#include <Arduino.h>
#include <CodeCell.h>
#include <WiFi.h>
#include <WebServer.h>

CodeCell myCodeCell;

const char* AP_SSID = "CodeCell-C6-Drive";
const char* AP_PASS = "12345678";

WebServer server(80);

enum ControlMode {
  CONTROL_DIRECT,
  CONTROL_TARGET
};

enum TargetPhase {
  TARGET_IDLE,
  TARGET_PRE_ROTATE,
  TARGET_MOVE_PI
};

enum DistanceMethod {
  DISTANCE_TIME,
  DISTANCE_IMU
};

struct RobotCommand {
  ControlMode mode;
  int8_t move;
  int8_t rotation;
  float targetDistanceM;
  float targetRotationDeg;
  bool useImuDistance;
  uint32_t sequence;
};

struct MotorOutput {
  bool m1Forward;
  bool m2Forward;
  uint8_t m1Speed;
  uint8_t m2Speed;
};

struct RobotStatus {
  uint16_t batteryMv;
  uint16_t batteryPercent;
  uint32_t uptimeMs;
  float headingDeg;
  float estimatedDistanceM;
  int8_t move;
  int8_t rotation;
  int16_t m1Signed;
  int16_t m2Signed;
};

RobotCommand commandBuffer = {
  CONTROL_DIRECT, 0, 0, 0.0f, 0.0f, false, 0
};

MotorOutput motorBuffer = {true, true, 0, 0};
RobotStatus statusBuffer;

SemaphoreHandle_t commandMutex;
SemaphoreHandle_t motorMutex;
SemaphoreHandle_t statusMutex;

uint32_t startTimeMs;

volatile bool emergencyStop = false;

// ======================================================
// Helpers
// ======================================================

float angleError(float target, float current) {
  float error = target - current;
  while (error > 180.0f) error -= 360.0f;
  while (error < -180.0f) error += 360.0f;
  return error;
}

float readYawDeg() {
  float roll = 0, pitch = 0, yaw = 0;
  myCodeCell.Motion_RotationRead(roll, pitch, yaw);
  return yaw;
}

void calculateMotorMix(int8_t move, int8_t rotation, int16_t &m1, int16_t &m2) {
  move = constrain(move, -100, 100);
  rotation = constrain(rotation, -100, 100);

  m1 = constrain(move + rotation, -100, 100);
  m2 = constrain(move - rotation, -100, 100);
}

int16_t applyStartRamp(int16_t targetSpeed, uint32_t elapsedMs) {
  const int START_BOOST = 70;
  const int RAMP_TIME_MS = 500;

  if (targetSpeed == 0) return 0;

  int sign = targetSpeed >= 0 ? 1 : -1;
  int absTarget = abs(targetSpeed);

  if (elapsedMs >= RAMP_TIME_MS) return targetSpeed;

  float t = (float)elapsedMs / (float)RAMP_TIME_MS;
  float decay = 1.0f - (t * t);

  int boosted = absTarget + (START_BOOST - absTarget) * decay;
  boosted = constrain(boosted, absTarget, 100);

  return boosted * sign;
}

void debugMotorState(const char* source, int8_t move, int8_t rotation, int16_t m1, int16_t m2) {
  Serial.println("================================");
  Serial.print("SOURCE: "); Serial.println(source);
  Serial.print("MOVE: "); Serial.println(move);
  Serial.print("ROTATION: "); Serial.println(rotation);
  Serial.print("M1 SIGNED: "); Serial.println(m1);
  Serial.print("M2 SIGNED: "); Serial.println(m2);
  Serial.print("M1 PWM: "); Serial.println(abs(m1));
  Serial.print("M2 PWM: "); Serial.println(abs(m2));
  Serial.println("================================");
}

void sendJson(int code, const String& json) {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
  server.send(code, "application/json", json);
}

void handleCorsOptions()
{
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
  server.send(204);
}

// ======================================================
// Shared buffers
// ======================================================

void setDirectControl(int8_t move, int8_t rotation) {
  if (xSemaphoreTake(commandMutex, portMAX_DELAY) == pdTRUE) {
    commandBuffer.mode = CONTROL_DIRECT;
    commandBuffer.move = constrain(move, -100, 100);
    commandBuffer.rotation = constrain(rotation, -100, 100);
    commandBuffer.sequence++;
    xSemaphoreGive(commandMutex);
  }
}

void setTargetControl(float distanceM, float rotationDeg, bool useImuDistance) {
  if (xSemaphoreTake(commandMutex, portMAX_DELAY) == pdTRUE) {
    commandBuffer.mode = CONTROL_TARGET;
    commandBuffer.targetDistanceM = distanceM;
    commandBuffer.targetRotationDeg = rotationDeg;
    commandBuffer.useImuDistance = useImuDistance;
    commandBuffer.sequence++;
    xSemaphoreGive(commandMutex);
  }
}

void writeMotorOutput(int16_t m1Signed, int16_t m2Signed) {
  if (xSemaphoreTake(motorMutex, portMAX_DELAY) == pdTRUE) {
    motorBuffer.m1Forward = m1Signed >= 0;
    motorBuffer.m2Forward = m2Signed >= 0;
    motorBuffer.m1Speed = abs(m1Signed);
    motorBuffer.m2Speed = abs(m2Signed);
    xSemaphoreGive(motorMutex);
  }
}

MotorOutput readMotorOutput() {
  MotorOutput out;
  if (xSemaphoreTake(motorMutex, portMAX_DELAY) == pdTRUE) {
    out = motorBuffer;
    xSemaphoreGive(motorMutex);
  }
  return out;
}

void emergencyStopNow() {
  emergencyStop = true;

  writeMotorOutput(0, 0);

  Serial.println("################################");
  Serial.println("### EMERGENCY STOP ACTIVATED ###");
  Serial.println("################################");
}

void clearEmergencyStop() {
  emergencyStop = false;
  Serial.println("Emergency stop cleared");
}

// ======================================================
// Control functions
// ======================================================

struct DistanceState {
  uint32_t startMs;
  uint32_t lastImuMs;
  float imuVelocityMps;
  float imuDistanceM;
};

void resetDistanceState(DistanceState &state) {
  state.startMs = millis();
  state.lastImuMs = millis();
  state.imuVelocityMps = 0.0f;
  state.imuDistanceM = 0.0f;
}

float getCurrentDistanceM(DistanceMethod method,DistanceState &state) {
  const float MS_PER_METER = 2500.0f;

  if (method == DISTANCE_TIME) {
    uint32_t elapsedMs = millis() - state.startMs;
    return (float)elapsedMs / MS_PER_METER;
  }

  return state.imuDistanceM;
}

void controlDirect(const RobotCommand &cmd, int16_t &m1, int16_t &m2) {
  calculateMotorMix(cmd.move, cmd.rotation, m1, m2);
  debugMotorState("DIRECT", cmd.move, cmd.rotation, m1, m2);
}


void controlMoveWithPiYaw(
  const RobotCommand &cmd,
  float targetYaw,
  DistanceState &distanceState,
  int16_t &m1,
  int16_t &m2,
  bool &targetRunning,
  float &yawIntegral,
  uint32_t &lastPiMs
) {
  const int8_t BASE_MOVE_SPEED = 50;

  const float KP = 0.2f;
  const float KI = 0.02f;
  const float MAX_CORRECTION = 35.0f;

  DistanceMethod method =
    cmd.useImuDistance ? DISTANCE_IMU : DISTANCE_TIME;

  float currentDistance =
    getCurrentDistanceM(method, distanceState);

  if (abs(currentDistance) >= abs(cmd.targetDistanceM)) {
    m1 = 0;
    m2 = 0;
    targetRunning = false;
    return;
  }

  uint32_t now = millis();
  float dt = (now - lastPiMs) / 1000.0f;
  lastPiMs = now;

  float yaw = readYawDeg();
  float err = angleError(targetYaw, yaw);

  yawIntegral += err * dt;
  yawIntegral = constrain(yawIntegral, -100.0f, 100.0f);

  float correction = KP * err + KI * yawIntegral;
  correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);

  int8_t move =
    cmd.targetDistanceM >= 0
    ? BASE_MOVE_SPEED
    : -BASE_MOVE_SPEED;

  int8_t rotationCorrection =
    constrain((int)correction, -100, 100);

  calculateMotorMix(move, rotationCorrection, m1, m2);

  debugMotorState(
    "MOVE_PI_YAW",
    move,
    rotationCorrection,
    m1,
    m2
  );
}

void controlPreliminaryRotation(
  float targetYaw,
  TargetPhase &targetPhase,
  int16_t &m1,
  int16_t &m2,
  DistanceState &distanceState,
  float &yawIntegral,
  uint32_t &lastPiMs
) {
  const float ANGLE_TOLERANCE_DEG = 8.0f;
  const int8_t ROTATE_SPEED = 50;

  float yaw = readYawDeg();
  float err = angleError(targetYaw, yaw);

  if (abs(err) <= ANGLE_TOLERANCE_DEG) {
    m1 = 0;
    m2 = 0;

    targetPhase = TARGET_MOVE_PI;

    resetDistanceState(distanceState);
    yawIntegral = 0.0f;
    lastPiMs = millis();

    return;
  }

  int8_t rot =
    err > 0
    ? ROTATE_SPEED
    : -ROTATE_SPEED;

  calculateMotorMix(0, rot, m1, m2);

  debugMotorState(
    "PRE_ROTATE",
    0,
    rot,
    m1,
    m2
  );
}

// ======================================================
// HTTP
// ======================================================

void handleEmergencyStop() {
  emergencyStopNow();

  sendJson(
    200,
    "{\"ok\":true,\"emergencyStop\":true,\"message\":\"EMERGENCY STOP ACTIVATED\"}"
  );
}

void handleEmergencyClear() {
  clearEmergencyStop();

  sendJson(
    200,
    "{\"ok\":true,\"emergencyStop\":false,\"message\":\"EMERGENCY STOP CLEARED\"}"
  );
}

void handleMoveCommand() {
  if (!server.hasArg("move") || !server.hasArg("rotation")) {
    sendJson(
      400,
      "{\"ok\":false,\"error\":\"Use /move?move=-100..100&rotation=-100..100\"}"
    );
    return;
  }

  int move = constrain(server.arg("move").toInt(), -100, 100);
  int rotation = constrain(server.arg("rotation").toInt(), -100, 100);

  setDirectControl(move, rotation);

  String json = "{";
  json += "\"ok\":true,";
  json += "\"mode\":\"direct\",";
  json += "\"move\":" + String(move) + ",";
  json += "\"rotation\":" + String(rotation);
  json += "}";

  sendJson(200, json);
}

void handleTargetCommand() {
  if (!server.hasArg("distance") || !server.hasArg("angle")) {
    sendJson(
      400,
      "{\"ok\":false,\"error\":\"Use /target?distance=1.0&angle=90&imu=0\"}"
    );
    return;
  }

  float distanceM = server.arg("distance").toFloat();
  float angleDeg = server.arg("angle").toFloat();
  bool useImu = server.hasArg("imu") && server.arg("imu").toInt() == 1;

  setTargetControl(distanceM, angleDeg, useImu);

  String json = "{";
  json += "\"ok\":true,";
  json += "\"mode\":\"target\",";
  json += "\"distanceM\":" + String(distanceM, 3) + ",";
  json += "\"angleDeg\":" + String(angleDeg, 1) + ",";
  json += "\"useImu\":" + String(useImu ? "true" : "false");
  json += "}";

  sendJson(200, json);
}

void handleStatus() {
  RobotStatus s;

  if (xSemaphoreTake(statusMutex, portMAX_DELAY) == pdTRUE) {
    s = statusBuffer;
    xSemaphoreGive(statusMutex);
  }

  String json = "{";
  json += "\"ok\":true,";
  json += "\"batteryMv\":" + String(s.batteryMv) + ",";
  json += "\"batteryPercent\":" + String(s.batteryPercent) + ",";
  json += "\"uptimeS\":" + String(s.uptimeMs / 1000.0f, 1) + ",";
  json += "\"headingDeg\":" + String(s.headingDeg, 1) + ",";
  json += "\"estimatedDistanceM\":" + String(s.estimatedDistanceM, 3) + ",";
  json += "\"move\":" + String(s.move) + ",";
  json += "\"rotation\":" + String(s.rotation) + ",";
  json += "\"m1\":" + String(s.m1Signed) + ",";
  json += "\"m2\":" + String(s.m2Signed);
  json += "}";

  sendJson(200, json);
}

// ======================================================
// Tasks
// ======================================================

void motorTask(void* parameter) {

  while (true) {
    if (emergencyStop) {

      myCodeCell.Drive1.Drive(true, 0);
      myCodeCell.Drive2.Drive(true, 0);

      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    MotorOutput out = readMotorOutput();

    myCodeCell.Drive1.Drive(!out.m1Forward, out.m1Speed);
    myCodeCell.Drive2.Drive(out.m2Forward, out.m2Speed);

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void controlTask(void* parameter) {
  uint32_t lastSeq = 0;
  uint32_t motionStartMs = 0;
  bool wasMoving = false;

  bool targetRunning = false;
  uint32_t moveStartMs = 0;
  float targetYaw = 0.0f;

  DistanceState distanceState;

  float yawIntegral = 0.0f;
  uint32_t lastPiMs = 0;
  TargetPhase targetPhase = TARGET_IDLE;

  while (true) {
    if (emergencyStop) {

      writeMotorOutput(0, 0);

      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    myCodeCell.Run(20);

    RobotCommand cmd;

    if (xSemaphoreTake(commandMutex, portMAX_DELAY) == pdTRUE) {
      cmd = commandBuffer;
      xSemaphoreGive(commandMutex);
    }

    int16_t m1 = 0;
    int16_t m2 = 0;

    if (cmd.sequence != lastSeq) {
      lastSeq = cmd.sequence;
      targetRunning = true;
      targetPhase = TARGET_PRE_ROTATE;

      resetDistanceState(distanceState);

      yawIntegral = 0.0f;
      lastPiMs = millis();

      targetYaw = readYawDeg() + cmd.targetRotationDeg;

      while (targetYaw > 180.0f) targetYaw -= 360.0f;
      while (targetYaw < -180.0f) targetYaw += 360.0f;

      Serial.print("NEW COMMAND SEQ: ");
      Serial.println(cmd.sequence);
    }

    if (cmd.mode == CONTROL_DIRECT) {
      targetRunning = false;
      controlDirect(cmd, m1, m2);
    }
    else if (cmd.mode == CONTROL_TARGET && targetRunning) {
      Serial.print("Target Phase:  ");  Serial.println(targetPhase);

      if (targetPhase == TARGET_PRE_ROTATE) {
        controlPreliminaryRotation(
          targetYaw,
          targetPhase,
          m1,
          m2,
          distanceState,
          yawIntegral,
          lastPiMs
        );
      }

      else if (targetPhase == TARGET_MOVE_PI) {
        controlMoveWithPiYaw(
          cmd,
          targetYaw,
          distanceState,
          m1,
          m2,
          targetRunning,
          yawIntegral,
          lastPiMs
        );
      }

      else {
        m1 = 0;
        m2 = 0;
        targetRunning = false;
      }
    }

    bool movingNow = (m1 != 0 || m2 != 0);

    if (movingNow && !wasMoving) {
      motionStartMs = millis();
    }

    uint32_t motionElapsed = millis() - motionStartMs;

    m1 = applyStartRamp(m1, motionElapsed);
    m2 = applyStartRamp(m2, motionElapsed);

    wasMoving = movingNow;

    writeMotorOutput(m1, m2);

    if (xSemaphoreTake(statusMutex, portMAX_DELAY) == pdTRUE) {
      statusBuffer.batteryMv = myCodeCell.BatteryVoltageRead();
      statusBuffer.batteryPercent = myCodeCell.BatteryLevelRead();
      statusBuffer.uptimeMs = millis() - startTimeMs;
      statusBuffer.headingDeg = readYawDeg();

      float currentDistanceM = getCurrentDistanceM(cmd.useImuDistance ? DISTANCE_IMU : DISTANCE_TIME,distanceState);

      statusBuffer.estimatedDistanceM = currentDistanceM;
      statusBuffer.move = cmd.move;
      statusBuffer.rotation = cmd.rotation;
      statusBuffer.m1Signed = m1;
      statusBuffer.m2Signed = m2;
      xSemaphoreGive(statusMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void communicationTask(void* parameter) {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  Serial.println("Hotspot started");
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());

  server.on("/move", HTTP_GET, handleMoveCommand);
  server.on("/target", HTTP_GET, handleTargetCommand);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/estop", HTTP_GET, handleEmergencyStop);
  server.on("/clear_estop", HTTP_GET, handleEmergencyClear);

  server.on("/move", HTTP_OPTIONS, handleCorsOptions);
  server.on("/target", HTTP_OPTIONS, handleCorsOptions);
  server.on("/status", HTTP_OPTIONS, handleCorsOptions);
  server.on("/estop", HTTP_OPTIONS, handleCorsOptions);
  server.on("/clear_estop", HTTP_OPTIONS, handleCorsOptions);

  server.begin();

  while (true) {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ======================================================
// Setup / loop
// ======================================================

void setup() {
  Serial.begin(115200);
  delay(500);

  startTimeMs = millis();

  commandMutex = xSemaphoreCreateMutex();
  motorMutex = xSemaphoreCreateMutex();
  statusMutex = xSemaphoreCreateMutex();

  myCodeCell.Init(
    LIGHT +
    MOTION_ROTATION +
    MOTION_LINEAR_ACC
  );

  myCodeCell.Drive1.Init();
  myCodeCell.Drive2.Init();

  setDirectControl(0, 0);

  xTaskCreate(motorTask, "MotorTask", 2048, NULL, 3, NULL);
  xTaskCreate(controlTask, "ControlTask", 4096, NULL, 2, NULL);
  xTaskCreate(communicationTask, "CommunicationTask", 8192, NULL, 1, NULL);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}