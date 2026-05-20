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

void controlDirect(const RobotCommand &cmd, int16_t &m1, int16_t &m2) {
  calculateMotorMix(cmd.move, cmd.rotation, m1, m2);
  debugMotorState("DIRECT", cmd.move, cmd.rotation, m1, m2);
}

void controlTargetByYawAndTime(
  const RobotCommand &cmd,
  int16_t &m1,
  int16_t &m2,
  bool &targetRunning,
  uint32_t &targetStartMs,
  float &targetYaw,
  uint32_t &moveStartMs
) {
  const float ANGLE_TOLERANCE_DEG = 8.0f;
  const float MS_PER_METER = 2500.0f;
  const int8_t TARGET_MOVE_SPEED = 50;
  const int8_t TARGET_ROTATE_SPEED = 50;

  float yaw = readYawDeg();
  float err = angleError(targetYaw, yaw);

  if (abs(err) > ANGLE_TOLERANCE_DEG) {
    int8_t rot = err > 0 ? TARGET_ROTATE_SPEED : -TARGET_ROTATE_SPEED;
    calculateMotorMix(0, rot, m1, m2);
    moveStartMs = 0;
    debugMotorState("TARGET_YAW", 0, rot, m1, m2);
    return;
  }

  if (moveStartMs == 0) moveStartMs = millis();

  uint32_t moveDurationMs = abs(cmd.targetDistanceM) * MS_PER_METER;
  uint32_t moveElapsed = millis() - moveStartMs;

  if (moveElapsed < moveDurationMs) {
    int8_t mv = cmd.targetDistanceM >= 0 ? TARGET_MOVE_SPEED : -TARGET_MOVE_SPEED;
    calculateMotorMix(mv, 0, m1, m2);
    debugMotorState("TARGET_DISTANCE_TIME", mv, 0, m1, m2);
  } else {
    m1 = 0;
    m2 = 0;
    targetRunning = false;
  }
}

void controlTargetByYawAndImu(
  const RobotCommand &cmd,
  int16_t &m1,
  int16_t &m2,
  bool &targetRunning,
  float &targetYaw,
  float &velocityMps,
  float &distanceM,
  uint32_t &lastImuMs
) {
  const float ANGLE_TOLERANCE_DEG = 16.0f;
  const int8_t TARGET_MOVE_SPEED = 50;
  const int8_t TARGET_ROTATE_SPEED = 50;

  float yaw = readYawDeg();
  float err = angleError(targetYaw, yaw);

  if (abs(err) > ANGLE_TOLERANCE_DEG) {
    int8_t rot = err > 0 ? TARGET_ROTATE_SPEED : -TARGET_ROTATE_SPEED;
    calculateMotorMix(0, rot, m1, m2);
    debugMotorState("TARGET_YAW_IMU", 0, rot, m1, m2);
    lastImuMs = millis();
    return;
  }

  uint32_t now = millis();
  float dt = (now - lastImuMs) / 1000.0f;
  lastImuMs = now;

  float ax = 0, ay = 0, az = 0;
  myCodeCell.Motion_LinearAccRead(ax, ay, az);

  // Change this axis if your robot forward direction is different.
  float forwardAcc = ax;

  velocityMps += forwardAcc * dt;
  distanceM += velocityMps * dt;

  if (abs(distanceM) < abs(cmd.targetDistanceM)) {
    int8_t mv = cmd.targetDistanceM >= 0 ? TARGET_MOVE_SPEED : -TARGET_MOVE_SPEED;
    calculateMotorMix(mv, 0, m1, m2);
    debugMotorState("TARGET_DISTANCE_IMU", mv, 0, m1, m2);
  } else {
    m1 = 0;
    m2 = 0;
    velocityMps = 0;
    targetRunning = false;
  }
}

// ======================================================
// HTTP
// ======================================================

void handleEmergencyStop() {
  emergencyStopNow();

  server.send(
    200,
    "text/plain",
    "EMERGENCY STOP ACTIVATED"
  );
}

void handleEmergencyClear()
{
  clearEmergencyStop();

  server.send(
    200,
    "text/plain",
    "EMERGENCY STOP CLEARED"
  );
}

void handleMoveCommand() {
  if (!server.hasArg("move") || !server.hasArg("rotation")) {
    server.send(400, "text/plain", "ERROR: /move?move=-100..100&rotation=-100..100");
    return;
  }

  setDirectControl(server.arg("move").toInt(), server.arg("rotation").toInt());
  server.send(200, "text/plain", "OK: direct control updated");
}

void handleTargetCommand() {
  if (!server.hasArg("distance") || !server.hasArg("angle")) {
    server.send(400, "text/plain", "ERROR: /target?distance=1.0&angle=90&imu=0");
    return;
  }

  float distanceM = server.arg("distance").toFloat();
  float angleDeg = server.arg("angle").toFloat();
  bool useImu = server.hasArg("imu") && server.arg("imu").toInt() == 1;

  setTargetControl(distanceM, angleDeg, useImu);

  server.send(200, "text/plain", useImu ? "OK: target IMU mode" : "OK: target time mode");
}

void handleStatus() {
  RobotStatus s;

  if (xSemaphoreTake(statusMutex, portMAX_DELAY) == pdTRUE) {
    s = statusBuffer;
    xSemaphoreGive(statusMutex);
  }

  String reply;
  reply += "Battery mV: " + String(s.batteryMv) + "\n";
  reply += "Battery %: " + String(s.batteryPercent) + "\n";
  reply += "Uptime s: " + String(s.uptimeMs / 1000.0f, 1) + "\n";
  reply += "Heading deg: " + String(s.headingDeg, 1) + "\n";
  reply += "Estimated distance m: " + String(s.estimatedDistanceM, 3) + "\n";
  reply += "Move: " + String(s.move) + "\n";
  reply += "Rotation: " + String(s.rotation) + "\n";
  reply += "M1: " + String(s.m1Signed) + "\n";
  reply += "M2: " + String(s.m2Signed) + "\n";

  server.send(200, "text/plain", reply);
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
  uint32_t targetStartMs = 0;
  uint32_t moveStartMs = 0;
  float targetYaw = 0;

  float imuVelocityMps = 0;
  float imuDistanceM = 0;
  uint32_t lastImuMs = 0;

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
      targetStartMs = millis();
      moveStartMs = 0;

      imuVelocityMps = 0;
      imuDistanceM = 0;
      lastImuMs = millis();

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
      if (cmd.useImuDistance) {
        controlTargetByYawAndImu(
          cmd, m1, m2, targetRunning,
          targetYaw, imuVelocityMps, imuDistanceM, lastImuMs
        );
      } else {
        controlTargetByYawAndTime(
          cmd, m1, m2, targetRunning,
          targetStartMs, targetYaw, moveStartMs
        );
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
      statusBuffer.estimatedDistanceM = cmd.useImuDistance ? imuDistanceM : 0.0f;
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