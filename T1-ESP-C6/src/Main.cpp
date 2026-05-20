#include <Arduino.h>
#include <CodeCell.h>
#include <WiFi.h>
#include <WebServer.h>

CodeCell myCodeCell;

// ======================================================
// WiFi AP
// ======================================================

const char* AP_SSID = "CodeCell-C6-Drive";
const char* AP_PASS = "12345678";

WebServer server(80);

// ======================================================
// Types
// ======================================================

enum ControlMode
{
  CONTROL_DIRECT,
  CONTROL_TARGET
};

struct RobotCommand
{
  ControlMode mode;

  int8_t move;
  int8_t rotation;

  float targetDistanceM;
  float targetRotationDeg;

  uint32_t sequence;
};

struct MotorOutput
{
  bool m1Forward;
  bool m2Forward;

  uint8_t m1Speed;
  uint8_t m2Speed;
};

struct RobotStatus
{
  uint16_t batteryMv;
  uint16_t batteryPercent;

  uint32_t uptimeMs;

  float headingDeg;

  int8_t move;
  int8_t rotation;

  int16_t m1Signed;
  int16_t m2Signed;
};

// ======================================================
// Shared buffers
// ======================================================

RobotCommand commandBuffer =
{
  CONTROL_DIRECT,
  0,
  0,
  0.0f,
  0.0f,
  0
};

MotorOutput motorBuffer =
{
  true,
  true,
  0,
  0
};

RobotStatus statusBuffer;

SemaphoreHandle_t commandMutex;
SemaphoreHandle_t motorMutex;
SemaphoreHandle_t statusMutex;

uint32_t startTimeMs;

// ======================================================
// Helpers
// ======================================================

void calculateMotorMix(
  int8_t move,
  int8_t rotation,
  int16_t &m1,
  int16_t &m2
)
{
  move = constrain(move, -100, 100);
  rotation = constrain(rotation, -100, 100);

  m1 = move + rotation;
  m2 = move - rotation;

  m1 = constrain(m1, -100, 100);
  m2 = constrain(m2, -100, 100);
}

// ======================================================
// Start boost ramp
// ======================================================

int16_t applyStartRamp(
  int16_t targetSpeed,
  uint32_t elapsedMs
)
{
  const int START_BOOST = 70;
  const int RAMP_TIME_MS = 500;

  if (targetSpeed == 0)
    return 0;

  int sign = targetSpeed >= 0 ? 1 : -1;

  int absTarget = abs(targetSpeed);

  if (elapsedMs >= RAMP_TIME_MS)
    return targetSpeed;

  float t = (float)elapsedMs / (float)RAMP_TIME_MS;

  // squared decay
  float decay = 1.0f - (t * t);

  int boosted =
    absTarget +
    (START_BOOST - absTarget) * decay;

  boosted = constrain(boosted, absTarget, 100);

  return boosted * sign;
}

// ======================================================
// Command access
// ======================================================

void setDirectControl(
  int8_t move,
  int8_t rotation
)
{
  if (xSemaphoreTake(commandMutex, portMAX_DELAY) == pdTRUE)
  {

    commandBuffer.mode = CONTROL_DIRECT;

    commandBuffer.move =
      constrain(move, -100, 100);

    commandBuffer.rotation =
      constrain(rotation, -100, 100);

    commandBuffer.sequence++;

    xSemaphoreGive(commandMutex);
  }
}

void setTargetControl(
  float distanceM,
  float rotationDeg
)
{
  if (xSemaphoreTake(commandMutex, portMAX_DELAY) == pdTRUE)
  {

    commandBuffer.mode = CONTROL_TARGET;

    commandBuffer.targetDistanceM = distanceM;
    commandBuffer.targetRotationDeg = rotationDeg;

    commandBuffer.sequence++;

    xSemaphoreGive(commandMutex);
  }
}

// ======================================================
// Motor output access
// ======================================================

void writeMotorOutput(
  int16_t m1Signed,
  int16_t m2Signed
)
{
  if (xSemaphoreTake(motorMutex, portMAX_DELAY) == pdTRUE)
  {

    motorBuffer.m1Forward = m1Signed >= 0;
    motorBuffer.m2Forward = m2Signed >= 0;

    motorBuffer.m1Speed = abs(m1Signed);
    motorBuffer.m2Speed = abs(m2Signed);

    xSemaphoreGive(motorMutex);
  }
}

MotorOutput readMotorOutput()
{
  MotorOutput out;

  if (xSemaphoreTake(motorMutex, portMAX_DELAY) == pdTRUE)
  {
    out = motorBuffer;
    xSemaphoreGive(motorMutex);
  }

  return out;
}

// ======================================================
// Debug print
// ======================================================

void debugMotorState(
  const char* source,
  int8_t move,
  int8_t rotation,
  int16_t m1,
  int16_t m2
) {

  Serial.println("================================");

  Serial.print("SOURCE: ");
  Serial.println(source);

  Serial.print("MOVE: ");
  Serial.println(move);

  Serial.print("ROTATION: ");
  Serial.println(rotation);

  Serial.print("M1 SIGNED: ");
  Serial.println(m1);

  Serial.print("M2 SIGNED: ");
  Serial.println(m2);

  Serial.print("M1 DIR: ");
  Serial.println(m1 >= 0 ? "FORWARD" : "REVERSE");

  Serial.print("M2 DIR: ");
  Serial.println(m2 >= 0 ? "FORWARD" : "REVERSE");

  Serial.print("M1 PWM: ");
  Serial.println(abs(m1));

  Serial.print("M2 PWM: ");
  Serial.println(abs(m2));

  Serial.println("================================");
}

// ======================================================
// HTTP handlers
// ======================================================

void handleMoveCommand()
{

  if (!server.hasArg("move") ||
      !server.hasArg("rotation"))
  {

    server.send(
      400,
      "text/plain",
      "ERROR: /move?move=-100..100&rotation=-100..100"
    );

    return;
  }

  int move = server.arg("move").toInt();
  int rotation = server.arg("rotation").toInt();

  setDirectControl(move, rotation);

  server.send(
    200,
    "text/plain",
    "OK: direct control updated"
  );
}

void handleTargetCommand()
{

  if (!server.hasArg("distance") ||
      !server.hasArg("angle"))
  {

    server.send(
      400,
      "text/plain",
      "ERROR: /target?distance=1.0&angle=90"
    );

    return;
  }

  float distanceM =
    server.arg("distance").toFloat();

  float angleDeg =
    server.arg("angle").toFloat();

  setTargetControl(distanceM, angleDeg);

  server.send(
    200,
    "text/plain",
    "OK: target control updated"
  );
}

void handleStatus()
{

  RobotStatus s;

  if (xSemaphoreTake(statusMutex, portMAX_DELAY) == pdTRUE)
  {
    s = statusBuffer;
    xSemaphoreGive(statusMutex);
  }

  String reply;

  reply += "Battery mV: ";
  reply += String(s.batteryMv);
  reply += "\n";

  reply += "Battery %: ";
  reply += String(s.batteryPercent);
  reply += "\n";

  reply += "Uptime s: ";
  reply += String(s.uptimeMs / 1000.0f, 1);
  reply += "\n";

  reply += "Heading deg: ";
  reply += String(s.headingDeg, 1);
  reply += "\n";

  reply += "Move: ";
  reply += String(s.move);
  reply += "\n";

  reply += "Rotation: ";
  reply += String(s.rotation);
  reply += "\n";

  reply += "M1: ";
  reply += String(s.m1Signed);
  reply += "\n";

  reply += "M2: ";
  reply += String(s.m2Signed);
  reply += "\n";

  server.send(200, "text/plain", reply);
}

// ======================================================
// Motor task
// Only writes to hardware
// ======================================================

void motorTask(void* parameter)
{

  while (true)
  {

    MotorOutput out = readMotorOutput();

    myCodeCell.Drive1.Drive(
      !out.m1Forward,
      out.m1Speed
    );

    myCodeCell.Drive2.Drive(
      out.m2Forward,
      out.m2Speed
    );

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// ======================================================
// Control task
// All motion logic here
// ======================================================

void controlTask(void* parameter)
{

  const float MS_PER_METER = 2500.0f;
  const float MS_PER_DEGREE = 10.0f;

  const int8_t TARGET_MOVE_SPEED = 50;
  const int8_t TARGET_ROTATE_SPEED = 40;

  uint32_t lastSeq = 0;

  uint32_t targetStartMs = 0;

  bool targetRunning = false;

  uint32_t rotateDurationMs = 0;
  uint32_t moveDurationMs = 0;

  uint32_t motionStartMs = 0;
  bool wasMoving = false;

  while (true)
  {
    RobotCommand cmd;

    if (xSemaphoreTake(commandMutex, portMAX_DELAY) == pdTRUE)
    {
      cmd = commandBuffer;
      xSemaphoreGive(commandMutex);
    }

    int16_t m1 = 0;
    int16_t m2 = 0;

    // ==========================================
    // DIRECT MODE
    // ==========================================

    if (cmd.mode == CONTROL_DIRECT)
    {
      if (cmd.mode == CONTROL_DIRECT) {

        calculateMotorMix(
          cmd.move,
          cmd.rotation,
          m1,
          m2
        );

        debugMotorState(
          "DIRECT",
          cmd.move,
          cmd.rotation,
          m1,
          m2
        );

      }
    }

    // ==========================================
    // TARGET MODE
    // ==========================================

    else if (cmd.mode == CONTROL_TARGET)
    {
      if (cmd.sequence != lastSeq)
      {
        lastSeq = cmd.sequence;

        targetStartMs = millis();

        rotateDurationMs =
          abs(cmd.targetRotationDeg) *
          MS_PER_DEGREE;

        moveDurationMs =
          abs(cmd.targetDistanceM) *
          MS_PER_METER;

        targetRunning = true;
      }

      uint32_t elapsed =
        millis() - targetStartMs;

      // rotate phase
      if (targetRunning &&
          elapsed < rotateDurationMs)
      {

        int8_t rot =
          cmd.targetRotationDeg >= 0
          ? TARGET_ROTATE_SPEED
          : -TARGET_ROTATE_SPEED;

        calculateMotorMix(
          0,
          rot,
          m1,
          m2
        );

        debugMotorState(
           "TARGET_ROTATE",
          0,
          rot,
          m1,
          m2
        );
      }

      // move phase
      else if (targetRunning &&
               elapsed <
               rotateDurationMs +
               moveDurationMs)
      {

        int8_t mv =
          cmd.targetDistanceM >= 0
          ? TARGET_MOVE_SPEED
          : -TARGET_MOVE_SPEED;

        calculateMotorMix(
          mv,
          0,
          m1,
          m2
        );
      }

      else
      {
        m1 = 0;
        m2 = 0;
        targetRunning = false;
      }
    }

    // ==========================================
    // Start boost ramp
    // ==========================================

    bool movingNow =
      (m1 != 0 || m2 != 0);

    if (movingNow && !wasMoving)
    {
      motionStartMs = millis();
    }

    uint32_t motionElapsed =
      millis() - motionStartMs;

    m1 = applyStartRamp(
      m1,
      motionElapsed
    );

    m2 = applyStartRamp(
      m2,
      motionElapsed
    );

    Serial.print("BOOSTED M1: ");

    Serial.println(m1);

    Serial.print("BOOSTED M2: ");

    Serial.println(m2);

    Serial.print("BOOST ELAPSED MS: ");

    Serial.println(motionElapsed);

    wasMoving = movingNow;

    // ==========================================
    // Write motor output
    // ==========================================

    writeMotorOutput(m1, m2);

    // ==========================================
    // Sensor/status update
    // ==========================================

    myCodeCell.Run(20);

    float roll = 0;
    float pitch = 0;
    float yaw = 0;

    myCodeCell.Motion_RotationRead(
      roll,
      pitch,
      yaw
    );

    if (xSemaphoreTake(statusMutex, portMAX_DELAY) == pdTRUE) {

      statusBuffer.batteryMv =
        myCodeCell.BatteryVoltageRead();

      statusBuffer.batteryPercent =
        myCodeCell.BatteryLevelRead();

      statusBuffer.uptimeMs =
        millis() - startTimeMs;

      statusBuffer.headingDeg = yaw;

      statusBuffer.move = cmd.move;
      statusBuffer.rotation = cmd.rotation;

      statusBuffer.m1Signed = m1;
      statusBuffer.m2Signed = m2;

      xSemaphoreGive(statusMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// ======================================================
// Communication task
// ======================================================

void communicationTask(void* parameter)
{

  WiFi.mode(WIFI_AP);

  WiFi.softAP(AP_SSID, AP_PASS);

  Serial.println("Hotspot started");
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());

  server.on(
    "/move",
    HTTP_GET,
    handleMoveCommand
  );

  server.on(
    "/target",
    HTTP_GET,
    handleTargetCommand
  );

  server.on(
    "/status",
    HTTP_GET,
    handleStatus
  );

  server.begin();

  while (true)
  {

    server.handleClient();

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ======================================================
// Setup
// ======================================================

void setup()
{

  Serial.begin(115200);

  delay(500);

  startTimeMs = millis();

  commandMutex =
    xSemaphoreCreateMutex();

  motorMutex =
    xSemaphoreCreateMutex();

  statusMutex =
    xSemaphoreCreateMutex();

  myCodeCell.Init(
    LIGHT +
    MOTION_ROTATION
  );

  myCodeCell.Drive1.Init();
  myCodeCell.Drive2.Init();

  setDirectControl(0, 0);

  xTaskCreate(
    motorTask,
    "MotorTask",
    2048,
    NULL,
    3,
    NULL
  );

  xTaskCreate(
    controlTask,
    "ControlTask",
    4096,
    NULL,
    2,
    NULL
  );

  xTaskCreate(
    communicationTask,
    "CommunicationTask",
    8192,
    NULL,
    1,
    NULL
  );
}

// ======================================================
// Loop
// ======================================================

void loop()
{
  vTaskDelay(pdMS_TO_TICKS(1000));
}