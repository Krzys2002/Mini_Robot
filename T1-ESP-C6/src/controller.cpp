//
// Created by Krzysztof Sawicki on 24/05/2026.
//

#include "controller.h"
#include "ActuatorSafety.hpp"
#include <cmath>

RobotCommand commandBuffer = {
  CONTROL_DIRECT, 0, 0, 0.0f, 0.0f, false, 0, 0
};

MotorOutput motorBuffer = {true, true, 0, 0};
RobotStatus statusBuffer;

SemaphoreHandle_t commandMutex;
SemaphoreHandle_t statusMutex;

uint32_t startTimeMs;

std::atomic<bool> emergencyStop{false};
static std::atomic<bool> runtimeFault{false};
static std::atomic<uint32_t> emergencyEpoch{0};

const uint32_t COMMAND_TIMEOUT_MS = 300;
static prototype::ActuatorSafety actuatorSafety{COMMAND_TIMEOUT_MS};
const float MAX_ANGLE_SETPOINT_RATE_DPS = 100.0f;

const float YAW_KP = 0.25f;
const float YAW_KI = 0.02f;
const float YAW_KD = 0.03f;
const float YAW_INTEGRAL_LIMIT = 120.0f;
const float MAX_YAW_CORRECTION = 45.0f;

const float IMU_ACCEL_DEADBAND_MPS2 = 0.08f;
const float IMU_STATIONARY_VELOCITY_DECAY = 0.90f;
const float IMU_MAX_DT_S = 0.10f;

static float debugYawError = 0.0f;
static float debugYawCorrection = 0.0f;

bool initController() {
  startTimeMs = millis();
  commandMutex = xSemaphoreCreateMutex();
  statusMutex = xSemaphoreCreateMutex();
  if (!commandMutex || !statusMutex) {
    controllerFaultNow();
    return false;
  }
  return true;
}
void controllerFaultNow() {
  runtimeFault.store(true);
  emergencyStop.store(true);
  // The actuator task tests these atomics even if a handoff mutex is unavailable.
}

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

bool setDirectControl(int8_t move, int8_t rotation) {
  if (!commandMutex || emergencyStop.load() || runtimeFault.load()
      || xSemaphoreTake(commandMutex, 0) != pdTRUE) return false;
  const auto now = millis();
  const auto generation = emergencyStop.load() ? 0 : actuatorSafety.accept(now);
  if (generation) {
    commandBuffer.mode = CONTROL_DIRECT;
    commandBuffer.move = constrain(move, -100, 100);
    commandBuffer.rotation = constrain(rotation, -100, 100);
    commandBuffer.updatedMs = now;
    commandBuffer.safetyGeneration = generation;
    ++commandBuffer.sequence;
  }
  xSemaphoreGive(commandMutex);
  return generation != 0;
}

bool setTargetControl(float distanceM, float rotationDeg, bool useImuDistance) {
  if (!std::isfinite(distanceM) || !std::isfinite(rotationDeg) || !commandMutex
      || emergencyStop.load() || runtimeFault.load()
      || xSemaphoreTake(commandMutex, 0) != pdTRUE) return false;
  const auto now = millis();
  const auto generation = emergencyStop.load() ? 0 : actuatorSafety.accept(now);
  if (generation) {
    commandBuffer.mode = CONTROL_TARGET;
    commandBuffer.targetDistanceM = distanceM;
    commandBuffer.targetRotationDeg = constrain(rotationDeg, -100.0f, 100.0f);
    commandBuffer.useImuDistance = useImuDistance;
    commandBuffer.updatedMs = now;
    commandBuffer.safetyGeneration = generation;
    ++commandBuffer.sequence;
  }
  xSemaphoreGive(commandMutex);
  return generation != 0;
}

void writeMotorOutput(int16_t m1Signed, int16_t m2Signed, uint64_t generation = 0) {
  if (!commandMutex || xSemaphoreTake(commandMutex, 0) != pdTRUE) return;
  if (emergencyStop.load() || runtimeFault.load() || !actuatorSafety.permits(generation, millis())) {
    motorBuffer = {true, true, 0, 0};
  } else {
    motorBuffer = {m1Signed >= 0, m2Signed >= 0,
      static_cast<uint8_t>(abs(m1Signed)), static_cast<uint8_t>(abs(m2Signed)), generation};
  }
  xSemaphoreGive(commandMutex);
}

RobotStatus readRobotStatus() {
  RobotStatus s{};

  if (xSemaphoreTake(statusMutex, 0) == pdTRUE) {
    s = statusBuffer;
    xSemaphoreGive(statusMutex);
  }

  return s;
}

void emergencyStopNow() {
  // Latch independently of command/control progress or a held handoff mutex.
  emergencyEpoch.fetch_add(1);
  emergencyStop.store(true);
  if (commandMutex && xSemaphoreTake(commandMutex, 0) == pdTRUE) {
    actuatorSafety.stop();
    motorBuffer = {true, true, 0, 0};
    xSemaphoreGive(commandMutex);
  }
  Serial.println("Emergency stop latched; actuator loop disables outputs");
}

bool clearEmergencyStop() {
  const auto epoch = emergencyEpoch.load();
  if (runtimeFault.load() || !commandMutex || xSemaphoreTake(commandMutex, 0) != pdTRUE) return false;
  // Clearing invalidates both pending intent and computed output. An old control
  // snapshot cannot replay after clear, even if a new command arrives meanwhile.
  actuatorSafety.clear();
  commandBuffer.move = commandBuffer.rotation = 0;
  commandBuffer.mode = CONTROL_DIRECT;
  commandBuffer.safetyGeneration = 0;
  ++commandBuffer.sequence;
  motorBuffer = {true, true, 0, 0};
  emergencyStop.store(false);
  const bool racedStop = epoch != emergencyEpoch.load();
  if (racedStop) emergencyStop.store(true);
  xSemaphoreGive(commandMutex);
  if (racedStop || runtimeFault.load()) return false;
  Serial.println("Emergency stop cleared to idle; new command required");
  return true;
}

void resetDistanceState(DistanceState &state) {
  state.startMs = millis();
  state.lastImuMs = millis();
  state.imuVelocityMps = 0.0f;
  state.imuDistanceM = 0.0f;
  state.imuAccelBiasMps2 = 0.0f;
  state.imuBiasReady = false;
}

float rateLimitAngle(float currentDeg, float targetDeg, float dtS) {
  float maxStep = MAX_ANGLE_SETPOINT_RATE_DPS * dtS;
  float err = angleError(targetDeg, currentDeg);
  err = constrain(err, -maxStep, maxStep);
  return currentDeg + err;
}

float wrapAngleDeg(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

float readForwardLinearAccelerationMps2() {
  float ax = 0, ay = 0, az = 0;
  myCodeCell.Motion_LinearAccRead(ax, ay, az);

  // CodeCell returns linear acceleration in m/s^2. For this robot the forward
  // axis is assumed to be sensor X. Change this to ay if your CodeCell is
  // mounted with Y pointing forward.
  return ax;
}

void updateImuDistance(DistanceState &state, bool moving) {
  uint32_t now = millis();
  if (state.lastImuMs == 0) {
    state.lastImuMs = now;
    return;
  }

  float dt = (now - state.lastImuMs) / 1000.0f;
  state.lastImuMs = now;

  if (dt <= 0.0f || dt > IMU_MAX_DT_S) return;

  float acc = readForwardLinearAccelerationMps2();

  if (!moving) {
    state.imuAccelBiasMps2 = state.imuBiasReady
      ? (0.98f * state.imuAccelBiasMps2 + 0.02f * acc)
      : acc;
    state.imuBiasReady = true;
    state.imuVelocityMps = 0.0f;
    return;
  }

  if (state.imuBiasReady) acc -= state.imuAccelBiasMps2;
  if (abs(acc) < IMU_ACCEL_DEADBAND_MPS2) acc = 0.0f;

  state.imuVelocityMps += acc * dt;
  if (acc == 0.0f) state.imuVelocityMps *= IMU_STATIONARY_VELOCITY_DECAY;

  state.imuDistanceM += state.imuVelocityMps * dt;
}

float getCurrentDistanceM(DistanceMethod method, DistanceState &state) {
  const float MS_PER_METER = 2500.0f;

  if (method == DISTANCE_TIME) {
    uint32_t elapsedMs = millis() - state.startMs;
    return (float)elapsedMs / MS_PER_METER;
  }

  return state.imuDistanceM;
}

int8_t yawPidCorrection(float targetYaw, float &yawIntegral, float &lastYawError, uint32_t &lastPidMs) {
  uint32_t now = millis();
  float dt = (now - lastPidMs) / 1000.0f;
  if (dt <= 0.0f || dt > 0.5f) dt = 0.02f;
  lastPidMs = now;

  float yaw = readYawDeg();
  float err = angleError(targetYaw, yaw);

  yawIntegral += err * dt;
  yawIntegral = constrain(yawIntegral, -YAW_INTEGRAL_LIMIT, YAW_INTEGRAL_LIMIT);

  float derivative = (err - lastYawError) / dt;
  lastYawError = err;

  float correction = YAW_KP * err + YAW_KI * yawIntegral + YAW_KD * derivative;
  correction = constrain(correction, -MAX_YAW_CORRECTION, MAX_YAW_CORRECTION);

  debugYawError = err;
  debugYawCorrection = correction;

  return constrain((int)correction, -100, 100);
}

void controlDirectRealtime(
  const RobotCommand &cmd,
  int16_t &m1,
  int16_t &m2,
  float &directYawSetpoint,
  float &yawIntegral,
  float &lastYawError,
  uint32_t &lastPidMs,
  uint32_t &lastSetpointMs
) {
  uint32_t now = millis();

  // Timeout only stops motors.
  // Do NOT reset directYawSetpoint.
  if (now - cmd.updatedMs >= COMMAND_TIMEOUT_MS) {
    m1 = 0;
    m2 = 0;
    return;
  }

  float dt = (now - lastSetpointMs) / 1000.0f;
  if (dt <= 0.0f || dt > 0.5f) dt = 0.02f;
  lastSetpointMs = now;

  // Joystick rotation is now relative yaw change, not absolute target.
  float yawDelta = (float)cmd.rotation * dt;
  directYawSetpoint = wrapAngleDeg(directYawSetpoint + yawDelta);

  int8_t correction = yawPidCorrection(
    directYawSetpoint,
    yawIntegral,
    lastYawError,
    lastPidMs
  );

  calculateMotorMix(cmd.move, correction, m1, m2);

  debugMotorState("DIRECT_RELATIVE_YAW_PID", cmd.move, correction, m1, m2);
}

void controlMoveWithPiYaw(
  const RobotCommand &cmd,
  float targetYaw,
  DistanceState &distanceState,
  int16_t &m1,
  int16_t &m2,
  bool &targetRunning,
  float &yawIntegral,
  float &lastYawError,
  uint32_t &lastPiMs
) {
  const int8_t BASE_MOVE_SPEED = 50;

  DistanceMethod method = cmd.useImuDistance ? DISTANCE_IMU : DISTANCE_TIME;

  float currentDistance = getCurrentDistanceM(method, distanceState);

  if (abs(currentDistance) >= abs(cmd.targetDistanceM)) {
    m1 = 0;
    m2 = 0;
    targetRunning = false;
    return;
  }

  int8_t move = cmd.targetDistanceM >= 0 ? BASE_MOVE_SPEED : -BASE_MOVE_SPEED;
  int8_t rotationCorrection = yawPidCorrection(targetYaw, yawIntegral, lastYawError, lastPiMs);

  calculateMotorMix(move, rotationCorrection, m1, m2);

  debugMotorState("MOVE_PI_YAW", move, rotationCorrection, m1, m2);
}

void controlPreliminaryRotation(
  float targetYaw,
  TargetPhase &targetPhase,
  int16_t &m1,
  int16_t &m2,
  DistanceState &distanceState,
  float &yawIntegral,
  float &lastYawError,
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
    lastYawError = 0.0f;
    lastPiMs = millis();

    return;
  }

  int8_t rot = err > 0 ? ROTATE_SPEED : -ROTATE_SPEED;

  calculateMotorMix(0, rot, m1, m2);

  debugMotorState("PRE_ROTATE", 0, rot, m1, m2);
}

void motorTask(void* /*parameter*/) {
  while (true) {
    MotorOutput out = {true, true, 0, 0};
    // Never wait on a stalled control/command task. Contention means zero output.
    const bool locked = commandMutex && xSemaphoreTake(commandMutex, 0) == pdTRUE;
    if (locked && !emergencyStop.load() && !runtimeFault.load()
        && actuatorSafety.permits(motorBuffer.safetyGeneration, millis())) out = motorBuffer;
    // Only this task writes actuators after setup. Keep gate evaluation and writes
    // serialized with clear/new intent; an E-stop latch is rechecked every cycle.
    if (emergencyStop.load() || runtimeFault.load()) out = {true, true, 0, 0};
    myCodeCell.Drive1.Drive(!out.m1Forward, out.m1Speed);
    myCodeCell.Drive2.Drive(out.m2Forward, out.m2Speed);
    if (locked) xSemaphoreGive(commandMutex);
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void controlTask(void* /*parameter*/) {
  uint32_t lastSeq = 0;
  uint32_t motionStartMs = 0;
  bool wasMoving = false;

  bool targetRunning = false;
  float targetYaw = 0.0f;

  DistanceState distanceState;

  float yawIntegral = 0.0f;
  float lastYawError = 0.0f;
  uint32_t lastPiMs = 0;
  float directYawSetpoint = 0.0f;
  uint32_t lastSetpointMs = 0;
  TargetPhase targetPhase = TARGET_IDLE;

  while (true) {
    if (emergencyStop) {
      writeMotorOutput(0, 0);
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    myCodeCell.Run(20);

    RobotCommand cmd{};

    if (xSemaphoreTake(commandMutex, 0) == pdTRUE) {
      cmd = commandBuffer;
      xSemaphoreGive(commandMutex);
    } else {
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    int16_t m1 = 0;
    int16_t m2 = 0;

    if (cmd.sequence != lastSeq) {
      lastSeq = cmd.sequence;
      resetDistanceState(distanceState);

      yawIntegral = 0.0f;
      lastYawError = 0.0f;
      lastPiMs = millis();
      lastSetpointMs = millis();

      if (cmd.mode == CONTROL_TARGET) {
        targetRunning = true;
        targetPhase = TARGET_PRE_ROTATE;
        targetYaw = wrapAngleDeg(readYawDeg() + cmd.targetRotationDeg);
      }
      else {
        targetRunning = false;
        targetPhase = TARGET_IDLE;

      }

      Serial.print("NEW COMMAND SEQ: ");
      Serial.println(cmd.sequence);
    }

    bool commandFresh = (millis() - cmd.updatedMs) < COMMAND_TIMEOUT_MS;

    if (cmd.mode == CONTROL_DIRECT) {
      targetRunning = false;
      controlDirectRealtime(cmd, m1, m2, directYawSetpoint, yawIntegral, lastYawError, lastPiMs, lastSetpointMs);
    }
    else if (cmd.mode == CONTROL_TARGET && targetRunning && commandFresh) {
      Serial.print("Target Phase: ");
      Serial.println(targetPhase);

      if (targetPhase == TARGET_PRE_ROTATE) {
        controlPreliminaryRotation(
          targetYaw,
          targetPhase,
          m1,
          m2,
          distanceState,
          yawIntegral,
          lastYawError,
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
          lastYawError,
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

    updateImuDistance(distanceState, movingNow);

    if (movingNow && !wasMoving) {
      motionStartMs = millis();
    }

    uint32_t motionElapsed = millis() - motionStartMs;

    m1 = applyStartRamp(m1, motionElapsed);
    m2 = applyStartRamp(m2, motionElapsed);

    wasMoving = movingNow;

    writeMotorOutput(m1, m2, cmd.safetyGeneration);

    if (xSemaphoreTake(statusMutex, 0) == pdTRUE) {
      statusBuffer.batteryMv = myCodeCell.BatteryVoltageRead();
      statusBuffer.batteryPercent = myCodeCell.BatteryLevelRead();
      statusBuffer.uptimeMs = millis() - startTimeMs;
      statusBuffer.headingDeg = readYawDeg();

      if (cmd.mode == CONTROL_DIRECT) {
        statusBuffer.headingSetpointDeg = directYawSetpoint;
      } else {
        statusBuffer.headingSetpointDeg = targetYaw;
      }

      statusBuffer.yawErrorDeg = debugYawError;
      statusBuffer.yawCorrection = debugYawCorrection;

      statusBuffer.estimatedDistanceM =
        getCurrentDistanceM(cmd.useImuDistance ? DISTANCE_IMU : DISTANCE_TIME, distanceState);

      statusBuffer.move = cmd.move;
      statusBuffer.rotation = cmd.rotation;
      statusBuffer.m1Signed = m1;
      statusBuffer.m2Signed = m2;

      xSemaphoreGive(statusMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}