#pragma once

#include <Arduino.h>
#include <CodeCell.h>

extern CodeCell myCodeCell;

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
    int8_t move;              // -100..100 speed command
    int8_t rotation;          // -100..100 yaw setpoint command in degrees
    float targetDistanceM;
    float targetRotationDeg;
    bool useImuDistance;
    uint32_t sequence;
    uint32_t updatedMs;       // command freshness watchdog timestamp
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
    float headingSetpointDeg;
    float yawErrorDeg;
    float yawCorrection;
    float estimatedDistanceM;
    int8_t move;
    int8_t rotation;
    int16_t m1Signed;
    int16_t m2Signed;
};

struct DistanceState {
    uint32_t startMs;
    uint32_t lastImuMs;
    float imuVelocityMps;
    float imuDistanceM;
    float imuAccelBiasMps2;
    bool imuBiasReady;
};

extern volatile bool emergencyStop;

void initController();

void setDirectControl(int8_t move, int8_t rotation);
void setTargetControl(float distanceM, float rotationDeg, bool useImuDistance);

void emergencyStopNow();
void clearEmergencyStop();

RobotStatus readRobotStatus();

void motorTask(void* parameter);
void controlTask(void* parameter);
