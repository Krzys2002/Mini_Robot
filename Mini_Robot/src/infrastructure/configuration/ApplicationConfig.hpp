#pragma once

#include "camera/CameraTypes.hpp"
#include "common/RobotId.hpp"
#include "infrastructure/logging/LogRecord.hpp"

#include <chrono>
#include <string>

namespace micro_robot {

// Typed application settings. Member initializers are the built-in defaults used
// when the configuration file is absent or an entry is invalid. The composition
// root loads one ApplicationConfig at startup and injects the relevant values into
// modules; modules never read configuration files or keep their own copies of keys.

struct LoggingConfig {
    LogLevel minimumLevel{LogLevel::Info};
    bool fileEnabled{true};
    // UTF-8 path of the log file; empty selects the per-user default location.
    std::string filePath;
};

struct RobotConfig {
    // Robot registered and driven by the current single-robot demo.
    RobotId id{1};
};

struct SimulationConfig {
    // Period of the timer advancing the simulated camera and transport.
    std::chrono::milliseconds updateInterval{100};
    // Frame period of the fake camera source.
    std::chrono::milliseconds cameraFrameInterval{33};
    // Minimum spacing between simulated robot telemetry samples; zero reports on
    // every simulation update.
    std::chrono::milliseconds telemetryInterval{0};
};

struct CameraConfig {
    // Applied by the live camera backend to every device it opens.
    CameraCaptureSettings capture;
};

struct ManualControlConfig {
    // Software bring-up bounds, not physical actuator/supply acceptance (#154).
    double maximumLinearMetersPerSecond{0.1};
    double maximumAngularRadiansPerSecond{1.0};
    std::chrono::milliseconds refreshInterval{100};
    std::chrono::milliseconds commandValidity{250};
};

struct ApplicationConfig {
    LoggingConfig logging;
    RobotConfig robot;
    ManualControlConfig manual;
    CameraConfig camera;
    SimulationConfig simulation;
};

} // namespace micro_robot
