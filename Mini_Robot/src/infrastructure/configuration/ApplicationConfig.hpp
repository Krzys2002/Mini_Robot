#pragma once

#include "camera/CameraTypes.hpp"
#include "common/RobotId.hpp"
#include "infrastructure/logging/LogRecord.hpp"

#include <chrono>
#include <numbers>
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

struct TrackingConfig {
    double positionAlpha{1.0};
    double headingAlpha{1.0};
    // Initial software thresholds; tune for camera cadence and experiment conditions.
    std::chrono::milliseconds staleAfter{250};
    std::chrono::milliseconds lostAfter{1000};
};

struct VisualControlConfig {
    // Software bring-up parameters only; physical tuning/acceptance is separate.
    double positionGainPerSecond{1.0};
    double bearingGainPerSecond{3.0};
    double finalHeadingGainPerSecond{2.0};
    double positionToleranceMeters{0.005};
    double positionExitToleranceMeters{0.01};
    double headingToleranceRadians{0.03};
    double turnInPlaceRadians{std::numbers::pi / 3.0};
    bool alignFinalHeading{true};
};

struct ApplicationConfig {
    LoggingConfig logging;
    RobotConfig robot;
    ManualControlConfig manual;
    TrackingConfig tracking;
    VisualControlConfig visualControl;
    CameraConfig camera;
    SimulationConfig simulation;
};

} // namespace micro_robot
