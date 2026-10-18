#pragma once

#include "common/Pose2D.hpp"
#include "transport/TransportTypes.hpp"

#include <optional>

namespace micro_robot {

// Ideal no-slip drive with equal wheel radii. Track is wheel-centre separation.
// Both dimensions must be finite and strictly positive; no hardware defaults.
struct DifferentialDriveGeometry final {
    Meters wheelRadius{};
    Meters wheelTrack{};
};

// Positive on either wheel means rolling the robot forward, independent of
// motor mounting/wiring. This is an angular setpoint, never PWM or PID output.
struct WheelAngularVelocity final {
    double leftRadiansPerSecond{};
    double rightRadiansPerSecond{};
};

struct WorldPoseDerivative final {
    double xMetersPerSecond{};
    double yMetersPerSecond{};
    double headingRadiansPerSecond{};
};

// Pure, reentrant value calculations: no state, threads, clocks or hardware.
// All functions return nullopt on nonfinite inputs or nonrepresentable results;
// conversions also reject invalid geometry. Callers must handle failure before
// issuing motion. No function performs safety/freshness checks or sends commands.
[[nodiscard]] std::optional<WorldPoseDerivative> unicycleDerivative(
    const Pose2D& pose, const VelocityCommand& velocity) noexcept;

[[nodiscard]] std::optional<WheelAngularVelocity> toWheelAngularVelocity(
    const VelocityCommand& velocity, const DifferentialDriveGeometry& geometry) noexcept;

[[nodiscard]] std::optional<VelocityCommand> toBodyVelocity(
    const WheelAngularVelocity& wheels, const DifferentialDriveGeometry& geometry) noexcept;

// Scale both wheel targets by one factor to preserve curvature. The symmetric
// limit is finite and >= 0 rad/s (zero produces a stationary target). Returns
// nullopt if the unbounded conversion fails, even when a bound was supplied.
[[nodiscard]] std::optional<WheelAngularVelocity> toBoundedWheelAngularVelocity(
    const VelocityCommand& velocity, const DifferentialDriveGeometry& geometry,
    double maxWheelRadiansPerSecond) noexcept;

} // namespace micro_robot
