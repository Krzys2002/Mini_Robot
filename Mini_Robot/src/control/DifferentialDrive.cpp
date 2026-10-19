#include "control/DifferentialDrive.hpp"

#include <algorithm>
#include <cmath>

namespace micro_robot {
namespace {

bool validGeometry(const DifferentialDriveGeometry& geometry) noexcept
{
    return std::isfinite(geometry.wheelRadius.value) && geometry.wheelRadius.value > 0.0
        && std::isfinite(geometry.wheelTrack.value) && geometry.wheelTrack.value > 0.0;
}

bool finiteVelocity(const VelocityCommand& velocity) noexcept
{
    return std::isfinite(velocity.linearMetersPerSecond)
        && std::isfinite(velocity.angularRadiansPerSecond);
}

bool finiteWheels(const WheelAngularVelocity& wheels) noexcept
{
    return std::isfinite(wheels.leftRadiansPerSecond)
        && std::isfinite(wheels.rightRadiansPerSecond);
}

} // namespace

std::optional<WorldPoseDerivative> unicycleDerivative(
    const Pose2D& pose, const VelocityCommand& velocity) noexcept
{
    if (!std::isfinite(pose.x.value) || !std::isfinite(pose.y.value)
        || !std::isfinite(pose.heading.value) || !finiteVelocity(velocity)) {
        return std::nullopt;
    }
    return WorldPoseDerivative{
        velocity.linearMetersPerSecond * std::cos(pose.heading.value),
        velocity.linearMetersPerSecond * std::sin(pose.heading.value),
        velocity.angularRadiansPerSecond};
}

std::optional<WheelAngularVelocity> toWheelAngularVelocity(
    const VelocityCommand& velocity, const DifferentialDriveGeometry& geometry) noexcept
{
    if (!validGeometry(geometry) || !finiteVelocity(velocity)) return std::nullopt;
    const double turnSpeed = velocity.angularRadiansPerSecond * (geometry.wheelTrack.value / 2.0);
    const WheelAngularVelocity wheels{
        (velocity.linearMetersPerSecond - turnSpeed) / geometry.wheelRadius.value,
        (velocity.linearMetersPerSecond + turnSpeed) / geometry.wheelRadius.value};
    if (!finiteWheels(wheels)) return std::nullopt;
    return wheels;
}

std::optional<VelocityCommand> toBodyVelocity(
    const WheelAngularVelocity& wheels, const DifferentialDriveGeometry& geometry) noexcept
{
    if (!validGeometry(geometry) || !finiteWheels(wheels)) return std::nullopt;
    const double left = wheels.leftRadiansPerSecond * geometry.wheelRadius.value;
    const double right = wheels.rightRadiansPerSecond * geometry.wheelRadius.value;
    if (!std::isfinite(left) || !std::isfinite(right)) return std::nullopt;
    const VelocityCommand velocity{
        left / 2.0 + right / 2.0,
        (right / geometry.wheelTrack.value) - (left / geometry.wheelTrack.value)};
    if (!finiteVelocity(velocity)) return std::nullopt;
    return velocity;
}

std::optional<WheelAngularVelocity> toBoundedWheelAngularVelocity(
    const VelocityCommand& velocity, const DifferentialDriveGeometry& geometry,
    double maxWheelRadiansPerSecond) noexcept
{
    if (!std::isfinite(maxWheelRadiansPerSecond) || maxWheelRadiansPerSecond < 0.0) {
        return std::nullopt;
    }
    auto wheels = toWheelAngularVelocity(velocity, geometry);
    if (!wheels) return std::nullopt;
    const double peak = std::max(std::abs(wheels->leftRadiansPerSecond),
        std::abs(wheels->rightRadiansPerSecond));
    if (peak > maxWheelRadiansPerSecond) {
        const double scale = maxWheelRadiansPerSecond / peak;
        // Clamp also guards against a last-bit rounding overshoot at the bound.
        wheels->leftRadiansPerSecond = std::clamp(wheels->leftRadiansPerSecond * scale,
            -maxWheelRadiansPerSecond, maxWheelRadiansPerSecond);
        wheels->rightRadiansPerSecond = std::clamp(wheels->rightRadiansPerSecond * scale,
            -maxWheelRadiansPerSecond, maxWheelRadiansPerSecond);
    }
    return wheels;
}

} // namespace micro_robot
