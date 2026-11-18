#include "control/GoalPointController.hpp"

#include "common/Angle.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace micro_robot {
namespace {
bool positive(double value) { return std::isfinite(value) && value > 0.0; }
bool finite(const Pose2D& pose)
{
    return std::isfinite(pose.x.value) && std::isfinite(pose.y.value)
        && std::isfinite(pose.heading.value);
}
// Saturate before multiplying to keep finite gains/errors from overflowing.
double boundedProportional(double gain, double error, double limit)
{
    if (error == 0.0) return 0.0;
    if (std::abs(error) >= limit / gain) return std::copysign(limit, error);
    return std::clamp(gain * error, -limit, limit);
}
}

GoalPointController::GoalPointController(GoalPointControllerConfig config) : config_(config)
{
    if (config_.positionExitTolerance.value == 0.0)
        config_.positionExitTolerance = Meters{2.0 * config_.positionTolerance.value};
    if (!positive(config.positionGainPerSecond) || !positive(config.bearingGainPerSecond)
        || config.bearingGainPerSecond <= config.positionGainPerSecond
        || !positive(config.finalHeadingGainPerSecond)
        || !positive(config.maximumLinearMetersPerSecond) || !positive(config.maximumAngularRadiansPerSecond)
        || !positive(config.positionTolerance.value) || !positive(config.headingTolerance.value)
        || config.headingTolerance.value > std::numbers::pi
        || !positive(config_.positionExitTolerance.value)
        || config_.positionExitTolerance.value <= config.positionTolerance.value
        || !positive(config.turnInPlaceAngle.value) || config.turnInPlaceAngle.value > std::numbers::pi / 2.0)
        throw std::invalid_argument("Goal controller requires positive finite gains/limits/tolerances, exit tolerance above entry tolerance, bearing gain > position gain, and turn angle <= pi/2");
}

std::optional<GoalPointEvaluation> GoalPointController::evaluate(const Pose2D& measured,
                                                               const Pose2D& target) const noexcept
{
    return evaluateWithPreviousState(measured, target, std::nullopt);
}

std::optional<GoalPointEvaluation> GoalPointController::evaluateWithPreviousState(
    const Pose2D& measured, const Pose2D& target,
    std::optional<GoalControlState> previousState) const noexcept
{
    if (!finite(measured) || !finite(target)) return {};
    const double heading = normalizeAngle(measured.heading).value;
    const double dx = target.x.value - measured.x.value, dy = target.y.value - measured.y.value;
    const double distance = std::hypot(dx, dy);
    if (!std::isfinite(distance)) return {};
    GoalPointError error{dx, dy, dx * std::cos(heading) + dy * std::sin(heading),
        -dx * std::sin(heading) + dy * std::cos(heading), distance,
        distance > 0.0 ? normalizeAngle(Radians{std::atan2(dy, dx) - heading}).value : 0.0,
        normalizeAngle(Radians{normalizeAngle(target.heading).value - heading}).value};
    if (!std::isfinite(error.forwardMeters) || !std::isfinite(error.leftMeters)) return {};
    const bool holdingGoalRegion = previousState == GoalControlState::AligningHeading
        || previousState == GoalControlState::Arrived;
    const double positionLimit = holdingGoalRegion
        ? config_.positionExitTolerance.value : config_.positionTolerance.value;
    if (distance <= positionLimit) {
        if (!config_.alignFinalHeading || std::abs(error.finalHeadingRadians) <= config_.headingTolerance.value)
            return GoalPointEvaluation{GoalControlState::Arrived, error, {}};
        return GoalPointEvaluation{GoalControlState::AligningHeading, error,
            {0.0, boundedProportional(config_.finalHeadingGainPerSecond, error.finalHeadingRadians,
                                      config_.maximumAngularRadiansPerSecond)}};
    }
    const double angular = boundedProportional(config_.bearingGainPerSecond, error.bearingRadians,
                                               config_.maximumAngularRadiansPerSecond);
    if (std::abs(error.bearingRadians) >= config_.turnInPlaceAngle.value)
        return GoalPointEvaluation{GoalControlState::TurningToPoint, error, {0.0, angular}};
    const double thresholdCosine = std::cos(config_.turnInPlaceAngle.value);
    const double bearingCosine = std::cos(error.bearingRadians);
    const double approachScale = std::clamp((bearingCosine - thresholdCosine)
        / (1.0 - thresholdCosine), 0.0, 1.0);
    const double linear = boundedProportional(config_.positionGainPerSecond, distance,
                                              config_.maximumLinearMetersPerSecond) * approachScale;
    return GoalPointEvaluation{GoalControlState::Approaching, error, {linear, angular}};
}

std::optional<VelocityCommand> GoalPointController::calculate(const ControlInput& input)
{
    if (!input.robot.pose || input.tracking.status != TrackingStatus::Fresh) return {};
    const auto result = evaluateWithPreviousState(*input.robot.pose, input.target, state_);
    if (!result) return std::nullopt;
    state_ = result->state;
    return result->command;
}

} // namespace micro_robot
