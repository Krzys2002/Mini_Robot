#pragma once

#include "control/IController.hpp"

namespace micro_robot {

struct GoalPointControllerConfig {
    double positionGainPerSecond{};
    double bearingGainPerSecond{};
    double finalHeadingGainPerSecond{};
    double maximumLinearMetersPerSecond{};
    double maximumAngularRadiansPerSecond{};
    Meters positionTolerance{};
    Radians headingTolerance{};
    Radians turnInPlaceAngle{};
    bool alignFinalHeading{true}; // false selects a position-only goal.
    // Zero selects twice the entry tolerance. A larger exit radius keeps
    // measurement jitter from switching heading alignment back to approach.
    Meters positionExitTolerance{};
};

enum class GoalControlState { TurningToPoint, Approaching, AligningHeading, Arrived };
struct GoalPointError {
    double worldXmeters{}, worldYmeters{};
    double forwardMeters{}, leftMeters{}, distanceMeters{};
    double bearingRadians{}, finalHeadingRadians{};
};
struct GoalPointEvaluation {
    GoalControlState state;
    GoalPointError error;
    VelocityCommand command;
};

// Forward-only baseline. Pure evaluation has no clock, GUI, transport or safety
// side effects. calculate() retains goal-phase state for hysteresis and can be
// reset between runs. Exactly opposite bearings choose CCW; otherwise turn along
// the shortest angular error. Arrival uses inclusive configured tolerances.
// ControlLoop remains responsible for freshness/session gates and command routing.
class GoalPointController final : public IController {
public:
    // Throws invalid_argument for invalid configuration. Bearing gain must exceed
    // position gain; turn-in-place threshold is in (0, pi/2]. No physical defaults.
    explicit GoalPointController(GoalPointControllerConfig config);
    [[nodiscard]] std::optional<GoalPointEvaluation> evaluate(const Pose2D& measured,
                                                             const Pose2D& target) const noexcept;
    [[nodiscard]] std::optional<VelocityCommand> calculate(const ControlInput& input) override;
    [[nodiscard]] const GoalPointControllerConfig& config() const noexcept { return config_; }
    [[nodiscard]] std::optional<GoalControlState> controlState() const noexcept { return state_; }
    void reset() noexcept { state_.reset(); }
private:
    [[nodiscard]] std::optional<GoalPointEvaluation> evaluateWithPreviousState(
        const Pose2D& measured, const Pose2D& target,
        std::optional<GoalControlState> previousState) const noexcept;
    GoalPointControllerConfig config_;
    std::optional<GoalControlState> state_;
};

} // namespace micro_robot
