#pragma once

#include "app/RobotManager.hpp"
#include "control/IController.hpp"

#include <functional>

namespace micro_robot {

struct ControlLoopConfig {
    std::chrono::milliseconds period;
    std::chrono::milliseconds commandValidity;
    Duration maximumObservationAge;
    double maximumLinearMetersPerSecond;
    double maximumAngularRadiansPerSecond;
};

enum class ControlLoopState { Stopped, Running, Inhibited };
enum class ControlOperationalState { Ready, Running, Paused, LostTracking, LostConnection, EStop, Fault };
// Shared translation of the authoritative RobotManager session into the
// control-domain state vocabulary. Tracking and loop lifecycle are layered on top.
[[nodiscard]] ControlOperationalState classifySessionOperationalState(
    const std::optional<RobotSession>& session) noexcept;
enum class ControlCycleResult {
    Submitted, Stopped, InvalidClock, TrackingUnavailable, SessionUnavailable,
    FrameChanged, ControllerUnavailable, ControllerFailure, InvalidOutput, SubmissionRejected
};
struct ControlError {
    double xMeters{}, yMeters{}, headingRadians{}, distanceMeters{};
};
struct ControlCycle {
    Timestamp scheduledAt{}, startedAt{}, finishedAt{};
    Duration lateness{};
    std::uint64_t skippedPeriods{};
    std::optional<Timestamp> observedAt;
    std::optional<Duration> observationAge;
    std::optional<ControlError> error;
    std::optional<VelocityCommand> requested, bounded;
    std::optional<CommandSubmission> submission, stopSubmission;
    std::optional<std::uint64_t> commandSequence, stopSequence; // Domain correlation, not wire sequence.
    ControlCycleResult result{ControlCycleResult::Stopped};
};

// One robot, one owning thread shared with WorldModel/RobotManager/transport.
// All dependencies outlive this service. poll() never waits: at most one policy
// call and one submission; missed periods are skipped, never queued/replayed.
// Composition must give this service exclusive normal motion authority while
// running. E-stop remains RobotManager's independent path. Stop before teardown.
// Detached lastCycle() is bounded in memory; callers can record it externally.
class ControlLoop final {
public:
    using Clock = std::function<Timestamp()>;
    using Sequence = std::function<std::uint64_t()>; // Prompt/nonthrowing; optional domain correlation.
    ControlLoop(RobotId id, WorldModel& world, RobotManager& robots, IController& controller,
                ControlLoopConfig config, Clock clock = MonotonicClock::now, Sequence sequence = {});
    ControlLoop(const ControlLoop&) = delete;
    ControlLoop& operator=(const ControlLoop&) = delete;

    // Target is in the current metric world frame. Invalid values are rejected
    // without modifying the old target. Target changes require a stopped loop.
    [[nodiscard]] bool setTarget(Pose2D target);
    // Explicit start/restart only; failure never arms. First poll is due now.
    [[nodiscard]] bool start();
    // Controller/clock faults remain latched until explicitly acknowledged while
    // all ordinary frame, tracking and manager-session gates are clear.
    [[nodiscard]] bool acknowledgeControllerFault();
    [[nodiscard]] bool hasLatchedControllerFault() const noexcept { return controllerFaultLatched_; }
    void stop();
    // True iff a cycle/inhibition record was produced. Safety is checked even
    // before a scheduled deadline. No automatic restart after any inhibition.
    [[nodiscard]] bool poll();
    [[nodiscard]] ControlLoopState state() const noexcept { return state_; }
    [[nodiscard]] ControlOperationalState operationalState() const noexcept { return operationalState_; }
    [[nodiscard]] const ControlLoopConfig& config() const noexcept { return config_; }
    [[nodiscard]] std::optional<ControlCycle> lastCycle() const { return last_; }

private:
    [[nodiscard]] std::optional<ControlCycleResult> gate(Timestamp now) const;
    [[nodiscard]] ControlOperationalState classify(ControlCycleResult reason,
        std::optional<CommandSubmission> submission = std::nullopt) const;
    void inhibit(ControlCycle cycle, ControlCycleResult reason, Timestamp now);
    RobotId id_;
    WorldModel& world_;
    RobotManager& robots_;
    IController& controller_;
    ControlLoopConfig config_;
    Clock clock_;
    Sequence sequence_;
    std::optional<Pose2D> target_;
    std::optional<std::string> frame_;
    ControlLoopState state_{ControlLoopState::Stopped};
    ControlOperationalState operationalState_{ControlOperationalState::Ready};
    bool controllerFaultLatched_{};
    Timestamp nextDue_{};
    std::optional<Timestamp> lastNow_;
    std::optional<ControlCycle> last_;
};

} // namespace micro_robot
