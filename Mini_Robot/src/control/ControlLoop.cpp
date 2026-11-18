#include "control/ControlLoop.hpp"

#include "common/Angle.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace micro_robot {
namespace {
bool finite(const Pose2D& pose)
{
    return std::isfinite(pose.x.value) && std::isfinite(pose.y.value)
        && std::isfinite(pose.heading.value);
}
}

ControlOperationalState classifySessionOperationalState(const std::optional<RobotSession>& session) noexcept
{
    if (!session) return ControlOperationalState::LostConnection;
    if (session->emergencyStopRequested || session->safety == DesktopSafetyState::EmergencyStop)
        return ControlOperationalState::EStop;
    if (session->connection == ConnectionState::Fault)
        return ControlOperationalState::Fault;
    if (!session->connectRequested || session->removing
        || session->connection != ConnectionState::Connected
        || session->safety == DesktopSafetyState::Disconnected
        || session->safety == DesktopSafetyState::LostConnection)
        return ControlOperationalState::LostConnection;
    if (session->safetyFault || session->safety == DesktopSafetyState::Fault)
        return ControlOperationalState::Fault;
    return ControlOperationalState::Ready;
}

ControlLoop::ControlLoop(RobotId id, WorldModel& world, RobotManager& robots, IController& controller,
                         ControlLoopConfig config, Clock clock, Sequence sequence)
    : id_(id), world_(world), robots_(robots), controller_(controller), config_(config), clock_(std::move(clock)), sequence_(std::move(sequence))
{
    if (!clock_ || config.period.count() <= 0 || config.commandValidity <= config.period
        || config.maximumObservationAge <= Duration::zero()
        || !std::isfinite(config.maximumLinearMetersPerSecond) || config.maximumLinearMetersPerSecond <= 0
        || !std::isfinite(config.maximumAngularRadiansPerSecond) || config.maximumAngularRadiansPerSecond <= 0)
        throw std::invalid_argument("Control loop requires positive finite limits, period, age and validity longer than period");
}

bool ControlLoop::setTarget(Pose2D target)
{
    const auto frame = world_.workspaceFrameId();
    if (state_ == ControlLoopState::Running || !finite(target) || !frame) return false;
    target.heading = normalizeAngle(target.heading);
    target_ = target;
    frame_ = frame;
    return true;
}

std::optional<ControlCycleResult> ControlLoop::gate(Timestamp now) const
{
    if (lastNow_ && now < *lastNow_) return ControlCycleResult::InvalidClock;
    if (world_.workspaceFrameId() != frame_) return ControlCycleResult::FrameChanged;
    const auto tracking = world_.tracking(id_, now);
    const auto robot = world_.robot(id_);
    if (!robot || !robot->pose || !robot->poseUpdatedAt || !finite(*robot->pose)
        || tracking.status != TrackingStatus::Fresh || !tracking.age
        || *tracking.age >= config_.maximumObservationAge)
        return ControlCycleResult::TrackingUnavailable;
    const auto session = robots_.session(id_);
    if (!session || !session->connectRequested || session->removing
        || session->connection != ConnectionState::Connected || session->safety != DesktopSafetyState::Ready
        || session->emergencyStopRequested || session->safetyFault)
        return ControlCycleResult::SessionUnavailable;
    return {};
}

void ControlLoop::captureSnapshot(ControlCycle& cycle, Timestamp now) const
{
    const auto robot = world_.robot(id_);
    if (robot) {
        cycle.measuredPose = robot->pose;
        cycle.observedAt = robot->poseUpdatedAt;
        if (robot->tracking) {
            cycle.rawPose = robot->tracking->rawPose;
            cycle.measurementQuality = robot->tracking->measurementQuality;
        }
        const auto tracking = world_.tracking(id_, now);
        cycle.observationAge = tracking.age;
        cycle.trackingConfidence = tracking.confidence;
    }
    cycle.frameId = world_.workspaceFrameId().value_or("");
    cycle.operationalState = operationalState_;
    if (target_) cycle.targetPose = *target_;
    if (!cycle.measuredPose || !cycle.targetPose || !finite(*cycle.measuredPose) || !finite(*cycle.targetPose))
        return;
    const auto dx = cycle.targetPose->x.value - cycle.measuredPose->x.value;
    const auto dy = cycle.targetPose->y.value - cycle.measuredPose->y.value;
    const double distance = std::hypot(dx, dy);
    if (!std::isfinite(distance)) return;
    cycle.error = ControlError{dx, dy, normalizeAngle(Radians{
        cycle.targetPose->heading.value - normalizeAngle(cycle.measuredPose->heading).value}).value, distance};
}

ControlOperationalState ControlLoop::classify(ControlCycleResult reason,
    std::optional<CommandSubmission> submission) const
{
    switch (reason) {
    case ControlCycleResult::Stopped:
    case ControlCycleResult::FrameChanged:
        return ControlOperationalState::Paused;
    case ControlCycleResult::TrackingUnavailable:
        return ControlOperationalState::LostTracking;
    case ControlCycleResult::SessionUnavailable:
        return classifySessionOperationalState(robots_.session(id_));
    case ControlCycleResult::SubmissionRejected:
        switch (submission.value_or(CommandSubmission::SafetyFault)) {
        case CommandSubmission::EmergencyStopped: return ControlOperationalState::EStop;
        case CommandSubmission::SafetyFault: return ControlOperationalState::Fault;
        case CommandSubmission::StaleSession: return ControlOperationalState::Paused;
        case CommandSubmission::UnknownRobot:
        case CommandSubmission::NotConnected: return ControlOperationalState::LostConnection;
        case CommandSubmission::ShuttingDown: return ControlOperationalState::Paused;
        case CommandSubmission::Submitted: return ControlOperationalState::Fault;
        }
        return ControlOperationalState::Fault;
    case ControlCycleResult::InvalidClock:
    case ControlCycleResult::ControllerUnavailable:
    case ControlCycleResult::ControllerFailure:
    case ControlCycleResult::InvalidOutput:
        return ControlOperationalState::Fault;
    case ControlCycleResult::Submitted:
        return ControlOperationalState::Running;
    }
    return ControlOperationalState::Fault;
}

bool ControlLoop::start()
{
    if (state_ == ControlLoopState::Running) return true;
    if (controllerFaultLatched_) {
        state_ = ControlLoopState::Inhibited;
        operationalState_ = ControlOperationalState::Fault;
        return false;
    }
    const auto now = clock_();
    if (!target_) {
        state_ = ControlLoopState::Stopped;
        operationalState_ = ControlOperationalState::Ready;
        return false;
    }
    if (const auto reason = gate(now)) {
        state_ = ControlLoopState::Stopped;
        operationalState_ = classify(*reason);
        return false;
    }
    state_ = ControlLoopState::Running;
    operationalState_ = ControlOperationalState::Running;
    nextDue_ = now;
    lastNow_ = now;
    return true;
}

bool ControlLoop::acknowledgeControllerFault()
{
    if (!controllerFaultLatched_ || state_ == ControlLoopState::Running) return false;
    const auto now = clock_();
    if (!target_) return false;
    if (const auto reason = gate(now)) {
        operationalState_ = classify(*reason);
        return false;
    }
    controllerFaultLatched_ = false;
    state_ = ControlLoopState::Stopped;
    operationalState_ = ControlOperationalState::Ready;
    lastNow_ = now;
    return true;
}

void ControlLoop::inhibit(ControlCycle cycle, ControlCycleResult reason, Timestamp now)
{
    state_ = ControlLoopState::Inhibited;
    if (reason == ControlCycleResult::InvalidClock || reason == ControlCycleResult::ControllerUnavailable
        || reason == ControlCycleResult::ControllerFailure || reason == ControlCycleResult::InvalidOutput)
        controllerFaultLatched_ = true;
    operationalState_ = classify(reason, cycle.submission);
    cycle.operationalState = operationalState_;
    cycle.finishedAt = now;
    cycle.result = reason;
    // Stop submission is an attempt, never evidence of delivery/physical stopping.
    if (sequence_) cycle.stopSequence = sequence_();
    cycle.stopSubmission = robots_.sendCommand(id_, {StopCommand{}, now, config_.commandValidity,
        {currentTransportModelVersion, cycle.stopSequence}});
    last_ = cycle;
}

void ControlLoop::stop()
{
    if (state_ == ControlLoopState::Running) {
        const auto now = clock_();
        ControlCycle cycle;
        cycle.scheduledAt = nextDue_;
        cycle.startedAt = now;
        captureSnapshot(cycle, now);
        inhibit(cycle, ControlCycleResult::Stopped, now);
    }
    state_ = ControlLoopState::Stopped;
}

bool ControlLoop::poll()
{
    if (state_ != ControlLoopState::Running) return false;
    if (operationalState_ != ControlOperationalState::Running) {
        const auto now = clock_();
        ControlCycle cycle;
        cycle.startedAt = now;
        cycle.scheduledAt = nextDue_;
        inhibit(cycle, ControlCycleResult::Stopped, now);
        return true;
    }
    const auto now = clock_();
    ControlCycle cycle;
    cycle.startedAt = now;
    cycle.scheduledAt = nextDue_;
    captureSnapshot(cycle, now);
    if (const auto reason = gate(now)) { inhibit(cycle, *reason, now); return true; }
    lastNow_ = now;
    if (now < nextDue_) return false;
    cycle.lateness = now - nextDue_;
    const auto skipped = cycle.lateness / config_.period;
    cycle.skippedPeriods = static_cast<std::uint64_t>(skipped);
    // Preserve phase, but run only once even after a long scheduler stall.
    nextDue_ = now + (config_.period - cycle.lateness % config_.period);
    const auto robot = *world_.robot(id_);
    const auto tracking = world_.tracking(id_, now);
    if (!cycle.error) {
        inhibit(cycle, ControlCycleResult::InvalidOutput, now); return true;
    }
    try { cycle.requested = controller_.calculate({robot, tracking, *target_, now}); }
    catch (...) { inhibit(cycle, ControlCycleResult::ControllerFailure, clock_()); return true; }
    const auto finished = clock_();
    if (state_ != ControlLoopState::Running) {
        inhibit(cycle, ControlCycleResult::Stopped, finished);
        return true;
    }
    // A calculation may have taken too long or triggered a session/frame change.
    // Re-read authority and freshness immediately before submitting any motion.
    if (const auto reason = gate(finished)) { inhibit(cycle, *reason, finished); return true; }
    if (!cycle.requested) { inhibit(cycle, ControlCycleResult::ControllerUnavailable, finished); return true; }
    const auto v = *cycle.requested;
    if (!std::isfinite(v.linearMetersPerSecond) || !std::isfinite(v.angularRadiansPerSecond)) {
        inhibit(cycle, ControlCycleResult::InvalidOutput, finished); return true;
    }
    const double scale = std::max({1.0, std::abs(v.linearMetersPerSecond) / config_.maximumLinearMetersPerSecond,
        std::abs(v.angularRadiansPerSecond) / config_.maximumAngularRadiansPerSecond});
    cycle.bounded = VelocityCommand{v.linearMetersPerSecond / scale, v.angularRadiansPerSecond / scale};
    cycle.finishedAt = finished;
    if (sequence_) cycle.commandSequence = sequence_();
    cycle.submission = robots_.sendCommand(id_, {*cycle.bounded, finished, config_.commandValidity,
        {currentTransportModelVersion, cycle.commandSequence}});
    if (*cycle.submission != CommandSubmission::Submitted) {
        inhibit(cycle, ControlCycleResult::SubmissionRejected, finished); return true;
    }
    cycle.result = ControlCycleResult::Submitted;
    lastNow_ = finished;
    last_ = cycle;
    return true;
}

} // namespace micro_robot
