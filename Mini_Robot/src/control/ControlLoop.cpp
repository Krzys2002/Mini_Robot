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

ControlLoop::ControlLoop(RobotId id, WorldModel& world, RobotManager& robots, IController& controller,
                         ControlLoopConfig config, Clock clock)
    : id_(id), world_(world), robots_(robots), controller_(controller), config_(config), clock_(std::move(clock))
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

bool ControlLoop::start()
{
    if (state_ == ControlLoopState::Running) return true;
    const auto now = clock_();
    if (!target_ || gate(now)) return false;
    state_ = ControlLoopState::Running;
    nextDue_ = now;
    lastNow_ = now;
    return true;
}

void ControlLoop::inhibit(ControlCycle cycle, ControlCycleResult reason, Timestamp now)
{
    state_ = ControlLoopState::Inhibited;
    cycle.finishedAt = now;
    cycle.result = reason;
    // Stop submission is an attempt, never evidence of delivery/physical stopping.
    cycle.stopSubmission = robots_.stop(id_, now, config_.commandValidity);
    last_ = cycle;
}

void ControlLoop::stop()
{
    if (state_ == ControlLoopState::Running) {
        const auto now = clock_();
        ControlCycle cycle;
        cycle.scheduledAt = nextDue_;
        cycle.startedAt = now;
        inhibit(cycle, ControlCycleResult::Stopped, now);
    }
    state_ = ControlLoopState::Stopped;
}

bool ControlLoop::poll()
{
    if (state_ != ControlLoopState::Running) return false;
    const auto now = clock_();
    ControlCycle cycle;
    cycle.startedAt = now;
    cycle.scheduledAt = nextDue_;
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
    cycle.observedAt = robot.poseUpdatedAt;
    cycle.observationAge = tracking.age;
    const auto dx = target_->x.value - robot.pose->x.value;
    const auto dy = target_->y.value - robot.pose->y.value;
    cycle.error = ControlError{dx, dy, normalizeAngle(Radians{
        target_->heading.value - normalizeAngle(robot.pose->heading).value}).value, std::hypot(dx, dy)};
    if (!std::isfinite(cycle.error->distanceMeters)) {
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
    cycle.submission = robots_.sendCommand(id_, {*cycle.bounded, finished, config_.commandValidity});
    if (*cycle.submission != CommandSubmission::Submitted) {
        inhibit(cycle, ControlCycleResult::SubmissionRejected, finished); return true;
    }
    cycle.result = ControlCycleResult::Submitted;
    lastNow_ = finished;
    last_ = cycle;
    return true;
}

} // namespace micro_robot
