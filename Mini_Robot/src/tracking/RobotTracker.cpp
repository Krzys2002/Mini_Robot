#include "tracking/RobotTracker.hpp"
#include <cmath>
#include "common/Angle.hpp"
#include <stdexcept>

namespace micro_robot {
RobotTracker::RobotTracker(WorldModel& world, TrackingThresholds thresholds, PoseFilterConfig filter)
    : world_(world), thresholds_(thresholds), filter_(filter)
{
    if (!std::isfinite(filter.positionAlpha) || filter.positionAlpha <= 0 || filter.positionAlpha > 1
        || !std::isfinite(filter.headingAlpha) || filter.headingAlpha <= 0 || filter.headingAlpha > 1)
        throw std::invalid_argument("Filter alpha must be finite in (0,1]");
    if (thresholds.staleAfter <= Duration::zero() || thresholds.lostAfter <= thresholds.staleAfter)
        throw std::invalid_argument("Tracking thresholds must be positive and ordered");
}
TrackerUpdate RobotTracker::observe(RobotId id, const Pose2D& pose, Timestamp capturedAt, Timestamp now)
{
    if (!std::isfinite(pose.x.value) || !std::isfinite(pose.y.value) || !std::isfinite(pose.heading.value))
        return TrackerUpdate::InvalidMeasurement;
    if (invalidatedAt_ && capturedAt <= *invalidatedAt_) return TrackerUpdate::RejectedByWorld;
    if (capturedAt > now) return TrackerUpdate::FutureTimestamp;
    if (now - capturedAt >= thresholds_.lostAfter) return TrackerUpdate::ExpiredMeasurement;
    (void)world_.registerRobot(id, capturedAt);
    const auto previous = world_.robot(id);
    Pose2D filtered = pose;
    double heading = pose.heading.value;
    if (previous && previous->poseUpdatedAt && capturedAt <= *previous->poseUpdatedAt)
        return TrackerUpdate::RejectedByWorld;
    if (previous && previous->pose && previous->tracking && previous->tracking->rawPose
        && world_.tracking(id, capturedAt).status != TrackingStatus::Lost) {
        heading = previous->tracking->continuousHeading + normalizeAngle(Radians{
            normalizeAngle(pose.heading).value - normalizeAngle(previous->tracking->rawPose->heading).value}).value;
        filtered.x.value = std::lerp(previous->pose->x.value, pose.x.value, filter_.positionAlpha);
        filtered.y.value = std::lerp(previous->pose->y.value, pose.y.value, filter_.positionAlpha);
        filtered.heading = normalizeAngle(Radians{previous->pose->heading.value + filter_.headingAlpha
            * normalizeAngle(Radians{normalizeAngle(pose.heading).value - normalizeAngle(previous->pose->heading).value}).value});
    }
    if (world_.updateTrackedPose(id, filtered, capturedAt, thresholds_, pose, heading) != PoseUpdateResult::Applied)
        return TrackerUpdate::RejectedByWorld;
    observed_.insert(id);
    return TrackerUpdate::Applied;
}
void RobotTracker::invalidate(Timestamp now)
{
    if (!invalidatedAt_ || now > *invalidatedAt_) invalidatedAt_ = now;
    for (auto id : observed_) world_.invalidateTracking(id);
}
} // namespace micro_robot
