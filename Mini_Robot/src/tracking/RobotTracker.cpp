#include "tracking/RobotTracker.hpp"
#include <cmath>
#include <stdexcept>

namespace micro_robot {
RobotTracker::RobotTracker(WorldModel& world, TrackingThresholds thresholds) : world_(world), thresholds_(thresholds)
{
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
    if (world_.updateTrackedPose(id, pose, capturedAt, thresholds_) != PoseUpdateResult::Applied)
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
