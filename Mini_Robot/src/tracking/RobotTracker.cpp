#include "tracking/RobotTracker.hpp"
#include <cmath>
#include "common/Angle.hpp"
#include <stdexcept>

namespace micro_robot {
RobotTracker::RobotTracker(WorldModel& world, TrackingThresholds thresholds, PoseFilterConfig filter)
    : world_(world), thresholds_(thresholds), filter_(filter)
{
    (void)KalmanPoseFilter{filter.noise}; // validate even when currently using exponential
    if (!std::isfinite(filter.positionAlpha) || filter.positionAlpha <= 0 || filter.positionAlpha > 1
        || !std::isfinite(filter.headingAlpha) || filter.headingAlpha <= 0 || filter.headingAlpha > 1)
        throw std::invalid_argument("Filter alpha must be finite in (0,1]");
    if (thresholds.staleAfter <= Duration::zero() || thresholds.lostAfter <= thresholds.staleAfter)
        throw std::invalid_argument("Tracking thresholds must be positive and ordered");
}
TrackerUpdate RobotTracker::observe(RobotId id, const Pose2D& pose, Timestamp capturedAt, Timestamp now,
                                    std::optional<double> quality)
{
    if (quality && (!std::isfinite(*quality) || *quality < 0 || *quality > 1))
        return TrackerUpdate::InvalidMeasurement;
    if (!std::isfinite(pose.x.value) || !std::isfinite(pose.y.value) || !std::isfinite(pose.heading.value))
        return TrackerUpdate::InvalidMeasurement;
    if (invalidatedAt_ && capturedAt <= *invalidatedAt_) return TrackerUpdate::RejectedByWorld;
    if (capturedAt > now) return TrackerUpdate::FutureTimestamp;
    if (now - capturedAt >= thresholds_.lostAfter) return TrackerUpdate::ExpiredMeasurement;
    (void)world_.registerRobot(id, capturedAt);
    const auto previous = world_.robot(id);
    Pose2D filtered = pose;
    double heading = pose.heading.value;
    std::optional<PlanarVelocity> velocity;
    if (previous && previous->poseUpdatedAt && capturedAt <= *previous->poseUpdatedAt)
        return TrackerUpdate::RejectedByWorld;
    const bool continuing = previous && previous->pose && previous->tracking && previous->tracking->rawPose
        && world_.tracking(id, capturedAt).status != TrackingStatus::Lost;
    if (continuing) {
        heading = previous->tracking->continuousHeading + normalizeAngle(Radians{
            normalizeAngle(pose.heading).value - normalizeAngle(previous->tracking->rawPose->heading).value}).value;
        // Raw finite differences avoid alpha-dependent velocity bias. Tiny dt
        // is accepted as a pose but does not publish an unstable derivative.
        const double dt = std::chrono::duration<double>(capturedAt - *previous->poseUpdatedAt).count();
        if (dt >= .001) {
            PlanarVelocity candidate{
                (pose.x.value - previous->tracking->rawPose->x.value) / dt,
                (pose.y.value - previous->tracking->rawPose->y.value) / dt,
                (heading - previous->tracking->continuousHeading) / dt};
            if (std::isfinite(candidate.xMetersPerSecond) && std::isfinite(candidate.yMetersPerSecond)
                && std::isfinite(candidate.radiansPerSecond)) velocity = candidate;
        }
        filtered.x.value = std::lerp(previous->pose->x.value, pose.x.value, filter_.positionAlpha);
        filtered.y.value = std::lerp(previous->pose->y.value, pose.y.value, filter_.positionAlpha);
        filtered.heading = normalizeAngle(Radians{previous->pose->heading.value + filter_.headingAlpha
            * normalizeAngle(Radians{normalizeAngle(pose.heading).value - normalizeAngle(previous->pose->heading).value}).value});
    }
    std::optional<KalmanPoseFilter> candidate;
    if (filter_.kind == PoseFilterKind::Kalman) {
        candidate.emplace(filter_.noise);
        const auto cached = kalman_.find(id);
        if (continuing && cached != kalman_.end() && cached->second.updatedAt == previous->poseUpdatedAt)
            candidate = cached->second.filter;
        const double dt = continuing
            ? std::chrono::duration<double>(capturedAt - *previous->poseUpdatedAt).count() : 0;
        if (!candidate->update(pose, dt)) return TrackerUpdate::InvalidMeasurement;
        filtered = *candidate->pose();
    }
    if (world_.updateTrackedPose(id, filtered, capturedAt, thresholds_, pose, heading, velocity, quality) != PoseUpdateResult::Applied)
        return TrackerUpdate::RejectedByWorld;
    if (candidate) kalman_.insert_or_assign(id, KalmanTrack{*candidate, capturedAt});
    observed_.insert(id);
    return TrackerUpdate::Applied;
}
void RobotTracker::invalidate(Timestamp now)
{
    if (!invalidatedAt_ || now > *invalidatedAt_) invalidatedAt_ = now;
    for (auto id : observed_) world_.invalidateTracking(id);
    kalman_.clear();
}
} // namespace micro_robot
