#include "tracking/RobotTracker.hpp"
#include <cmath>
#include "common/Angle.hpp"
#include <stdexcept>

namespace micro_robot {
const char* trackerUpdateName(TrackerUpdate u) {
    switch(u) {
    case TrackerUpdate::Applied: return "applied";
    case TrackerUpdate::InvalidMeasurement: return "invalid measurement";
    case TrackerUpdate::FutureTimestamp: return "future observation timestamp";
    case TrackerUpdate::ExpiredMeasurement: return "expired measurement";
    case TrackerUpdate::RejectedByWorld: return "invalidated or non-increasing WorldModel observation";
    }
    return "unknown tracker rejection";
}
RobotTracker::RobotTracker(WorldModel& world, TrackingThresholds thresholds, PoseFilterConfig filter)
    : world_(world), thresholds_(thresholds), filter_(filter)
{
    if (!std::isfinite(filter.positionAlpha) || filter.positionAlpha <= 0 || filter.positionAlpha > 1
        || !std::isfinite(filter.headingAlpha) || filter.headingAlpha <= 0 || filter.headingAlpha > 1)
        throw std::invalid_argument("Filter alpha must be finite in (0,1]");
    if (thresholds.staleAfter <= Duration::zero() || thresholds.lostAfter <= thresholds.staleAfter)
        throw std::invalid_argument("Tracking thresholds must be positive and ordered");
}
TrackerUpdate RobotTracker::observe(RobotId id, const Pose2D& pose, Timestamp capturedAt, Timestamp now,
                                    std::optional<double> quality, std::optional<MeasurementProvenance> provenance)
{
    if (provenance && provenance->observedAt != capturedAt) return TrackerUpdate::InvalidMeasurement;
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
    const auto sameBinding=[&] {
        if(!previous || !previous->tracking) return true;
        const auto& old=previous->tracking->provenance;
        if(bool(old)!=bool(provenance)) return false;
        if(!old) return true;
        const auto& a=*old; const auto& b=*provenance;
        const bool calibration=a.calibrationRevision==b.calibrationRevision
            || (a.calibrationRevision && b.calibrationRevision && *a.calibrationRevision==*b.calibrationRevision);
        return a.sourceId==b.sourceId && a.inputGeneration==b.inputGeneration && a.frameId==b.frameId
            && calibration && a.mountingRevision==b.mountingRevision && a.width==b.width && a.height==b.height;
    };
    if (sameBinding() && previous && previous->pose && previous->tracking && previous->tracking->rawPose
        && world_.tracking(id, capturedAt).status != TrackingStatus::Lost) {
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
    if (world_.updateTrackedPose(id, filtered, capturedAt, thresholds_, pose, heading, velocity, quality, std::move(provenance), now) != PoseUpdateResult::Applied)
        return TrackerUpdate::RejectedByWorld;
    observed_.insert(id);
    return TrackerUpdate::Applied;
}
void RobotTracker::clearMeasurements(Timestamp now)
{
    invalidate(now);
    world_.clearMeasurements();
    observed_.clear();
}
void RobotTracker::invalidate(Timestamp now)
{
    if (!invalidatedAt_ || now > *invalidatedAt_) invalidatedAt_ = now;
    for (auto id : observed_) world_.invalidateTracking(id);
}
} // namespace micro_robot
