#include "world/WorldModel.hpp"

#include <cmath>
#include <algorithm>

namespace micro_robot {

bool WorldModel::registerRobot(RobotId id, Timestamp registeredAt)
{
    return robots_.try_emplace(id, RobotState{id, registeredAt, std::nullopt, std::nullopt, std::nullopt}).second;
}

bool WorldModel::removeRobot(RobotId id) { return robots_.erase(id) != 0; }

PoseUpdateResult WorldModel::updatePose(RobotId id, const Pose2D& pose, Timestamp updatedAt)
{
    const auto found = robots_.find(id);
    if (found == robots_.end()) {
        return PoseUpdateResult::UnknownRobot;
    }
    if (!std::isfinite(pose.x.value) || !std::isfinite(pose.y.value)
        || !std::isfinite(pose.heading.value)) {
        return PoseUpdateResult::InvalidPose;
    }
    auto& state = found->second;
    if (updatedAt < state.registeredAt
        || (state.poseUpdatedAt && updatedAt <= *state.poseUpdatedAt)) {
        return PoseUpdateResult::StaleTimestamp;
    }
    state.pose = pose;
    state.poseUpdatedAt = updatedAt;
    state.tracking.reset();
    return PoseUpdateResult::Applied;
}

PoseUpdateResult WorldModel::updateTrackedPose(RobotId id, const Pose2D& pose,
                                             Timestamp capturedAt, TrackingThresholds thresholds,
                                             std::optional<Pose2D> rawPose, double continuousHeading,
                                             std::optional<PlanarVelocity> velocity, std::optional<double> quality)
{
    if (thresholds.staleAfter <= Duration::zero() || thresholds.lostAfter <= thresholds.staleAfter)
        return PoseUpdateResult::InvalidPose;
    if (!std::isfinite(continuousHeading)
        || (quality && (!std::isfinite(*quality) || *quality < 0 || *quality > 1))
        || (velocity && (!std::isfinite(velocity->xMetersPerSecond)
            || !std::isfinite(velocity->yMetersPerSecond) || !std::isfinite(velocity->radiansPerSecond)))
        || (rawPose && (!std::isfinite(rawPose->x.value) || !std::isfinite(rawPose->y.value)
            || !std::isfinite(rawPose->heading.value)))) return PoseUpdateResult::InvalidPose;
    const auto result = updatePose(id, pose, capturedAt);
    if (result == PoseUpdateResult::Applied) robots_.at(id).tracking = TrackingMetadata{thresholds, false, rawPose, velocity, quality, continuousHeading};
    return result;
}
void WorldModel::invalidateTracking(RobotId id)
{
    const auto found = robots_.find(id);
    if (found != robots_.end() && found->second.tracking) found->second.tracking->invalidated = true;
}
TrackingInfo WorldModel::tracking(RobotId id, Timestamp now) const
{
    const auto found = robots_.find(id);
    if (found == robots_.end() || !found->second.tracking || !found->second.poseUpdatedAt) return {};
    const auto& state = found->second;
    if (now < *state.poseUpdatedAt) return {TrackingStatus::Lost, {}};
    const auto age = now - *state.poseUpdatedAt;
    const double confidence = state.tracking->measurementQuality.value_or(1.0)
        * std::max(0.0, 1.0 - std::chrono::duration<double>(age).count()
            / std::chrono::duration<double>(state.tracking->thresholds.lostAfter).count());
    if (state.tracking->invalidated || age >= state.tracking->thresholds.lostAfter)
        return {TrackingStatus::Lost, age, 0.0};
    if (age >= state.tracking->thresholds.staleAfter) return {TrackingStatus::Stale, age, confidence};
    return {TrackingStatus::Fresh, age, confidence};
}

std::optional<RobotState> WorldModel::robot(RobotId id) const
{
    const auto found = robots_.find(id);
    if (found == robots_.end()) {
        return std::nullopt;
    }
    return found->second;
}

std::size_t WorldModel::robotCount() const noexcept { return robots_.size(); }

WorldSnapshot WorldModel::snapshot() const { return {robots_, workspace_}; }

std::optional<WorkspaceBounds> WorldModel::workspaceBounds() const { return workspace_; }

void WorldModel::setWorkspaceBounds(std::optional<WorkspaceBounds> bounds) { workspace_ = bounds; }

} // namespace micro_robot
