#include "world/WorldModel.hpp"

#include <cmath>

namespace micro_robot {

bool WorldModel::registerRobot(RobotId id, Timestamp registeredAt)
{
    return robots_.try_emplace(id, RobotState{id, registeredAt, std::nullopt, std::nullopt}).second;
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
    return PoseUpdateResult::Applied;
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

WorldSnapshot WorldModel::snapshot() const { return {robots_}; }

} // namespace micro_robot
