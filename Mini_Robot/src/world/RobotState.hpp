#pragma once

#include "common/Pose2D.hpp"
#include "common/RobotId.hpp"
#include "common/Time.hpp"

#include <optional>

namespace micro_robot {

// Stored application state only. Registration does not imply connection or
// tracking. Pose and its timestamp are either both present or both absent.
struct RobotState {
    RobotId id;
    Timestamp registeredAt;
    std::optional<Pose2D> pose;
    std::optional<Timestamp> poseUpdatedAt;
};

} // namespace micro_robot
