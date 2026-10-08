#pragma once

#include "world/RobotState.hpp"

#include <map>

namespace micro_robot {

// Independent value copy, ordered by RobotId. Mutating a snapshot cannot change
// WorldModel, and later model updates cannot change an existing snapshot.
struct WorldSnapshot {
    std::map<RobotId, RobotState> robots;
};

} // namespace micro_robot
