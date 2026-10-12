#pragma once

#include "world/RobotState.hpp"

#include <map>

namespace micro_robot {

// Independent value copy, ordered by RobotId. Mutating a snapshot cannot change
// WorldModel, and later model updates cannot change an existing snapshot.
// Future workspace/obstacle/task state should be copied here alongside robots
// when those issue-scoped domain types are introduced.
struct WorldSnapshot {
    std::map<RobotId, RobotState> robots;
};

} // namespace micro_robot
