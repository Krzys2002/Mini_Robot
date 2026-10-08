#pragma once

#include "world/WorldSnapshot.hpp"

#include <cstddef>
#include <map>
#include <optional>

namespace micro_robot {

enum class PoseUpdateResult { Applied, UnknownRobot, StaleTimestamp, InvalidPose };

// One authoritative registry, used on its owning thread. No background work,
// automatic clock reads, filtering, communication, capture, or rendering.
// Snapshot copies may be handed off to other threads through the application.
class WorldModel final {
public:
    // False for an existing ID; duplicates never reset its pose or timestamps.
    [[nodiscard]] bool registerRobot(RobotId id, Timestamp registeredAt);
    [[nodiscard]] bool removeRobot(RobotId id);

    // Pose is already in world metres/radians. Heading is stored as supplied.
    // Updates cannot precede registration. Once a pose exists, only a strictly
    // newer timestamp is accepted. Failed updates leave all state unchanged.
    [[nodiscard]] PoseUpdateResult updatePose(RobotId id, const Pose2D& pose,
                                            Timestamp updatedAt);

    [[nodiscard]] std::optional<RobotState> robot(RobotId id) const;
    [[nodiscard]] std::size_t robotCount() const noexcept;
    [[nodiscard]] WorldSnapshot snapshot() const;

private:
    std::map<RobotId, RobotState> robots_;
};

} // namespace micro_robot
