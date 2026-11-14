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
// The composition root owns one instance and injects references into services.
// Copying/moving the registry is forbidden; use snapshot() for detached state.
// Future obstacle/task domain storage belongs here, with corresponding
// value fields in WorldSnapshot, rather than separate authoritative registries.
class WorldModel final {
public:
    WorldModel() = default;
    WorldModel(const WorldModel&) = delete;
    WorldModel& operator=(const WorldModel&) = delete;
    WorldModel(WorldModel&&) = delete;
    WorldModel& operator=(WorldModel&&) = delete;

    // False for an existing ID; duplicates never reset its pose or timestamps.
    [[nodiscard]] bool registerRobot(RobotId id, Timestamp registeredAt);
    [[nodiscard]] bool removeRobot(RobotId id);

    // Pose is already in world metres/radians. Heading is stored as supplied.
    // Updates cannot precede registration. Once a pose exists, only a strictly
    // newer timestamp is accepted. Failed updates leave all state unchanged.
    [[nodiscard]] PoseUpdateResult updatePose(RobotId id, const Pose2D& pose,
                                            Timestamp updatedAt);

    // Apply tracked measurements on the owner thread. Rejections leave state intact.
    [[nodiscard]] PoseUpdateResult updateTrackedPose(RobotId id, const Pose2D& pose,
                                                     Timestamp capturedAt, TrackingThresholds thresholds);
    void invalidateTracking(RobotId id);
    // Explicit active-frame replacement: old coordinates cannot survive in a new frame.
    // Registry/transport identity remains intact. Owner thread only.
    void clearMeasurements();
    // Explicit time avoids hidden clocks. A time before capture is conservatively lost.
    [[nodiscard]] TrackingInfo tracking(RobotId id, Timestamp now) const;
    [[nodiscard]] std::optional<RobotState> robot(RobotId id) const;
    [[nodiscard]] std::size_t robotCount() const noexcept;
    [[nodiscard]] WorldSnapshot snapshot() const;

    // Same owning thread as robot state. Bounds describe the table; they do not
    // establish camera calibration or clamp/rewrite existing robot measurements.
    void setWorkspaceBounds(std::optional<WorkspaceBounds> bounds);
    [[nodiscard]] std::optional<WorkspaceBounds> workspaceBounds() const;
    // Explicit active-frame installation clears old coordinates and keeps its identity
    // in detached snapshots. A nonempty frame identity is required with bounds.
    void setWorkspaceFrame(WorkspaceBounds bounds, std::string frameId);

private:
    std::map<RobotId, RobotState> robots_;
    std::optional<WorkspaceBounds> workspace_;
    std::optional<std::string> workspaceFrameId_;
};

} // namespace micro_robot
