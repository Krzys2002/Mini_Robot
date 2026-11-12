#pragma once
#include "calibration/TabletopHomography.hpp"
#include <cstdint>
#include <memory>
#include <limits>

namespace micro_robot {
inline constexpr CalibrationTarget WorkspaceBoardTarget{}; // Same intrinsic 9x6 / 25 mm target.
inline constexpr double WorkspaceBoardRoundoffUlps = 16; // Only machine-roundoff at a declared boundary.
inline constexpr std::size_t MaximumWorkspacePlacements = 128;
inline constexpr std::size_t MaximumWorkspaceAcquisitionBytes = 4 * 1024 * 1024;
inline constexpr std::size_t MaximumWorkspacePlacementNoteBytes = 512;
inline constexpr std::size_t MaximumWorkspaceAcquisitionIdBytes = 128;

struct WorkspaceBoardPlacement {
    std::uint64_t id{};
    CalibrationTarget target{WorkspaceBoardTarget};
    std::size_t anchorIndex{}; // Logical row-major inner corner after explicit ordering selection.
    WorldPoint2D anchor;
    Radians columnAngle{std::numeric_limits<double>::quiet_NaN()}; // +columns angle from world +X, positive CCW.
    int rowDirection{}; // +1: +rows is +90 degrees from +columns; -1: -90 degrees.
    bool reverseColumns{}, reverseRows{};
    std::optional<std::uint64_t> relativeReferenceId;
    WorldPoint2D relativeReferenceSnapshot; // Original reference coordinates, even if later edited/removed.
    WorldPoint2D measuredOffset; // World-axis offset; anchor is the resolved absolute position.
    std::uint64_t frameSequence{};
    std::int64_t capturedTicksNanoseconds{}; // Session steady-clock tick, not UTC.
    std::string note;
    std::vector<ImageCorner> detected; // Original complete detection on that exact undistorted frame.
    std::vector<WorkspaceReferenceRejection> reviewRejections; // Logical order, original automatic review result.
    std::vector<bool> included; // Explicit review mask; original detections remain even when excluded.
};
struct WorkspaceBoardSource { std::uint64_t placementId{}; std::size_t cornerIndex{}; };
struct WorkspaceReference {
    std::uint64_t id{};
    WorkspaceCorrespondence point;
    bool check{}, boundary{};
    std::optional<WorkspaceBoardSource> source;
};
struct WorkspaceAcquisitionRecord {
    std::string id;
    WorkspaceCalibrationProfile profile; // Exact H, frame, optics and acquisition binding.
    std::vector<WorkspaceReference> references;
    std::vector<WorkspaceBoardPlacement> placements;
};
struct WorkspaceAcquisitionLoadOutcome {
    std::shared_ptr<WorkspaceAcquisitionRecord> record;
    std::string error;
};
// Pure generation: explicit orientation/order, no inference of physical identity or movement.
// Returns all 54 pairs, including any outside bounds for explicit review/exclusion.
struct WorkspaceBoardGeneration { std::vector<WorkspaceCorrespondence> points; std::string error; };
[[nodiscard]] WorkspaceBoardGeneration generateWorkspaceBoard(const WorkspaceBoardPlacement& placement, std::optional<WorkspaceBounds> bounds = {});
[[nodiscard]] std::string validateWorkspaceAcquisition(const WorkspaceAcquisitionRecord& record);
// Canonical strict format, no raster storage. Caller must retain physical evidence separately.
[[nodiscard]] std::string serializeWorkspaceAcquisition(const WorkspaceAcquisitionRecord& record);
[[nodiscard]] WorkspaceAcquisitionLoadOutcome parseWorkspaceAcquisition(std::string_view text);
} // namespace micro_robot
