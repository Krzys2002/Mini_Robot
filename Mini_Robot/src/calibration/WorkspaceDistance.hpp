#pragma once
#include "calibration/WorkspaceDiagnostics.hpp"

namespace micro_robot
{
inline constexpr double WorkspaceDistanceSupportRoundoffUlps = 32;
inline constexpr std::size_t MaximumWorkspaceSelectionMethodBytes = 512;
inline constexpr double WorkspaceDistanceCoverage = 0.95;
inline constexpr std::size_t WorkspaceDistanceEndpointCount = 2;
enum class WorkspaceSelectionSpace { NativeUndistortedPixels, WorldMeters };
struct WorkspaceEndpointSelection {
    WorkspaceSelectionSpace space{WorkspaceSelectionSpace::NativeUndistortedPixels};
    std::optional<double> maximumRadius; // Explicit deterministic disk bound; null is UNKNOWN, never zero.
    std::string justification; // Required before reporting ±; not an automatically inferred precision.
};
struct WorkspaceDistanceEndpoint {
    WorldPoint2D world;
    WorkspaceEndpointSelection selection;
};
struct WorkspaceDistanceResult {
    double distanceMeters{};
    std::optional<double> plusMinusMeters, lowerMeters, upperMeters;
    std::array<std::optional<double>, WorkspaceDistanceEndpointCount> selectionRadiiMeters,
        calibrationRadiiMeters;
    std::string uncertaintyUnavailable;
    double coverage{WorkspaceDistanceCoverage}; // Conditional for a fixed pair; no endpoint independence.
};
struct WorkspaceDistanceOutcome {
    std::optional<WorkspaceDistanceResult> result;
    std::string error; // Invalid centre/mapping -> no measurement; unknown footprint -> distance without ±.
};
// GUI-independent letterboxing and native pixel-centre convention. All view values
// are logical coordinates; device-pixel ratio is not applied a second time.
struct WorkspaceCameraViewport {
    double left{}, top{}, width{}, height{};
    std::uint32_t nativeWidth{}, nativeHeight{};
};
[[nodiscard]] std::optional<WorkspaceCameraViewport> workspaceCameraViewport(std::uint32_t nativeWidth,
                                                                             std::uint32_t nativeHeight,
                                                                             double viewWidth,
                                                                             double viewHeight);
[[nodiscard]] std::optional<ImageCorner> workspaceCameraPick(const WorkspaceCameraViewport&, double x,
                                                             double y);
[[nodiscard]] std::optional<WorkspaceViewPoint> workspaceCameraDisplay(const WorkspaceCameraViewport&,
                                                                       ImageCorner);
// Exact projective support of a world-space disk: bounds, native image, available
// fit hulls; optionally the mapped survey/prediction hull. No sampling shortcut.
[[nodiscard]] bool workspaceDistanceDiskSupported(const WorkspaceDiagnostics&, WorldPoint2D,
                                                  double radiusMeters,
                                                  bool requirePredictionCoverage = false);
[[nodiscard]] std::optional<WorldPoint2D> workspaceDistancePointFromImage(const WorkspaceDiagnostics&,
                                                                          ImageCorner);
[[nodiscard]] WorkspaceDistanceOutcome
measureWorkspaceDistance(const WorkspaceDiagnostics&,
                         const std::array<WorkspaceDistanceEndpoint, WorkspaceDistanceEndpointCount>&);
} // namespace micro_robot
