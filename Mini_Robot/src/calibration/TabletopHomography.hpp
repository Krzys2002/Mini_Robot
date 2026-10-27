#pragma once
#include "calibration/WorkspaceCalibration.hpp"
#include "calibration/CalibrationProfile.hpp"

namespace micro_robot {
struct WorkspaceCorrespondence {
    ImageCorner image; // Undistorted native-K pixels, never display coordinates.
    WorldPoint2D world; // Measured metres.
};
struct WorkspaceHomographyInput {
    WorkspaceCalibrationProfile profile;
    // Convex perimeter order, paired physical points. Either winding is allowed
    // independently in each frame; image v-down often reverses world CCW winding.
    std::vector<WorkspaceCorrespondence> fitPoints;
    std::vector<WorkspaceCorrespondence> checkPoints; // Held out of estimation.
};
struct WorkspaceMappingErrors {
    std::vector<double> distancesMeters;
    double rmsMeters{};
    double maximumMeters{};
};
struct WorkspaceHomographyResult {
    WorkspaceCalibrationProfile profile;
    WorkspaceMappingErrors fit;
    std::optional<WorkspaceMappingErrors> check; // None is not zero validation error.
};
struct WorkspaceHomographyOutcome {
    std::optional<WorkspaceHomographyResult> result;
    std::string error;
};
// Geometry helper used to evaluate calibration; does not transform detections or
// update WorldModel (#58). Null at a horizon/non-finite input or invalid matrix.
[[nodiscard]] std::optional<WorldPoint2D> projectWorkspacePoint(
    const WorkspaceHomography& transform, ImageCorner point);
[[nodiscard]] std::string validateWorkspaceHomographyInput(const WorkspaceHomographyInput& input);
[[nodiscard]] WorkspaceHomographyOutcome evaluateWorkspaceHomography(
    const WorkspaceHomographyInput& input, const WorkspaceHomography& transform);
class IWorkspaceHomographySolver {
public:
    virtual ~IWorkspaceHomographySolver() = default;
    // Synchronous worker-only adapter operation; failure has no usable result.
    [[nodiscard]] virtual WorkspaceHomographyOutcome solve(const WorkspaceHomographyInput& input) = 0;
};
} // namespace micro_robot
