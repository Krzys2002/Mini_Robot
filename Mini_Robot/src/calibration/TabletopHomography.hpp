#pragma once
#include "calibration/CalibrationProfile.hpp"
#include "calibration/WorkspaceCalibration.hpp"

namespace micro_robot {
struct WorkspaceCorrespondence {
    ImageCorner image;  // Undistorted native-K pixels, never display coordinates.
    WorldPoint2D world; // Measured metres.
};

struct WorkspaceHomographyInput {
    WorkspaceCalibrationProfile profile;
    // Paired physical points in any order, including edge/interior references.
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
[[nodiscard]] std::optional<WorldPoint2D> projectWorkspacePoint(const WorkspaceHomography& transform,
                                                                ImageCorner point);
// World metres to undistorted native-K pixels; no viewport or lens conversion.
[[nodiscard]] std::optional<ImageCorner> unprojectWorkspacePoint(const WorkspaceHomography& transform,
                                                               WorldPoint2D point);
[[nodiscard]] std::string validateWorkspaceHomographyInput(const WorkspaceHomographyInput& input);
// Evaluate held-out references against a stored transform without refitting it.
[[nodiscard]] std::optional<WorkspaceMappingErrors>
evaluateWorkspaceChecks(const WorkspaceCalibrationProfile& profile,
                        const std::vector<WorkspaceCorrespondence>& points);
[[nodiscard]] WorkspaceHomographyOutcome evaluateWorkspaceHomography(const WorkspaceHomographyInput& input,
                                                                     const WorkspaceHomography& transform);

class IWorkspaceHomographySolver {
public:
    virtual ~IWorkspaceHomographySolver() = default;
    // Synchronous worker-only adapter operation; failure has no usable result.
    [[nodiscard]] virtual WorkspaceHomographyOutcome solve(const WorkspaceHomographyInput& input) = 0;
};
} // namespace micro_robot
