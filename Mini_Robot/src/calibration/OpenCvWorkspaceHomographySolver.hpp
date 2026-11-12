#pragma once
#include "calibration/TabletopHomography.hpp"

namespace micro_robot {
inline constexpr double WorkspaceMinimumSingularValueRatio = 1e-9;
// Dimension-scaled DLT/SVD guard plus all-points OpenCV geometric refinement.
class OpenCvWorkspaceHomographySolver final : public IWorkspaceHomographySolver {
public:
    [[nodiscard]] WorkspaceHomographyOutcome solve(const WorkspaceHomographyInput& input) override;
};
} // namespace micro_robot
