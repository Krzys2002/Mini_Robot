#pragma once
#include "calibration/TabletopHomography.hpp"

namespace micro_robot {
// Dimension-scaled DLT/SVD guard plus all-points OpenCV geometric refinement.
class OpenCvWorkspaceHomographySolver final : public IWorkspaceHomographySolver {
public:
    [[nodiscard]] WorkspaceHomographyOutcome solve(const WorkspaceHomographyInput& input) override;
};
} // namespace micro_robot
