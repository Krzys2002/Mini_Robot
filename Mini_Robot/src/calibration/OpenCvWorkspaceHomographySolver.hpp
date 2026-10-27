#pragma once
#include "calibration/TabletopHomography.hpp"
namespace micro_robot {
class OpenCvWorkspaceHomographySolver final : public IWorkspaceHomographySolver {
public:
    [[nodiscard]] WorkspaceHomographyOutcome solve(const WorkspaceHomographyInput& input) override;
};
}
