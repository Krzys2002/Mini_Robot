#pragma once
#include "calibration/ElevatedLocalization.hpp"
namespace micro_robot {
class OpenCvExtrinsicsSolver final : public IExtrinsicsSolver {
public:
    ExtrinsicSolveOutcome solve(const WorkspaceCalibrationProfile&, const ExtrinsicSolveInput&) override;
};
}
