#pragma once
#include "calibration/IntrinsicCalibration.hpp"

namespace micro_robot {
class OpenCvIntrinsicCalibrationSolver final : public IIntrinsicCalibrationSolver {
public:
    [[nodiscard]] IntrinsicCalibrationOutcome solve(const IntrinsicCalibrationInput& input,
        const IntrinsicCalibrationConfig& config = {}) override;
};
} // namespace micro_robot
