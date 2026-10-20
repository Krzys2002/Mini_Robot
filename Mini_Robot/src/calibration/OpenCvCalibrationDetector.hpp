#pragma once
#include "calibration/CalibrationCapture.hpp"

namespace micro_robot {
// Portable image-processing adapter; no camera ownership, Qt, or calibration solver.
class OpenCvCalibrationDetector final : public ICalibrationDetector {
public:
    [[nodiscard]] CalibrationDetection detect(const Frame& frame,
                                              const CalibrationTarget& target) override;
};
} // namespace micro_robot
