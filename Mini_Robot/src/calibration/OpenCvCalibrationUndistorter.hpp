#pragma once
#include "calibration/CalibrationProfile.hpp"

namespace micro_robot {
class OpenCvCalibrationUndistorter final : public ICalibrationUndistorter {
public:
    [[nodiscard]] UndistortedPreviewOutcome undistort(const Frame& frame,
        const IntrinsicCalibration& calibration) override;
};
} // namespace micro_robot
