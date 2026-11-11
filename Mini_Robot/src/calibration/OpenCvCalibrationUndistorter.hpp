#pragma once
#include "calibration/CalibrationProfile.hpp"

namespace micro_robot {
class OpenCvCalibrationUndistorter final : public ICalibrationUndistorter {
public:
    [[nodiscard]] UndistortedPreviewOutcome undistort(const Frame& frame,
        const IntrinsicCalibration& calibration) override;
    [[nodiscard]] UndistortedPreviewOutcome undistortWithCorners(const Frame& frame,
        const IntrinsicCalibration& calibration, const std::vector<ImageCorner>& corners) override;
};
} // namespace micro_robot
