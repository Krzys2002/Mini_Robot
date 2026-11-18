#pragma once
#include "calibration/CalibrationProfile.hpp"

namespace micro_robot {
class OpenCvCalibrationUndistorter final : public ICalibrationUndistorter {
public:
    [[nodiscard]] static std::optional<ImageCorner> undistortPoint(ImageCorner point,
        const IntrinsicCalibration& calibration);
    // Maps ideal undistorted native-K pixels back onto the raw distorted image.
    [[nodiscard]] static std::optional<ImageCorner> distortPoint(ImageCorner point,
        const IntrinsicCalibration& calibration);
    [[nodiscard]] UndistortedPreviewOutcome undistort(const Frame& frame,
        const IntrinsicCalibration& calibration) override;
    [[nodiscard]] UndistortedPreviewOutcome undistortWithCorners(const Frame& frame,
        const IntrinsicCalibration& calibration, const std::vector<ImageCorner>& corners) override;
};
} // namespace micro_robot
