#pragma once
#include "calibration/CalibrationCornerTracker.hpp"

namespace micro_robot {
struct CalibrationCornerTrackingConfig {
    double maximumRoundTripPixels{1.0};
    double maximumPatchError{20.0};
    double maximumSpacingFraction{0.4}; // Avoid jumping to another grid intersection.
    std::chrono::milliseconds maximumStep{150};
};
class OpenCvCalibrationCornerTracker final : public ICalibrationCornerTracker {
public:
    explicit OpenCvCalibrationCornerTracker(CalibrationCornerTrackingConfig config = {});
    [[nodiscard]] std::vector<ImageCorner> track(const Frame& previous,
        const Frame& current, const std::vector<ImageCorner>& corners) override;
private:
    CalibrationCornerTrackingConfig config_;
};
} // namespace micro_robot
