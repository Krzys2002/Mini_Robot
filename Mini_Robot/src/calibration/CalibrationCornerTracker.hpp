#pragma once
#include "calibration/CalibrationCapture.hpp"

namespace micro_robot {
// Presentation estimates only, never calibration observations or quality scores.
// Synchronous on a worker; an empty result means hide the complete overlay.
class ICalibrationCornerTracker {
public:
    virtual ~ICalibrationCornerTracker() = default;
    [[nodiscard]] virtual std::vector<ImageCorner> track(const Frame& previous,
        const Frame& current, const std::vector<ImageCorner>& corners) = 0;
};
} // namespace micro_robot
