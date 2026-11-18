#pragma once
#include "vision/MetricObservation.hpp"
namespace micro_robot {
inline constexpr int ReferenceUndistortMaximumIterations = 20;
inline constexpr double ReferenceUndistortConvergence = 1e-10;
[[nodiscard]] ReferenceFramePtr acquireReferenceFrame(const MarkerDetections&, const std::string&,
    const RobotMarkerMap&, const WorkspaceCalibrationProfile&);
}
