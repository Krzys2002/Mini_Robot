#include "vision/MetricObservation.hpp"
#include "common/Angle.hpp"
#include <cmath>

namespace micro_robot {
MetricObservationResult metricObservation(const RobotObservation& observation,
    const std::array<MarkerCorner, 4>& corners,
    const std::optional<WorkspaceCalibrationProfile>& calibration)
{
    if (!calibration || !calibration->homography) return {MetricObservationStatus::Uncalibrated, {}};
    if (!validateWorkspaceCalibrationProfile(*calibration).empty())
        return {MetricObservationStatus::InvalidCalibration, {}};
    const auto geometry = markerGeometry(corners);
    if (!geometry) return {MetricObservationStatus::InvalidGeometry, {}};
    const auto& h = *calibration->homography;
    std::array<WorldPoint2D, 4> world;
    double sign = 0;
    for (std::size_t i = 0; i < corners.size(); ++i) {
        const auto& p = corners[i];
        const double w = h.imageToWorld[6] * p.u + h.imageToWorld[7] * p.v + h.imageToWorld[8];
        const auto point = projectWorkspacePoint(h, {p.u, p.v});
        if (!point || !std::isfinite(w) || w == 0 || (i && std::signbit(w) != std::signbit(sign)))
            return {MetricObservationStatus::InvalidGeometry, {}};
        sign = w;
        world[i] = *point;
    }
    for (const auto& point : world)
        if (!calibration->bounds.contains(point)) return {MetricObservationStatus::OutsideWorkspace, {}};
    const auto center = projectWorkspacePoint(h, {geometry->center.u, geometry->center.v});
    if (!center) return {MetricObservationStatus::InvalidGeometry, {}};
    if (!calibration->bounds.contains(*center)) return {MetricObservationStatus::OutsideWorkspace, {}};
    const double dx = world[0].x.value + world[1].x.value - world[2].x.value - world[3].x.value;
    const double dy = world[0].y.value + world[1].y.value - world[2].y.value - world[3].y.value;
    if (!std::isfinite(dx) || !std::isfinite(dy) || std::hypot(dx, dy) <= 1e-12)
        return {MetricObservationStatus::InvalidGeometry, {}};
    return {MetricObservationStatus::Valid, MetricPoseMeasurement{observation.robotId,
        {center->x, center->y, normalizeAngle(Radians{std::atan2(dy, dx)})},
        observation.capturedAt, observation.frameSequence, observation.minimumEdgePixels, observation.decisionMargin}};
}
} // namespace micro_robot
