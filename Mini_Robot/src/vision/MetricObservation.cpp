#include "vision/MetricObservation.hpp"
#include "common/Angle.hpp"
#include <cmath>
#include <algorithm>

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
    if (calibration->localization) {
        const auto& e = *calibration->localization;
        const auto mount = std::find_if(e.mounts.begin(), e.mounts.end(), [&](const auto& m) { return m.robotId == observation.robotId; });
        if (mount == e.mounts.end()) return {MetricObservationStatus::UnknownMount, {}};
        std::array<WorldPoint2D,4> world;
        for (std::size_t i=0;i<corners.size();++i) {
            const auto& p=corners[i];
            if (p.u<0 || p.v<0 || p.u>calibration->intrinsics.width-1 || p.v>calibration->intrinsics.height-1)
                return {MetricObservationStatus::UnsupportedRay,{}};
            const auto w=intersectTagPlane(e,calibration->intrinsics,{p.u,p.v},mount->tagHeightMeters);
            if (!w) return {MetricObservationStatus::UnsupportedRay,{}};
            if (!calibration->bounds.contains(*w)) return {MetricObservationStatus::OutsideWorkspace,{}};
            world[i]=*w;
        }
        const auto center=intersectTagPlane(e,calibration->intrinsics,{geometry->center.u,geometry->center.v},mount->tagHeightMeters);
        if (!center) return {MetricObservationStatus::UnsupportedRay,{}};
        if (!calibration->bounds.contains(*center)) return {MetricObservationStatus::OutsideWorkspace,{}};
        const double dx=world[0].x.value+world[1].x.value-world[2].x.value-world[3].x.value;
        const double dy=world[0].y.value+world[1].y.value-world[2].y.value-world[3].y.value;
        if (!std::isfinite(dx) || !std::isfinite(dy) || std::hypot(dx,dy)<=MinimumHeadingLengthMeters)
            return {MetricObservationStatus::InvalidGeometry,{}};
        const double tagHeading=std::atan2(dy,dx);
        const double cosine=std::cos(tagHeading), sine=std::sin(tagHeading);
        const WorldPoint2D body{Meters{center->x.value+cosine*mount->bodyForwardMeters-sine*mount->bodyLeftMeters},
                               Meters{center->y.value+sine*mount->bodyForwardMeters+cosine*mount->bodyLeftMeters}};
        if (!calibration->bounds.contains(body)) return {MetricObservationStatus::OutsideWorkspace,{}};
        auto provenance = observation.provenance;
        provenance.mountingRevision = mount->revision;
        return {MetricObservationStatus::Valid,MetricPoseMeasurement{observation.robotId,
            {body.x,body.y,normalizeAngle(Radians{tagHeading+mount->bodyHeadingOffsetRadians})},
            observation.capturedAt,observation.frameSequence,observation.minimumEdgePixels,observation.decisionMargin,
            MetricPoseReference::AxleBodyCenter,calibration->intrinsics.sourceId,calibration->frameId,e.revision,mount->revision,std::move(provenance)}};
    }
    const auto& h = *calibration->homography;
    std::array<WorldPoint2D, 4> world;
    // Match projection's homogeneous normalization before checking horizon sides.
    // Finite equivalent H coefficients can otherwise overflow h6*u+h7*v+h8.
    auto denominator = h.imageToWorld;
    double scale = 0;
    for (auto value : denominator) scale = std::max(scale, std::abs(value));
    for (auto& value : denominator) value /= scale; // Profile validation established nonzero finite H.
    double sign = 0;
    for (std::size_t i = 0; i < corners.size(); ++i) {
        const auto& p = corners[i];
        const double w = denominator[6] * p.u + denominator[7] * p.v + denominator[8];
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
    if (!std::isfinite(dx) || !std::isfinite(dy) || std::hypot(dx, dy) <= MinimumHeadingLengthMeters)
        return {MetricObservationStatus::InvalidGeometry, {}};
    return {MetricObservationStatus::Valid, MetricPoseMeasurement{observation.robotId,
        {center->x, center->y, normalizeAngle(Radians{std::atan2(dy, dx)})},
        observation.capturedAt, observation.frameSequence, observation.minimumEdgePixels, observation.decisionMargin,
        MetricPoseReference::TabletopTagCenter,calibration->intrinsics.sourceId,calibration->frameId,calibration->acquisitionId,{},observation.provenance}};
}
} // namespace micro_robot
