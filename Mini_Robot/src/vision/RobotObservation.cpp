#include "vision/RobotObservation.hpp"

#include "common/Angle.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>

namespace micro_robot {
namespace {
double cross(double au, double av, double bu, double bv) { return au * bv - av * bu; }
} // namespace

RobotMarkerMap::RobotMarkerMap(RobotMarkerProfile profile) : profile_(profile)
{
    const auto maximumRobot = std::numeric_limits<std::uint32_t>::max();
    if (profile.markerCount == 0 || profile.firstMarkerId < 0
        || profile.firstMarkerId >= aprilTag36h11MarkerCount
        || profile.markerCount > static_cast<std::uint32_t>(aprilTag36h11MarkerCount - profile.firstMarkerId)
        || profile.firstRobotId > maximumRobot - (profile.markerCount - 1)) {
        throw std::invalid_argument("Robot marker range must be non-empty, inside the dictionary and RobotId range");
    }
}

std::optional<RobotId> RobotMarkerMap::robotFor(MarkerFamily family, int markerId) const noexcept
{
    if (family != profile_.family || markerId < profile_.firstMarkerId) return std::nullopt;
    const auto offset = static_cast<std::uint32_t>(markerId - profile_.firstMarkerId);
    if (offset >= profile_.markerCount) return std::nullopt;
    return RobotId{profile_.firstRobotId + offset};
}

std::optional<MarkerGeometry> markerGeometry(const std::array<MarkerCorner, 4>& corners)
{
    double shortest = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < corners.size(); ++i) {
        const auto& a = corners[i];
        const auto& b = corners[(i + 1) % corners.size()];
        const auto& c = corners[(i + 2) % corners.size()];
        if (!std::isfinite(a.u) || !std::isfinite(a.v)) return std::nullopt;
        // With +v down, a positive turn at every corner is clockwise as displayed:
        // the canonical unmirrored TL, TR, BR, BL order. Zero rejects collinear corners.
        if (!(cross(b.u - a.u, b.v - a.v, c.u - b.u, c.v - b.v) > 0)) return std::nullopt;
        shortest = std::min(shortest, std::hypot(b.u - a.u, b.v - a.v));
    }
    const auto& [tl, tr, br, bl] = corners;
    // A strictly convex quadrilateral has intersecting diagonals TL-BR and TR-BL.
    // Their intersection is the image of the square's center under perspective.
    const double du1 = br.u - tl.u, dv1 = br.v - tl.v;
    const double du2 = bl.u - tr.u, dv2 = bl.v - tr.v;
    const double t = cross(tr.u - tl.u, tr.v - tl.v, du2, dv2) / cross(du1, dv1, du2, dv2);
    const ImagePoint center{tl.u + t * du1, tl.v + t * dv1};
    const double frontU = (tl.u + tr.u - bl.u - br.u) / 2;
    const double frontV = (tl.v + tr.v - bl.v - br.v) / 2;
    // Negate v so that heading increases counterclockwise as displayed.
    const Radians heading = normalizeAngle(Radians{std::atan2(-frontV, frontU)});
    if (!std::isfinite(center.u) || !std::isfinite(center.v) || !std::isfinite(heading.value)) {
        return std::nullopt;
    }
    return MarkerGeometry{center, heading, shortest};
}

RobotObservations observeRobots(const MarkerDetections& detections, const RobotMarkerMap& map)
{
    RobotObservations result;
    result.frameSequence = detections.frameSequence;
    result.capturedAt = detections.capturedAt;
    result.width = detections.width;
    result.height = detections.height;
    if (!detections.error.empty()) {
        result.error = detections.error;
        return result;
    }

    std::map<RobotId, std::size_t> copies;
    for (const auto& marker : detections.markers) {
        if (const auto id = map.robotFor(marker.family, marker.id)) ++copies[*id];
    }
    for (const auto& marker : detections.markers) {
        const auto id = map.robotFor(marker.family, marker.id);
        if (!id) {
            result.rejected.push_back({marker, marker.family != map.profile().family
                    ? MarkerRejection::UnsupportedFamily : MarkerRejection::UnassignedId});
        } else if (copies[*id] > 1) {
            // Duplicate printed tags are ambiguous: never pick one arbitrarily.
            result.rejected.push_back({marker, MarkerRejection::DuplicateRobot});
        } else if (const auto geometry = markerGeometry(marker.corners)) {
            result.robots.push_back({*id, marker.family, marker.id, marker.corners,
                geometry->center, geometry->imageHeading, detections.frameSequence,
                detections.capturedAt, geometry->minimumEdgePixels, marker.decisionMargin});
        } else {
            result.rejected.push_back({marker, MarkerRejection::InvalidGeometry});
        }
    }
    std::sort(result.robots.begin(), result.robots.end(),
              [](const auto& a, const auto& b) { return a.robotId < b.robotId; });
    return result;
}
} // namespace micro_robot
