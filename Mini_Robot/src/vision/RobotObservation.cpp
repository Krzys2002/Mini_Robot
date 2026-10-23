#include "vision/RobotObservation.hpp"

#include <algorithm>
#include <limits>
#include <map>
#include <stdexcept>

namespace micro_robot {
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
        } else {
            result.robots.push_back({*id, marker.family, marker.id, marker.corners,
                detections.frameSequence, detections.capturedAt, marker.decisionMargin});
        }
    }
    std::sort(result.robots.begin(), result.robots.end(),
              [](const auto& a, const auto& b) { return a.robotId < b.robotId; });
    return result;
}
} // namespace micro_robot
