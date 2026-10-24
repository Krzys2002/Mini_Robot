#include "vision/VisionDebugFrame.hpp"

namespace micro_robot {
std::shared_ptr<VisionDebugFrame> makeVisionDebugFrame(
    FramePtr frame, MarkerDetectionsPtr detections, const RobotMarkerMap& map)
{
    if (!frame || !isValidFrame(*frame) || !detections
        || frame->sequence != detections->frameSequence || frame->capturedAt != detections->capturedAt
        || frame->width != detections->width || frame->height != detections->height) return {};
    auto result = std::make_shared<VisionDebugFrame>();
    result->frame = std::move(frame);
    result->detections = std::move(detections);
    const auto robots = observeRobots(*result->detections, map);
    result->markers.reserve(robots.robots.size() + robots.rejected.size());
    for (const auto& robot : robots.robots) {
        result->markers.push_back({{robot.family, robot.markerId, robot.corners, robot.decisionMargin},
            robot.robotId, MarkerGeometry{robot.center, robot.imageHeading, robot.minimumEdgePixels}, {}});
    }
    for (const auto& rejected : robots.rejected) {
        result->markers.push_back({rejected.marker, {}, markerGeometry(rejected.marker.corners), rejected.reason});
    }
    return result;
}
void VisionOutputRate::reset() { completions_.clear(); }
std::optional<double> VisionOutputRate::record(Timestamp at)
{
    if (!completions_.empty() && at < completions_.back()) reset();
    completions_.push_back(at);
    if (completions_.size() > WindowSize) completions_.pop_front();
    if (completions_.size() < 2 || at == completions_.front()) return {};
    const double seconds = std::chrono::duration<double>(at - completions_.front()).count();
    return static_cast<double>(completions_.size() - 1) / seconds;
}
} // namespace micro_robot
