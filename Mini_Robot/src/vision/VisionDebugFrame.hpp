#pragma once

#include "vision/RobotObservation.hpp"

#include <deque>

namespace micro_robot {
// Presentation data prepared off the GUI thread. Rejected tags stay diagnostic:
// no RobotId is assigned to an unassigned, duplicate or invalid measurement.
struct MarkerDebugOverlay {
    MarkerObservation marker;
    std::optional<RobotId> robotId;
    std::optional<MarkerGeometry> geometry;
    std::optional<MarkerRejection> rejection;
};
struct VisionDebugFrame {
    std::uint64_t inputGeneration{}; // Application publication epoch, not a camera sequence.
    FramePtr frame; // Exact raw image used by the detector, never a newer live frame.
    MarkerDetectionsPtr detections;
    std::vector<MarkerDebugOverlay> markers;
    std::optional<double> outputFramesPerSecond; // Completed output cadence, not camera FPS.
};
using VisionDebugFramePtr = std::shared_ptr<const VisionDebugFrame>;
// Returns empty for mismatched/invalid frame metadata. Mutable only while being
// assembled; publish/retain through VisionDebugFramePtr after preparation.
[[nodiscard]] std::shared_ptr<VisionDebugFrame> makeVisionDebugFrame(
    FramePtr frame, MarkerDetectionsPtr detections, const RobotMarkerMap& map);

// Bounded rolling output cadence; caller supplies steady completion times. Reset
// on source/resolution/generation changes. A single result has no measurable FPS.
class VisionOutputRate final {
public:
    void reset();
    [[nodiscard]] std::optional<double> record(Timestamp completedAt);
private:
    static constexpr std::size_t WindowSize = 16;
    std::deque<Timestamp> completions_;
};
} // namespace micro_robot
