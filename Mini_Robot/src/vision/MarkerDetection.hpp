#pragma once

#include "camera/CameraTypes.hpp"

#include <array>

namespace micro_robot {
enum class MarkerFamily { AprilTag36h11 };
struct MarkerCorner { double u{}, v{}; }; // Pixels: +u right, +v down.
struct MarkerObservation {
    MarkerFamily family{MarkerFamily::AprilTag36h11};
    int id{}; // Dictionary ID, not RobotId. Duplicate IDs remain separate observations.
    // Canonical decoded TL, TR, BR, BL. Rotates with the printed marker; never
    // re-sort by screen position. Corner 0->1 is the canonical front/top edge.
    std::array<MarkerCorner, 4> corners;
    // Detector-native decision margin when supplied; not a probability.
    // OpenCV ArUco does not expose one and leaves this empty.
    std::optional<double> decisionMargin;
};
struct MarkerDetections {
    std::uint64_t frameSequence{};
    Timestamp capturedAt{}; // Unchanged source capture time, not completion time.
    std::uint32_t width{}, height{};
    Duration processingTime{}; // Conversion + detection + result mapping; no queue delay.
    std::vector<MarkerObservation> markers;
    std::string error; // Empty with zero markers is a successful negative observation.
};
using MarkerDetectionsPtr = std::shared_ptr<const MarkerDetections>;

// Called serially on a processing worker. Never retains or mutates input pixels.
// A failed frame must produce an error without partial observations. Application
// scheduling also catches throwing detector implementations at its boundary.
class IMarkerDetector {
public:
    virtual ~IMarkerDetector() = default;
    [[nodiscard]] virtual MarkerDetections detect(const Frame& frame) = 0;
};
} // namespace micro_robot
