#pragma once

#include "common/RobotId.hpp"
#include "vision/MarkerDetection.hpp"

namespace micro_robot {
inline constexpr int aprilTag36h11MarkerCount = 587; // Dictionary IDs 0..586.

// Allocated robot marker range: markerCount consecutive dictionary IDs starting at
// firstMarkerId map to consecutive RobotIds starting at firstRobotId. Defaults are
// the #49 profile: tag36h11 IDs 0-15 -> RobotId 1-16. Other IDs are unassigned.
struct RobotMarkerProfile {
    MarkerFamily family{MarkerFamily::AprilTag36h11};
    int firstMarkerId{0};
    std::uint32_t markerCount{16};
    std::uint32_t firstRobotId{1};
};

// Validated, immutable identity lookup. The constructor throws
// std::invalid_argument for an empty range, a range outside the family's
// dictionary, or RobotIds that would overflow uint32.
class RobotMarkerMap final {
public:
    explicit RobotMarkerMap(RobotMarkerProfile profile = {});
    [[nodiscard]] const RobotMarkerProfile& profile() const noexcept { return profile_; }
    // Empty for another family or an unassigned/negative dictionary ID.
    [[nodiscard]] std::optional<RobotId> robotFor(MarkerFamily family, int markerId) const noexcept;
private:
    RobotMarkerProfile profile_;
};

enum class MarkerRejection {
    UnsupportedFamily, // Family differs from the profile.
    UnassignedId,      // Dictionary ID outside the allocated robot range.
    DuplicateRobot,    // Several markers in one frame map to the same RobotId.
};
struct RejectedMarker {
    MarkerObservation marker;
    MarkerRejection reason{};
};

// One identified robot marker in one frame: a measurement, not robot state.
// Pixel geometry only; no undistortion or world conversion has been applied.
struct RobotObservation {
    RobotId robotId{0};
    MarkerFamily family{MarkerFamily::AprilTag36h11};
    int markerId{};
    std::array<MarkerCorner, 4> corners; // Unchanged canonical TL, TR, BR, BL.
    std::uint64_t frameSequence{};
    Timestamp capturedAt{}; // Source capture time of the frame.
    std::optional<double> decisionMargin; // Detector-native; empty when unsupported.
};

// Per-frame result. A detection error is propagated without observations or
// rejections. Robots are ordered by RobotId and contain at most one entry per
// RobotId; every other marker appears in rejected, in detection order.
struct RobotObservations {
    std::uint64_t frameSequence{};
    Timestamp capturedAt{};
    std::uint32_t width{}, height{};
    std::vector<RobotObservation> robots;
    std::vector<RejectedMarker> rejected;
    std::string error;
};

// Pure and deterministic; safe to call from any thread with distinct arguments.
[[nodiscard]] RobotObservations observeRobots(const MarkerDetections& detections,
                                              const RobotMarkerMap& map);
} // namespace micro_robot
