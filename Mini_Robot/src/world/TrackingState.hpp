#pragma once
#include "common/Time.hpp"
#include "common/Pose2D.hpp"
#include <optional>

namespace micro_robot {
enum class TrackingStatus { Unknown, Fresh, Stale, Lost };
// Fresh below staleAfter, stale from staleAfter until lostAfter, then lost.
struct TrackingThresholds {
    Duration staleAfter;
    Duration lostAfter;
};
struct PlanarVelocity {
    double xMetersPerSecond{}, yMetersPerSecond{}, radiansPerSecond{};
};
struct TrackingInfo {
    TrackingStatus status{TrackingStatus::Unknown};
    std::optional<Duration> age;
    std::optional<double> confidence; // age-weighted reliability, not a probability
};
// Stored only with a tracked pose; poseUpdatedAt is its capture timestamp.
struct TrackingMetadata {
    TrackingThresholds thresholds;
    bool invalidated{};
    std::optional<Pose2D> rawPose;
    std::optional<PlanarVelocity> velocity;
    std::optional<double> measurementQuality; // [0,1], absent if unknown
    double continuousHeading{}; // unfiltered, unwrapped radians for diagnostics
};
} // namespace micro_robot
