#pragma once
#include "common/Time.hpp"
#include <optional>

namespace micro_robot {
enum class TrackingStatus { Unknown, Fresh, Stale, Lost };
// Fresh below staleAfter, stale from staleAfter until lostAfter, then lost.
struct TrackingThresholds {
    Duration staleAfter;
    Duration lostAfter;
};
struct TrackingInfo {
    TrackingStatus status{TrackingStatus::Unknown};
    std::optional<Duration> age;
};
// Stored only with a tracked pose; poseUpdatedAt is its capture timestamp.
struct TrackingMetadata {
    TrackingThresholds thresholds;
    bool invalidated{};
};
} // namespace micro_robot
