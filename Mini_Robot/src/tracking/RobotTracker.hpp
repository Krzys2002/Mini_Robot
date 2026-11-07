#pragma once
#include "world/WorldModel.hpp"
#include <set>
#include "tracking/KalmanPoseFilter.hpp"
#include <map>

namespace micro_robot {
enum class PoseFilterKind { Exponential, Kalman };
struct PoseFilterConfig {
    double positionAlpha{1.0}; // (0,1], 1 preserves raw position
    double headingAlpha{1.0};
    PoseFilterKind kind{PoseFilterKind::Exponential};
    KalmanNoise noise{};
};
enum class TrackerUpdate { Applied, InvalidMeasurement, FutureTimestamp, ExpiredMeasurement, RejectedByWorld };
// Owner-thread service. WorldModel remains the sole pose authority. `now` must
// use the same steady clock as capturedAt. No motion control. Filter state is derived from the authoritative world.
class RobotTracker final {
public:
    RobotTracker(WorldModel& world, TrackingThresholds thresholds, PoseFilterConfig filter = {});
    [[nodiscard]] TrackerUpdate observe(RobotId id, const Pose2D& pose, Timestamp capturedAt, Timestamp now,
                                        std::optional<double> quality = {});
    // Input/calibration invalidation makes all poses from this tracker lost.
    void invalidate(Timestamp now);
private:
    WorldModel& world_;
    TrackingThresholds thresholds_;
    PoseFilterConfig filter_;
    struct KalmanTrack { KalmanPoseFilter filter; Timestamp updatedAt; };
    std::map<RobotId, KalmanTrack> kalman_; // derived estimator cache, never pose authority
    std::set<RobotId> observed_;
    std::optional<Timestamp> invalidatedAt_;
};
} // namespace micro_robot
