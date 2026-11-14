#pragma once
#include "world/WorldModel.hpp"
#include <set>

namespace micro_robot {
enum class TrackerUpdate { Applied, InvalidMeasurement, FutureTimestamp, ExpiredMeasurement, RejectedByWorld };
// Owner-thread service. WorldModel remains the sole pose authority. `now` must
// use the same steady clock as capturedAt. No filtering or motion control.
class RobotTracker final {
public:
    RobotTracker(WorldModel& world, TrackingThresholds thresholds);
    [[nodiscard]] TrackerUpdate observe(RobotId id, const Pose2D& pose, Timestamp capturedAt, Timestamp now);
    // Input/calibration invalidation makes all poses from this tracker lost.
    void invalidate(Timestamp now);
    // Exclusive camera/frame owner: discard raw/tracked coordinates while retaining robot identities.
    void clearMeasurements(Timestamp now);
private:
    WorldModel& world_;
    TrackingThresholds thresholds_;
    std::set<RobotId> observed_;
    std::optional<Timestamp> invalidatedAt_;
};
} // namespace micro_robot
