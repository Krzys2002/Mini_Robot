#include "tracking/MetricTracking.hpp"
namespace micro_robot {
void ingestMetricBatch(RobotTracker& tracker, const MetricObservationBatchPtr& batch, Timestamp now)
{
    if (!batch || batch->status != MetricObservationStatus::Valid) {
        tracker.invalidate(now);
        return;
    }
    for (const auto& measurement : batch->measurements)
        (void)tracker.observe(measurement.robotId, measurement.pose, measurement.capturedAt, now);
}
} // namespace micro_robot
