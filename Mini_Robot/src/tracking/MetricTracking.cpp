#include "tracking/MetricTracking.hpp"
#include <cmath>
namespace micro_robot {
void ingestMetricBatch(RobotTracker& tracker, const MetricObservationBatchPtr& batch, Timestamp now)
{
    if (!batch || batch->status != MetricObservationStatus::Valid) {
        tracker.invalidate(now);
        return;
    }
    for (const auto& measurement : batch->measurements) {
        // Geometry reliability heuristic, not a calibrated detector probability.
        const auto edge = measurement.minimumEdgePixels;
        const std::optional<double> quality = std::isfinite(edge) && edge > 0
            ? std::optional<double>{edge / (edge + 10.0)} : std::nullopt;
        (void)tracker.observe(measurement.robotId, measurement.pose, measurement.capturedAt, now, quality);
    }
}
} // namespace micro_robot
