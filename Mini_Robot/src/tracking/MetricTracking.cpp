#include "tracking/MetricTracking.hpp"
#include <cmath>
namespace micro_robot {
MetricObservationBatchPtr ingestMetricBatch(RobotTracker& tracker, const MetricObservationBatchPtr& batch, Timestamp now)
{
    if (!batch) {
        tracker.clearMeasurements(now);
        return {};
    }
    if (batch->status != MetricObservationStatus::Valid) {
        tracker.invalidate(now);
        return batch;
    }
    auto accepted = std::make_shared<MetricObservationBatch>();
    accepted->status = batch->status;
    accepted->rejected = batch->rejected;
    for (const auto& measurement : batch->measurements) {
        // Geometry reliability heuristic, not a calibrated detector probability.
        const auto edge = measurement.minimumEdgePixels;
        const std::optional<double> quality = std::isfinite(edge) && edge > 0
            ? std::optional<double>{edge / (edge + 10.0)} : std::nullopt;
        if (tracker.observe(measurement.robotId, measurement.pose, measurement.capturedAt, now, quality) == TrackerUpdate::Applied)
            accepted->measurements.push_back(measurement);
    }
    return accepted;
}
} // namespace micro_robot
