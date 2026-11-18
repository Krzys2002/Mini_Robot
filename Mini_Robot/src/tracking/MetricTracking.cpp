#include "tracking/MetricTracking.hpp"
#include <cmath>
namespace micro_robot {
MetricIngestion ingestMetricBatch(RobotTracker& tracker, const MetricObservationBatchPtr& batch, Timestamp now)
{
    MetricIngestion result;
    if (!batch) {
        tracker.clearMeasurements(now);
        return result;
    }
    if (batch->status != MetricObservationStatus::Valid) {
        tracker.invalidate(now);
        return result;
    }
    for (const auto& measurement : batch->measurements) {
        // Geometry reliability heuristic, not a calibrated detector probability.
        const auto edge = measurement.minimumEdgePixels;
        const std::optional<double> quality = std::isfinite(edge) && edge > 0
            ? std::optional<double>{edge / (edge + 10.0)} : std::nullopt;
        auto provenance=measurement.provenance;
        const auto bound=!provenance.sourceId.empty();
        const auto outcome=tracker.observe(measurement.robotId, measurement.pose, measurement.capturedAt, now, quality,
            bound?std::optional<MeasurementProvenance>{provenance}:std::nullopt);
        (outcome==TrackerUpdate::Applied?result.accepted:result.rejected).push_back(measurement.robotId);
        result.outcomes.emplace_back(measurement.robotId,outcome);
    }
    return result;
}
} // namespace micro_robot
