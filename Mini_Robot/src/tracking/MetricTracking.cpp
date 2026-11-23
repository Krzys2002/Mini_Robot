#include "tracking/MetricTracking.hpp"
#include <cmath>
namespace micro_robot {
MetricIngestion ingestMetricBatchWithOutcomes(RobotTracker& tracker, const MetricObservationBatchPtr& batch, Timestamp now)
{
    MetricIngestion result;
    if (!batch) {
        tracker.clearMeasurements(now);
        return result;
    }
    if (batch->status != MetricObservationStatus::Valid) {
        tracker.invalidate(now);
        result.applied = batch;
        return result;
    }
    auto accepted = std::make_shared<MetricObservationBatch>();
    accepted->status = batch->status;
    accepted->provenance = batch->provenance;
    accepted->rejected = batch->rejected;
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
        if (outcome == TrackerUpdate::Applied) accepted->measurements.push_back(measurement);
        result.outcomes.emplace_back(measurement.robotId,outcome);
    }
    result.applied = std::move(accepted);
    return result;
}

MetricObservationBatchPtr ingestMetricBatch(RobotTracker& tracker, const MetricObservationBatchPtr& batch, Timestamp now)
{
    return ingestMetricBatchWithOutcomes(tracker, batch, now).applied;
}
} // namespace micro_robot
