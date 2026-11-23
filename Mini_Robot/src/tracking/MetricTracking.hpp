#pragma once
#include "tracking/RobotTracker.hpp"
#include "vision/MetricObservation.hpp"

namespace micro_robot {
// Called on the tracker owner thread. A cleared or invalid batch makes previous
// measurements unavailable immediately; a cleared batch discards old-frame coordinates.
// An invalid non-null batch marks measurements lost; a valid empty batch ages normally.
struct MetricIngestion {
    // Only tracker-applied measurements can authorize downstream control.
    MetricObservationBatchPtr applied;
    std::vector<RobotId> accepted, rejected;
    std::vector<std::pair<RobotId,TrackerUpdate>> outcomes;
};
MetricIngestion ingestMetricBatchWithOutcomes(RobotTracker& tracker, const MetricObservationBatchPtr& batch, Timestamp now);
// Returns immutable metadata for only Applied measurements. Rejected/duplicate
// detections must not authorize downstream control provenance. Invalidation/null
// status is preserved. Callers not observing acceptance may ignore the result.
MetricObservationBatchPtr ingestMetricBatch(RobotTracker& tracker, const MetricObservationBatchPtr& batch, Timestamp now);
} // namespace micro_robot
