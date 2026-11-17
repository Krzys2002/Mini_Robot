#pragma once
#include "tracking/RobotTracker.hpp"
#include "vision/MetricObservation.hpp"

namespace micro_robot {
// Called on the tracker owner thread. A cleared or invalid batch makes previous
// measurements unavailable immediately; a cleared batch discards old-frame coordinates.
// An invalid non-null batch marks measurements lost; a valid empty batch ages normally.
struct MetricIngestion {
    std::vector<RobotId> accepted, rejected;
    std::vector<std::pair<RobotId,TrackerUpdate>> outcomes;
};
MetricIngestion ingestMetricBatch(RobotTracker& tracker, const MetricObservationBatchPtr& batch, Timestamp now);
} // namespace micro_robot
