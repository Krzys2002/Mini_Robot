#pragma once
#include "tracking/RobotTracker.hpp"
#include "vision/MetricObservation.hpp"

namespace micro_robot {
// Called on the tracker owner thread. A cleared or invalid batch makes previous
// measurements unavailable immediately; a cleared batch discards old-frame coordinates.
// An invalid non-null batch marks measurements lost; a valid empty batch ages normally.
void ingestMetricBatch(RobotTracker& tracker, const MetricObservationBatchPtr& batch, Timestamp now);
} // namespace micro_robot
