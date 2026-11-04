#pragma once
#include "tracking/RobotTracker.hpp"
#include "vision/MetricObservation.hpp"

namespace micro_robot {
// Called on the tracker owner thread. A cleared or invalid batch makes previous
// measurements lost immediately; a valid empty batch leaves them to age normally.
void ingestMetricBatch(RobotTracker& tracker, const MetricObservationBatchPtr& batch, Timestamp now);
} // namespace micro_robot
