#pragma once
#include "app/VisionController.hpp"
#include "tracking/MetricTracking.hpp"
#include <functional>

namespace micro_robot {
// One VisionController exclusively owns all tracks in this tracker. Its input
// generation fences source changes/restarts before publication. sourceId is
// diagnostic: never connect a second camera to the same tracker with this bridge.
// Multi-camera support requires per-source invalidation and explicit fusion ownership.
inline QMetaObject::Connection connectVisionTracking(VisionController& vision, RobotTracker& tracker,
    QObject& context, std::function<void()> refresh,
    std::function<Timestamp()> clock = [] { return MonotonicClock::now(); },
    std::function<void(const MetricObservationBatchPtr&)> afterIngest = {})
{
    return QObject::connect(&vision, &VisionController::metricObservationsAvailable, &context,
        [&vision, &tracker, refresh = std::move(refresh), clock = std::move(clock), afterIngest = std::move(afterIngest)](const QString&, MetricObservationBatchPtr batch) {
            if(batch && batch->provenance.inputGeneration!=vision.inputGeneration()) return;
            const auto start=clock();
            if (batch && batch->status == MetricObservationStatus::Valid && !batch->measurements.empty()) {
                auto filtered = std::make_shared<MetricObservationBatch>(*batch);
                applyQualityGates(*filtered, vision.qualityGates(), start);
                batch = std::move(filtered);
            }
            const auto result = ingestMetricBatchWithOutcomes(tracker, batch, start);
            const auto completed = clock();
            if (batch && vision.recordPublication(*batch, result.accepted, result.rejected, start, completed)) {
                for (const auto& [id, outcome] : result.outcomes) {
                    if (outcome != TrackerUpdate::Applied)
                        vision.recordTrackingRejection(id, trackerUpdateName(outcome));
                }
            }
            if (afterIngest) afterIngest(result.applied);
            refresh();
        });
}
} // namespace micro_robot
