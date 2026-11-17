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
    std::function<Timestamp()> clock = [] { return MonotonicClock::now(); })
{
    return QObject::connect(&vision, &VisionController::metricObservationsAvailable, &context,
        [&vision, &tracker, refresh = std::move(refresh), clock = std::move(clock)](const QString&, MetricObservationBatchPtr batch) {
            if(batch && batch->provenance.inputGeneration!=vision.inputGeneration()) return;
            const auto start=clock();
            auto filtered=batch?std::make_shared<MetricObservationBatch>(*batch):nullptr;
            if(filtered) applyQualityGates(*filtered,vision.qualityGates(),start);
            const auto result=ingestMetricBatch(tracker,filtered,start);
            const auto completed=clock();
            if(filtered) {
                if(vision.recordPublication(*filtered,result.accepted,result.rejected,start,completed))
                for(const auto& [id,outcome]:result.outcomes) if(outcome!=TrackerUpdate::Applied)
                    vision.recordTrackingRejection(id,trackerUpdateName(outcome));
            }
            refresh();
        });
}
} // namespace micro_robot
