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
        [&tracker, refresh = std::move(refresh), clock = std::move(clock)](const QString&, MetricObservationBatchPtr batch) {
            ingestMetricBatch(tracker, batch, clock());
            refresh();
        });
}
} // namespace micro_robot
