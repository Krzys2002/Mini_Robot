#pragma once
#include "vision/MetricObservation.hpp"
namespace micro_robot {
// Called serially on the vision worker.
class OpenCvMetricObservationConverter final : public IMetricObservationConverter {
public:
    MetricObservationBatch convert(const MarkerDetections&, const std::string& sourceId,
        const RobotMarkerMap&, const std::optional<WorkspaceCalibrationProfile>&) override;
};
} // namespace micro_robot
