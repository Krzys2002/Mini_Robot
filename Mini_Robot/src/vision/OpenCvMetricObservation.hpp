#pragma once
#include "vision/MetricObservation.hpp"
namespace micro_robot {
// Called serially on the vision worker.
class OpenCvMetricObservationConverter final : public IMetricObservationConverter {
public:
    ReferenceFramePtr referenceObservations(const MarkerDetections&, const std::string&, const RobotMarkerMap&, const WorkspaceCalibrationProfile&) override;
    MetricObservationBatch convert(const MarkerDetections&, const std::string& sourceId,
        const RobotMarkerMap&, const std::optional<WorkspaceCalibrationProfile>&) override;
private:
    ReferenceMonitor monitor_;
};
} // namespace micro_robot
