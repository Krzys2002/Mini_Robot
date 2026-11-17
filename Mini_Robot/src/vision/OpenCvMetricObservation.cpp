#include "vision/OpenCvMetricObservation.hpp"
#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR >= 5
#include <opencv2/geometry/3d.hpp>
#else
#include <opencv2/calib3d.hpp>
#endif
#include <exception>
#include "calibration/WorkspaceProfile.hpp"

namespace micro_robot {
MetricObservationBatch OpenCvMetricObservationConverter::convert(const MarkerDetections& detections,
    const std::string& sourceId, const RobotMarkerMap& map,
    const std::optional<WorkspaceCalibrationProfile>& profile)
{
    if (!profile || !profile->homography) return {MetricObservationStatus::Uncalibrated,{},"Workspace calibration unavailable"};
    if (!validateWorkspaceCalibrationProfile(*profile).empty() || sourceId != profile->intrinsics.sourceId
        || detections.width != profile->intrinsics.width || detections.height != profile->intrinsics.height)
        return {MetricObservationStatus::InvalidCalibration, {}, "Incompatible workspace calibration"};
    if (!detections.error.empty())
        return {MetricObservationStatus::InvalidGeometry, {}, detections.error};
    MetricObservationBatch batch{MetricObservationStatus::Valid, {}, {}};
    batch.provenance=detections.provenance;
    batch.provenance.sourceId=sourceId; batch.provenance.observedAt=detections.capturedAt;
    batch.provenance.frameSequence=detections.frameSequence;
    batch.provenance.width=detections.width; batch.provenance.height=detections.height;
    batch.provenance.frameId=profile->frameId;
    if(!batch.provenance.calibrationRevision) batch.provenance.calibrationRevision=std::make_shared<const std::string>(serializeWorkspaceProfile(*profile));
    auto bound=detections; bound.provenance=batch.provenance;
    const auto observations = observeRobots(bound, map);
    for (const auto& rejected : observations.rejected) {
        const auto geometry=markerGeometry(rejected.marker.corners);
        batch.diagnostics.push_back({rejected.marker.id,map.robotFor(rejected.marker.family,rejected.marker.id),
            rejected.reason==MarkerRejection::UnsupportedFamily?MeasurementRejection::UnsupportedFamily:
            rejected.reason==MarkerRejection::UnassignedId?MeasurementRejection::UnassignedId:
            rejected.reason==MarkerRejection::DuplicateRobot?MeasurementRejection::DuplicateRobot:MeasurementRejection::InvalidGeometry,
            geometry?std::optional<double>{geometry->minimumEdgePixels}:std::nullopt});
        switch (rejected.reason) {
        case MarkerRejection::UnsupportedFamily: ++batch.rejected.unsupportedFamily; break;
        case MarkerRejection::UnassignedId: ++batch.rejected.unassignedId; break;
        case MarkerRejection::DuplicateRobot: ++batch.rejected.duplicateRobot; break;
        case MarkerRejection::InvalidGeometry: ++batch.rejected.invalidGeometry; break;
        }
    }
    try {
        const auto& intrinsics = profile->intrinsics;
        const cv::Mat k(3, 3, CV_64F, const_cast<double*>(intrinsics.cameraMatrix.data()));
        const cv::Mat d(1, 5, CV_64F, const_cast<double*>(intrinsics.distortion.data()));
        for (const auto& observation : observations.robots) {
            std::vector<cv::Point2d> input, output;
            for (const auto& p : observation.corners) input.emplace_back(p.u, p.v);
            cv::undistortPoints(input, output, k, d, cv::noArray(), k);
            if (output.size() != 4) {
                ++batch.rejected.invalidGeometry;
                batch.diagnostics.push_back({observation.markerId,observation.robotId,MeasurementRejection::InvalidGeometry,observation.minimumEdgePixels});
                continue;
            }
            std::array<MarkerCorner, 4> corners;
            for (std::size_t i = 0; i < 4; ++i) corners[i] = {output[i].x, output[i].y};
            auto result = metricObservation(observation, corners, profile);
            batch.diagnostics.push_back({observation.markerId,observation.robotId,
                result.measurement?MeasurementRejection::None:result.status==MetricObservationStatus::OutsideWorkspace?MeasurementRejection::OutsideWorkspace:MeasurementRejection::InvalidGeometry,observation.minimumEdgePixels});
            if (result.measurement) batch.measurements.push_back(*result.measurement);
            else if (result.status == MetricObservationStatus::OutsideWorkspace) ++batch.rejected.outsideWorkspace;
            else ++batch.rejected.invalidGeometry;
        }
    } catch (const std::exception& error) {
        return {MetricObservationStatus::InvalidGeometry, {}, error.what()};
    }
    return batch;
}
} // namespace micro_robot
