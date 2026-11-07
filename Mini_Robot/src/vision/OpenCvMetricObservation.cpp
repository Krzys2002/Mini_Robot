#include "vision/OpenCvMetricObservation.hpp"
#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR >= 5
#include <opencv2/geometry/3d.hpp>
#else
#include <opencv2/calib3d.hpp>
#endif
#include <exception>

namespace micro_robot {
MetricObservationBatch OpenCvMetricObservationConverter::convert(const MarkerDetections& detections,
    const std::string& sourceId, const RobotMarkerMap& map,
    const std::optional<WorkspaceCalibrationProfile>& profile)
{
    if (!profile || !profile->homography) return {};
    if (!validateWorkspaceCalibrationProfile(*profile).empty() || sourceId != profile->intrinsics.sourceId
        || detections.width != profile->intrinsics.width || detections.height != profile->intrinsics.height)
        return {MetricObservationStatus::InvalidCalibration, {}, "Incompatible workspace calibration"};
    if (!detections.error.empty())
        return {MetricObservationStatus::InvalidGeometry, {}, detections.error};
    MetricObservationBatch batch{MetricObservationStatus::Valid, {}, {}};
    const auto observations = observeRobots(detections, map);
    for (const auto& rejected : observations.rejected) {
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
            if (output.size() != 4) { ++batch.rejected.invalidGeometry; continue; }
            std::array<MarkerCorner, 4> corners;
            for (std::size_t i = 0; i < 4; ++i) corners[i] = {output[i].x, output[i].y};
            auto result = metricObservation(observation, corners, profile);
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
