#pragma once

#include "calibration/TabletopHomography.hpp"
#include "common/Pose2D.hpp"
#include "vision/RobotObservation.hpp"

namespace micro_robot {
// Undistorted native-K corners, ordered as the decoded marker TL, TR, BR, BL.
// The caller must undistort corners before computing their projective center.
enum class MetricPoseReference { TabletopTagCenter, AxleBodyCenter };
struct MetricPoseMeasurement {
    RobotId robotId;
    Pose2D pose;
    Timestamp capturedAt;
    std::uint64_t frameSequence;
    double minimumEdgePixels;
    std::optional<double> decisionMargin;
    MetricPoseReference reference{MetricPoseReference::TabletopTagCenter};
    std::string cameraId{}, worldFrameId{}, calibrationRevision{}, mountingRevision{};
};

enum class MetricObservationStatus { Valid, Uncalibrated, InvalidCalibration, InvalidGeometry, OutsideWorkspace, UnknownMount, UnsupportedRay, PlacementUnverified, PlacementDrift };
struct MetricObservationResult {
    MetricObservationStatus status{MetricObservationStatus::Uncalibrated};
    std::optional<MetricPoseMeasurement> measurement;
};

// Pure boundary for already-undistorted geometry; no raw pixel pose leaves it.
// Returns a measurement, never tracked state. Center and all corners must lie
// inside the workspace; a horizon crossing or collapsed heading is rejected.
// V4 additionally checks native image support and publishes the measured body
// transform with revision metadata. Legacy measurements remain tabletop-tag-only.
[[nodiscard]] MetricObservationResult metricObservation(
    const RobotObservation& observation, const std::array<MarkerCorner, 4>& undistortedCorners,
    const std::optional<WorkspaceCalibrationProfile>& calibration);
// Fixed-size per-frame counters; no marker identities or unbounded diagnostics retained.
struct MetricRejections {
    std::size_t unsupportedFamily{}, unassignedId{}, duplicateRobot{}, invalidGeometry{}, outsideWorkspace{}, unknownMount{}, unsupportedRay{};
};
struct MetricObservationBatch {
    MetricObservationStatus status{MetricObservationStatus::Uncalibrated};
    std::vector<MetricPoseMeasurement> measurements;
    std::string error;
    MetricRejections rejected{};
    std::optional<PlacementReport> placement{};
    ReferenceFramePtr references{};
};
using MetricObservationBatchPtr = std::shared_ptr<const MetricObservationBatch>;
class IMetricObservationConverter {
public:
    virtual ~IMetricObservationConverter() = default;
    virtual ReferenceFramePtr referenceObservations(const MarkerDetections&, const std::string&, const RobotMarkerMap&, const WorkspaceCalibrationProfile&) { return {}; }
    virtual MetricObservationBatch convert(const MarkerDetections&, const std::string& sourceId,
        const RobotMarkerMap&, const std::optional<WorkspaceCalibrationProfile>&) = 0;
};
} // namespace micro_robot
