#pragma once

#include "calibration/TabletopHomography.hpp"
#include "common/Pose2D.hpp"
#include "vision/RobotObservation.hpp"
#include "vision/LocalizationMetrics.hpp"

namespace micro_robot {
// Undistorted native-K corners, ordered as the decoded marker TL, TR, BR, BL.
// The caller must undistort corners before computing their projective center.
struct MetricPoseMeasurement {
    RobotId robotId;
    Pose2D pose;
    Timestamp capturedAt;
    std::uint64_t frameSequence;
    double minimumEdgePixels;
    std::optional<double> decisionMargin;
    MeasurementProvenance provenance{};
};

enum class MetricObservationStatus { Valid, Uncalibrated, InvalidCalibration, InvalidGeometry, OutsideWorkspace };
struct MetricObservationResult {
    MetricObservationStatus status{MetricObservationStatus::Uncalibrated};
    std::optional<MetricPoseMeasurement> measurement;
};

// Pure boundary for already-undistorted geometry; no raw pixel pose leaves it.
// Returns a measurement, never tracked state. Center and all corners must lie
// inside the workspace; a horizon crossing or collapsed heading is rejected.
[[nodiscard]] MetricObservationResult metricObservation(
    const RobotObservation& observation, const std::array<MarkerCorner, 4>& undistortedCorners,
    const std::optional<WorkspaceCalibrationProfile>& calibration);
// Fixed-size per-frame counters; no marker identities or unbounded diagnostics retained.
struct MetricRejections {
    std::size_t unsupportedFamily{}, unassignedId{}, duplicateRobot{}, invalidGeometry{}, outsideWorkspace{};
};
enum class MeasurementRejection { None, UnsupportedFamily, UnassignedId, DuplicateRobot, InvalidGeometry, OutsideWorkspace, TooSmall, MarginUnavailable, LowMargin, TooOld, InvalidClock };
[[nodiscard]] const char* measurementRejectionName(MeasurementRejection);
struct MeasurementDiagnostic {
    int markerId{};
    std::optional<RobotId> robotId;
    MeasurementRejection reason{};
    std::optional<double> minimumEdgePixels;
};
// Disabled gates use zero/no optional limit. Values are native image pixels and
// detector-native margin units, never calibrated probability or covariance.
struct LocalizationQualityGates {
    double minimumEdgePixels{};
    std::optional<double> minimumDecisionMargin;
    std::optional<Duration> maximumObservationAge;
};
void validateQualityGates(const LocalizationQualityGates&);
void applyQualityGates(struct MetricObservationBatch&, const LocalizationQualityGates&, Timestamp now);
struct MetricObservationBatch {
    MetricObservationStatus status{MetricObservationStatus::Uncalibrated};
    std::vector<MetricPoseMeasurement> measurements;
    std::string error;
    MetricRejections rejected{};
    MeasurementProvenance provenance{};
    PipelineTiming timing{};
    std::vector<MeasurementDiagnostic> diagnostics{};
    std::vector<RobotId> missingRobots{}; // Absent detection, never a measurement.
};
using MetricObservationBatchPtr = std::shared_ptr<const MetricObservationBatch>;
class IMetricObservationConverter {
public:
    virtual ~IMetricObservationConverter() = default;
    virtual MetricObservationBatch convert(const MarkerDetections&, const std::string& sourceId,
        const RobotMarkerMap&, const std::optional<WorkspaceCalibrationProfile>&) = 0;
};
} // namespace micro_robot
