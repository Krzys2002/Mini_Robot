#pragma once
#include "calibration/ElevatedLocalization.hpp"
#include "vision/RobotObservation.hpp"
#include <memory>

namespace micro_robot {
inline constexpr std::size_t MaximumReferenceMarkers = 32;
inline constexpr std::size_t MinimumReferenceSupportMarkers = 3;
inline constexpr std::size_t MinimumReferenceDriftSamples = 2;
inline constexpr double MinimumReferenceEdgePixels = 4;
inline constexpr double MinimumReferenceImageSpan = .05;
inline constexpr double MinimumReferenceImageTriangleArea = .0025;
inline constexpr double ReferenceSquareToleranceMeters = 1e-6;
inline constexpr double ReferenceSizeRelativeTolerance = .02;
inline constexpr double ReferenceGlobalFraction = .75;
inline constexpr auto MinimumReferenceCadence = std::chrono::milliseconds{20};
inline constexpr auto MaximumReferenceCadence = std::chrono::seconds{10};
inline constexpr auto ReferenceWatchdogPeriod = std::chrono::milliseconds{100};
enum class ReferenceRole { Fit, IndependentCheck, DriftMonitor };
struct SurveyedReferenceMarker {
    MarkerFamily family{MarkerFamily::AprilTag36h11};
    int id{};
    // Absolute measured decoded TL,TR,BR,BL corners in world X/Y; all at surfaceHeightMeters.
    std::array<WorldPoint2D,4> corners;
    double surfaceHeightMeters{}, blackEdgeMeters{}, standardUncertaintyMeters{};
    ReferenceRole role{ReferenceRole::Fit};
    std::string installationRevision, surveyRevision, method;
    bool independentlySurveyed{}; // Required declaration for held-out Checks, not a physical certification.
};
struct ReferencePolicy {
    std::chrono::milliseconds cadence{200};
    std::chrono::milliseconds maximumEvidenceAge{750};
    std::chrono::milliseconds minimumDriftDuration{400};
    std::size_t consecutiveDriftSamples{3};
    double maximumResidualPixels{2}, maximumResidualAxisMeters{.005};
};
enum class ReferenceRejection { Included, Missing, Duplicate, InvalidCorners, UnsupportedRay };
struct ReferenceObservation {
    int id{};
    ReferenceRejection rejection{ReferenceRejection::Missing};
    std::array<MarkerCorner,4> raw{}, undistorted{};
};
struct ReferenceFrame {
    std::string cameraBinding, worldFrameId, registryRevision, inputCalibrationRevision;
    std::uint64_t frameSequence{}, inputGeneration{};
    Timestamp capturedAt{};
    std::vector<ReferenceObservation> observations; // Exactly one bounded row per registry marker, including missing rows.
    std::string error;
};
using ReferenceFramePtr=std::shared_ptr<const ReferenceFrame>;
struct ReferenceConfiguration {
    std::string revision, tabletopBinding, sharedErrorNotes;
    std::vector<SurveyedReferenceMarker> markers;
    ReferencePolicy policy;
    std::optional<ReferenceFrame> acquisition; // Immutable commissioning snapshot; monitoring never overwrites it.
    std::string validatedLocalizationRevision; // Set only by successful explicit reference solve, never by monitoring.
};
[[nodiscard]] std::string referenceCameraBinding(const IntrinsicCalibration&);
[[nodiscard]] std::string validateReferenceConfiguration(const ReferenceConfiguration&, const WorkspaceCalibrationProfile&,
                                                        const RobotMarkerProfile& allocation = {});
[[nodiscard]] std::string validateReferenceFrame(const ReferenceFrame&, const ReferenceConfiguration&,
                                                const WorkspaceCalibrationProfile&);
[[nodiscard]] bool referenceSupportCoversWorkspace(const std::vector<WorldPoint2D>& centres,const WorkspaceBounds&);
[[nodiscard]] std::optional<ExtrinsicSolveInput> referenceExtrinsicInput(const ReferenceFrame&,const ReferenceConfiguration&,
    const WorkspaceCalibrationProfile&, std::string revision, std::string& error);
[[nodiscard]] std::string serializeReferenceConfiguration(const ReferenceConfiguration&);
[[nodiscard]] std::optional<ReferenceConfiguration> parseReferenceConfiguration(std::string_view);

enum class PlacementConsistency { Unvalidated, Consistent, IndividualOutlier, SuspectedDrift, ConfirmedDrift, InsufficientEvidence };
struct PlacementReport {
    PlacementConsistency state{PlacementConsistency::Unvalidated};
    Timestamp assessedAt{};
    std::uint64_t frameSequence{};
    std::string registryRevision, calibrationRevision, message;
    std::size_t visible{}, outliers{};
    bool permitsMeasurements{};
};
// Serialized worker-only state. No camera access, unbounded history or implicit clock reads.
class ReferenceMonitor final {
public:
    void configure(const ReferenceConfiguration&,std::string calibrationRevision,std::uint64_t inputGeneration);
    [[nodiscard]] bool due(Timestamp now) const;
    PlacementReport observe(const ReferenceFrame&,const ReferenceConfiguration&,const WorkspaceCalibrationProfile&);
    [[nodiscard]] PlacementReport status(Timestamp now,const ReferencePolicy&) const;
private:
    std::string configurationKey_, calibrationRevision_;
    std::uint64_t generation_{};
    std::chrono::milliseconds cadence_{200};
    std::optional<Timestamp> lastAssessment_, driftStarted_;
    std::size_t driftSamples_{};
    bool driftLatched_{};
    PlacementReport report_;
};
} // namespace micro_robot
