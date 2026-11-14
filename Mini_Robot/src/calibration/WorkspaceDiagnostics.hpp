#pragma once
#include "calibration/WorkspaceAcquisition.hpp"
#include <array>

namespace micro_robot
{
inline constexpr std::size_t DefaultWorkspaceMapColumns = 41, DefaultWorkspaceMapRows = 25;
inline constexpr std::size_t MinimumWorkspaceMapDimension = 2, MaximumWorkspaceMapDimension = 65;
inline constexpr std::size_t MinimumWorkspaceSurveyChecks = 6;
inline constexpr double WorkspaceMapHullTolerance = 1e-12; // Normalized cross product.
inline constexpr double MinimumWorkspaceSurveyAreaFraction = 1e-4;
inline constexpr double MaximumWorkspaceRegressionCondition = 1e8;
inline constexpr double MinimumWorkspaceRelativeDenominatorMeters = 1e-9;
inline constexpr double WorkspaceMapCentimetersPerMeter = 100;
inline constexpr double WorkspaceObservedAxisToleranceMeters = 0.005;
inline constexpr double WorkspacePointCoverage = 0.95;
inline constexpr double WorkspaceEndpointCoverage = 0.975; // Two endpoints: union bound >=95%.
inline constexpr double MaximumWorkspaceSurveyStandardUncertaintyMeters = 1000;
inline constexpr std::size_t MaximumWorkspaceSurveyMethodBytes = 2048;
inline constexpr int WorkspaceBetaIterations = 200, WorkspaceQuantileIterations = 64;
inline constexpr double WorkspaceBetaTolerance = 1e-13, WorkspaceBetaTiny = 1e-300;
inline constexpr double MaximumWorkspaceStudentQuantile = 1e6;
inline constexpr std::size_t MaximumWorkspaceDiagnosticsExportBytes = 16 * 1024 * 1024;

struct WorkspaceSurveyEvidence {
    std::vector<std::uint64_t> referenceIds;
    bool independentSurveyConfirmed{};
    bool gaussianAffineModelConfirmed{};
    double additionalStandardUncertaintyMeters{}; // Positive, per-axis survey/model floor; never inferred.
    std::string method; // Survey, uncertainty budget and model-assumption justification.
};
struct WorkspaceErrorObservation {
    std::uint64_t referenceId{};
    WorkspaceCorrespondence point;
    bool check{}, surveyed{};
    WorldPoint2D mappedWorld; // H(pixel); prediction uses these observed coordinates as fixed predictors.
    double errorXMeters{}, errorYMeters{}, absoluteMeters{};
    std::optional<double> relativePercent; // 100*absolute/workspace diagonal; origin invariant.
};
struct WorkspacePredictionModel {
    std::array<double, 9> inverseNormal{};
    std::array<double, 3> betaX{}, betaY{};
    double varianceX{}, varianceY{}; // Residual SSE/(n-3) plus declared positive floor squared.
    std::size_t degreesOfFreedom{};
    double pointCritical{}, endpointCritical{}; // Axis-wise two-sided Bonferroni Student-t.
};
struct WorkspaceMapSample {
    WorldPoint2D world;
    bool geometrySupported{};
    bool fitCoverageKnown{};
    std::optional<double> xPixelsPerCentimeter, yPixelsPerCentimeter;
    std::optional<double> interpolatedAbsoluteMeters, interpolatedRelativePercent;
    std::optional<double>
        predictionRadiusMeters; // Pointwise conditional >=95%, NOT simultaneous map coverage.
};
struct WorkspaceDiagnostics {
    WorkspaceCalibrationProfile profile;
    std::shared_ptr<const WorkspaceAcquisitionRecord> acquisition;
    WorkspaceSurveyEvidence evidence;
    std::size_t columns{}, rows{};
    // Normalized world coordinates (x,y), image hull uses normalized native pixels.
    std::vector<WorkspaceViewPoint> fitHull, imageFitHull, surveyHull, predictionHull;
    std::vector<WorkspaceErrorObservation> observations;
    std::optional<WorkspacePredictionModel> prediction;
    std::string uncertaintyUnavailable;
    std::vector<WorkspaceMapSample> samples; // Row-major, world +Y up (first row at minimum Y).
    std::string diagnosticPlane{"tabletop z=0"}; // Never axle/body uncertainty without propagation and new evidence.
};
struct WorkspaceDiagnosticsOutcome {
    std::shared_ptr<const WorkspaceDiagnostics> result;
    std::string error;
};
// Worker-only bounded preparation/validation/sampling. No Qt, OpenCV or hardware.
[[nodiscard]] WorkspaceDiagnosticsOutcome buildWorkspaceDiagnostics(
    WorkspaceCalibrationProfile profile, std::shared_ptr<const WorkspaceAcquisitionRecord> acquisition = {},
    WorkspaceSurveyEvidence evidence = {}, std::size_t columns = DefaultWorkspaceMapColumns,
    std::size_t rows = DefaultWorkspaceMapRows);
// Reusable geometric support/scale and point prediction contract for #250.
[[nodiscard]] WorkspaceMapSample sampleWorkspaceDiagnostics(const WorkspaceDiagnostics&, WorldPoint2D);
// A circle around the unchanged H coordinate containing the axis-wise prediction rectangle.
// Only the two declared coverage levels are supported; no endpoint independence is assumed.
[[nodiscard]] std::optional<double> workspacePredictionRadius(const WorkspaceDiagnostics&, WorldPoint2D,
                                                              double coverage = WorkspacePointCoverage);
[[nodiscard]] std::optional<double> workspaceRelativeErrorPercent(double errorMeters, const WorkspaceBounds&);
// Equal-tailed Student-t quantile used by the prediction model; null on failed convergence/invalid input.
[[nodiscard]] std::optional<double> workspaceStudentCritical(double cumulativeProbability,
                                                             std::size_t degreesOfFreedom);
} // namespace micro_robot
