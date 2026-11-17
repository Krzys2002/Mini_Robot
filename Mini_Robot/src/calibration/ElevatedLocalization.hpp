#pragma once
#include "calibration/IntrinsicCalibration.hpp"
#include "common/RobotId.hpp"
#include "world/Workspace.hpp"
#include <array>
#include <string_view>

namespace micro_robot {
struct WorkspaceCalibrationProfile;
inline constexpr std::size_t MaximumExtrinsicReferences = 256;
inline constexpr std::size_t MaximumRobotMounts = 64;
inline constexpr std::size_t MaximumGeometryNoteBytes = 512;
inline constexpr double RotationTolerance = 1e-8;
inline constexpr double RayTolerance = 1e-10;
inline constexpr double MinimumHeadingLengthMeters = 1e-12;
inline constexpr double DefaultExtrinsicPixelLimit = 2.0;
inline constexpr double DefaultExtrinsicAxisLimitMeters = 0.005;
inline constexpr double MinimumPoseSeparationPixels = 0.25;
inline constexpr double MaximumSurveyEntryMeters = 100;
inline constexpr double MaximumSurveyPixelLimit = 100;
// Pixels are already undistorted native-K pixels. All references share the
// explicitly measured parallel surface z, including board/backing thickness.
struct ExtrinsicReference {
    ImageCorner image;
    WorldPoint2D world;
    bool check{}; // Held out of estimation; operator must survey independently.
};
struct RobotMount {
    RobotId robotId{0};
    double tagHeightMeters{};
    double bodyForwardMeters{}, bodyLeftMeters{}; // Tag centre -> axle/body in tag-heading axes.
    double bodyHeadingOffsetRadians{};
    std::string revision, measurementMethod;
};
struct ExtrinsicSurvey {
    double surfaceHeightMeters{}, boardThicknessMeters{}, backingThicknessMeters{};
    std::string measurementMethod;
    std::vector<ExtrinsicReference> references;
    double maximumReprojectionPixels{DefaultExtrinsicPixelLimit};
    double maximumCheckAxisMeters{DefaultExtrinsicAxisLimitMeters};
};
struct ElevatedLocalization {
    std::string revision;
    std::string tabletopBinding; // Exact canonical legacy H/K/frame/sidecar binding; never relabel evidence.
    ExtrinsicSurvey survey;
    // X_camera = R_world_to_camera * X_world + t, OpenCV camera +X right,
    // +Y down, +Z forward. World +Z is above tabletop, +heading CCW from +X.
    std::array<double,9> worldToCamera{};
    std::array<double,3> translationMeters{};
    // Absent only when every alternate planar solution failed cheirality/above-table checks.
    std::optional<double> alternativeGapPixels;
    std::vector<RobotMount> mounts; // Independent of marker identity mapping.
};
struct ExtrinsicSolveInput {
    ExtrinsicSurvey survey;
    std::vector<RobotMount> mounts;
    std::string revision;
};
struct ExtrinsicSolveOutcome {
    std::optional<ElevatedLocalization> calibration;
    std::string error;
};
class IExtrinsicsSolver {
public:
    virtual ~IExtrinsicsSolver() = default;
    // Worker only. Estimate from measured references, never from the runtime H.
    virtual ExtrinsicSolveOutcome solve(const WorkspaceCalibrationProfile&, const ExtrinsicSolveInput&) = 0;
};
[[nodiscard]] std::string tabletopProfileBinding(const WorkspaceCalibrationProfile&);
[[nodiscard]] std::string validateExtrinsicSurvey(const ExtrinsicSurvey&, const IntrinsicCalibration&);
[[nodiscard]] std::string validateRobotMounts(const std::vector<RobotMount>&);
[[nodiscard]] std::string validateElevatedLocalization(const ElevatedLocalization&, const WorkspaceCalibrationProfile&);
[[nodiscard]] std::optional<std::array<double,3>> cameraWorldPosition(const ElevatedLocalization&);
[[nodiscard]] std::optional<WorldPoint2D> intersectTagPlane(const ElevatedLocalization&, const IntrinsicCalibration&, ImageCorner, double heightMeters);
[[nodiscard]] std::optional<ImageCorner> projectReference(const ElevatedLocalization&, const IntrinsicCalibration&, WorldPoint2D, double heightMeters);
[[nodiscard]] std::string serializeElevatedLocalization(const ElevatedLocalization&);
// Strict bounded parser; caller validates against its enclosing workspace before use.
[[nodiscard]] std::optional<ElevatedLocalization> parseElevatedLocalization(std::string_view);
} // namespace micro_robot
