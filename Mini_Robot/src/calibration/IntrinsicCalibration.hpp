#pragma once

#include "calibration/CalibrationCapture.hpp"

#include <array>
#include <optional>

namespace micro_robot {

struct IntrinsicObservation {
    std::uint64_t frameSequence{};
    std::vector<ImageCorner> corners;
};
// Owned corner-only snapshot: no image storage, widgets, hardware or OpenCV types.
struct IntrinsicCalibrationInput {
    std::string sourceId;
    std::uint32_t width{};
    std::uint32_t height{};
    CalibrationTarget target;
    std::vector<IntrinsicObservation> observations;
};
struct ReprojectionError {
    std::uint64_t frameSequence{};
    std::size_t pointCount{};
    double rmsPixels{}; // sqrt(sum(||observed - projected||^2) / pointCount).
};
struct IntrinsicCalibration {
    std::string sourceId;
    std::uint32_t width{};
    std::uint32_t height{};
    // Row-major K = [fx,0,cx; 0,fy,cy; 0,0,1], in pixels.
    std::array<double, 9> cameraMatrix{};
    // Supported model: pinhole Brown-Conrady, exactly [k1,k2,p1,p2,k3].
    std::vector<double> distortion;
    double rmsPixels{}; // Point-weighted aggregate RMS, not mean per-view RMS.
    std::vector<ReprojectionError> perViewErrors;
};
struct IntrinsicCalibrationConfig {
    double warningRmsPixels{1.0}; // Advisory; a high finite RMS is not a model-format failure.
    double minimumFocalImageRatio{0.1}; // fx/width and fy/height.
    double maximumFocalImageRatio{10.0};
    double maximumFocalAspectRatio{2.0}; // max(fx/fy, fy/fx).
    double maximumAbsoluteDistortion{10.0};
};
// Validation returns an actionable error, or an empty string. No Qt/OpenCV needed.
// Principal point must lie in the image; homogeneous K entries must be canonical.
[[nodiscard]] std::string validateIntrinsicConfig(const IntrinsicCalibrationConfig& config);
[[nodiscard]] std::string validateIntrinsicCalibration(const IntrinsicCalibration& result,
                                                       const IntrinsicCalibrationConfig& config = {});
[[nodiscard]] bool hasReprojectionWarning(const IntrinsicCalibration& result,
                                          const IntrinsicCalibrationConfig& config = {});
// Throws std::invalid_argument unless capture is complete. Copies corners only.
[[nodiscard]] IntrinsicCalibrationInput intrinsicCalibrationInput(const CalibrationCapture& capture);

struct IntrinsicCalibrationOutcome {
    std::optional<IntrinsicCalibration> result;
    std::string error;
};
class IIntrinsicCalibrationSolver {
public:
    virtual ~IIntrinsicCalibrationSolver() = default;
    // Synchronous, worker-thread use only. Failures carry no consumable result.
    [[nodiscard]] virtual IntrinsicCalibrationOutcome solve(const IntrinsicCalibrationInput& input,
        const IntrinsicCalibrationConfig& config) = 0;
};
} // namespace micro_robot
