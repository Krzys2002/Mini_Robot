#pragma once

#include "camera/CameraTypes.hpp"

#include <optional>
#include <string>
#include <vector>

namespace micro_robot {

// Target coordinates: top-left inner corner is (0,0,0), x along columns,
// y along rows on the board, all in metres. Image corners use pixels.
struct CalibrationTarget {
    int columns{9};
    int rows{6};
    double squareSizeMeters{0.025};
};
struct ImageCorner { double u{}; double v{}; };
struct TargetPoint { double x{}; double y{}; double z{}; };
[[nodiscard]] std::vector<TargetPoint> calibrationObjectPoints(const CalibrationTarget& target);

struct CalibrationDetection {
    std::vector<ImageCorner> corners; // Complete row-major inner-corner grid.
    double sharpness{}; // Variance of grayscale Laplacian inside target bounds.
    std::string error;
};
class ICalibrationDetector {
public:
    virtual ~ICalibrationDetector() = default;
    // Synchronous on an acquisition/vision worker, never on the GUI thread.
    // No hardware/widget dependency; frame storage remains owned by the caller.
    [[nodiscard]] virtual CalibrationDetection detect(const Frame& frame,
                                                      const CalibrationTarget& target) = 0;
};
struct CalibrationCaptureConfig {
    CalibrationTarget target;
    std::size_t minimumViews{10};
    std::size_t maximumViews{24};
    std::size_t maximumStoredBytes{128 * 1024 * 1024};
    double minimumSharpness{40.0};
    double minimumAreaFraction{0.02};
    double minimumCornerMarginPixels{4.0};
    double duplicateDistance{0.025}; // RMS corner displacement / image diagonal.
    std::size_t minimumLocationCells{3}; // Occupied board-centre cells in a 3x3 grid.
    double minimumScaleSpread{0.08}; // Difference in sqrt(inner-board area / image area).
    double minimumAngleSpreadRadians{0.20}; // Board edge rotation, modulo pi.
};
enum class CalibrationCaptureState { Idle, Collecting, Complete, Cancelled };
struct CalibrationView {
    FramePtr frame;
    CalibrationDetection detection;
};
struct CalibrationProgress {
    std::size_t count{};
    std::size_t locationCells{};
    double scaleSpread{};
    double angleSpreadRadians{};
    bool ready{};
};
struct CaptureDecision {
    bool accepted{};
    std::string message;
};

// Single-thread-affine pure capture state machine. Restart/cancel clear all
// observations; completed observations remain available for the intrinsic solver.
// A session belongs to exactly one camera identity and resolution. Source loss
// is handled by the application adapter; mixed resolutions are rejected here.
class CalibrationCapture final {
public:
    // Throws std::invalid_argument for an invalid target or capture configuration.
    explicit CalibrationCapture(CalibrationCaptureConfig config = {});
    void restart(std::string sourceId);
    void cancel();
    [[nodiscard]] CaptureDecision add(FramePtr frame, CalibrationDetection detection);
    [[nodiscard]] bool remove(std::size_t index);
    [[nodiscard]] bool finish();
    [[nodiscard]] CalibrationProgress progress() const;
    [[nodiscard]] CalibrationCaptureState state() const noexcept { return state_; }
    [[nodiscard]] const CalibrationCaptureConfig& config() const noexcept { return config_; }
    [[nodiscard]] const std::vector<CalibrationView>& views() const noexcept { return views_; }
    [[nodiscard]] const std::string& sourceId() const noexcept { return sourceId_; }
private:
    CalibrationCaptureConfig config_;
    CalibrationCaptureState state_{CalibrationCaptureState::Idle};
    std::string sourceId_;
    std::vector<CalibrationView> views_;
    std::size_t storedBytes_{};
};

} // namespace micro_robot
