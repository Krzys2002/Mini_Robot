#pragma once

#include "camera/CameraTypes.hpp"

#include <array>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace micro_robot {

inline constexpr int CalibrationLocationGridSide = 3;
inline constexpr std::size_t CalibrationLocationCellCount =
    CalibrationLocationGridSide * CalibrationLocationGridSide;
inline constexpr std::uint32_t CalibrationMaximumImageDimension = 4096;
inline constexpr std::uint64_t CalibrationMaximumImagePixels = 4'000'000;
inline constexpr std::size_t CalibrationMaximumViews = 24;
inline constexpr std::size_t CalibrationDefaultStoredPixelBudget = 128 * 1024 * 1024;
inline constexpr std::size_t CalibrationBytesPerMiB = 1024 * 1024;
// Integer luma approximation for review only; detection uses original pixels.
inline constexpr unsigned CalibrationReviewRedWeight = 77;
inline constexpr unsigned CalibrationReviewGreenWeight = 150;
inline constexpr unsigned CalibrationReviewBlueWeight = 29;
inline constexpr unsigned CalibrationReviewWeightSum = 256;

[[nodiscard]] inline bool supportedCalibrationResolution(std::uint32_t width, std::uint32_t height) noexcept
{
    return width > 0 && height > 0 && width <= CalibrationMaximumImageDimension
        && height <= CalibrationMaximumImageDimension
        && static_cast<std::uint64_t>(width) * height <= CalibrationMaximumImagePixels;
}
// Keeps sequence/time, full dimensions and pixel-centre coordinates. Removes
// colour, padding and unused vector capacity. Throws on invalid input/allocation
// failure. Production callers perform bulk conversion on their capture worker.
[[nodiscard]] FramePtr prepareCalibrationReviewFrame(FramePtr frame);
using CalibrationReviewFrameFactory = std::function<FramePtr(FramePtr)>;

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
    std::size_t maximumViews{CalibrationMaximumViews};
    std::size_t maximumStoredBytes{CalibrationDefaultStoredPixelBudget};
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
    FramePtr frame; // Full-resolution, tightly packed Gray8 review; no raw colour retention.
    CalibrationDetection detection;
};
struct CalibrationProgress {
    std::size_t count{};
    std::size_t locationCells{};
    double scaleSpread{};
    double angleSpreadRadians{};
    bool ready{};
    // Row-major image-space board-centre coverage; not corner/tilt coverage.
    std::array<bool, CalibrationLocationCellCount> occupiedLocationCells{};
};
enum class CalibrationSuggestion {
    Start, FirstView, ChangeLocation, ChangeScale, RotateInPlane,
    MoreViews, Ready, ReviewResult, ReplaceView
};
struct CalibrationGuidance {
    CalibrationSuggestion suggestion{CalibrationSuggestion::Start};
    // Optional row-major image region to place the board centre. Keep the entire
    // target visible; this is a suggestion, never an acceptance/safety guarantee.
    std::optional<std::size_t> nextLocationCell;
};
struct CaptureDecision {
    bool accepted{};
    std::string message;
};

// Single-thread-affine pure capture state machine. Restart/cancel clear all
// observations; completed observations remain available for the intrinsic solver.
// Removing a completed view reopens collection without discarding the other views.
// A session belongs to exactly one camera identity and resolution. Source loss
// is handled by the application adapter; mixed resolutions are rejected here.
class CalibrationCapture final {
public:
    // Throws std::invalid_argument for an invalid target or capture configuration.
    explicit CalibrationCapture(CalibrationCaptureConfig config = {});
    void restart(std::string sourceId);
    void cancel();
    // Read-only quality/diversity/budget preflight; never retains or copies pixels.
    [[nodiscard]] CaptureDecision assess(FramePtr frame, const CalibrationDetection& detection) const;
    // Strong storage guarantee: rejected/failed preparation leaves captures/bytes
    // unchanged. Injectable preparation also permits deterministic failure tests.
    [[nodiscard]] CaptureDecision add(FramePtr frame, CalibrationDetection detection,
        const CalibrationReviewFrameFactory& prepare = prepareCalibrationReviewFrame);
    [[nodiscard]] bool remove(std::size_t index);
    [[nodiscard]] bool finish();
    [[nodiscard]] CalibrationProgress progress() const;
    // Uses accepted-view measurements only. Out-of-plane tilt is not measured.
    [[nodiscard]] CalibrationGuidance guidance() const;
    [[nodiscard]] CalibrationCaptureState state() const noexcept { return state_; }
    [[nodiscard]] const CalibrationCaptureConfig& config() const noexcept { return config_; }
    [[nodiscard]] const std::vector<CalibrationView>& views() const noexcept { return views_; }
    [[nodiscard]] const std::string& sourceId() const noexcept { return sourceId_; }
    [[nodiscard]] std::size_t storedPixelBytes() const noexcept { return storedBytes_; }
    [[nodiscard]] std::size_t remainingPixelBytes() const noexcept { return config_.maximumStoredBytes - storedBytes_; }
private:
    CalibrationCaptureConfig config_;
    CalibrationCaptureState state_{CalibrationCaptureState::Idle};
    std::string sourceId_;
    std::vector<CalibrationView> views_;
    std::size_t storedBytes_{};
};

} // namespace micro_robot
