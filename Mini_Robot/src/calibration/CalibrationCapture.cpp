#include "calibration/CalibrationCapture.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <iterator>
#include <stdexcept>
#include <utility>

namespace micro_robot {
namespace {
void validateTarget(const CalibrationTarget& target)
{
    if (target.columns < 3 || target.columns > 30 || target.rows < 3 || target.rows > 30
        || target.columns == target.rows || !std::isfinite(target.squareSizeMeters)
        || target.squareSizeMeters <= 0) {
        throw std::invalid_argument("Use a non-square 3..30 inner-corner grid and positive square size");
    }
}
double boardArea(const std::vector<ImageCorner>& points, int columns)
{
    const std::array<ImageCorner, 4> quad{points.front(), points[columns - 1],
                                       points.back(), points[points.size() - columns]};
    double area = 0;
    for (std::size_t i = 0; i < quad.size(); ++i) {
        const auto& a = quad[i];
        const auto& b = quad[(i + 1) % quad.size()];
        area += a.u * b.v - b.u * a.v;
    }
    return std::abs(area) * 0.5;
}
double angle(const CalibrationView& view, int columns)
{
    const auto& a = view.detection.corners.front();
    const auto& b = view.detection.corners[columns - 1];
    return std::atan2(b.v - a.v, b.u - a.u);
}
} // namespace

std::vector<TargetPoint> calibrationObjectPoints(const CalibrationTarget& target)
{
    validateTarget(target);
    std::vector<TargetPoint> result;
    for (int row = 0; row < target.rows; ++row) {
        for (int column = 0; column < target.columns; ++column) {
            result.push_back({column * target.squareSizeMeters, row * target.squareSizeMeters, 0});
        }
    }
    return result;
}

CalibrationCapture::CalibrationCapture(CalibrationCaptureConfig config) : config_(config)
{
    validateTarget(config.target);
    const double values[]{config.minimumSharpness, config.minimumAreaFraction,
        config.minimumCornerMarginPixels, config.duplicateDistance,
        config.minimumScaleSpread, config.minimumAngleSpreadRadians};
    if (std::any_of(std::begin(values), std::end(values), [](double v) {
            return !std::isfinite(v) || v < 0;
        }) || config.minimumViews < 3 || config.maximumViews < config.minimumViews
        || config.maximumViews > 100 || config.maximumStoredBytes == 0
        || config.minimumLocationCells < 1 || config.minimumLocationCells > CalibrationLocationCellCount
        || config.minimumAreaFraction >= 1 || config.duplicateDistance <= 0
        || config.minimumScaleSpread > 1
        || config.minimumAngleSpreadRadians > std::numbers::pi / 2) {
        throw std::invalid_argument("Invalid calibration capture thresholds");
    }
}

void CalibrationCapture::restart(std::string sourceId)
{
    views_.clear();
    storedBytes_ = 0;
    sourceId_ = std::move(sourceId);
    state_ = sourceId_.empty() ? CalibrationCaptureState::Idle : CalibrationCaptureState::Collecting;
}
void CalibrationCapture::cancel()
{
    views_.clear();
    storedBytes_ = 0;
    sourceId_.clear();
    state_ = CalibrationCaptureState::Cancelled;
}
CaptureDecision CalibrationCapture::add(FramePtr frame, CalibrationDetection detection)
{
    if (state_ != CalibrationCaptureState::Collecting) return {false, "Start or restart capture first."};
    if (!frame || !isValidFrame(*frame)) return {false, "Invalid frame."};
    if (!detection.error.empty()) return {false, std::move(detection.error)};
    if (detection.corners.size() != static_cast<std::size_t>(config_.target.columns * config_.target.rows)) {
        return {false, "Full chessboard not found. Show all inner corners with a white border."};
    }
    if (!views_.empty() && (frame->width != views_.front().frame->width
                           || frame->height != views_.front().frame->height)) {
        return {false, "Resolution changed. Restart capture at a fixed camera resolution."};
    }
    const double margin = config_.minimumCornerMarginPixels;
    for (const auto& point : detection.corners) {
        if (!std::isfinite(point.u) || !std::isfinite(point.v) || point.u < margin || point.v < margin
            || point.u >= frame->width - margin || point.v >= frame->height - margin) {
            return {false, "Board is clipped or too close to the image edge."};
        }
    }
    if (!std::isfinite(detection.sharpness) || detection.sharpness < config_.minimumSharpness) {
        return {false, "Board is blurry or low contrast. Improve focus/lighting and hold still."};
    }
    if (boardArea(detection.corners, config_.target.columns)
        / (static_cast<double>(frame->width) * frame->height) < config_.minimumAreaFraction) {
        return {false, "Board is too small. Move closer to the camera."};
    }
    const double diagonal = std::hypot(frame->width, frame->height);
    for (const auto& view : views_) {
        if (frame->sequence == view.frame->sequence || frame->capturedAt <= view.frame->capturedAt) {
            return {false, "Frame is repeated or older than an accepted capture. Wait for a fresh frame."};
        }
        double direct = 0;
        double reversed = 0;
        const auto n = detection.corners.size();
        for (std::size_t i = 0; i < n; ++i) {
            const auto& a = detection.corners[i];
            const auto& b = view.detection.corners[i];
            const auto& c = view.detection.corners[n - 1 - i];
            direct += (a.u - b.u) * (a.u - b.u) + (a.v - b.v) * (a.v - b.v);
            reversed += (a.u - c.u) * (a.u - c.u) + (a.v - c.v) * (a.v - c.v);
        }
        if (std::sqrt(std::min(direct, reversed) / static_cast<double>(n)) / diagonal
            < config_.duplicateDistance) {
            return {false, "Redundant view. Move, tilt, rotate the board or change its distance."};
        }
    }
    if (views_.size() >= config_.maximumViews) return {false, "Capture limit reached. Remove a view to recapture."};
    if (frame->pixels->size() > config_.maximumStoredBytes - storedBytes_) {
        return {false, "Capture memory limit reached. Remove views or lower the camera resolution and restart."};
    }
    storedBytes_ += frame->pixels->size();
    views_.push_back({std::move(frame), std::move(detection)});
    return {true, "View accepted. Review its corners, then vary position, distance and rotation."};
}
bool CalibrationCapture::remove(std::size_t index)
{
    if ((state_ != CalibrationCaptureState::Collecting && state_ != CalibrationCaptureState::Complete)
        || index >= views_.size()) return false;
    state_ = CalibrationCaptureState::Collecting;
    storedBytes_ -= views_[index].frame->pixels->size();
    views_.erase(views_.begin() + static_cast<std::ptrdiff_t>(index));
    return true;
}
CalibrationProgress CalibrationCapture::progress() const
{
    CalibrationProgress result;
    result.count = views_.size();
    double minScale = 1;
    double maxScale = 0;
    for (const auto& view : views_) {
        double u = 0;
        double v = 0;
        for (const auto& p : view.detection.corners) { u += p.u; v += p.v; }
        u /= static_cast<double>(view.detection.corners.size()) * view.frame->width;
        v /= static_cast<double>(view.detection.corners.size()) * view.frame->height;
        const int column = std::clamp(static_cast<int>(u * CalibrationLocationGridSide),
            0, CalibrationLocationGridSide - 1);
        const int row = std::clamp(static_cast<int>(v * CalibrationLocationGridSide),
            0, CalibrationLocationGridSide - 1);
        result.occupiedLocationCells[row * CalibrationLocationGridSide + column] = true;
        const double scale = std::sqrt(boardArea(view.detection.corners, config_.target.columns)
            / (static_cast<double>(view.frame->width) * view.frame->height));
        minScale = std::min(minScale, scale);
        maxScale = std::max(maxScale, scale);
        for (const auto& other : views_) {
            const double difference = std::abs(std::remainder(angle(view, config_.target.columns)
                - angle(other, config_.target.columns), std::numbers::pi));
            result.angleSpreadRadians = std::max(result.angleSpreadRadians, difference);
        }
    }
    result.locationCells = static_cast<std::size_t>(std::count(
        result.occupiedLocationCells.begin(), result.occupiedLocationCells.end(), true));
    result.scaleSpread = views_.empty() ? 0 : maxScale - minScale;
    result.ready = result.count >= config_.minimumViews
        && result.locationCells >= config_.minimumLocationCells
        && result.scaleSpread >= config_.minimumScaleSpread
        && result.angleSpreadRadians >= config_.minimumAngleSpreadRadians;
    return result;
}
CalibrationGuidance CalibrationCapture::guidance() const
{
    if (state_ == CalibrationCaptureState::Idle || state_ == CalibrationCaptureState::Cancelled)
        return {CalibrationSuggestion::Start, {}};
    if (state_ == CalibrationCaptureState::Complete)
        return {CalibrationSuggestion::ReviewResult, {}};
    const auto measured = progress();
    if (measured.ready) return {CalibrationSuggestion::Ready, {}};
    if (measured.count >= config_.maximumViews) return {CalibrationSuggestion::ReplaceView, {}};
    if (measured.count == 0) return {CalibrationSuggestion::FirstView, {}};
    if (measured.locationCells < config_.minimumLocationCells) {
        const auto unused = std::find(measured.occupiedLocationCells.begin(),
            measured.occupiedLocationCells.end(), false);
        return {CalibrationSuggestion::ChangeLocation, static_cast<std::size_t>(
            unused - measured.occupiedLocationCells.begin())};
    }
    if (measured.scaleSpread < config_.minimumScaleSpread)
        return {CalibrationSuggestion::ChangeScale, {}};
    if (measured.angleSpreadRadians < config_.minimumAngleSpreadRadians)
        return {CalibrationSuggestion::RotateInPlane, {}};
    return {CalibrationSuggestion::MoreViews, {}};
}
bool CalibrationCapture::finish()
{
    if (state_ != CalibrationCaptureState::Collecting || !progress().ready) return false;
    state_ = CalibrationCaptureState::Complete;
    return true;
}
} // namespace micro_robot
