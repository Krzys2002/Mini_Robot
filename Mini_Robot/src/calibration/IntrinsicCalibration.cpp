#include "calibration/IntrinsicCalibration.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

namespace micro_robot {
std::string validateIntrinsicConfig(const IntrinsicCalibrationConfig& c)
{
    if (!std::isfinite(c.warningRmsPixels) || c.warningRmsPixels <= 0
        || !std::isfinite(c.minimumFocalImageRatio) || c.minimumFocalImageRatio <= 0
        || !std::isfinite(c.maximumFocalImageRatio) || c.maximumFocalImageRatio < c.minimumFocalImageRatio
        || !std::isfinite(c.maximumFocalAspectRatio) || c.maximumFocalAspectRatio < 1
        || !std::isfinite(c.maximumAbsoluteDistortion) || c.maximumAbsoluteDistortion <= 0)
        return "Invalid intrinsic calibration thresholds.";
    return {};
}
std::string validateIntrinsicCalibration(const IntrinsicCalibration& r, const IntrinsicCalibrationConfig& c)
{
    if (const auto error = validateIntrinsicConfig(c); !error.empty()) return error;
    if (r.sourceId.empty() || !supportedCalibrationResolution(r.width, r.height))
        return "Calibration needs camera identity and supported resolution: at most 4 megapixels and 4096 per dimension.";
    const auto& k = r.cameraMatrix;
    if (!std::all_of(k.begin(), k.end(), [](double v) { return std::isfinite(v); }))
        return "Camera matrix contains non-finite values.";
    if (k[1] != 0 || k[3] != 0 || k[6] != 0 || k[7] != 0 || k[8] != 1)
        return "Camera matrix is not a supported pinhole matrix.";
    const auto plausibleFocal = [&c](double focal, double extent) {
        return focal / extent >= c.minimumFocalImageRatio && focal / extent <= c.maximumFocalImageRatio;
    };
    if (!plausibleFocal(k[0], r.width) || !plausibleFocal(k[4], r.height)
        || std::max(k[0] / k[4], k[4] / k[0]) > c.maximumFocalAspectRatio)
        return "Implausible focal lengths. Capture more tilted, diverse views.";
    if (k[2] < 0 || k[2] >= r.width || k[5] < 0 || k[5] >= r.height)
        return "Principal point lies outside the calibrated image.";
    if (r.distortion.size() != 5)
        return "Unsupported distortion vector; expected k1,k2,p1,p2,k3.";
    for (double v : r.distortion) {
        if (!std::isfinite(v) || std::abs(v) > c.maximumAbsoluteDistortion)
            return "Non-finite or implausible distortion coefficient.";
    }
    if (!std::isfinite(r.rmsPixels) || r.rmsPixels < 0 || r.perViewErrors.size() < 3)
        return "Invalid aggregate reprojection error or insufficient per-view errors.";
    double weightedSquares = 0;
    double points = 0;
    std::set<std::uint64_t> sequences;
    for (const auto& view : r.perViewErrors) {
        if (view.pointCount < 4 || !std::isfinite(view.rmsPixels) || view.rmsPixels < 0
            || !sequences.insert(view.frameSequence).second)
            return "Invalid or duplicate per-view reprojection error.";
        weightedSquares += view.rmsPixels * view.rmsPixels * static_cast<double>(view.pointCount);
        points += static_cast<double>(view.pointCount);
    }
    const double aggregate = std::sqrt(weightedSquares / points);
    if (!std::isfinite(aggregate) || std::abs(aggregate - r.rmsPixels) > 1e-6 * std::max(1.0, aggregate))
        return "Aggregate reprojection error does not match per-view errors.";
    return {};
}
bool hasReprojectionWarning(const IntrinsicCalibration& r, const IntrinsicCalibrationConfig& c)
{
    return r.rmsPixels > c.warningRmsPixels
        || std::any_of(r.perViewErrors.begin(), r.perViewErrors.end(), [&c](const auto& view) {
            return view.rmsPixels > c.warningRmsPixels;
        });
}
IntrinsicCalibrationInput intrinsicCalibrationInput(const CalibrationCapture& capture)
{
    if (capture.state() != CalibrationCaptureState::Complete || capture.views().empty())
        throw std::invalid_argument("Finish a diverse capture before calculating intrinsics.");
    const auto& first = *capture.views().front().frame;
    IntrinsicCalibrationInput input{capture.sourceId(), first.width, first.height, capture.config().target, {}};
    for (const auto& view : capture.views()) {
        input.observations.push_back({view.frame->sequence, view.detection.corners});
    }
    return input;
}
} // namespace micro_robot
