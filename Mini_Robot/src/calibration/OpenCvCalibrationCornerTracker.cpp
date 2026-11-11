#include "calibration/OpenCvCalibrationCornerTracker.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace micro_robot {
namespace {
cv::Mat gray(const Frame& frame)
{
    const int channels = frame.pixelFormat == PixelFormat::Gray8 ? 1 : frame.pixelFormat == PixelFormat::Rgb8 ? 3 : 4;
    const cv::Mat input(static_cast<int>(frame.height), static_cast<int>(frame.width),
        CV_MAKETYPE(CV_8U, channels), const_cast<std::byte*>(frame.pixels->data()), frame.strideBytes);
    cv::Mat output;
    if (channels == 1) return input;
    cv::cvtColor(input, output, channels == 3 ? cv::COLOR_RGB2GRAY : cv::COLOR_RGBA2GRAY);
    return output;
}
bool inside(const ImageCorner& point, const Frame& frame)
{
    return std::isfinite(point.u) && std::isfinite(point.v) && point.u >= 4 && point.v >= 4
        && point.u < frame.width - 4.0 && point.v < frame.height - 4.0;
}
}
OpenCvCalibrationCornerTracker::OpenCvCalibrationCornerTracker(CalibrationCornerTrackingConfig config)
    : config_(config)
{
    if (!std::isfinite(config.maximumRoundTripPixels) || config.maximumRoundTripPixels <= 0
        || !std::isfinite(config.maximumPatchError) || config.maximumPatchError <= 0
        || !std::isfinite(config.maximumSpacingFraction) || config.maximumSpacingFraction <= 0
        || config.maximumSpacingFraction >= .5 || config.maximumStep <= std::chrono::milliseconds::zero())
        throw std::invalid_argument("Invalid calibration corner tracking configuration");
}
std::vector<ImageCorner> OpenCvCalibrationCornerTracker::track(const Frame& previous,
    const Frame& current, const std::vector<ImageCorner>& corners)
{
    if (!isValidFrame(previous) || !isValidFrame(current) || previous.width != current.width
        || previous.height != current.height || previous.pixelFormat != current.pixelFormat
        || current.width > 4096 || current.height > 4096
        || static_cast<std::uint64_t>(current.width)*current.height > 4'000'000
        || current.sequence <= previous.sequence || current.capturedAt <= previous.capturedAt
        || current.capturedAt-previous.capturedAt > config_.maximumStep
        || corners.size() < 4 || corners.size() > 1024) return {};
    double spacing = std::numeric_limits<double>::infinity();
    std::vector<cv::Point2f> input;
    for (std::size_t i=0; i<corners.size(); ++i) {
        if (!inside(corners[i], previous)) return {};
        input.emplace_back(static_cast<float>(corners[i].u), static_cast<float>(corners[i].v));
        for (std::size_t j=0; j<i; ++j)
            spacing = std::min(spacing, std::hypot(corners[i].u-corners[j].u, corners[i].v-corners[j].v));
    }
    if (spacing < 4 || !std::isfinite(spacing)) return {};
    try {
        const auto from=gray(previous), to=gray(current);
        std::vector<cv::Point2f> forward, backward;
        std::vector<uchar> forwardOk, backwardOk;
        std::vector<float> forwardError, backwardError;
        const cv::Size window{21,21};
        const cv::TermCriteria criteria{cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, .01};
        cv::calcOpticalFlowPyrLK(from, to, input, forward, forwardOk, forwardError, window, 3, criteria);
        cv::calcOpticalFlowPyrLK(to, from, forward, backward, backwardOk, backwardError, window, 3, criteria);
        std::vector<ImageCorner> result;
        for (std::size_t i=0; i<input.size(); ++i) {
            const ImageCorner point{forward[i].x, forward[i].y};
            const auto roundTrip=cv::norm(input[i]-backward[i]);
            const auto displacement=cv::norm(input[i]-forward[i]);
            if (!forwardOk[i] || !backwardOk[i] || !inside(point, current)
                || !std::isfinite(roundTrip) || roundTrip > config_.maximumRoundTripPixels
                || !std::isfinite(forwardError[i]) || !std::isfinite(backwardError[i])
                || forwardError[i] > config_.maximumPatchError || backwardError[i] > config_.maximumPatchError
                || displacement > spacing*config_.maximumSpacingFraction) return {};
            result.push_back(point);
        }
        return result;
    } catch (const cv::Exception&) { return {}; }
}
} // namespace micro_robot
