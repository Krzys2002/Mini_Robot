#include "calibration/OpenCvCalibrationUndistorter.hpp"
#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR >= 5
#include <opencv2/calib.hpp>
#include <opencv2/geometry.hpp>
#else
#include <opencv2/calib3d.hpp>
#endif
#include <opencv2/imgproc.hpp>

#include <cstring>
#include <cmath>
#include <exception>

namespace micro_robot {
UndistortedPreviewOutcome OpenCvCalibrationUndistorter::undistort(const Frame& frame,
    const IntrinsicCalibration& c)
{
    if (const auto error = validateIntrinsicCalibration(c); !error.empty()) return {{}, error};
    if (!isValidFrame(frame) || frame.width != c.width || frame.height != c.height
        || !supportedCalibrationResolution(frame.width, frame.height))
        return {{}, "Invalid, oversized or incompatible preview frame."};
    try {
        const int channels = frame.pixelFormat == PixelFormat::Gray8 ? 1 : frame.pixelFormat == PixelFormat::Rgb8 ? 3 : 4;
        // InputArray is read-only; output is a separate allocation.
        const cv::Mat source(static_cast<int>(frame.height), static_cast<int>(frame.width), CV_MAKETYPE(CV_8U, channels),
            const_cast<std::byte*>(frame.pixels->data()), frame.strideBytes);
        const cv::Mat k(3, 3, CV_64F, const_cast<double*>(c.cameraMatrix.data()));
        const cv::Mat d(1, 5, CV_64F, const_cast<double*>(c.distortion.data()));
        cv::Mat output;
        // Keep the original K and full resolution; black border pixels are expected.
        cv::undistort(source, output, k, d, k);
        auto result = std::make_shared<Frame>(frame);
        result->strideBytes = static_cast<std::size_t>(frame.width) * static_cast<std::size_t>(channels);
        auto pixels = std::make_shared<std::vector<std::byte>>(result->strideBytes * frame.height);
        for (std::uint32_t y = 0; y < frame.height; ++y)
            std::memcpy(pixels->data() + y * result->strideBytes, output.ptr(static_cast<int>(y)), result->strideBytes);
        result->pixels = std::move(pixels);
        return {std::move(result), {}};
    } catch (const std::exception& error) { return {{}, std::string{"Undistortion failed: "} + error.what()}; }
}
UndistortedPreviewOutcome OpenCvCalibrationUndistorter::undistortWithCorners(const Frame& frame,
    const IntrinsicCalibration& c, const std::vector<ImageCorner>& corners)
{
    auto result = undistort(frame, c);
    if (!result.frame || !result.error.empty() || corners.empty()) return result;
    try {
        std::vector<cv::Point2d> input, output;
        for (const auto& point : corners) input.emplace_back(point.u, point.v);
        const cv::Mat k(3, 3, CV_64F, const_cast<double*>(c.cameraMatrix.data()));
        const cv::Mat d(1, 5, CV_64F, const_cast<double*>(c.distortion.data()));
        cv::undistortPoints(input, output, k, d, cv::noArray(), k);
        for (const auto& point : output) {
            if (!std::isfinite(point.x) || !std::isfinite(point.y))
                return {{}, "Non-finite undistorted corner."};
            result.corners.push_back({point.x, point.y});
        }
    } catch (const std::exception& error) { return {{}, error.what()}; }
    return result;
}
} // namespace micro_robot
