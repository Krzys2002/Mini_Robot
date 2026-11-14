#include "calibration/OpenCvCalibrationDetector.hpp"

#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR >= 5
#include <opencv2/objdetect.hpp>
#else
#include <opencv2/calib3d.hpp>
#endif
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>

namespace micro_robot {
CalibrationDetection OpenCvCalibrationDetector::detect(const Frame& frame,
                                                       const CalibrationTarget& target)
{
    CalibrationDetection result;
    if (!isValidFrame(frame) || !supportedCalibrationResolution(frame.width, frame.height)) {
        result.error = "Invalid or oversized frame. Select at most 4 megapixels, neither dimension above 4096.";
        return result;
    }
    try {
        (void)calibrationObjectPoints(target); // Validate before passing dimensions to OpenCV.
        const int type = frame.pixelFormat == PixelFormat::Gray8 ? CV_8UC1
            : frame.pixelFormat == PixelFormat::Rgb8 ? CV_8UC3 : CV_8UC4;
        // OpenCV's read-only InputArray still requires a mutable Mat view. Never write to it.
        const cv::Mat image(static_cast<int>(frame.height), static_cast<int>(frame.width), type,
            const_cast<std::byte*>(frame.pixels->data()), frame.strideBytes);
        cv::Mat gray;
        if (frame.pixelFormat == PixelFormat::Gray8) gray = image;
        else cv::cvtColor(image, gray, frame.pixelFormat == PixelFormat::Rgb8
                         ? cv::COLOR_RGB2GRAY : cv::COLOR_RGBA2GRAY);
        std::vector<cv::Point2f> corners;
        if (!cv::findChessboardCornersSB(gray, {target.columns, target.rows}, corners,
                                        cv::CALIB_CB_NORMALIZE_IMAGE)) return result;
        for (const auto& p : corners) result.corners.push_back({p.x, p.y});
        double minU = gray.cols, minV = gray.rows, maxU = 0, maxV = 0;
        for (const auto& p : corners) {
            minU = std::min(minU, static_cast<double>(p.x));
            minV = std::min(minV, static_cast<double>(p.y));
            maxU = std::max(maxU, static_cast<double>(p.x));
            maxV = std::max(maxV, static_cast<double>(p.y));
        }
        const int left = std::clamp(static_cast<int>(std::floor(minU)), 0, gray.cols - 1);
        const int top = std::clamp(static_cast<int>(std::floor(minV)), 0, gray.rows - 1);
        const cv::Rect bounds{left, top,
            std::clamp(static_cast<int>(std::ceil(maxU)) + 1, left + 1, gray.cols) - left,
            std::clamp(static_cast<int>(std::ceil(maxV)) + 1, top + 1, gray.rows) - top};
        cv::Mat laplacian;
        cv::Laplacian(gray(bounds), laplacian, CV_64F);
        cv::Scalar mean;
        cv::Scalar deviation;
        cv::meanStdDev(laplacian, mean, deviation);
        result.sharpness = deviation[0] * deviation[0];
    } catch (const std::exception& error) {
        result.error = std::string{"Calibration detection failed: "} + error.what();
    }
    return result;
}
} // namespace micro_robot
