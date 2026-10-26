#include "vision/OpenCvAprilTagDetector.hpp"

#include <opencv2/core/version.hpp>
#include <opencv2/imgproc.hpp>
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
#include <opencv2/objdetect/aruco_detector.hpp>
#else
#include <opencv2/aruco.hpp>
#endif

#include <cmath>
#include <limits>
#include <stdexcept>

namespace micro_robot {
class OpenCvAprilTagDetector::Impl {
public:
    explicit Impl(AprilTagDetectorConfig value) : config(value)
    {
        if (!config.maximumPixels || !config.maximumDimension
            || config.maximumDimension > static_cast<std::uint32_t>(std::numeric_limits<int>::max())
            || !std::isfinite(config.minimumPerimeterRate) || config.minimumPerimeterRate <= 0
            || config.minimumPerimeterRate >= 4)
            throw std::invalid_argument("Invalid AprilTag detector resource/perimeter limits");
        parameters->markerBorderBits = 1;
        parameters->errorCorrectionRate = 0; // Conservative: no data-bit correction.
        parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
        parameters->minMarkerPerimeterRate = config.minimumPerimeterRate;
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
        detector = std::make_unique<cv::aruco::ArucoDetector>(dictionary, *parameters);
#endif
    }
    AprilTagDetectorConfig config;
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
    cv::aruco::Dictionary dictionary{cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11)};
    cv::Ptr<cv::aruco::DetectorParameters> parameters{cv::makePtr<cv::aruco::DetectorParameters>()};
    std::unique_ptr<cv::aruco::ArucoDetector> detector;
#else
    cv::Ptr<cv::aruco::Dictionary> dictionary{cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11)};
    cv::Ptr<cv::aruco::DetectorParameters> parameters{cv::aruco::DetectorParameters::create()};
#endif
};
OpenCvAprilTagDetector::OpenCvAprilTagDetector(AprilTagDetectorConfig config)
    : impl_(std::make_unique<Impl>(config)) {}
OpenCvAprilTagDetector::~OpenCvAprilTagDetector() = default;

MarkerDetections OpenCvAprilTagDetector::detect(const Frame& frame)
{
    const auto started = MonotonicClock::now();
    MarkerDetections result;
    result.frameSequence = frame.sequence;
    result.capturedAt = frame.capturedAt;
    result.width = frame.width;
    result.height = frame.height;
    if (!isValidFrame(frame) || frame.width > impl_->config.maximumDimension
        || frame.height > impl_->config.maximumDimension
        || static_cast<std::uint64_t>(frame.width) * frame.height > impl_->config.maximumPixels) {
        result.error = "Invalid frame or AprilTag detector resource limit exceeded";
    } else try {
        const int type = frame.pixelFormat == PixelFormat::Gray8 ? CV_8UC1
            : frame.pixelFormat == PixelFormat::Rgb8 ? CV_8UC3 : CV_8UC4;
        // InputArray is read-only despite Mat's mutable constructor. The frame
        // owns storage throughout this call. Gray uses a stride-aware zero-copy view.
        const cv::Mat image(static_cast<int>(frame.height), static_cast<int>(frame.width), type,
            const_cast<std::byte*>(frame.pixels->data()), frame.strideBytes);
        cv::Mat gray;
        if (frame.pixelFormat == PixelFormat::Gray8) gray = image;
        else cv::cvtColor(image, gray, frame.pixelFormat == PixelFormat::Rgb8
                         ? cv::COLOR_RGB2GRAY : cv::COLOR_RGBA2GRAY);
        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<int> ids;
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
        impl_->detector->detectMarkers(gray, corners, ids);
#else
        cv::aruco::detectMarkers(gray, impl_->dictionary, corners, ids, impl_->parameters);
#endif
        if (ids.size() != corners.size()) throw std::runtime_error("Inconsistent marker output");
        result.markers.reserve(ids.size());
        for (std::size_t i = 0; i < ids.size(); ++i) {
            if (ids[i] < 0 || ids[i] >= 587 || corners[i].size() != 4)
                throw std::runtime_error("Invalid marker output");
            MarkerObservation marker;
            marker.id = ids[i];
            for (std::size_t j = 0; j < 4; ++j) {
                const auto& p = corners[i][j];
                if (!std::isfinite(p.x) || !std::isfinite(p.y)) throw std::runtime_error("Invalid marker corner");
                marker.corners[j] = {p.x, p.y};
            }
            result.markers.push_back(marker);
        }
    } catch (const std::exception& error) {
        result.markers.clear();
        result.error = std::string{"AprilTag detection failed: "} + error.what();
    } catch (...) {
        result.markers.clear();
        result.error = "AprilTag detection failed with an unknown error";
    }
    result.processingTime = MonotonicClock::now() - started;
    return result;
}
} // namespace micro_robot
