#pragma once
#include "camera/CameraTypes.hpp"
#include <opencv2/core/version.hpp>
#include <opencv2/imgproc.hpp>
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
#include <opencv2/objdetect/aruco_detector.hpp>
#else
#include <opencv2/aruco.hpp>
#endif
#include <cstring>

namespace micro_robot::test {
inline cv::Mat tag(int id, int side = 160)
{
    const auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    cv::Mat image;
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
    cv::aruco::generateImageMarker(dictionary, id, side, image, 1);
#else
    cv::aruco::drawMarker(dictionary, id, side, image, 1);
#endif
    return image;
}
inline cv::Mat tagScene(int id = 7)
{
    cv::Mat image(400, 400, CV_8UC1, cv::Scalar{255});
    tag(id).copyTo(image(cv::Rect{100, 100, 160, 160}));
    return image;
}
inline FramePtr markerFrame(const cv::Mat& input, PixelFormat format = PixelFormat::Gray8,
                            std::uint64_t sequence = 42)
{
    cv::Mat image;
    if (format == PixelFormat::Rgb8) cv::cvtColor(input, image, cv::COLOR_GRAY2RGB);
    else if (format == PixelFormat::Rgba8) cv::cvtColor(input, image, cv::COLOR_GRAY2RGBA);
    else image = input;
    auto frame = std::make_shared<Frame>();
    frame->sequence = sequence;
    frame->capturedAt = Timestamp{} + std::chrono::milliseconds{sequence};
    frame->width = static_cast<std::uint32_t>(image.cols);
    frame->height = static_cast<std::uint32_t>(image.rows);
    frame->strideBytes = image.cols * image.elemSize() + 13;
    frame->pixelFormat = format;
    auto storage = std::make_shared<std::vector<std::byte>>(frame->strideBytes * frame->height);
    for (int row = 0; row < image.rows; ++row)
        std::memcpy(storage->data() + row * frame->strideBytes, image.ptr(row), image.cols * image.elemSize());
    frame->pixels = storage;
    return frame;
}
} // namespace micro_robot::test
