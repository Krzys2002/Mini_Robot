#pragma once

#include "ui/VisionDebugView.hpp"

#include <algorithm>
#include <cstddef>
#include <vector>

namespace micro_robot::test {
inline std::shared_ptr<VisionDebugFrame> debugPreview(PixelFormat format = PixelFormat::Gray8,
    std::uint32_t width = 320, std::uint32_t height = 200,
    Timestamp capturedAt = MonotonicClock::now())
{
    auto frame = std::make_shared<Frame>();
    frame->sequence = 42;
    frame->capturedAt = capturedAt;
    frame->width = width;
    frame->height = height;
    frame->pixelFormat = format;
    const std::size_t channels = format == PixelFormat::Gray8 ? 1
        : format == PixelFormat::Rgb8 ? 3 : 4;
    frame->strideBytes = static_cast<std::size_t>(width) * channels + 8;
    auto pixels = std::make_shared<std::vector<std::byte>>(frame->strideBytes * height, std::byte{32});
    if (channels == 4) {
        for (std::size_t y = 0; y < height; ++y)
            for (std::size_t x = 0; x < width; ++x)
                (*pixels)[y * frame->strideBytes + 4 * x + 3] = std::byte{255};
    }
    for (std::size_t y = 40; y < 60 && y < height; ++y)
        for (std::size_t x = 130; x < 150 && x < width; ++x)
            for (std::size_t c = 0; c < std::min<std::size_t>(channels, 3); ++c)
                (*pixels)[y * frame->strideBytes + channels * x + c] = std::byte{240};
    frame->pixels = pixels;

    auto detections = std::make_shared<MarkerDetections>();
    detections->frameSequence = frame->sequence;
    detections->capturedAt = frame->capturedAt;
    detections->width = width;
    detections->height = height;
    detections->processingTime = std::chrono::milliseconds{2};
    detections->markers = {
        {MarkerFamily::AprilTag36h11, 0, {{{40, 40}, {100, 40}, {100, 100}, {40, 100}}}, std::nullopt},
        {MarkerFamily::AprilTag36h11, 2, {{{250, 90}, {250, 150}, {190, 150}, {190, 90}}}, 37.5}};
    return makeVisionDebugFrame(frame, detections, RobotMarkerMap{});
}
} // namespace micro_robot::test
