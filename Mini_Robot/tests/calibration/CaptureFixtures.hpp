#pragma once
#include "calibration/CalibrationCapture.hpp"
#include <chrono>
#include <cmath>

namespace micro_robot::test {
inline FramePtr captureFrame(std::uint64_t sequence = 1, std::uint32_t width = 1000,
                             std::uint32_t height = 800)
{
    auto frame = std::make_shared<Frame>();
    frame->sequence = sequence;
    frame->capturedAt = Timestamp{std::chrono::seconds{static_cast<int>(sequence)}};
    frame->width = width;
    frame->height = height;
    frame->strideBytes = width;
    frame->pixelFormat = PixelFormat::Gray8;
    frame->pixels = std::make_shared<const std::vector<std::byte>>(width * height);
    return frame;
}
inline CalibrationDetection captureDetection(double cx = 500, double cy = 400,
                                              double step = 35, double angle = 0)
{
    CalibrationDetection result;
    result.sharpness = 100;
    for (int row = 0; row < 6; ++row) {
        for (int col = 0; col < 9; ++col) {
            const double x = (col - 4) * step;
            const double y = (row - 2.5) * step;
            result.corners.push_back({cx + x * std::cos(angle) - y * std::sin(angle),
                                      cy + x * std::sin(angle) + y * std::cos(angle)});
        }
    }
    return result;
}
} // namespace micro_robot::test
