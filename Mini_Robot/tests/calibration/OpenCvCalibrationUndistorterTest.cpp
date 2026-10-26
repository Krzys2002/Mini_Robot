#include "calibration/OpenCvCalibrationUndistorter.hpp"
#include "IntrinsicFixtures.hpp"
#include <gtest/gtest.h>
#include <cmath>

namespace {
using namespace micro_robot;
using namespace micro_robot::test;
IntrinsicCalibration smallModel()
{
    auto c = validIntrinsicCalibration();
    c.width = 64; c.height = 48;
    c.cameraMatrix = {50, 0, 32, 0, 50, 24, 0, 0, 1};
    return c;
}
Frame image(PixelFormat format)
{
    const std::size_t channels = format == PixelFormat::Gray8 ? 1 : format == PixelFormat::Rgb8 ? 3 : 4;
    Frame f{42, Timestamp{} + std::chrono::seconds{1}, 64, 48, 64 * channels + 7, format, {}};
    auto pixels = std::make_shared<std::vector<std::byte>>(f.strideBytes * f.height, std::byte{0xAA});
    for (std::size_t y = 0; y < f.height; ++y)
        for (std::size_t x = 0; x < f.width; ++x)
            for (std::size_t ch = 0; ch < channels; ++ch)
                (*pixels)[y * f.strideBytes + x * channels + ch] = static_cast<std::byte>(x * 3);
    f.pixels = pixels;
    return f;
}
TEST(OpenCvCalibrationUndistorterTest, ZeroDistortionPreservesPackedFormatsMetadataAndOriginalStorage)
{
    auto model = smallModel();
    model.distortion.assign(5, 0);
    OpenCvCalibrationUndistorter undistorter;
    for (auto format : {PixelFormat::Gray8, PixelFormat::Rgb8, PixelFormat::Rgba8}) {
        const auto frame = image(format);
        const auto before = *frame.pixels;
        const auto outcome = undistorter.undistort(frame, model);
        ASSERT_TRUE(outcome.frame) << outcome.error;
        EXPECT_NE(outcome.frame->pixels, frame.pixels);
        EXPECT_EQ(*frame.pixels, before);
        EXPECT_EQ(outcome.frame->capturedAt, frame.capturedAt);
        EXPECT_EQ(outcome.frame->sequence, frame.sequence);
        EXPECT_EQ(outcome.frame->pixelFormat, frame.pixelFormat);
        const std::size_t row = outcome.frame->strideBytes;
        for (std::size_t y = 0; y < frame.height; ++y)
            EXPECT_TRUE(std::equal(before.begin() + static_cast<std::ptrdiff_t>(y * frame.strideBytes),
                before.begin() + static_cast<std::ptrdiff_t>(y * frame.strideBytes + row),
                outcome.frame->pixels->begin() + static_cast<std::ptrdiff_t>(y * row)));
    }
}
TEST(OpenCvCalibrationUndistorterTest, MatchesAnalyticalBrownConradySampling)
{
    const auto frame = image(PixelFormat::Gray8);
    const auto model = smallModel();
    OpenCvCalibrationUndistorter undistorter;
    const auto result = undistorter.undistort(frame, model);
    ASSERT_TRUE(result.frame) << result.error;
    for (int v = 5; v < 44; v += 6) for (int u = 5; u < 60; u += 6) {
        const double x = (u - 32) / 50.0, y = (v - 24) / 50.0;
        const double r2 = x*x + y*y;
        const auto& d = model.distortion;
        const double radial = 1 + d[0]*r2 + d[1]*r2*r2 + d[4]*r2*r2*r2;
        const double xd = x*radial + 2*d[2]*x*y + d[3]*(r2 + 2*x*x);
        const double sampleU = 50*xd + 32;
        const int value = std::to_integer<int>((*result.frame->pixels)[static_cast<std::size_t>(v)*result.frame->strideBytes + static_cast<std::size_t>(u)]);
        EXPECT_NEAR(value, 3*sampleU, 1.0);
    }
}
TEST(OpenCvCalibrationUndistorterTest, RejectsInvalidModelsFramesAndResolutionMismatch)
{
    auto frame = image(PixelFormat::Gray8);
    auto model = smallModel();
    OpenCvCalibrationUndistorter undistorter;
    model.width++;
    EXPECT_FALSE(undistorter.undistort(frame, model).frame);
    model = smallModel();
    model.cameraMatrix[0] = 0;
    EXPECT_FALSE(undistorter.undistort(frame, model).frame);
    model = smallModel();
    frame.pixels.reset();
    EXPECT_FALSE(undistorter.undistort(frame, model).frame);
}
} // namespace
