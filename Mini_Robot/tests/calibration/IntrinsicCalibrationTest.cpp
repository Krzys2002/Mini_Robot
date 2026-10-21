#include "calibration/IntrinsicCalibration.hpp"
#include "CaptureFixtures.hpp"
#include "IntrinsicFixtures.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <functional>
#include <limits>

namespace {
using namespace micro_robot;
using namespace micro_robot::test;
TEST(IntrinsicCalibrationTest, ValidatesModelAndSeparatesWarningFromRejection)
{
    auto result = validIntrinsicCalibration();
    EXPECT_TRUE(validateIntrinsicCalibration(result).empty());
    EXPECT_FALSE(hasReprojectionWarning(result));
    IntrinsicCalibrationConfig config;
    config.warningRmsPixels = 0.4;
    EXPECT_TRUE(validateIntrinsicCalibration(result, config).empty());
    EXPECT_TRUE(hasReprojectionWarning(result, config));
    config.warningRmsPixels = 0.5;
    EXPECT_FALSE(hasReprojectionWarning(result, config)); // Equality is within threshold.
    result.perViewErrors[2].rmsPixels = 1.2;
    result.rmsPixels = std::sqrt((0.25 + 0.25 + 1.44) / 3);
    EXPECT_TRUE(validateIntrinsicCalibration(result).empty());
    EXPECT_LT(result.rmsPixels, 1.0);
    EXPECT_TRUE(hasReprojectionWarning(result)); // One poor view must still warn.
}
TEST(IntrinsicCalibrationTest, RejectsMalformedAndImplausibleModels)
{
    const std::vector<std::function<void(IntrinsicCalibration&)>> corruptions{
        [](auto& r) { r.sourceId.clear(); }, [](auto& r) { r.width = 0; },
        [](auto& r) { r.height = 0; }, [](auto& r) { r.cameraMatrix[0] = -1; },
        [](auto& r) { r.cameraMatrix[4] = 1e9; }, [](auto& r) { r.cameraMatrix[0] = 200; },
        [](auto& r) { r.cameraMatrix[2] = -1; }, [](auto& r) { r.cameraMatrix[5] = r.height; },
        [](auto& r) { r.cameraMatrix[1] = 1; }, [](auto& r) { r.cameraMatrix[8] = 0; },
        [](auto& r) { r.distortion.pop_back(); }, [](auto& r) { r.distortion.push_back(0); },
        [](auto& r) { r.distortion[0] = 100; }, [](auto& r) { r.rmsPixels = -1; },
        [](auto& r) { r.perViewErrors.pop_back(); },
        [](auto& r) { r.perViewErrors[0].pointCount = 0; },
        [](auto& r) { r.perViewErrors[0].rmsPixels = -1; },
        [](auto& r) { r.perViewErrors[0].frameSequence = 2; },
        [](auto& r) { r.rmsPixels = 0; }
    };
    for (std::size_t i = 0; i < corruptions.size(); ++i) {
        auto r = validIntrinsicCalibration();
        corruptions[i](r);
        EXPECT_FALSE(validateIntrinsicCalibration(r).empty()) << i;
    }
}
TEST(IntrinsicCalibrationTest, RejectsNonFiniteMatrixDistortionAndErrors)
{
    for (double invalid : {std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity()}) {
        for (std::size_t i = 0; i < 9; ++i) {
            auto r = validIntrinsicCalibration();
            r.cameraMatrix[i] = invalid;
            EXPECT_FALSE(validateIntrinsicCalibration(r).empty());
        }
        for (std::size_t i = 0; i < 5; ++i) {
            auto r = validIntrinsicCalibration();
            r.distortion[i] = invalid;
            EXPECT_FALSE(validateIntrinsicCalibration(r).empty());
        }
        auto r = validIntrinsicCalibration();
        r.rmsPixels = invalid;
        EXPECT_FALSE(validateIntrinsicCalibration(r).empty());
        r = validIntrinsicCalibration();
        r.perViewErrors[0].rmsPixels = invalid;
        EXPECT_FALSE(validateIntrinsicCalibration(r).empty());
    }
}
TEST(IntrinsicCalibrationTest, ValidatesThresholdsAndWeightedAggregate)
{
    IntrinsicCalibrationConfig config;
    config.warningRmsPixels = 0;
    EXPECT_FALSE(validateIntrinsicConfig(config).empty());
    config = {};
    config.maximumFocalImageRatio = 0.01;
    EXPECT_FALSE(validateIntrinsicConfig(config).empty());
    config = {};
    config.maximumFocalAspectRatio = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(validateIntrinsicConfig(config).empty());
    auto r = validIntrinsicCalibration();
    r.perViewErrors = {{1, 4, 0}, {2, 4, 0}, {3, 8, 2}};
    r.rmsPixels = std::sqrt(2.0);
    EXPECT_TRUE(validateIntrinsicCalibration(r).empty());
    r.rmsPixels = 2.0 / 3; // Unweighted average is not RMS.
    EXPECT_FALSE(validateIntrinsicCalibration(r).empty());
}
TEST(IntrinsicCalibrationTest, SnapshotsOnlyCompletedCaptureAndOwnsCorners)
{
    CalibrationCaptureConfig config;
    config.minimumViews = 3;
    config.minimumLocationCells = 1;
    config.minimumScaleSpread = config.minimumAngleSpreadRadians = 0;
    CalibrationCapture capture{config};
    EXPECT_THROW((void)intrinsicCalibrationInput(capture), std::invalid_argument);
    capture.restart("camera-a");
    for (int i = 1; i <= 3; ++i)
        ASSERT_TRUE(capture.add(captureFrame(i), captureDetection(250.0 * i)).accepted);
    EXPECT_THROW((void)intrinsicCalibrationInput(capture), std::invalid_argument);
    ASSERT_TRUE(capture.finish());
    const auto input = intrinsicCalibrationInput(capture);
    capture.cancel();
    EXPECT_EQ(input.sourceId, "camera-a");
    EXPECT_EQ(input.width, 1000U);
    EXPECT_EQ(input.height, 800U);
    ASSERT_EQ(input.observations.size(), 3U);
    EXPECT_EQ(input.observations[2].frameSequence, 3U);
    EXPECT_EQ(input.observations[0].corners.size(), 54U);
}
} // namespace
