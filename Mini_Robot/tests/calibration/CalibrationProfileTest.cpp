#include "calibration/CalibrationProfile.hpp"
#include "IntrinsicFixtures.hpp"
#include <gtest/gtest.h>
#include <limits>

namespace {
using namespace micro_robot;
using namespace micro_robot::test;
TEST(CalibrationProfileTest, RoundTripsAllFieldsWithoutPrecisionLoss)
{
    auto c = validIntrinsicCalibration();
    c.sourceId = "camera \\\" Unicode: żółć/../camera";
    c.cameraMatrix[0] = 812.1234567890123;
    c.distortion[4] = std::numeric_limits<double>::denorm_min();
    const auto encoded = serializeCalibrationProfile(c);
    const auto loaded = parseCalibrationProfile(encoded);
    ASSERT_TRUE(loaded.result) << loaded.error;
    const auto& r = *loaded.result;
    EXPECT_EQ(r.sourceId, c.sourceId);
    EXPECT_EQ(r.width, c.width);
    EXPECT_EQ(r.height, c.height);
    EXPECT_EQ(r.cameraMatrix, c.cameraMatrix);
    EXPECT_EQ(r.distortion, c.distortion);
    EXPECT_EQ(r.rmsPixels, c.rmsPixels);
    ASSERT_EQ(r.perViewErrors.size(), c.perViewErrors.size());
    for (std::size_t i = 0; i < r.perViewErrors.size(); ++i) {
        EXPECT_EQ(r.perViewErrors[i].frameSequence, c.perViewErrors[i].frameSequence);
        EXPECT_EQ(r.perViewErrors[i].pointCount, c.perViewErrors[i].pointCount);
        EXPECT_EQ(r.perViewErrors[i].rmsPixels, c.perViewErrors[i].rmsPixels);
    }
    EXPECT_EQ(serializeCalibrationProfile(r), encoded);
}
TEST(CalibrationProfileTest, RejectsUnknownVersionModelTruncationAndTrailingData)
{
    const auto good = serializeCalibrationProfile(validIntrinsicCalibration());
    for (std::size_t n = 0; n < good.find("end"); ++n)
        EXPECT_FALSE(parseCalibrationProfile(good.substr(0, n)).result) << n;
    auto changed = good;
    changed.replace(changed.find("INTRINSICS 1"), 12, "INTRINSICS 2");
    EXPECT_FALSE(parseCalibrationProfile(changed).result);
    changed = good;
    changed.replace(changed.find("pinhole_brown_conrady_5"), 22, "fisheye");
    EXPECT_FALSE(parseCalibrationProfile(changed).result);
    EXPECT_FALSE(parseCalibrationProfile(good + "extra").result);
    EXPECT_FALSE(parseCalibrationProfile(std::string(MaximumCalibrationProfileBytes + 1, 'x')).result);
}
TEST(CalibrationProfileTest, RejectsMalformedNumericsAndInconsistentErrors)
{
    const auto good = serializeCalibrationProfile(validIntrinsicCalibration());
    for (const std::string bad : {"nan", "inf", "-5", "1e999", "garbage"}) {
        auto changed = good;
        const auto start = changed.find("matrix ") + 7;
        changed.replace(start, changed.find(' ', start) - start, bad);
        EXPECT_FALSE(parseCalibrationProfile(changed).result) << bad;
    }
    auto c = validIntrinsicCalibration();
    c.rmsPixels = 100;
    EXPECT_THROW((void)serializeCalibrationProfile(c), std::invalid_argument);
    c = validIntrinsicCalibration();
    c.perViewErrors.resize(25);
    EXPECT_THROW((void)serializeCalibrationProfile(c), std::invalid_argument);
    c = validIntrinsicCalibration();
    c.sourceId = "camera\nwrong";
    EXPECT_THROW((void)serializeCalibrationProfile(c), std::invalid_argument);
}
TEST(CalibrationProfileTest, RequiresExactIdentityAndDeliveredResolution)
{
    const auto c = validIntrinsicCalibration();
    EXPECT_TRUE(calibrationCompatibilityError(c, "camera-a", 1000, 800).empty());
    EXPECT_FALSE(calibrationCompatibilityError(c, "camera-b", 1000, 800).empty());
    EXPECT_FALSE(calibrationCompatibilityError(c, "camera-a", 800, 1000).empty());
    EXPECT_FALSE(calibrationCompatibilityError(c, "camera-a", 500, 400).empty());
    EXPECT_FALSE(calibrationCompatibilityError(c, "", 1000, 800).empty());
}
} // namespace
