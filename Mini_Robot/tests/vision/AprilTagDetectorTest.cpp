#include "vision/OpenCvAprilTagDetector.hpp"
#include "vision/RobotObservation.hpp"
#include "MarkerFixtures.hpp"
#if CV_VERSION_MAJOR >= 5
#include <opencv2/geometry.hpp>
#endif
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

namespace {
using namespace micro_robot;
using namespace micro_robot::test;

TEST(AprilTagDetectorTest, PreservesPixelsMetadataAndCornersForEveryPackedFormat)
{
    OpenCvAprilTagDetector detector;
    for (const auto format : {PixelFormat::Gray8, PixelFormat::Rgb8, PixelFormat::Rgba8}) {
        const auto frame = markerFrame(tagScene(), format);
        const auto before = *frame->pixels;
        const auto result = detector.detect(*frame);
        ASSERT_TRUE(result.error.empty()) << result.error;
        ASSERT_EQ(result.markers.size(), 1U);
        EXPECT_EQ(result.frameSequence, frame->sequence);
        EXPECT_EQ(result.capturedAt, frame->capturedAt);
        EXPECT_EQ(result.width, frame->width);
        EXPECT_EQ(result.height, frame->height);
        EXPECT_GT(result.processingTime, Duration::zero());
        EXPECT_EQ(*frame->pixels, before);
        const auto& marker = result.markers.front();
        EXPECT_EQ(marker.family, MarkerFamily::AprilTag36h11);
        EXPECT_EQ(marker.id, 7);
        EXPECT_FALSE(marker.decisionMargin);
        const std::array<MarkerCorner, 4> expected{{{100,100}, {259,100}, {259,259}, {100,259}}};
        for (std::size_t i = 0; i < 4; ++i) {
            EXPECT_NEAR(marker.corners[i].u, expected[i].u, 1);
            EXPECT_NEAR(marker.corners[i].v, expected[i].v, 1);
        }
    }
}
TEST(AprilTagDetectorTest, CanonicalCornersRotateWithTheDecodedMarker)
{
    OpenCvAprilTagDetector detector;
    auto image = tagScene(15);
    std::array<MarkerCorner, 4> expected{{{100,100}, {259,100}, {259,259}, {100,259}}};
    for (int rotation = 0; rotation < 4; ++rotation) {
        const auto result = detector.detect(*markerFrame(image));
        ASSERT_TRUE(result.error.empty());
        ASSERT_EQ(result.markers.size(), 1U);
        EXPECT_EQ(result.markers.front().id, 15);
        for (std::size_t i = 0; i < 4; ++i) {
            EXPECT_NEAR(result.markers.front().corners[i].u, expected[i].u, 1);
            EXPECT_NEAR(result.markers.front().corners[i].v, expected[i].v, 1);
        }
        cv::rotate(image, image, cv::ROTATE_90_CLOCKWISE);
        for (auto& p : expected) p = {399 - p.v, p.u};
    }
}
TEST(AprilTagDetectorTest, DecodedCanonicalCornersGiveRobotHeadingInEveryQuarterTurn)
{
    OpenCvAprilTagDetector detector;
    const RobotMarkerMap map;
    auto image = tagScene(15);
    MarkerCorner center{179.5, 179.5};
    double heading = std::numbers::pi / 2; // Printed top edge toward image up.
    for (int rotation = 0; rotation < 4; ++rotation) {
        const auto result = observeRobots(detector.detect(*markerFrame(image)), map);
        ASSERT_TRUE(result.error.empty());
        ASSERT_EQ(result.robots.size(), 1U);
        const auto& robot = result.robots.front();
        EXPECT_EQ(robot.robotId, RobotId{16});
        EXPECT_NEAR(robot.center.u, center.u, 1);
        EXPECT_NEAR(robot.center.v, center.v, 1);
        EXPECT_NEAR(std::remainder(robot.imageHeading.value - heading, 2 * std::numbers::pi), 0, .02);
        cv::rotate(image, image, cv::ROTATE_90_CLOCKWISE);
        center = {399 - center.v, center.u};
        heading -= std::numbers::pi / 2;
    }
}
TEST(AprilTagDetectorTest, ReturnsMultipleDuplicateAndUnassignedDictionaryIdsWithoutAssociation)
{
    OpenCvAprilTagDetector detector;
    cv::Mat image(400, 900, CV_8UC1, cv::Scalar{255});
    tag(0).copyTo(image(cv::Rect{50,100,160,160}));
    tag(0).copyTo(image(cv::Rect{350,100,160,160}));
    tag(123).copyTo(image(cv::Rect{650,100,160,160}));
    const auto result = detector.detect(*markerFrame(image));
    ASSERT_TRUE(result.error.empty()) << result.error;
    ASSERT_EQ(result.markers.size(), 3U);
    std::vector<int> ids;
    for (const auto& marker : result.markers) ids.push_back(marker.id);
    std::sort(ids.begin(), ids.end());
    EXPECT_EQ(ids, (std::vector<int>{0, 0, 123}));
}
TEST(AprilTagDetectorTest, HandlesScaleRotationAndNegativeImagesWithoutFalseRobotObservations)
{
    OpenCvAprilTagDetector detector;
    const auto original = tagScene();
    cv::Mat rotated;
    cv::warpAffine(original, rotated, cv::getRotationMatrix2D({180,180}, 27, .7), original.size(),
                   cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar{255});
    EXPECT_EQ(detector.detect(*markerFrame(rotated)).markers.size(), 1U);
    cv::Mat blank(400,400,CV_8UC1,cv::Scalar{255});
    EXPECT_TRUE(detector.detect(*markerFrame(blank)).markers.empty());
    cv::Mat noise(400,400,CV_8UC1);
    cv::RNG{12345}.fill(noise, cv::RNG::UNIFORM, 0, 256);
    EXPECT_TRUE(detector.detect(*markerFrame(noise)).markers.empty());
    auto occluded = original.clone();
    occluded(cv::Rect{100,100,80,160}).setTo(255);
    EXPECT_TRUE(detector.detect(*markerFrame(occluded)).markers.empty());
}
TEST(AprilTagDetectorTest, RejectsMalformedAndOversizedFramesAndRecovers)
{
    OpenCvAprilTagDetector detector{{160'000, 500, .005}};
    auto frame = *markerFrame(tagScene());
    frame.strideBytes = 1;
    EXPECT_FALSE(detector.detect(frame).error.empty());
    frame = *markerFrame(tagScene());
    frame.pixelFormat = static_cast<PixelFormat>(99);
    EXPECT_FALSE(detector.detect(frame).error.empty());
    frame = *markerFrame(tagScene());
    frame.pixels.reset();
    EXPECT_FALSE(detector.detect(frame).error.empty());
    auto big = markerFrame(cv::Mat(401,400,CV_8UC1,cv::Scalar{255}));
    auto result = detector.detect(*big);
    EXPECT_FALSE(result.error.empty());
    EXPECT_TRUE(result.markers.empty());
    EXPECT_EQ(detector.detect(*markerFrame(tagScene())).markers.size(), 1U);
}
TEST(AprilTagDetectorTest, ValidatesResourceConfiguration)
{
    EXPECT_THROW((OpenCvAprilTagDetector{{0, 4096, .005}}), std::invalid_argument);
    EXPECT_THROW((OpenCvAprilTagDetector{{100, 0, .005}}), std::invalid_argument);
    EXPECT_THROW((OpenCvAprilTagDetector{{100, 4096, std::numeric_limits<double>::quiet_NaN()}}), std::invalid_argument);
}
} // namespace
