#include "vision/OpenCvMetricObservation.hpp"
#include "HomographyFixtures.hpp"
#include <gtest/gtest.h>
#include <numbers>
using namespace micro_robot;
using namespace micro_robot::test;
TEST(OpenCvMetricObservationTest, DeliversOnlyCompatibleIdentifiedMetricMeasurements)
{
    auto profile = homographyInput().profile;
    profile.homography = knownHomography();
    profile.intrinsics.distortion = {0, 0, 0, 0, 0};
    MarkerDetections input;
    input.width = profile.intrinsics.width;
    input.height = profile.intrinsics.height;
    input.capturedAt = Timestamp{} + std::chrono::seconds{3};
    input.markers.push_back({MarkerFamily::AprilTag36h11, 0,
        {{{400, 300}, {600, 300}, {600, 500}, {400, 500}}}, {}});
    OpenCvMetricObservationConverter converter;
    auto result = converter.convert(input, profile.intrinsics.sourceId, RobotMarkerMap{}, profile);
    ASSERT_EQ(result.measurements.size(), 1);
    EXPECT_NEAR(result.measurements[0].pose.x.value, .5, 1e-10);
    EXPECT_NEAR(result.measurements[0].pose.y.value, .3, 1e-10);
    EXPECT_NEAR(result.measurements[0].pose.heading.value, std::numbers::pi / 2, 1e-10);
    EXPECT_EQ(result.measurements[0].capturedAt, input.capturedAt);
    EXPECT_EQ(converter.convert(input, "different", RobotMarkerMap{}, profile).status,
        MetricObservationStatus::InvalidCalibration);
    EXPECT_EQ(converter.convert(input, profile.intrinsics.sourceId, RobotMarkerMap{}, {}).status,
        MetricObservationStatus::Uncalibrated);
    input.markers.push_back(input.markers[0]);
    EXPECT_TRUE(converter.convert(input, profile.intrinsics.sourceId, RobotMarkerMap{}, profile).measurements.empty());
}
TEST(OpenCvMetricObservationTest, RemovesLensDistortionBeforeWorldMapping)
{
    auto profile = homographyInput().profile;
    profile.homography = knownHomography();
    profile.intrinsics.distortion = {.15, 0, 0, 0, 0};
    MarkerDetections input;
    input.width = profile.intrinsics.width;
    input.height = profile.intrinsics.height;
    std::array<MarkerCorner, 4> corners{{{400, 300}, {600, 300}, {600, 500}, {400, 500}}};
    for (auto& p : corners) {
        const double x = (p.u - 500) / 800, y = (p.v - 400) / 810;
        const double scale = 1 + .15 * (x*x + y*y);
        p = {500 + 800*x*scale, 400 + 810*y*scale};
    }
    input.markers.push_back({MarkerFamily::AprilTag36h11, 0, corners, {}});
    OpenCvMetricObservationConverter converter;
    const auto result = converter.convert(input, profile.intrinsics.sourceId, RobotMarkerMap{}, profile);
    ASSERT_EQ(result.measurements.size(), 1);
    EXPECT_NEAR(result.measurements[0].pose.x.value, .5, 1e-8);
    EXPECT_NEAR(result.measurements[0].pose.y.value, .3, 1e-8);
    EXPECT_NEAR(result.measurements[0].pose.heading.value, std::numbers::pi / 2, 1e-8);
}
