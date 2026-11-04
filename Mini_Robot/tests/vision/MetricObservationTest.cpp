#include "vision/MetricObservation.hpp"
#include "../calibration/HomographyFixtures.hpp"
#include <gtest/gtest.h>
#include <numbers>
#include <algorithm>

using namespace micro_robot;
using namespace micro_robot::test;

TEST(MetricObservationTest, TransformsCenterHeadingAndMetadata)
{
    auto profile = homographyInput().profile;
    profile.homography = knownHomography();
    RobotObservation observation{};
    observation.robotId = RobotId{3};
    observation.frameSequence = 42;
    observation.capturedAt = Timestamp{} + std::chrono::seconds{2};
    observation.minimumEdgePixels = 50;
    observation.decisionMargin = 12.;
    std::array<MarkerCorner, 4> corners{{{400, 300}, {600, 300}, {600, 500}, {400, 500}}};
    const auto result = metricObservation(observation, corners, profile);
    ASSERT_TRUE(result.measurement);
    EXPECT_NEAR(result.measurement->pose.x.value, .5, 1e-12);
    EXPECT_NEAR(result.measurement->pose.y.value, .3, 1e-12);
    EXPECT_NEAR(result.measurement->pose.heading.value, std::numbers::pi / 2, 1e-12);
    EXPECT_EQ(result.measurement->robotId, observation.robotId);
    EXPECT_EQ(result.measurement->capturedAt, observation.capturedAt);
    EXPECT_EQ(result.measurement->frameSequence, 42);
    EXPECT_EQ(result.measurement->decisionMargin, 12.);
}

TEST(MetricObservationTest, RejectsMissingCalibrationAndInvalidGeometry)
{
    auto profile = homographyInput().profile;
    std::array<MarkerCorner, 4> corners{{{400, 300}, {600, 300}, {600, 500}, {400, 500}}};
    EXPECT_EQ(metricObservation({}, corners, {}).status, MetricObservationStatus::Uncalibrated);
    EXPECT_EQ(metricObservation({}, corners, profile).status, MetricObservationStatus::Uncalibrated);
    profile.homography = knownHomography();
    corners[0] = corners[1];
    EXPECT_EQ(metricObservation({}, corners, profile).status, MetricObservationStatus::InvalidGeometry);
}

TEST(MetricObservationTest, WorldHeadingUsesTransformedCornersAcrossQuarterTurns)
{
    auto profile = homographyInput().profile;
    profile.homography = knownHomography();
    std::array<MarkerCorner, 4> corners{{{400, 300}, {600, 300}, {600, 500}, {400, 500}}};
    const double expected[]{std::numbers::pi / 2, 0, -std::numbers::pi / 2, std::numbers::pi};
    for (double angle : expected) {
        const auto result = metricObservation({}, corners, profile);
        ASSERT_TRUE(result.measurement);
        EXPECT_NEAR(result.measurement->pose.heading.value, angle, 1e-12);
        std::rotate(corners.begin(), corners.begin() + 1, corners.end());
    }
}

TEST(MetricObservationTest, RejectsOutsideWorkspaceAndHorizonCrossing)
{
    auto profile = homographyInput().profile;
    profile.homography = knownHomography();
    std::array<MarkerCorner, 4> corners{{{0, 300}, {200, 300}, {200, 500}, {0, 500}}};
    EXPECT_EQ(metricObservation({}, corners, profile).status, MetricObservationStatus::OutsideWorkspace);
    profile.homography = WorkspaceHomography{{1, 0, 0, 0, 1, 0, 1, 0, -100}};
    EXPECT_EQ(metricObservation({}, corners, profile).status, MetricObservationStatus::InvalidGeometry);
}
