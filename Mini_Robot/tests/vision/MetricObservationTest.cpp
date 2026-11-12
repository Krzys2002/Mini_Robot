#include <cmath>
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

TEST(MetricObservationTest, PerspectiveObliqueOffCenterPosesMatchIndependentWorldConstruction)
{
    auto profile = homographyInput().profile;
    // Inverse of independently specified world-to-image pinhole plane mapping below.
    profile.homography = WorkspaceHomography{{.0013379310344827585, .00012413793103448277, -.2206896551724138,
        -.00012413793103448277, -.0010758620689655172, .7655172413793103,
        -.0002551724137931034, .00008275862068965517, 1}};
    for (const auto center : {std::array<double, 2>{.18, .15}, std::array<double, 2>{.78, .44}}) {
        for (double heading : {.37, -2.4, 3.13}) {
            std::array<MarkerCorner, 4> corners;
            const double fx = std::cos(heading), fy = std::sin(heading);
            const double rx = fy, ry = -fx;
            const double front[]{1, 1, -1, -1}, right[]{-1, 1, 1, -1};
            for (int i = 0; i < 4; ++i) {
                const double x = center[0] + .03 * (front[i]*fx + right[i]*rx);
                const double y = center[1] + .03 * (front[i]*fy + right[i]*ry);
                const double w = 1 + .2*x + .1*y;
                corners[i] = {(100 + 800*x + 100*y)/w, (700 + 50*x - 900*y)/w};
            }
            const auto result = metricObservation({}, corners, profile);
            ASSERT_TRUE(result.measurement);
            const auto& pose = result.measurement->pose;
            EXPECT_NEAR(std::hypot(pose.x.value-center[0], pose.y.value-center[1]), 0, 1e-12);
            EXPECT_NEAR(std::remainder(pose.heading.value-heading, 2*std::numbers::pi), 0, 1e-12);
            EXPECT_TRUE(std::isfinite(pose.heading.value));
        }
    }
}
TEST(MetricObservationTest, NearHorizonFiniteMappingAndSingularRejection)
{
    auto profile = homographyInput().profile;
    profile.bounds = WorkspaceBounds{Meters{1e9}, Meters{1e9}};
    profile.homography = WorkspaceHomography{{1,0,0, 0,1,0, 0, -.001, 1}};
    std::array<MarkerCorner,4> corners{{{20,998}, {22,998}, {22,999}, {20,999}}};
    auto result = metricObservation({}, corners, profile);
    ASSERT_TRUE(result.measurement);
    EXPECT_NEAR(result.measurement->pose.x.value, 14000, 1e-6);
    EXPECT_NEAR(result.measurement->pose.y.value, 998.5/.0015, 1e-5);
    EXPECT_NEAR(result.measurement->pose.heading.value, std::atan2(-250000., -5250.), 1e-10);
    corners[2].v = corners[3].v = 1000;
    EXPECT_EQ(metricObservation({}, corners, profile).status, MetricObservationStatus::InvalidGeometry);
    corners[2].v = corners[3].v = 1001;
    EXPECT_EQ(metricObservation({}, corners, profile).status, MetricObservationStatus::InvalidGeometry);
}

TEST(MetricObservationTest, InteriorOriginProducesNegativePositionsWithoutChangingHeading)
{
    auto p = homographyInput().profile;
    p.bounds = WorkspaceBounds{Meters{1}, Meters{0.6}, {Meters{-0.75}, Meters{-0.4}}};
    p.homography = knownHomography();
    p.homography->imageToWorld[2] -= 0.75;
    p.homography->imageToWorld[5] -= 0.4;
    const std::array<MarkerCorner, 4> corners{{{400,300}, {600,300}, {600,500}, {400,500}}};
    const auto result = metricObservation({}, corners, p);
    ASSERT_TRUE(result.measurement);
    EXPECT_NEAR(result.measurement->pose.x.value, -0.25, 1e-12);
    EXPECT_NEAR(result.measurement->pose.y.value, -0.1, 1e-12);
    EXPECT_NEAR(result.measurement->pose.heading.value, std::numbers::pi / 2, 1e-12);
}

TEST(MetricObservationTest, ProjectiveMetricMappingIsInvariantToLargeFiniteHomogeneousScale)
{
    auto profile = homographyInput().profile;
    const WorkspaceHomography h{{0.001,0,0, 0,0.001,0, 0,0.001,1}};
    profile.homography = h;
    const std::array<MarkerCorner,4> corners{{{400,300},{600,300},{600,500},{400,500}}};
    const auto expected = metricObservation({},corners,profile);
    ASSERT_TRUE(expected.measurement);
    for (double scale : {1.7e308,-1.7e308,1e-200}) {
        profile.homography = h;
        for (auto& value : profile.homography->imageToWorld) value *= scale;
        ASSERT_TRUE(validateWorkspaceCalibrationProfile(profile).empty());
        const auto actual = metricObservation({},corners,profile);
        ASSERT_TRUE(actual.measurement) << "scale " << scale;
        EXPECT_NEAR(actual.measurement->pose.x.value,expected.measurement->pose.x.value,1e-12);
        EXPECT_NEAR(actual.measurement->pose.y.value,expected.measurement->pose.y.value,1e-12);
        EXPECT_NEAR(actual.measurement->pose.heading.value,expected.measurement->pose.heading.value,1e-12);
    }
}

#include "../calibration/ObliqueWorkspaceFixtures.hpp"

TEST(MetricObservationObliqueTest, NearFarTagsHaveKnownMetricPositionAndWorldHeading)
{
    auto profile = obliqueWorkspaceInput().profile; profile.homography = obliqueHomography();
    for (double y : {0.1,1.1,2.3}) for (double heading : {0.0,0.7,-2.1}) {
        std::array<MarkerCorner,4> corners;
        const double front[]{1,1,-1,-1}, right[]{-1,1,1,-1};
        for (std::size_t i=0;i<corners.size();++i) {
            const double x = 0.4+0.025*(front[i]*std::cos(heading)+right[i]*std::sin(heading));
            const double wy = y+0.025*(front[i]*std::sin(heading)-right[i]*std::cos(heading));
            const auto image = obliqueImage({Meters{x},Meters{wy}}); corners[i] = {image.u,image.v};
        }
        const auto result = metricObservation({},corners,profile);
        ASSERT_TRUE(result.measurement);
        EXPECT_NEAR(result.measurement->pose.x.value,0.4,1e-12);
        EXPECT_NEAR(result.measurement->pose.y.value,y,1e-12);
        EXPECT_NEAR(std::remainder(result.measurement->pose.heading.value-heading,2*std::numbers::pi),0,1e-12);
    }
}
