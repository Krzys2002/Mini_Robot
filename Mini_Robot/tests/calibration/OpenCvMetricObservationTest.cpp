#include <cmath>
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

TEST(OpenCvMetricObservationTest, OffCenterObliqueTagWithRadialAndTangentialDistortion)
{
    auto profile = homographyInput().profile;
    profile.homography = knownHomography();
    profile.intrinsics.distortion = {.45, .12, .03, -.025, .04};
    MarkerDetections input;
    input.width = profile.intrinsics.width; input.height = profile.intrinsics.height;
    const double heading = -.63, fx = std::cos(heading), fy = std::sin(heading);
    const double front[]{1,1,-1,-1}, right[]{-1,1,1,-1};
    std::array<MarkerCorner,4> corners;
    for (int i=0; i<4; ++i) {
        const double wx = .2 + .025*(front[i]*fx + right[i]*fy);
        const double wy = .15 + .025*(front[i]*fy - right[i]*fx);
        const double x = (100 + 800*wx - 500)/800, y = (700 - 1000*wy - 400)/810;
        const double r2=x*x+y*y, radial=1+.45*r2+.12*r2*r2+.04*r2*r2*r2;
        corners[i] = {500+800*(x*radial+2*.03*x*y-.025*(r2+2*x*x)),
                      400+810*(y*radial+.03*(r2+2*y*y)-2*.025*x*y)};
    }
    input.markers.push_back({MarkerFamily::AprilTag36h11,0,corners,{}});
    OpenCvMetricObservationConverter converter;
    const auto batch=converter.convert(input,profile.intrinsics.sourceId,RobotMarkerMap{},profile);
    ASSERT_EQ(batch.measurements.size(),1U);
    const auto& pose=batch.measurements[0].pose;
    EXPECT_LT(std::hypot(pose.x.value-.2,pose.y.value-.15), 5e-5);
    EXPECT_LT(std::abs(std::remainder(pose.heading.value-heading,2*std::numbers::pi)), 2e-4);
    EXPECT_TRUE(std::isfinite(pose.x.value)); EXPECT_TRUE(std::isfinite(pose.y.value));
    EXPECT_TRUE(std::isfinite(pose.heading.value));
}
TEST(OpenCvMetricObservationTest, BoundedReasonsDistinguishEmptyRejectedAndInvalidFrames)
{
    auto profile=homographyInput().profile; profile.homography=knownHomography();
    profile.intrinsics.distortion={0,0,0,0,0};
    MarkerDetections input; input.width=profile.intrinsics.width; input.height=profile.intrinsics.height;
    OpenCvMetricObservationConverter converter;
    auto convert=[&]{return converter.convert(input,profile.intrinsics.sourceId,RobotMarkerMap{},profile);};
    auto empty=convert(); EXPECT_EQ(empty.status,MetricObservationStatus::Valid);
    EXPECT_EQ(empty.rejected.outsideWorkspace,0U);
    const std::array<MarkerCorner,4> outside{{{0,200},{30,200},{30,230},{0,230}}};
    input.markers.push_back({MarkerFamily::AprilTag36h11,0,outside,{}});
    input.markers.push_back({MarkerFamily::AprilTag36h11,1,{}, {}});
    input.markers.push_back({MarkerFamily::AprilTag36h11,2,outside,{}});
    input.markers.push_back(input.markers.back());
    input.markers.push_back({MarkerFamily::AprilTag36h11,100,outside,{}});
    auto rejected=convert(); EXPECT_EQ(rejected.status,MetricObservationStatus::Valid);
    EXPECT_TRUE(rejected.measurements.empty());
    EXPECT_EQ(rejected.rejected.outsideWorkspace,1U); EXPECT_EQ(rejected.rejected.invalidGeometry,1U);
    EXPECT_EQ(rejected.rejected.duplicateRobot,2U); EXPECT_EQ(rejected.rejected.unassignedId,1U);
    input.error="detector failure";
    EXPECT_EQ(convert().status,MetricObservationStatus::InvalidGeometry);
}

#include "ObliqueWorkspaceFixtures.hpp"

TEST(OpenCvMetricObservationObliqueTest, UndistortsStrongNearFarPerspectiveInNativeCameraGeometry)
{
    auto profile = obliqueWorkspaceInput().profile;
    profile.homography = obliqueHomography();
    profile.intrinsics.distortion = {0.08,-0.01,0.002,-0.003,0.001};
    OpenCvMetricObservationConverter converter;
    for (double y : {0.1,1.1,2.3}) {
        const double heading = 0.7;
        std::array<MarkerCorner,4> corners;
        const double front[]{1,1,-1,-1}, right[]{-1,1,1,-1};
        for (std::size_t i=0;i<corners.size();++i) {
            const auto ideal = obliqueImage({Meters{0.4+0.025*(front[i]*std::cos(heading)+right[i]*std::sin(heading))},
                Meters{y+0.025*(front[i]*std::sin(heading)-right[i]*std::cos(heading))}});
            const double x = (ideal.u-500)/400, ny = (ideal.v-300)/410;
            const double r2 = x*x+ny*ny, radial = 1+0.08*r2-0.01*r2*r2+0.001*r2*r2*r2;
            corners[i] = {500+400*(x*radial+2*0.002*x*ny-0.003*(r2+2*x*x)),
                300+410*(ny*radial+0.002*(r2+2*ny*ny)-2*0.003*x*ny)};
        }
        MarkerDetections input; input.width=1000; input.height=800;
        input.markers.push_back({MarkerFamily::AprilTag36h11,0,corners,{}});
        const auto result = converter.convert(input,profile.intrinsics.sourceId,RobotMarkerMap{},profile);
        ASSERT_EQ(result.measurements.size(),1);
        EXPECT_NEAR(result.measurements[0].pose.x.value,0.4,1e-7);
        EXPECT_NEAR(result.measurements[0].pose.y.value,y,1e-7);
        EXPECT_NEAR(result.measurements[0].pose.heading.value,heading,1e-6);
    }
}
