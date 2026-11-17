#include "../calibration/ElevatedFixtures.hpp"
#include "common/Angle.hpp"
#include <gtest/gtest.h>
#include <numbers>
using namespace micro_robot;
using namespace micro_robot::test;
TEST(ElevatedMetricObservationTest, CorrectsNearFarEdgesInteriorMultipleHeightsAndRotatedOffsetMounts) {
    for(double height:{.03,.05,.07}) for(double x:{-.45,0.,.45}) for(double y:{-.20,0.,.20}) for(double heading:{0.,.7,-2.4,3.13}) {
        auto p=elevatedProfile(); auto& m=p.localization->mounts[0]; m.tagHeightMeters=height;
        const auto corners=elevatedCorners(x,y,height,heading);
        RobotObservation o{}; o.robotId=RobotId{0}; o.frameSequence=42; o.capturedAt=Timestamp{}+std::chrono::seconds{3};
        const auto result=metricObservation(o,corners,p);
        ASSERT_TRUE(result.measurement)<<static_cast<int>(result.status)<<" at "<<x<<','<<y;
        const auto& measurement=*result.measurement;
        EXPECT_NEAR(measurement.pose.x.value,x+std::cos(heading)*m.bodyForwardMeters-std::sin(heading)*m.bodyLeftMeters,1e-12);
        EXPECT_NEAR(measurement.pose.y.value,y+std::sin(heading)*m.bodyForwardMeters+std::cos(heading)*m.bodyLeftMeters,1e-12);
        EXPECT_NEAR(measurement.pose.heading.value,normalizeAngle(Radians{heading+m.bodyHeadingOffsetRadians}).value,1e-12);
        EXPECT_EQ(measurement.reference,MetricPoseReference::AxleBodyCenter);
        EXPECT_EQ(measurement.mountingRevision,m.revision); EXPECT_EQ(measurement.calibrationRevision,p.localization->revision);
        EXPECT_EQ(measurement.worldFrameId,p.frameId); EXPECT_EQ(measurement.cameraId,p.intrinsics.sourceId);
        EXPECT_EQ(measurement.frameSequence,42U); EXPECT_EQ(measurement.capturedAt,o.capturedAt);
    }
}
TEST(ElevatedMetricObservationTest, UnknownMountAndOutsideTagOrBodyNeverProduceCorrectedPose) {
    auto p=elevatedProfile(); auto corners=elevatedCorners(0,0,.05,0);
    RobotObservation unknown{}; unknown.robotId=RobotId{8};
    EXPECT_EQ(metricObservation(unknown,corners,p).status,MetricObservationStatus::UnknownMount);
    p.localization->mounts[0].bodyForwardMeters=2;
    EXPECT_EQ(metricObservation({},corners,p).status,MetricObservationStatus::OutsideWorkspace);
    p=elevatedProfile(); corners=elevatedCorners(.495,0,.05,0);
    EXPECT_EQ(metricObservation({},corners,p).status,MetricObservationStatus::OutsideWorkspace);
    p.localization.reset();
    const auto legacy=metricObservation({},elevatedCorners(0,0,.05,0),p); ASSERT_TRUE(legacy.measurement);
    EXPECT_EQ(legacy.measurement->reference,MetricPoseReference::TabletopTagCenter);
    EXPECT_GT(std::abs(legacy.measurement->pose.y.value),.01); // Explicit parallax, no body-pose claim.
}
