#include "ElevatedFixtures.hpp"
#include "calibration/OpenCvExtrinsicsSolver.hpp"
#include "vision/OpenCvMetricObservation.hpp"
#include <gtest/gtest.h>
using namespace micro_robot;
using namespace micro_robot::test;
TEST(OpenCvExtrinsicsTest, EstimatesFromMeasuredElevatedSurfaceInsteadOfNoisyRuntimeH) {
    auto p=elevatedProfile(); const auto truth=*p.localization; p.localization.reset();
    p.homography->imageToWorld[2]+=100; // Estimator must not decompose the runtime H.
    OpenCvExtrinsicsSolver solver;
    const auto result=solver.solve(p,{truth.survey,truth.mounts,"new-camera-r2"}); ASSERT_TRUE(result.calibration)<<result.error;
    for(std::size_t i=0;i<9;++i) EXPECT_NEAR(result.calibration->worldToCamera[i],truth.worldToCamera[i],1e-10);
    for(std::size_t i=0;i<3;++i) EXPECT_NEAR(result.calibration->translationMeters[i],truth.translationMeters[i],1e-10);
    EXPECT_TRUE(validateElevatedLocalization(*result.calibration,p).empty());
}
TEST(OpenCvExtrinsicsTest, RejectsBadHeldOutChecksAndCollinearOrUnknownSurvey) {
    auto p=elevatedProfile(); const auto truth=*p.localization; p.localization.reset();
    OpenCvExtrinsicsSolver solver; auto input=ExtrinsicSolveInput{truth.survey,truth.mounts,"new-r2"};
    input.survey.references.back().image.u+=30; EXPECT_FALSE(solver.solve(p,input).calibration);
    input.survey=truth.survey; input.survey.measurementMethod.clear(); EXPECT_FALSE(solver.solve(p,input).calibration);
    input.survey=truth.survey; for(auto& point:input.survey.references) point.world.y=Meters{0};
    EXPECT_FALSE(solver.solve(p,input).calibration);
}
TEST(OpenCvExtrinsicsTest, NativeCornersAreUndistortedExactlyOnceAndEachRobotUsesItsOwnHeight) {
    auto p=elevatedProfile();
    p.intrinsics.distortion={.08,-.01,.002,-.003,.001};
    p.localization->tabletopBinding=tabletopProfileBinding(p);
    auto other=p.localization->mounts[0]; other.robotId=RobotId{1}; other.tagHeightMeters=.07; other.revision="other-mount";
    p.localization->mounts.push_back(other);
    MarkerDetections input; input.width=p.intrinsics.width; input.height=p.intrinsics.height;
    for(int id:{0,1}) {
        const double height=id==0?.05:.07, x=id==0?-.3:.3;
        auto corners=elevatedCorners(x,.1,height,.7);
        for(auto& pixel:corners) {
            const double nx=(pixel.u-960)/1100,ny=(pixel.v-540)/1100,rr=nx*nx+ny*ny;
            const double radial=1+.08*rr-.01*rr*rr+.001*rr*rr*rr;
            pixel={960+1100*(nx*radial+2*.002*nx*ny-.003*(rr+2*nx*nx)),
                540+1100*(ny*radial+.002*(rr+2*ny*ny)-2*.003*nx*ny)};
        }
        input.markers.push_back({MarkerFamily::AprilTag36h11,id,corners,{}});
    }
    OpenCvMetricObservationConverter converter;
    const auto batch=converter.convert(input,p.intrinsics.sourceId,RobotMarkerMap{RobotMarkerProfile{MarkerFamily::AprilTag36h11,0,16,0}},p); ASSERT_EQ(batch.measurements.size(),2U)<<batch.error;
    for(const auto& m:batch.measurements) {
        const auto& mount=p.localization->mounts[m.robotId.value()];
        EXPECT_NEAR(m.pose.x.value,(m.robotId.value()==0?-.3:.3)+std::cos(.7)*mount.bodyForwardMeters-std::sin(.7)*mount.bodyLeftMeters,1e-8);
        EXPECT_NEAR(m.pose.y.value,.1+std::sin(.7)*mount.bodyForwardMeters+std::cos(.7)*mount.bodyLeftMeters,1e-8);
        EXPECT_EQ(m.reference,MetricPoseReference::AxleBodyCenter);
    }
}

TEST(OpenCvExtrinsicsTest, RejectsUnresolvedPlanarAmbiguityAndSupportsBoundedFullSurvey) {
    auto p=elevatedProfile(); auto truth=*p.localization; p.localization.reset();
    OpenCvExtrinsicsSolver solver;
    auto front=truth; front.worldToCamera={1,0,0,0,-1,0,0,0,-1}; front.translationMeters={0,0,.75};
    for(auto& ref:front.survey.references) ref.image=*projectReference(front,p.intrinsics,ref.world,front.survey.surfaceHeightMeters);
    const auto ambiguous=solver.solve(p,{front.survey,front.mounts,"front-r2"});
    EXPECT_FALSE(ambiguous.calibration); EXPECT_FALSE(ambiguous.error.empty());
    truth.survey.references.clear();
    for(int row=0;row<16;++row) for(int col=0;col<16;++col) {
        const WorldPoint2D point{Meters{-.44+.88*col/15},Meters{-.2+.4*row/15}};
        truth.survey.references.push_back({*projectReference(truth,p.intrinsics,point,truth.survey.surfaceHeightMeters),point,(row+col)%2==0});
    }
    ASSERT_EQ(truth.survey.references.size(),MaximumExtrinsicReferences);
    const auto full=solver.solve(p,{truth.survey,truth.mounts,"full-r2"}); ASSERT_TRUE(full.calibration)<<full.error;
    EXPECT_EQ(full.calibration->survey.references.size(),MaximumExtrinsicReferences);
    truth.survey.references.push_back(truth.survey.references.back());
    EXPECT_FALSE(solver.solve(p,{truth.survey,truth.mounts,"over-cap"}).calibration);
}
