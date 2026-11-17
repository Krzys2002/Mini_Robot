#include "ReferenceFixtures.hpp"
#include "calibration/OpenCvExtrinsicsSolver.hpp"
#include "vision/OpenCvMetricObservation.hpp"
#include <gtest/gtest.h>
using namespace micro_robot;
using namespace micro_robot::test;
using namespace std::chrono_literals;
namespace {
MarkerDetections detections(const WorkspaceCalibrationProfile& p,Timestamp now=Timestamp{}+10s,const ElevatedLocalization* actual=nullptr) {
    auto f=referenceFrame(p,now,actual); MarkerDetections d; d.width=p.intrinsics.width; d.height=p.intrinsics.height;
    d.capturedAt=now; d.frameSequence=42; d.inputGeneration=7;
    for(const auto& row:f.observations) d.markers.push_back({MarkerFamily::AprilTag36h11,row.id,row.raw,{}});
    return d;
}
}
TEST(OpenCvReferenceMarkersTest, MixedHeightOutsideReferencesEstimatePoseAndPreserveNativeAcquisition) {
    auto p=referenceProfile(); OpenCvMetricObservationConverter converter;
    const auto frame=converter.referenceObservations(detections(p),p.intrinsics.sourceId,RobotMarkerMap{},p); ASSERT_TRUE(frame);
    ASSERT_TRUE(frame->error.empty())<<frame->error;
    for(const auto& row:frame->observations) EXPECT_EQ(row.rejection,ReferenceRejection::Included);
    std::string error; auto input=referenceExtrinsicInput(*frame,*p.references,p,"reference-r2",error); ASSERT_TRUE(input)<<error;
    OpenCvExtrinsicsSolver solver; const auto solved=solver.solve(p,*input); ASSERT_TRUE(solved.calibration)<<solved.error;
    for(std::size_t i=0;i<9;++i) EXPECT_NEAR(solved.calibration->worldToCamera[i],p.localization->worldToCamera[i],1e-8);
    for(std::size_t i=0;i<3;++i) EXPECT_NEAR(solved.calibration->translationMeters[i],p.localization->translationMeters[i],1e-8);
    p.localization=solved.calibration; p.references->acquisition=*frame; p.references->validatedLocalizationRevision=p.localization->revision;
    EXPECT_TRUE(validateWorkspaceCalibrationProfile(p).empty())<<validateWorkspaceCalibrationProfile(p);
}
TEST(OpenCvReferenceMarkersTest, ExplicitMissingDuplicateGeometryAndRoleConflictsNeverCreateReferenceRobotTracks) {
    auto p=commissionedReferenceProfile(); OpenCvMetricObservationConverter converter; auto d=detections(p);
    d.markers.erase(d.markers.begin()); d.markers.push_back(d.markers[0]);
    d.markers[2].corners[0]=d.markers[2].corners[1];
    auto frame=converter.referenceObservations(d,p.intrinsics.sourceId,RobotMarkerMap{},p); ASSERT_TRUE(frame);
    EXPECT_EQ(frame->observations[0].rejection,ReferenceRejection::Missing);
    EXPECT_EQ(frame->observations[1].rejection,ReferenceRejection::Duplicate);
    EXPECT_EQ(frame->observations[3].rejection,ReferenceRejection::InvalidCorners);
    const auto batch=converter.convert(d,p.intrinsics.sourceId,RobotMarkerMap{},p);
    EXPECT_TRUE(batch.measurements.empty()); // References never map to RobotIds.
    auto bad=p; bad.references->markers[0].id=0;
    EXPECT_EQ(converter.convert(d,bad.intrinsics.sourceId,RobotMarkerMap{},bad).status,MetricObservationStatus::InvalidCalibration);
    EXPECT_FALSE(converter.referenceObservations(d,"other camera",RobotMarkerMap{},p)->error.empty());
}
TEST(OpenCvReferenceMarkersTest, BoundedMonitorCadenceWithholdsRobotMeasurementsOnDriftAndRetainsLatch) {
    auto p=commissionedReferenceProfile(); auto robotMount=p.localization->mounts[0]; robotMount.robotId=RobotId{1}; p.localization->mounts={robotMount};
    OpenCvMetricObservationConverter converter;
    auto d=detections(p);
    std::array<MarkerCorner,4> robot;
    const double front[]{1,1,-1,-1},right[]{-1,1,1,-1};
    for(std::size_t i=0;i<4;++i) { auto pixel=projectReference(*p.localization,p.intrinsics,{Meters{.016*front[i]},Meters{-.016*right[i]}},.05); robot[i]={pixel->u,pixel->v}; }
    d.markers.push_back({MarkerFamily::AprilTag36h11,0,robot,{}});
    auto healthy=converter.convert(d,p.intrinsics.sourceId,RobotMarkerMap{},p); ASSERT_EQ(healthy.measurements.size(),1U)<<healthy.error;
    EXPECT_EQ(healthy.placement->state,PlacementConsistency::Consistent);
    d.capturedAt+=100ms;
    auto skipped=converter.convert(d,p.intrinsics.sourceId,RobotMarkerMap{},p); EXPECT_FALSE(skipped.references); EXPECT_TRUE(skipped.placement->permitsMeasurements);
    auto moved=*p.localization; moved.translationMeters[0]+=.012;
    for(int i=0;i<3;++i) {
        d=detections(p,Timestamp{}+10s+200ms*(i+1),&moved); d.markers.push_back({MarkerFamily::AprilTag36h11,0,robot,{}});
        const auto bad=converter.convert(d,p.intrinsics.sourceId,RobotMarkerMap{},p); EXPECT_TRUE(bad.measurements.empty());
        EXPECT_EQ(bad.status,i==2?MetricObservationStatus::PlacementDrift:MetricObservationStatus::PlacementUnverified);
    }
    d=detections(p,Timestamp{}+11s); d.inputGeneration=8;
    EXPECT_EQ(converter.convert(d,p.intrinsics.sourceId,RobotMarkerMap{},p).status,MetricObservationStatus::PlacementDrift);
}

TEST(OpenCvReferenceMarkersTest, UndistortsNativeReferenceCornersOnceAndHandlesCanonicalRotation) {
    auto p=referenceProfile(); p.intrinsics.distortion={.03,-.004,.001,-.001,.0002};
    p.localization->tabletopBinding=tabletopProfileBinding(p); p.references->tabletopBinding=tabletopProfileBinding(p);
    for(auto& reference:p.localization->survey.references) reference.image=*projectReference(*p.localization,p.intrinsics,reference.world,p.localization->survey.surfaceHeightMeters);
    std::rotate(p.references->markers[0].corners.begin(),p.references->markers[0].corners.begin()+1,p.references->markers[0].corners.end());
    const auto ideal=detections(p); auto raw=ideal;
    for(auto& m:raw.markers) for(auto& pixel:m.corners) {
        const double x=(pixel.u-960)/700,y=(pixel.v-540)/700,r2=x*x+y*y;
        const double radial=1+.03*r2-.004*r2*r2+.0002*r2*r2*r2;
        pixel={960+700*(x*radial+.002*x*y-.001*(r2+2*x*x)),540+700*(y*radial+.001*(r2+2*y*y)-.002*x*y)};
    }
    OpenCvMetricObservationConverter converter;
    const auto frame=converter.referenceObservations(raw,p.intrinsics.sourceId,RobotMarkerMap{},p); ASSERT_TRUE(frame); ASSERT_TRUE(frame->error.empty())<<frame->error;
    for(std::size_t i=0;i<frame->observations.size();++i) for(std::size_t j=0;j<4;++j) {
        EXPECT_EQ(frame->observations[i].rejection,ReferenceRejection::Included);
        EXPECT_NEAR(frame->observations[i].undistorted[j].u,ideal.markers[i].corners[j].u,1e-5);
        EXPECT_NEAR(frame->observations[i].undistorted[j].v,ideal.markers[i].corners[j].v,1e-5);
        EXPECT_DOUBLE_EQ(frame->observations[i].raw[j].u,raw.markers[i].corners[j].u);
    }
}

TEST(OpenCvReferenceMarkersTest, DetectorErrorBreaksPendingDriftConfirmationEvenBetweenAssessmentCadences) {
    const auto p=commissionedReferenceProfile(); auto moved=*p.localization; moved.translationMeters[0]+=.012;
    OpenCvMetricObservationConverter converter;
    const auto convert=[&](MarkerDetections frame){return converter.convert(frame,p.intrinsics.sourceId,RobotMarkerMap{},p);};
    EXPECT_EQ(convert(detections(p,Timestamp{}+10s,&moved)).placement->state,PlacementConsistency::SuspectedDrift);
    EXPECT_EQ(convert(detections(p,Timestamp{}+10s+200ms,&moved)).placement->state,PlacementConsistency::SuspectedDrift);
    auto error=detections(p,Timestamp{}+10s+250ms); error.error="detector failed";
    const auto unavailable=convert(error); EXPECT_TRUE(unavailable.measurements.empty());
    EXPECT_TRUE(unavailable.placement);
    if(unavailable.placement) { EXPECT_EQ(unavailable.placement->state,PlacementConsistency::InsufficientEvidence); EXPECT_FALSE(unavailable.placement->permitsMeasurements); }
    EXPECT_NE(convert(detections(p,Timestamp{}+10s+400ms,&moved)).status,MetricObservationStatus::PlacementDrift);
    EXPECT_EQ(convert(detections(p,Timestamp{}+10s+450ms,&moved)).placement->state,PlacementConsistency::SuspectedDrift);
    EXPECT_EQ(convert(detections(p,Timestamp{}+10s+650ms,&moved)).placement->state,PlacementConsistency::SuspectedDrift);
    EXPECT_EQ(convert(detections(p,Timestamp{}+10s+850ms,&moved)).placement->state,PlacementConsistency::ConfirmedDrift);
    error.capturedAt=Timestamp{}+10s+900ms;
    const auto latched=convert(error); ASSERT_TRUE(latched.placement);
    EXPECT_EQ(latched.placement->state,PlacementConsistency::ConfirmedDrift); EXPECT_FALSE(latched.placement->permitsMeasurements);
}
