#include "ReferenceFixtures.hpp"
#include <gtest/gtest.h>
#include <algorithm>
using namespace micro_robot;
using namespace micro_robot::test;
using namespace std::chrono_literals;
TEST(ReferenceMarkersTest, SeparateRolesOutsideTravelBoundsAndMeasuredHeightsProduceHeldOutCorrespondences) {
    auto p=referenceProfile(); ASSERT_TRUE(validateWorkspaceCalibrationProfile(p).empty())<<validateWorkspaceCalibrationProfile(p);
    const auto bounds=p.bounds;
    for(const auto& m:p.references->markers) EXPECT_FALSE(p.bounds.contains(m.corners[0]));
    const auto f=referenceFrame(p); std::string error;
    const auto input=referenceExtrinsicInput(f,*p.references,p,"new-r2",error); ASSERT_TRUE(input)<<error;
    EXPECT_EQ(input->survey.references.size(),32U);
    EXPECT_EQ(std::count_if(input->survey.references.begin(),input->survey.references.end(),[](auto& r){return r.check;}),16);
    EXPECT_EQ(input->survey.references[0].heightMeters,p.references->markers[0].surfaceHeightMeters);
    EXPECT_NE(input->survey.references[0].heightMeters,input->survey.references[4].heightMeters);
    EXPECT_TRUE(p.bounds==bounds);
}
TEST(ReferenceMarkersTest, RejectsRobotConflictsDuplicatesWrongCornersMissingUncertaintyAndUnconfirmedChecks) {
    const auto original=referenceProfile(); auto p=original;
    p.references->markers[0].id=0; EXPECT_FALSE(validateWorkspaceCalibrationProfile(p).empty()); p=original;
    p.references->markers[1].id=p.references->markers[0].id; EXPECT_FALSE(validateWorkspaceCalibrationProfile(p).empty()); p=original;
    std::swap(p.references->markers[0].corners[0],p.references->markers[0].corners[1]); EXPECT_FALSE(validateWorkspaceCalibrationProfile(p).empty()); p=original;
    p.references->markers[0].standardUncertaintyMeters=0; EXPECT_FALSE(validateWorkspaceCalibrationProfile(p).empty()); p=original;
    p.references->markers[4].independentlySurveyed=false; EXPECT_FALSE(validateWorkspaceCalibrationProfile(p).empty()); p=original;
    p.references->policy.cadence=0ms; EXPECT_FALSE(validateWorkspaceCalibrationProfile(p).empty()); p=original;
    p.references->tabletopBinding="other camera"; EXPECT_FALSE(validateWorkspaceCalibrationProfile(p).empty());
}
TEST(ReferenceMarkersTest, ClusterOcclusionAndMismatchedFrameCannotEstablishSupport) {
    auto p=referenceProfile(); auto f=referenceFrame(p); std::string error;
    f.observations[0].rejection=ReferenceRejection::Missing;
    EXPECT_FALSE(referenceExtrinsicInput(f,*p.references,p,"new",error)); EXPECT_TRUE(error.find("surround")!=std::string::npos);
    f=referenceFrame(p); f.registryRevision="old"; EXPECT_FALSE(referenceExtrinsicInput(f,*p.references,p,"new",error));
    EXPECT_FALSE(referenceSupportCoversWorkspace({{Meters{0},Meters{0}},{Meters{.1},Meters{0}},{Meters{0},Meters{.1}}},p.bounds));
    f=referenceFrame(p); f.cameraBinding="old optics"; EXPECT_FALSE(validateReferenceFrame(f,*p.references,p).empty());
}
TEST(ReferenceMarkersTest, V5RetainsOriginalReferenceObservationsRevisionsUncertaintyAndLegacyProfileMeaning) {
    const auto p=commissionedReferenceProfile(); ASSERT_TRUE(validateWorkspaceCalibrationProfile(p).empty())<<validateWorkspaceCalibrationProfile(p);
    const auto text=serializeWorkspaceProfile(p); EXPECT_TRUE(text.starts_with("micro_robot_workspace 5\n"));
    const auto loaded=parseWorkspaceProfile(text); ASSERT_TRUE(loaded.profile)<<loaded.error;
    EXPECT_EQ(serializeWorkspaceProfile(*loaded.profile),text);
    EXPECT_EQ(loaded.profile->references->acquisition->inputCalibrationRevision,"camera-pose-r1");
    EXPECT_EQ(loaded.profile->references->markers[0].standardUncertaintyMeters,.0005);
    EXPECT_EQ(loaded.profile->localization->survey.references[0].heightMeters,p.localization->survey.references[0].heightMeters);
    auto legacy=p; legacy.references.reset(); legacy.localization=referenceProfile().localization;
    const auto old=serializeWorkspaceProfile(legacy); EXPECT_TRUE(old.starts_with("micro_robot_workspace 4\n"));
    const auto restored=parseWorkspaceProfile(old); ASSERT_TRUE(restored.profile); EXPECT_FALSE(restored.profile->references);
    EXPECT_EQ(tabletopProfileBinding(p),tabletopProfileBinding(legacy));
    EXPECT_FALSE(parseReferenceConfiguration(serializeReferenceConfiguration(*p.references)+"garbage"));
    auto changed=p; changed.frameId="new frame"; EXPECT_FALSE(validateWorkspaceCalibrationProfile(changed).empty());
}
TEST(ReferenceMonitorTest, CadenceOcclusionAndExpiryNeverCertifyHealthOrMovement) {
    auto p=commissionedReferenceProfile(); const auto& c=*p.references; ReferenceMonitor monitor;
    monitor.configure(c,p.localization->revision,7); EXPECT_FALSE(monitor.status(Timestamp{}+10s,c.policy).permitsMeasurements);
    auto f=referenceFrame(p); auto result=monitor.observe(f,c,p); EXPECT_EQ(result.state,PlacementConsistency::Consistent); EXPECT_TRUE(result.permitsMeasurements);
    EXPECT_FALSE(monitor.due(f.capturedAt+100ms)); EXPECT_TRUE(monitor.due(f.capturedAt+200ms));
    EXPECT_FALSE(monitor.status(f.capturedAt+1s,c.policy).permitsMeasurements);
    f.capturedAt+=200ms; for(auto& row:f.observations) row.rejection=ReferenceRejection::Missing;
    result=monitor.observe(f,c,p); EXPECT_EQ(result.state,PlacementConsistency::InsufficientEvidence); EXPECT_FALSE(result.permitsMeasurements);
    EXPECT_EQ(result.visible,0U);
}
TEST(ReferenceMonitorTest, IndividualMovedReferenceIsNotConfirmedGlobalDrift) {
    auto p=commissionedReferenceProfile(); ReferenceMonitor monitor; const auto& c=*p.references;
    monitor.configure(c,p.localization->revision,7);
    auto f=referenceFrame(p); for(auto& corner:f.observations[8].undistorted) corner.u+=10;
    for(int i=0;i<8;++i) {
        const auto result=monitor.observe(f,c,p); EXPECT_EQ(result.state,PlacementConsistency::IndividualOutlier); EXPECT_EQ(result.outliers,1U); EXPECT_FALSE(result.permitsMeasurements);
        f.capturedAt+=200ms;
    }
}
TEST(ReferenceMonitorTest, PersistentCoherentDriftLatchesAcrossRecoveryOcclusionAndSourceRestartUntilRevalidation) {
    auto p=commissionedReferenceProfile(); ReferenceMonitor monitor; const auto& c=*p.references;
    monitor.configure(c,p.localization->revision,7);
    auto moved=*p.localization; moved.translationMeters[0]+=.012;
    auto f=referenceFrame(p,Timestamp{}+10s,&moved);
    EXPECT_EQ(monitor.observe(f,c,p).state,PlacementConsistency::SuspectedDrift);
    f.capturedAt+=200ms; EXPECT_EQ(monitor.observe(f,c,p).state,PlacementConsistency::SuspectedDrift);
    f.capturedAt+=200ms; EXPECT_EQ(monitor.observe(f,c,p).state,PlacementConsistency::ConfirmedDrift);
    f=referenceFrame(p,f.capturedAt+200ms); EXPECT_EQ(monitor.observe(f,c,p).state,PlacementConsistency::ConfirmedDrift);
    monitor.configure(c,p.localization->revision,8); f.capturedAt+=200ms;
    for(auto& row:f.observations) row.rejection=ReferenceRejection::Missing;
    EXPECT_EQ(monitor.observe(f,c,p).state,PlacementConsistency::ConfirmedDrift);
    auto revalidated=p; revalidated.localization->revision="explicit-revalidation-r3"; revalidated.references->validatedLocalizationRevision=revalidated.localization->revision;
    monitor.configure(*revalidated.references,revalidated.localization->revision,9);
    const auto valid=monitor.observe(referenceFrame(revalidated,f.capturedAt+200ms),*revalidated.references,revalidated);
    EXPECT_EQ(valid.state,PlacementConsistency::Consistent); EXPECT_TRUE(valid.permitsMeasurements);
}
TEST(ReferenceMonitorTest, IntermittentDriftWithoutPersistenceOrWideSupportRemainsUnconfirmed) {
    auto p=commissionedReferenceProfile(); auto& c=*p.references; ReferenceMonitor monitor; monitor.configure(c,p.localization->revision,7);
    auto moved=*p.localization; moved.translationMeters[0]+=.01;
    auto f=referenceFrame(p,Timestamp{}+10s,&moved); EXPECT_EQ(monitor.observe(f,c,p).state,PlacementConsistency::SuspectedDrift);
    f=referenceFrame(p,f.capturedAt+200ms); EXPECT_EQ(monitor.observe(f,c,p).state,PlacementConsistency::Consistent);
    f=referenceFrame(p,f.capturedAt+200ms,&moved); EXPECT_EQ(monitor.observe(f,c,p).state,PlacementConsistency::SuspectedDrift);
    f.capturedAt+=2s; EXPECT_EQ(monitor.observe(f,c,p).state,PlacementConsistency::SuspectedDrift);
}

TEST(ReferenceMonitorTest, CameraRotationWithOppositeResidualDirectionsStillConfirmsWidespreadInconsistency) {
    const auto p=commissionedReferenceProfile(); const auto& c=*p.references;
    ReferenceMonitor monitor; monitor.configure(c,p.localization->revision,7);
    auto rotated=*p.localization; const double angle=.02,co=std::cos(angle),si=std::sin(angle);
    for(std::size_t j=0;j<3;++j) {
        rotated.worldToCamera[j]=co*p.localization->worldToCamera[j]-si*p.localization->worldToCamera[3+j];
        rotated.worldToCamera[3+j]=si*p.localization->worldToCamera[j]+co*p.localization->worldToCamera[3+j];
    }
    rotated.translationMeters[0]=co*p.localization->translationMeters[0]-si*p.localization->translationMeters[1];
    rotated.translationMeters[1]=si*p.localization->translationMeters[0]+co*p.localization->translationMeters[1];
    auto f=referenceFrame(p,Timestamp{}+10s,&rotated);
    EXPECT_EQ(monitor.observe(f,c,p).state,PlacementConsistency::SuspectedDrift);
    f.capturedAt+=200ms; EXPECT_EQ(monitor.observe(f,c,p).state,PlacementConsistency::SuspectedDrift);
    f.capturedAt+=200ms; const auto confirmed=monitor.observe(f,c,p);
    EXPECT_EQ(confirmed.state,PlacementConsistency::ConfirmedDrift); EXPECT_FALSE(confirmed.permitsMeasurements);
}

TEST(ReferenceMarkersTest, NativeImageClusterCannotClaimCoverageDespiteWideWorldSurvey) {
    auto p=referenceProfile(); auto f=referenceFrame(p);
    // Keep individual tags valid and sampled, but collapse their centre distribution.
    for(std::size_t i=0;i<f.observations.size();++i) {
        auto& row=f.observations[i]; const double u=900+8*(i%4),v=500+8*(i/4);
        row.raw=row.undistorted={MarkerCorner{u,v},MarkerCorner{u+5,v},MarkerCorner{u+5,v+5},MarkerCorner{u,v+5}};
    }
    std::string error; EXPECT_FALSE(referenceExtrinsicInput(f,*p.references,p,"cluster",error));
    EXPECT_NE(error.find("image support"),std::string::npos);
}

TEST(ReferenceMarkersTest, BoundedFullRegistryRoundtripAndOverCapacityFailAtomically) {
    auto p=referenceProfile(); auto& c=*p.references;
    for(int i=0;i<20;++i) {
        auto marker=c.markers.back(); marker.id=200+i; marker.role=ReferenceRole::DriftMonitor;
        const double x=-.7+.07*i,y=.46,h=marker.blackEdgeMeters/2;
        marker.corners={WorldPoint2D{Meters{x+h},Meters{y+h}},WorldPoint2D{Meters{x+h},Meters{y-h}},WorldPoint2D{Meters{x-h},Meters{y-h}},WorldPoint2D{Meters{x-h},Meters{y+h}}};
        c.markers.push_back(marker);
    }
    ASSERT_EQ(c.markers.size(),MaximumReferenceMarkers);
    c.acquisition=referenceFrame(p);
    ASSERT_TRUE(validateWorkspaceCalibrationProfile(p).empty())<<validateWorkspaceCalibrationProfile(p);
    const auto text=serializeWorkspaceProfile(p); EXPECT_LE(text.size(),MaximumWorkspaceProfileBytes);
    const auto restored=parseWorkspaceProfile(text); ASSERT_TRUE(restored.profile)<<restored.error;
    EXPECT_EQ(restored.profile->references->acquisition->observations.size(),MaximumReferenceMarkers);
    c.markers.push_back(c.markers.back()); EXPECT_FALSE(validateWorkspaceCalibrationProfile(p).empty());
    EXPECT_THROW((void)serializeWorkspaceProfile(p),std::invalid_argument);
}

TEST(ReferenceMarkersTest, SameXYAtDifferentMeasuredHeightsAreDifferentThreeDimensionalReferences) {
    auto p=referenceProfile(); auto survey=p.localization->survey;
    auto& check=survey.references[4]; check.world=survey.references[0].world; check.heightMeters=.04;
    check.image=*projectReference(*p.localization,p.intrinsics,check.world,*check.heightMeters);
    EXPECT_TRUE(validateExtrinsicSurvey(survey,p.intrinsics).empty());
    check.heightMeters=survey.surfaceHeightMeters; check.image=survey.references[0].image;
    EXPECT_FALSE(validateExtrinsicSurvey(survey,p.intrinsics).empty());
}
