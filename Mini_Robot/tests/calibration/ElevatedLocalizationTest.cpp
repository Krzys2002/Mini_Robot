#include "ElevatedFixtures.hpp"
#include <gtest/gtest.h>
#include <limits>
using namespace micro_robot;
using namespace micro_robot::test;

TEST(ElevatedLocalizationTest, KnownObliquePoseReprojectsMeasuredElevatedReferencesAndRejectsChangedBindings) {
    auto p=elevatedProfile(); ASSERT_TRUE(validateWorkspaceCalibrationProfile(p).empty())<<validateWorkspaceCalibrationProfile(p);
    const auto original=p;
    p.frameId="different"; EXPECT_FALSE(validateWorkspaceCalibrationProfile(p).empty()); p=original;
    p.intrinsics.cameraMatrix[0]+=1; EXPECT_FALSE(validateWorkspaceCalibrationProfile(p).empty()); p=original;
    p.intrinsics.distortion[0]=.1; EXPECT_FALSE(validateWorkspaceCalibrationProfile(p).empty()); p=original;
    ++p.intrinsics.width; EXPECT_FALSE(validateWorkspaceCalibrationProfile(p).empty()); p=original;
    p.homography->imageToWorld[0]+=1; EXPECT_FALSE(validateWorkspaceCalibrationProfile(p).empty()); p=original;
    p.localization->worldToCamera[0]=-1; EXPECT_FALSE(validateWorkspaceCalibrationProfile(p).empty());
}
TEST(ElevatedLocalizationTest, RayIntersectionMatchesTabletopHAtZeroAndHandlesSignedOriginsAndHomogeneousScaling) {
    auto p=elevatedProfile();
    for(double y:{-.23,0.,.23}) for(double x:{-.48,0.,.48}) {
        const auto pixel=projectReference(*p.localization,p.intrinsics,{Meters{x},Meters{y}},0); ASSERT_TRUE(pixel);
        const auto ray=intersectTagPlane(*p.localization,p.intrinsics,*pixel,0); ASSERT_TRUE(ray);
        const auto h=projectWorkspacePoint(*p.homography,*pixel); ASSERT_TRUE(h);
        EXPECT_NEAR(ray->x.value,x,1e-12); EXPECT_NEAR(ray->y.value,y,1e-12);
        EXPECT_NEAR(h->x.value,x,1e-12); EXPECT_NEAR(h->y.value,y,1e-12);
    }
    for(auto& v:p.homography->imageToWorld) v*=1e280;
    p.localization->tabletopBinding=tabletopProfileBinding(p);
    EXPECT_TRUE(validateWorkspaceCalibrationProfile(p).empty());
    const auto point=intersectTagPlane(*p.localization,p.intrinsics,{960,540},0); ASSERT_TRUE(point);
    EXPECT_NEAR(point->y.value,0,1e-12);
}
TEST(ElevatedLocalizationTest, DiagnosesUnknownOrInvalidSurveyMountsAmbiguityAndUnsupportedRays) {
    const auto original=elevatedProfile(); auto p=original;
    p.localization->mounts.clear(); EXPECT_FALSE(validateWorkspaceCalibrationProfile(p).empty()); p=original;
    p.localization->mounts.push_back(p.localization->mounts.front()); EXPECT_FALSE(validateWorkspaceCalibrationProfile(p).empty()); p=original;
    p.localization->mounts[0].tagHeightMeters=std::numeric_limits<double>::quiet_NaN(); EXPECT_FALSE(validateWorkspaceCalibrationProfile(p).empty()); p=original;
    p.localization->mounts[0].revision.clear(); EXPECT_FALSE(validateWorkspaceCalibrationProfile(p).empty()); p=original;
    p.localization->survey.surfaceHeightMeters=0; EXPECT_FALSE(validateWorkspaceCalibrationProfile(p).empty()); p=original;
    p.localization->survey.references[4].world.x=Meters{.1}; EXPECT_FALSE(validateWorkspaceCalibrationProfile(p).empty()); p=original;
    p.localization->alternativeGapPixels=.1; EXPECT_FALSE(validateWorkspaceCalibrationProfile(p).empty()); p=original;
    p.localization->translationMeters[2]=-1; EXPECT_FALSE(validateWorkspaceCalibrationProfile(p).empty()); p=original;
    EXPECT_FALSE(intersectTagPlane(*p.localization,p.intrinsics,{960,540},.75));
    // Camera ray whose world z direction is exactly horizontal.
    const double a=.45/std::hypot(.45,.75), b=.75/std::hypot(.45,.75);
    EXPECT_FALSE(intersectTagPlane(*p.localization,p.intrinsics,{960,540-1100*b/a},.05));
    EXPECT_FALSE(intersectTagPlane(*p.localization,p.intrinsics,{960,540-2200*b/a},.05));
    p.localization->survey.references.resize(MaximumExtrinsicReferences+1); EXPECT_FALSE(validateWorkspaceCalibrationProfile(p).empty());
    p=original; p.workspaceId.assign(MaximumWorkspaceProfileBytes, 'x');
    EXPECT_NO_THROW(EXPECT_FALSE(validateWorkspaceCalibrationProfile(p).empty()));
}
TEST(ElevatedLocalizationTest, V4RoundtripPreservesLegacyBindingAndRejectsMissingCorruptOrFutureGeometry) {
    const auto p=elevatedProfile(); const auto text=serializeWorkspaceProfile(p);
    EXPECT_TRUE(text.starts_with("micro_robot_workspace 4\n"));
    const auto loaded=parseWorkspaceProfile(text); ASSERT_TRUE(loaded.profile)<<loaded.error;
    ASSERT_TRUE(loaded.profile->localization); EXPECT_EQ(serializeWorkspaceProfile(*loaded.profile),text);
    auto legacy=p; legacy.localization.reset();
    const auto v3=serializeWorkspaceProfile(legacy); EXPECT_TRUE(v3.starts_with("micro_robot_workspace 3\n"));
    const auto old=parseWorkspaceProfile(v3); ASSERT_TRUE(old.profile); EXPECT_FALSE(old.profile->localization);
    EXPECT_EQ(tabletopProfileBinding(p),v3);
    auto broken=text; broken.replace(22,1,"9"); EXPECT_FALSE(parseWorkspaceProfile(broken).profile);
    EXPECT_FALSE(parseWorkspaceProfile(text.substr(0,text.find("localization "))).profile);
    EXPECT_FALSE(parseWorkspaceProfile(text+"extra").profile);
    EXPECT_FALSE(parseElevatedLocalization("micro_robot_elevated 1\n"));
    const auto geometry=serializeElevatedLocalization(*p.localization);
    EXPECT_FALSE(parseElevatedLocalization(geometry+"extra"));
}
