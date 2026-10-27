#include "HomographyFixtures.hpp"
#include "calibration/WorkspaceProfile.hpp"
#include <gtest/gtest.h>
#include <algorithm>
#include <limits>
namespace {
using namespace micro_robot;
using namespace micro_robot::test;
TEST(TabletopHomographyTest, KnownMappingAndIndependentResidualsUseMetres)
{
    auto input = homographyInput();
    auto result = evaluateWorkspaceHomography(input, knownHomography());
    ASSERT_TRUE(result.result) << result.error;
    EXPECT_LT(result.result->fit.rmsMeters, 1e-14);
    ASSERT_TRUE(result.result->check); EXPECT_LT(result.result->check->rmsMeters, 1e-14);
    input.checkPoints[0].world.x.value += 0.02;
    result = evaluateWorkspaceHomography(input, knownHomography());
    ASSERT_TRUE(result.result); EXPECT_NEAR(result.result->check->maximumMeters, 0.02, 1e-14);
    EXPECT_LT(result.result->fit.maximumMeters, 1e-14);
    input.checkPoints.clear(); result = evaluateWorkspaceHomography(input, knownHomography());
    ASSERT_TRUE(result.result); EXPECT_FALSE(result.result->check);
}
TEST(TabletopHomographyTest, RejectsBadCountsDuplicatesCrossingsCollinearityAndOutOfBounds)
{
    const auto valid = homographyInput();
    EXPECT_TRUE(validateWorkspaceHomographyInput(valid).empty());
    auto input = valid; input.fitPoints.pop_back(); EXPECT_FALSE(validateWorkspaceHomographyInput(input).empty());
    input = valid; input.fitPoints[1] = input.fitPoints[0]; EXPECT_FALSE(validateWorkspaceHomographyInput(input).empty());
    input = valid; std::swap(input.fitPoints[1], input.fitPoints[2]); EXPECT_FALSE(validateWorkspaceHomographyInput(input).empty());
    input = valid; input.fitPoints[1].world = {Meters{0.5}, Meters{0.3}}; EXPECT_FALSE(validateWorkspaceHomographyInput(input).empty());
    input = valid; input.fitPoints[0].image.u = -1; EXPECT_FALSE(validateWorkspaceHomographyInput(input).empty());
    input = valid; input.fitPoints[0].image.u = std::numeric_limits<double>::quiet_NaN(); EXPECT_FALSE(validateWorkspaceHomographyInput(input).empty());
    input = valid; input.fitPoints[0].world.x.value = 2; EXPECT_FALSE(validateWorkspaceHomographyInput(input).empty());
    input = valid; input.checkPoints = {input.fitPoints[0]}; EXPECT_FALSE(validateWorkspaceHomographyInput(input).empty());
    input = valid; input.checkPoints.push_back(input.checkPoints.front()); EXPECT_FALSE(validateWorkspaceHomographyInput(input).empty());
}
TEST(TabletopHomographyTest, EitherPerimeterWindingIsValidButHorizonsAndSingularMatricesAreRejected)
{
    auto input = homographyInput(); std::reverse(input.fitPoints.begin(), input.fitPoints.end());
    EXPECT_TRUE(validateWorkspaceHomographyInput(input).empty());
    EXPECT_FALSE(evaluateWorkspaceHomography(input, WorkspaceHomography{}).result);
    const WorkspaceHomography horizon{{1, 0, 0, 0, 1, 0, 1, 0, -500}};
    EXPECT_FALSE(evaluateWorkspaceHomography(input, horizon).result);
    EXPECT_FALSE(projectWorkspacePoint(horizon, {500, 0}));
    for (const double scale : {1e-100, -1.0, 1e100}) {
        auto h = knownHomography(); for (auto& value : h.imageToWorld) value *= scale;
        const auto point = projectWorkspacePoint(h, {500, 400}); ASSERT_TRUE(point);
        EXPECT_NEAR(point->x.value, 0.5, 1e-14); EXPECT_NEAR(point->y.value, 0.3, 1e-14);
    }
}
TEST(WorkspaceProfileTest, StrictRoundTripPreservesTransformOpticsAndUncalibratedState)
{
    auto profile = homographyInput().profile; profile.workspaceId = "table \"A\" / żółć";
    profile.homography = knownHomography();
    const auto text = serializeWorkspaceProfile(profile);
    auto parsed = parseWorkspaceProfile(text); ASSERT_TRUE(parsed.profile) << parsed.error;
    EXPECT_EQ(parsed.profile->workspaceId, profile.workspaceId);
    EXPECT_EQ(parsed.profile->homography->imageToWorld, profile.homography->imageToWorld);
    EXPECT_EQ(parsed.profile->intrinsics.cameraMatrix, profile.intrinsics.cameraMatrix);
    EXPECT_EQ(parsed.profile->intrinsics.distortion, profile.intrinsics.distortion);
    EXPECT_TRUE(workspaceCalibrationCompatibilityError(*parsed.profile, profile.intrinsics).empty());
    profile.homography.reset(); parsed = parseWorkspaceProfile(serializeWorkspaceProfile(profile));
    ASSERT_TRUE(parsed.profile); EXPECT_FALSE(parsed.profile->homography);
}
TEST(WorkspaceProfileTest, RejectsUnknownVersionsTruncationGarbageOversizeAndInvalidNumbers)
{
    auto p = homographyInput().profile; p.homography = knownHomography();
    const auto text = serializeWorkspaceProfile(p);
    for (const auto& [from, to] : std::vector<std::pair<std::string, std::string>>{
        {"workspace 1", "workspace -1"}, {"dimensions 1", "dimensions nan"},
        {"undistorted_native_k", "raw_distorted"}, {"homography 1", "homography -1"}}) {
        auto bad = text; const auto pos = bad.find(from); ASSERT_NE(pos, std::string::npos); bad.replace(pos, from.size(), to);
        EXPECT_FALSE(parseWorkspaceProfile(bad).profile) << from;
    }
    EXPECT_FALSE(parseWorkspaceProfile(text + "garbage").profile);
    EXPECT_FALSE(parseWorkspaceProfile(text.substr(0, text.size() / 2)).profile);
    EXPECT_FALSE(parseWorkspaceProfile(std::string(MaximumWorkspaceProfileBytes + 1, 'x')).profile);
    p.workspaceId.clear(); EXPECT_THROW(serializeWorkspaceProfile(p), std::invalid_argument);
}
}
