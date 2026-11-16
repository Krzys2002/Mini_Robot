#include "HomographyFixtures.hpp"
#include "calibration/WorkspaceProfile.hpp"
#include <algorithm>
#include <gtest/gtest.h>
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
    ASSERT_TRUE(result.result->check);
    EXPECT_LT(result.result->check->rmsMeters, 1e-14);
    input.checkPoints[0].world.x.value += 0.02;
    result = evaluateWorkspaceHomography(input, knownHomography());
    ASSERT_TRUE(result.result);
    EXPECT_NEAR(result.result->check->maximumMeters, 0.02, 1e-14);
    EXPECT_LT(result.result->fit.maximumMeters, 1e-14);
    input.checkPoints.clear();
    result = evaluateWorkspaceHomography(input, knownHomography());
    ASSERT_TRUE(result.result);
    EXPECT_FALSE(result.result->check);
}

TEST(TabletopHomographyTest, RejectsBadCountsDuplicatesCollinearityAndOutOfBounds)
{
    const auto valid = homographyInput();
    EXPECT_TRUE(validateWorkspaceHomographyInput(valid).empty());
    auto input = valid;
    input.fitPoints.pop_back();
    EXPECT_FALSE(validateWorkspaceHomographyInput(input).empty());
    input = valid;
    input.fitPoints[1] = input.fitPoints[0];
    EXPECT_FALSE(validateWorkspaceHomographyInput(input).empty());
    input = valid;
    std::swap(input.fitPoints[1], input.fitPoints[2]);
    EXPECT_TRUE(validateWorkspaceHomographyInput(input).empty());
    input = valid;
    for (auto& p : input.fitPoints)
        p.world.y = Meters{0.3};
    EXPECT_FALSE(validateWorkspaceHomographyInput(input).empty());
    input = valid;
    input.fitPoints[0].image.u = -1;
    EXPECT_FALSE(validateWorkspaceHomographyInput(input).empty());
    input = valid;
    input.fitPoints[0].image.u = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(validateWorkspaceHomographyInput(input).empty());
    input = valid;
    input.fitPoints[0].world.x.value = 2;
    EXPECT_FALSE(validateWorkspaceHomographyInput(input).empty());
    input = valid;
    input.checkPoints = {input.fitPoints[0]};
    EXPECT_FALSE(validateWorkspaceHomographyInput(input).empty());
    input = valid;
    input.checkPoints.push_back(input.checkPoints.front());
    EXPECT_FALSE(validateWorkspaceHomographyInput(input).empty());
}

TEST(TabletopHomographyTest, EitherPerimeterWindingIsValidButHorizonsAndSingularMatricesAreRejected)
{
    auto input = homographyInput();
    std::reverse(input.fitPoints.begin(), input.fitPoints.end());
    EXPECT_TRUE(validateWorkspaceHomographyInput(input).empty());
    EXPECT_FALSE(evaluateWorkspaceHomography(input, WorkspaceHomography{}).result);
    const WorkspaceHomography horizon{{1, 0, 0, 0, 1, 0, 1, 0, -500}};
    EXPECT_FALSE(evaluateWorkspaceHomography(input, horizon).result);
    EXPECT_FALSE(projectWorkspacePoint(horizon, {500, 0}));
    for (const double scale : {1e-100, -1.0, 1e100}) {
        auto h = knownHomography();
        for (auto& value : h.imageToWorld)
            value *= scale;
        const auto point = projectWorkspacePoint(h, {500, 400});
        ASSERT_TRUE(point);
        EXPECT_NEAR(point->x.value, 0.5, 1e-14);
        EXPECT_NEAR(point->y.value, 0.3, 1e-14);
    }
}

TEST(WorkspaceProfileTest, StrictRoundTripPreservesTransformOpticsAndUncalibratedState)
{
    auto profile = homographyInput().profile;
    profile.workspaceId = "table \"A\" / żółć";
    profile.homography = knownHomography();
    const auto text = serializeWorkspaceProfile(profile);
    auto parsed = parseWorkspaceProfile(text);
    ASSERT_TRUE(parsed.profile) << parsed.error;
    EXPECT_EQ(parsed.profile->workspaceId, profile.workspaceId);
    EXPECT_EQ(parsed.profile->homography->imageToWorld, profile.homography->imageToWorld);
    EXPECT_EQ(parsed.profile->intrinsics.cameraMatrix, profile.intrinsics.cameraMatrix);
    EXPECT_EQ(parsed.profile->intrinsics.distortion, profile.intrinsics.distortion);
    EXPECT_TRUE(workspaceCalibrationCompatibilityError(*parsed.profile, profile.intrinsics).empty());
    profile.homography.reset();
    parsed = parseWorkspaceProfile(serializeWorkspaceProfile(profile));
    ASSERT_TRUE(parsed.profile);
    EXPECT_FALSE(parsed.profile->homography);
}

TEST(WorkspaceProfileTest, RejectsUnknownVersionsTruncationGarbageOversizeAndInvalidNumbers)
{
    auto p = homographyInput().profile;
    p.homography = knownHomography();
    const auto text = serializeWorkspaceProfile(p);
    for (const auto& [from, to] :
         std::vector<std::pair<std::string, std::string>>{{"workspace 1", "workspace -1"},
                                                          {"dimensions 1", "dimensions nan"},
                                                          {"undistorted_native_k", "raw_distorted"},
                                                          {"homography 1", "homography -1"}}) {
        auto bad = text;
        const auto pos = bad.find(from);
        ASSERT_NE(pos, std::string::npos);
        bad.replace(pos, from.size(), to);
        EXPECT_FALSE(parseWorkspaceProfile(bad).profile) << from;
    }
    EXPECT_FALSE(parseWorkspaceProfile(text + "garbage").profile);
    EXPECT_FALSE(parseWorkspaceProfile(text.substr(0, text.size() / 2)).profile);
    EXPECT_FALSE(parseWorkspaceProfile(std::string(MaximumWorkspaceProfileBytes + 1, 'x')).profile);
    p.workspaceId.clear();
    EXPECT_THROW((void)serializeWorkspaceProfile(p), std::invalid_argument);
}
} // namespace

TEST(TabletopHomographyReviewTest, AcceptsUnorderedCornersEdgeMidpointsAndInteriorFitPoints)
{
    auto input = homographyInput();
    input.checkPoints.clear();
    for (ImageCorner point : std::vector<ImageCorner>{{500, 100}, {900, 400}, {500, 400}}) {
        const auto world = projectWorkspacePoint(knownHomography(), point);
        ASSERT_TRUE(world);
        input.fitPoints.push_back({point, *world});
    }
    std::rotate(input.fitPoints.begin(), input.fitPoints.begin() + 2, input.fitPoints.end());
    ASSERT_TRUE(validateWorkspaceHomographyInput(input).empty());
    const auto result = evaluateWorkspaceHomography(input, knownHomography());
    ASSERT_TRUE(result.result) << result.error;
    EXPECT_LT(result.result->fit.maximumMeters, 1e-12);
    const WorkspaceHomography horizon{{1, 0, 0, 0, 1, 0, 1, 0, -500}};
    EXPECT_FALSE(evaluateWorkspaceHomography(input, horizon).result);
}

#include "ObliqueWorkspaceFixtures.hpp"

TEST(TabletopObliqueTest, KnownPinholePlaneChecksNearFarEdgesAndIndependentInterior)
{
    const auto input = obliqueWorkspaceInput();
    const auto h = obliqueHomography();
    const auto result = evaluateWorkspaceHomography(input,h);
    ASSERT_TRUE(result.result) << result.error;
    EXPECT_LT(result.result->fit.maximumMeters,1e-12);
    EXPECT_LT(result.result->check->maximumMeters,1e-12);
    EXPECT_NEAR(result.result->coverage.workspaceFraction,1,1e-14);
    EXPECT_EQ(result.result->coverage.extrapolatedCheckCount,0);
    EXPECT_LT(result.result->coverage.imageFraction,0.1); // A side view occupies little image area; not poor table coverage.
    const auto nearA = obliqueImage({Meters{0.2},Meters{0}}), nearB = obliqueImage({Meters{0.3},Meters{0}});
    const auto farA = obliqueImage({Meters{0.2},Meters{2.4}}), farB = obliqueImage({Meters{0.3},Meters{2.4}});
    EXPECT_NEAR((nearB.u-nearA.u)/(farB.u-farA.u),3.4,1e-12);
    const auto forward = projectWorkspacePoint(h,obliqueImage({Meters{0.75},Meters{0.5}}));
    const auto farther = projectWorkspacePoint(h,obliqueImage({Meters{0.75},Meters{1.5}}));
    ASSERT_TRUE(forward); ASSERT_TRUE(farther);
    EXPECT_GT(farther->y.value,forward->y.value); // World +Y is up despite far edge shrinking in image.
}

TEST(TabletopObliqueTest, CoverageSeparatesUnsupportedExtrapolatedChecksFromFitResiduals)
{
    auto input = homographyInput(); input.fitPoints.clear(); input.checkPoints.clear();
    for (const auto xy : std::array<std::array<double,2>,4>{{{0.4,0.2},{0.6,0.2},{0.6,0.4},{0.4,0.4}}})
        input.fitPoints.push_back({{100+800*xy[0],700-1000*xy[1]},{Meters{xy[0]},Meters{xy[1]}}});
    input.checkPoints = {{{500,400},{Meters{0.5},Meters{0.3}}},{{180,600},{Meters{0.1},Meters{0.1}}}};
    auto result = evaluateWorkspaceHomography(input,knownHomography());
    ASSERT_TRUE(result.result) << result.error;
    EXPECT_NEAR(result.result->coverage.workspaceFraction,0.04/0.6,1e-14);
    EXPECT_EQ(result.result->coverage.extrapolatedCheckCount,1);
    EXPECT_LT(result.result->fit.maximumMeters,1e-12);
    EXPECT_LT(result.result->check->maximumMeters,1e-12); // Low synthetic residual does not erase extrapolation.
    std::reverse(input.fitPoints.begin(),input.fitPoints.end());
    result = evaluateWorkspaceHomography(input,knownHomography());
    ASSERT_TRUE(result.result);
    EXPECT_EQ(result.result->coverage.extrapolatedCheckCount,1);
}

TEST(TabletopObliqueTest, ElevatedPointHasExpectedParallaxAndHorizonFailureIsActionable)
{
    const WorldPoint2D actual{Meters{0.3},Meters{1}};
    const auto onPlane = projectWorkspacePoint(obliqueHomography(),obliqueImage(actual));
    const auto elevated = projectWorkspacePoint(obliqueHomography(),obliqueImage(actual,0.05));
    ASSERT_TRUE(onPlane); ASSERT_TRUE(elevated);
    EXPECT_NEAR(onPlane->y.value,actual.y.value,1e-12);
    EXPECT_NEAR(elevated->y.value,2.0*0.5/0.45-1,1e-12);
    EXPECT_GT(std::hypot(elevated->x.value-actual.x.value,elevated->y.value-actual.y.value),0.2);
    auto input = obliqueWorkspaceInput();
    const WorkspaceHomography horizon{{1,0,0,0,1,0,0,1,-400}};
    const auto result = evaluateWorkspaceHomography(input,horizon);
    EXPECT_FALSE(result.result);
    EXPECT_NE(result.error.find("horizon"),std::string::npos);
    EXPECT_NE(result.error.find("raise"),std::string::npos);
}
