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
         std::vector<std::pair<std::string, std::string>>{{"workspace 3", "workspace -1"},
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

TEST(WorkspaceProfileTest, VersionTwoPreservesFrameAndVersionOneMigratesWithoutMovingCoordinates)
{
    auto p = homographyInput().profile;
    p.homography = knownHomography();
    auto old = serializeWorkspaceProfile(p);
    old.replace(old.find("workspace 3"), std::string("workspace 3").size(), "workspace 1");
    old.erase(old.find("minimum "), old.find("image_coordinates") - old.find("minimum "));
    const auto migrated = parseWorkspaceProfile(old);
    ASSERT_TRUE(migrated.profile) << migrated.error;
    EXPECT_EQ(migrated.profile->version, WorkspaceCalibrationProfileVersion);
    EXPECT_EQ(migrated.profile->homography->imageToWorld, p.homography->imageToWorld);
    EXPECT_TRUE(migrated.profile->bounds == p.bounds);
    EXPECT_EQ(migrated.profile->frameId, "legacy-lower-left");
    EXPECT_FALSE(migrated.profile->boundary);
    p.bounds = WorkspaceBounds{Meters{1}, Meters{0.6}, {Meters{-0.25}, Meters{-0.3}}};
    p.frameId = "frame-246";
    p.boundary = WorkspaceBoundary{{{100,700}, {900,700}, {900,100}, {100,100}}};
    const auto roundTrip = parseWorkspaceProfile(serializeWorkspaceProfile(p));
    ASSERT_TRUE(roundTrip.profile) << roundTrip.error;
    EXPECT_TRUE(roundTrip.profile->bounds == p.bounds);
    EXPECT_EQ(roundTrip.profile->frameId, p.frameId);
    ASSERT_TRUE(roundTrip.profile->boundary);
    for (std::size_t i = 0; i < WorkspaceCornerCount; ++i) {
        EXPECT_DOUBLE_EQ((*roundTrip.profile->boundary)[i].u, (*p.boundary)[i].u);
        EXPECT_DOUBLE_EQ((*roundTrip.profile->boundary)[i].v, (*p.boundary)[i].v);
    }
}

TEST(WorkspaceBoundaryTest, PhysicalOrderAcceptsBothImageWindingsAndRejectsBadPerimeters)
{
    const WorkspaceBoundary valid{{{100,700}, {900,700}, {900,100}, {100,100}}};
    EXPECT_TRUE(validateWorkspaceBoundary(valid, 1000, 800).empty());
    auto reversed = valid;
    std::swap(reversed[1], reversed[3]);
    EXPECT_TRUE(validateWorkspaceBoundary(reversed, 1000, 800).empty());
    auto bad = valid;
    std::swap(bad[1], bad[2]);
    EXPECT_FALSE(validateWorkspaceBoundary(bad, 1000, 800).empty());
    bad = valid; bad[2] = bad[1];
    EXPECT_FALSE(validateWorkspaceBoundary(bad, 1000, 800).empty());
    bad = valid; bad[2] = {500,700};
    EXPECT_FALSE(validateWorkspaceBoundary(bad, 1000, 800).empty());
    bad = valid; bad[2] = {200,600};
    EXPECT_FALSE(validateWorkspaceBoundary(bad, 1000, 800).empty());
    bad = valid; bad[0].u = -1;
    EXPECT_FALSE(validateWorkspaceBoundary(bad, 1000, 800).empty());
}

TEST(WorkspaceProfileTest, RejectsIncompleteOrInvalidFrameDescriptors)
{
    auto p = homographyInput().profile;
    for (auto bounds : {WorkspaceBounds{Meters{1},Meters{0.6},{Meters{0.1},Meters{0}}},
                        WorkspaceBounds{Meters{1},Meters{0.6},{Meters{-2},Meters{0}}}}) {
        p.bounds = bounds;
        EXPECT_THROW((void)serializeWorkspaceProfile(p),std::invalid_argument);
    }
    p = homographyInput().profile;
    p.frameId.clear();
    EXPECT_THROW((void)serializeWorkspaceProfile(p),std::invalid_argument);
    p.frameId = std::string(MaximumWorkspaceFrameIdBytes+1,'x');
    EXPECT_THROW((void)serializeWorkspaceProfile(p),std::invalid_argument);
    auto text = serializeWorkspaceProfile(homographyInput().profile);
    text.erase(text.find("minimum "),text.find("image_coordinates")-text.find("minimum "));
    EXPECT_FALSE(parseWorkspaceProfile(text).profile); // v2 never silently defaults a missing frame.
}

TEST(TabletopHomographyTest, NativeProjectionInverseRoundTripsSignedWorldAndRejectsSingularMaps)
{
    WorkspaceHomography h{{0.001,0.0001,-0.5, 0.00005,-0.0007,0.2, 0.0001,0.0002,1}};
    for (const auto image : std::array<ImageCorner,3>{{{100,100},{500,400},{900,700}}}) {
        const auto world = projectWorkspacePoint(h,image);
        ASSERT_TRUE(world);
        const auto restored = unprojectWorkspacePoint(h,*world);
        ASSERT_TRUE(restored);
        EXPECT_NEAR(restored->u,image.u,1e-10);
        EXPECT_NEAR(restored->v,image.v,1e-10);
    }
    EXPECT_FALSE(unprojectWorkspacePoint(WorkspaceHomography{},{}));
}
