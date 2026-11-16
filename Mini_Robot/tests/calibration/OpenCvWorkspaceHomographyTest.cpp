#include "HomographyFixtures.hpp"
#include "calibration/OpenCvWorkspaceHomographySolver.hpp"
#include <gtest/gtest.h>

namespace {
using namespace micro_robot;
using namespace micro_robot::test;

TEST(OpenCvWorkspaceHomographyTest, FitsKnownAffineMappingAndIndependentInteriorPoint)
{
    OpenCvWorkspaceHomographySolver solver;
    const auto result = solver.solve(homographyInput());
    ASSERT_TRUE(result.result) << result.error;
    EXPECT_LT(result.result->fit.rmsMeters, 1e-10);
    ASSERT_TRUE(result.result->check);
    EXPECT_LT(result.result->check->rmsMeters, 1e-10);
}

TEST(OpenCvWorkspaceHomographyTest, RecoversPerspectiveMappingWithMoreThanFourPoints)
{
    OpenCvWorkspaceHomographySolver solver;
    auto input = homographyInput();
    input.fitPoints.clear();
    input.checkPoints.clear();
    WorkspaceHomography h{{0.001, 0.0001, 0, 0.00005, 0.0005, 0, 0.0001, 0.0002, 1}};
    for (ImageCorner image :
         std::vector<ImageCorner>{{100, 100}, {500, 50}, {900, 100}, {950, 400}, {800, 700}, {100, 700}}) {
        const auto world = projectWorkspacePoint(h, image);
        ASSERT_TRUE(world);
        input.fitPoints.push_back({image, *world});
    }
    auto world = projectWorkspacePoint(h, {500, 400});
    input.checkPoints.push_back({{500, 400}, *world});
    auto result = solver.solve(input);
    ASSERT_TRUE(result.result) << result.error;
    EXPECT_LT(result.result->fit.rmsMeters, 1e-8);
    EXPECT_LT(result.result->check->rmsMeters, 1e-8);
    input.fitPoints[1].world.x.value += 0.002;
    result = solver.solve(input);
    ASSERT_TRUE(result.result) << result.error;
    EXPECT_GT(result.result->fit.rmsMeters, 1e-5); // Noise is reported, not hidden by RANSAC.
}

TEST(OpenCvWorkspaceHomographyTest, RejectsInvalidAndNearlyCollinearGeometry)
{
    OpenCvWorkspaceHomographySolver solver;
    auto input = homographyInput();
    input.fitPoints[2].image = input.fitPoints[1].image;
    EXPECT_FALSE(solver.solve(input).result);
    input = homographyInput();
    for (auto& p : input.fitPoints)
        p.image.v = 400 + (p.image.v - 400) * 1e-10;
    EXPECT_FALSE(solver.solve(input).result);
}
} // namespace

TEST(OpenCvWorkspaceHomographyReviewTest, FitsUnorderedRectangleWithEdgeAndInteriorReferences)
{
    auto input = homographyInput();
    input.checkPoints.clear();
    for (ImageCorner p : std::vector<ImageCorner>{{500, 100}, {900, 400}, {500, 400}})
        input.fitPoints.push_back({p, *projectWorkspacePoint(knownHomography(), p)});
    std::swap(input.fitPoints[0], input.fitPoints[2]);
    OpenCvWorkspaceHomographySolver solver;
    const auto result = solver.solve(input);
    ASSERT_TRUE(result.result) << result.error;
    EXPECT_LT(result.result->fit.maximumMeters, 1e-10);
}

#include "ObliqueWorkspaceFixtures.hpp"

TEST(OpenCvWorkspaceHomographyObliqueTest, RecoversStrongPinholePlaneFromFourAndManyReferences)
{
    OpenCvWorkspaceHomographySolver solver;
    auto input = obliqueWorkspaceInput();
    for (bool many : {false,true}) {
        if (many) {
            for (const auto xy : std::array<std::array<double,2>,3>{{{0.4,0.8},{1.2,0.4},{0.8,1.8}}}) {
                WorldPoint2D world{Meters{xy[0]},Meters{xy[1]}};
                input.fitPoints.push_back({obliqueImage(world),world});
            }
        }
        const auto result = solver.solve(input);
        ASSERT_TRUE(result.result) << result.error;
        EXPECT_LT(result.result->fit.maximumMeters,1e-9);
        EXPECT_LT(result.result->check->maximumMeters,1e-9);
        EXPECT_NEAR(result.result->coverage.workspaceFraction,1,1e-12);
        for (const auto& check : input.checkPoints) {
            const auto mapped = projectWorkspacePoint(*result.result->profile.homography,check.image);
            ASSERT_TRUE(mapped);
            EXPECT_NEAR(mapped->x.value,check.world.x.value,1e-9);
            EXPECT_NEAR(mapped->y.value,check.world.y.value,1e-9);
        }
    }
}

TEST(OpenCvWorkspaceHomographyObliqueTest, NearEdgeOnReferencesAreRejectedWithRecoveryAdvice)
{
    auto input = obliqueWorkspaceInput();
    for (auto& p : input.fitPoints) p.image.v = 400 + (p.image.v-400)*1e-10;
    const auto result = OpenCvWorkspaceHomographySolver{}.solve(input);
    EXPECT_FALSE(result.result);
    EXPECT_NE(result.error.find("raise"),std::string::npos);
}
