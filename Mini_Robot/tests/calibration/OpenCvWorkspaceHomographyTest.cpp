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

TEST(OpenCvWorkspaceHomographyTest, InteriorOriginAndReversedAxesPreserveMetricMapping)
{
    OpenCvWorkspaceHomographySolver solver;
    for (bool reverse : {false, true}) {
        auto input = homographyInput();
        input.profile.bounds = WorkspaceBounds{Meters{1}, Meters{0.6}, {Meters{-0.5}, Meters{-0.3}}};
        for (auto* set : {&input.fitPoints, &input.checkPoints})
            for (auto& p : *set) {
                p.world.x.value = (reverse ? 1 - p.world.x.value : p.world.x.value) - 0.5;
                p.world.y.value -= 0.3;
            }
        auto result = solver.solve(input);
        ASSERT_TRUE(result.result) << result.error;
        EXPECT_LT(result.result->fit.maximumMeters, 1e-10);
        EXPECT_LT(result.result->check->maximumMeters, 1e-10);
        const auto p = projectWorkspacePoint(*result.result->profile.homography, {300, 600});
        ASSERT_TRUE(p);
        EXPECT_NEAR(p->x.value, reverse ? 0.25 : -0.25, 1e-10);
        EXPECT_NEAR(p->y.value, -0.2, 1e-10);
    }
}

TEST(OpenCvWorkspaceHomographyCapacityTest, Solves1525And4096PointsWithIndependentChecks)
{
    OpenCvWorkspaceHomographySolver solver;
    for (const auto shape : std::array<std::array<int,2>,2>{{{61,25},{64,64}}}) {
        auto input=homographyInput(); input.profile.bounds=WorkspaceBounds{Meters{1.5},Meters{0.6}};
        input.fitPoints.clear(); input.checkPoints.clear();
        for (int row=0;row<shape[1];++row) for (int col=0;col<shape[0];++col) {
            const double x=1.5*col/(shape[0]-1), y=0.6*row/(shape[1]-1);
            input.fitPoints.push_back({{50+500*x,730-900*y},{Meters{x},Meters{y}}});
        }
        input.checkPoints={{{50+500*0.213,730-900*0.257},{Meters{0.213},Meters{0.257}}}};
        const auto result=solver.solve(input);
        ASSERT_TRUE(result.result)<<result.error;
        EXPECT_EQ(result.result->fit.distancesMeters.size(),static_cast<std::size_t>(shape[0]*shape[1]));
        EXPECT_LT(result.result->fit.maximumMeters,1e-9);
        EXPECT_LT(result.result->check->maximumMeters,1e-9);
        if (input.fitPoints.size()==MaximumWorkspaceFitPoints) {
            input.fitPoints.push_back(input.fitPoints.back());
            EXPECT_FALSE(solver.solve(input).result); // No truncation at capacity.
        }
    }
}

#include <opencv2/core.hpp>
#include <opencv2/core/version.hpp>
#include <cstdlib>
#include <iostream>

TEST(WorkspaceSolverMemoryAudit, BoundedThinDecomposition)
{
    const char* setting=std::getenv("MICRO_ROBOT_WORKSPACE_SVD_AUDIT");
    if (!setting) GTEST_SKIP()<<"Opt-in isolated process memory audit; run tools/calibration/audit_workspace_solver.py.";
    const std::string mode{setting};
    ASSERT_TRUE(mode=="legacy-full-1525" || mode=="thin-1525" || mode=="thin-4096");
    const int count=mode=="thin-4096"?4096:1525;
    cv::Mat a=cv::Mat::zeros(count*2,9,CV_64F);
    for (int i=0;i<count;++i) {
        const double u=0.05+0.8*(i%61)/60, v=0.1+0.7*(i/61)/68;
        const double x=u*1.2+0.03*v, y=0.02*u+0.8*v;
        auto* first=a.ptr<double>(i*2); auto* second=a.ptr<double>(i*2+1);
        first[0]=-u; first[1]=-v; first[2]=-1; first[6]=x*u; first[7]=x*v; first[8]=x;
        second[3]=-u; second[4]=-v; second[5]=-1; second[6]=y*u; second[7]=y*v; second[8]=y;
    }
    cv::Mat w,u,vt;
    const bool legacy=mode=="legacy-full-1525";
    cv::SVD::compute(a,w,u,vt,legacy?cv::SVD::FULL_UV:0);
    ASSERT_EQ(vt.rows,9); ASSERT_EQ(vt.cols,9);
    EXPECT_EQ(u.rows,count*2); EXPECT_EQ(u.cols,legacy?count*2:9);
    std::cout<<"OpenCV "<<CV_VERSION<<"; points="<<count<<"; U="<<u.rows<<'x'<<u.cols
             <<"; U_bytes="<<u.total()*u.elemSize()<<'\n';
}
