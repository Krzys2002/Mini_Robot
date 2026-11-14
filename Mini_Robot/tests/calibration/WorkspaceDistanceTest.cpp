#include "calibration/WorkspaceDistance.hpp"
#include "WorkspaceDiagnosticsFixtures.hpp"
#include <gtest/gtest.h>
#include <numbers>
namespace
{
using namespace micro_robot;
using namespace micro_robot::test;
std::shared_ptr<const WorkspaceDiagnostics> diagnostics(bool surveyed = false)
{
    auto a = diagnosticsAcquisition(0.001);
    return buildWorkspaceDiagnostics(a->profile, a,
                                     surveyed ? diagnosticsEvidence(*a) : WorkspaceSurveyEvidence{}, 2, 2)
        .result;
}
WorkspaceDistanceEndpoint endpoint(double x, double y, std::optional<double> radius = {},
                                   WorkspaceSelectionSpace space = WorkspaceSelectionSpace::WorldMeters)
{
    return {{Meters{x}, Meters{y}}, {space, radius, radius ? "Synthetic declared deterministic disk." : ""}};
}
TEST(WorkspaceDistanceTest, KnownPlanarLengthsOrientationsAndUnavailableEvidence)
{
    auto d = diagnostics();
    ASSERT_TRUE(d);
    for (auto b : {endpoint(0.8, 0.3), endpoint(0.2, 0.5), endpoint(0.8, 0.5)}) {
        auto m = measureWorkspaceDistance(*d, {endpoint(0.2, 0.1, 0), b});
        ASSERT_TRUE(m.result) << m.error;
        EXPECT_NEAR(m.result->distanceMeters, std::hypot(b.world.x.value - 0.2, b.world.y.value - 0.1),
                    1e-15);
        EXPECT_EQ(m.result->diagnosticPlane,"tabletop z=0");
        EXPECT_EQ(m.result->frameId,d->profile.frameId);
        EXPECT_FALSE(m.result->calibrationRevision.empty());
        EXPECT_FALSE(m.result->plusMinusMeters);
        EXPECT_FALSE(m.result->uncertaintyUnavailable.empty());
    }
    auto m = measureWorkspaceDistance(*d, {endpoint(0.2, 0.1, 0), endpoint(0.8, 0.5, 0)});
    ASSERT_TRUE(m.result);
    EXPECT_TRUE(m.result->uncertaintyUnavailable.find("survey") != std::string::npos);
    EXPECT_FALSE(measureWorkspaceDistance(*d, {endpoint(-0.1, 0.1), endpoint(0.8, 0.5)}).result);
}
TEST(WorkspaceDistanceTest, PixelCentresLetterboxingResizingAndLogicalCoordinatesRoundTrip)
{
    for (auto size : {std::pair{1000.0, 1000.0}, std::pair{533.3, 777.7}, std::pair{1500.0, 400.0}}) {
        auto vp = workspaceCameraViewport(1000, 800, size.first, size.second);
        ASSERT_TRUE(vp);
        for (auto p : {ImageCorner{0, 0}, ImageCorner{999, 799}, ImageCorner{500.25, 400.75}}) {
            auto view = workspaceCameraDisplay(*vp, p);
            ASSERT_TRUE(view);
            auto back = workspaceCameraPick(*vp, view->u, view->v);
            ASSERT_TRUE(back);
            EXPECT_NEAR(back->u, p.u, 1e-10);
            EXPECT_NEAR(back->v, p.v, 1e-10);
        }
    }
    auto vp = workspaceCameraViewport(1000, 800, 1000, 1000);
    ASSERT_TRUE(vp);
    EXPECT_EQ(vp->top, 100);
    EXPECT_FALSE(workspaceCameraPick(*vp, 500, 50));
    EXPECT_FALSE(workspaceCameraPick(*vp, 0, 100));
    EXPECT_FALSE(workspaceCameraViewport(0, 800, 1000, 1000));
    EXPECT_FALSE(workspaceCameraViewport(1000, 800, 0, 1000));
    EXPECT_FALSE(workspaceCameraDisplay({}, {}));
}
TEST(WorkspaceDistanceTest, PerspectivePicksAndSignedOriginsRemainMetric)
{
    auto p = diagnosticsProfile();
    p.homography = WorkspaceHomography{
        {1.0 / 600, 1.0 / 5400, -5.0 / 27, 0, 1.0 / 450, -2.0 / 9, 0, -1.0 / 900, 10.0 / 9}};
    auto d = buildWorkspaceDiagnostics(p, {}, {}, 2, 2);
    ASSERT_TRUE(d.result) << d.error;
    const WorldPoint2D a{Meters{0.2}, Meters{0.1}}, b{Meters{0.8}, Meters{0.5}};
    auto qa = workspaceDistancePointFromImage(*d.result, *unprojectWorkspacePoint(*p.homography, a));
    auto qb = workspaceDistancePointFromImage(*d.result, *unprojectWorkspacePoint(*p.homography, b));
    ASSERT_TRUE(qa);
    ASSERT_TRUE(qb);
    EXPECT_NEAR(measureWorkspaceDistance(*d.result, {{{*qa, {}}, {*qb, {}}}}).result->distanceMeters,
                std::hypot(0.6, 0.4), 1e-12);
    auto source = diagnosticsAcquisition();
    source->profile.bounds = WorkspaceBounds{Meters{1}, Meters{0.6}, {Meters{-0.5}, Meters{-0.3}}};
    source->profile.homography->imageToWorld[2] -= 0.5;
    source->profile.homography->imageToWorld[5] -= 0.3;
    for (auto& r : source->references) {
        r.point.world.x.value -= 0.5;
        r.point.world.y.value -= 0.3;
    }
    auto signedD = buildWorkspaceDiagnostics(source->profile, source, {}, 2, 2);
    ASSERT_TRUE(signedD.result) << signedD.error;
    auto m = measureWorkspaceDistance(*signedD.result, {endpoint(-0.3, -0.2), endpoint(0.3, 0.2)});
    ASSERT_TRUE(m.result);
    EXPECT_NEAR(m.result->distanceMeters, std::hypot(0.6, 0.4), 1e-15);
}
TEST(WorkspaceDistanceTest, CalibrationRadiiCombineWithoutEndpointIndependenceOrFitRms)
{
    auto d = diagnostics(true);
    ASSERT_TRUE(d && d->prediction);
    auto a = endpoint(0.35, 0.25, 0), b = endpoint(0.65, 0.4, 0);
    auto m = measureWorkspaceDistance(*d, {a, b});
    ASSERT_TRUE(m.result);
    ASSERT_TRUE(m.result->plusMinusMeters) << m.result->uncertaintyUnavailable;
    const double expected = *workspacePredictionRadius(*d, a.world, WorkspaceEndpointCoverage) +
                            *workspacePredictionRadius(*d, b.world, WorkspaceEndpointCoverage);
    EXPECT_NEAR(*m.result->plusMinusMeters, expected, 1e-12);
    EXPECT_EQ(m.result->coverage, 0.95);
    EXPECT_NEAR(*m.result->lowerMeters, m.result->distanceMeters - expected, 1e-12);
    EXPECT_NEAR(*m.result->upperMeters, m.result->distanceMeters + expected, 1e-12);
    auto unknown = measureWorkspaceDistance(*d, {endpoint(0.35, 0.25), b});
    ASSERT_TRUE(unknown.result);
    EXPECT_FALSE(unknown.result->plusMinusMeters);
    auto invalid = measureWorkspaceDistance(*d, {endpoint(0.35, 0.25, -1), b});
    ASSERT_TRUE(invalid.result);
    EXPECT_FALSE(invalid.result->plusMinusMeters);
}
TEST(WorkspaceDistanceTest, WholeSelectionDiskIsBoundedIncludingSpatialModelVariation)
{
    auto d = diagnostics(true);
    ASSERT_TRUE(d && d->prediction);
    auto a = endpoint(0.35, 0.25, 2, WorkspaceSelectionSpace::NativeUndistortedPixels),
         b = endpoint(0.65, 0.4, 0.001);
    auto m = measureWorkspaceDistance(*d, {a, b});
    ASSERT_TRUE(m.result);
    ASSERT_TRUE(m.result->plusMinusMeters) << m.result->uncertaintyUnavailable;
    EXPECT_NEAR(*m.result->selectionRadiiMeters[0], 0.0025,
                1e-12); // 2 px times largest singular scale = 1.25 mm/px.
    EXPECT_EQ(*m.result->selectionRadiiMeters[1], 0.001);
    for (std::size_t i = 0; i < 2; ++i)
        for (int step = 0; step < 360; ++step) {
            const auto& e = i == 0 ? a : b;
            const double angle = step * std::numbers::pi / 180, r = *m.result->selectionRadiiMeters[i];
            WorldPoint2D q{Meters{e.world.x.value + r * std::cos(angle)},
                           Meters{e.world.y.value + r * std::sin(angle)}};
            auto pointRadius = workspacePredictionRadius(*d, q, WorkspaceEndpointCoverage);
            ASSERT_TRUE(pointRadius);
            EXPECT_GE(*m.result->calibrationRadiiMeters[i] + 1e-12, *pointRadius);
        }
    auto zero = measureWorkspaceDistance(*d, {endpoint(0.35, 0.25, 0), endpoint(0.65, 0.4, 0)});
    EXPECT_GT(*m.result->plusMinusMeters, *zero.result->plusMinusMeters + 0.0035);
    auto edge = measureWorkspaceDistance(*d, {endpoint(0.205, 0.2, 0.02), b});
    ASSERT_TRUE(edge.result);
    EXPECT_FALSE(edge.result->plusMinusMeters);
    EXPECT_TRUE(edge.result->uncertaintyUnavailable.find("footprint") != std::string::npos);
}
TEST(WorkspaceDistanceTest, PerspectivePixelDiskBoundEnclosesMappedPerturbations)
{
    auto p = diagnosticsProfile();
    p.homography = WorkspaceHomography{
        {1.0 / 600, 1.0 / 5400, -5.0 / 27, 0, 1.0 / 450, -2.0 / 9, 0, -1.0 / 900, 10.0 / 9}};
    auto d = buildWorkspaceDiagnostics(p, {}, {}, 2, 2);
    ASSERT_TRUE(d.result);
    auto a = endpoint(0.5, 0.3, 4, WorkspaceSelectionSpace::NativeUndistortedPixels);
    auto m = measureWorkspaceDistance(*d.result, {a, endpoint(0.6, 0.4, 0)});
    ASSERT_TRUE(m.result);
    ASSERT_TRUE(m.result->selectionRadiiMeters[0]);
    auto pixel = *unprojectWorkspacePoint(*p.homography, a.world);
    for (int step = 0; step < 360; ++step) {
        const double angle = step * std::numbers::pi / 180;
        auto q = projectWorkspacePoint(*p.homography,
                                       {pixel.u + 4 * std::cos(angle), pixel.v + 4 * std::sin(angle)});
        ASSERT_TRUE(q);
        EXPECT_LE(std::hypot(q->x.value - a.world.x.value, q->y.value - a.world.y.value),
                  *m.result->selectionRadiiMeters[0] + 1e-12);
    }
}
TEST(WorkspaceDistanceTest, ZeroNearZeroAndUnsupportedMappingsHaveExplicitResults)
{
    auto d = diagnostics(true);
    ASSERT_TRUE(d);
    for (double delta : {0.0, 1e-12}) {
        auto m = measureWorkspaceDistance(*d, {endpoint(0.5, 0.3, 0), endpoint(0.5 + delta, 0.3, 0)});
        ASSERT_TRUE(m.result);
        ASSERT_TRUE(m.result->plusMinusMeters);
        EXPECT_NEAR(m.result->distanceMeters, delta, 1e-16);
        EXPECT_EQ(*m.result->lowerMeters, 0);
        EXPECT_GT(*m.result->upperMeters, 0);
    }
    EXPECT_FALSE(workspaceDistancePointFromImage(*d, {-1, 100}));
    EXPECT_FALSE(workspaceDistancePointFromImage(*d, {500, 799}));
    EXPECT_FALSE(workspaceDistanceDiskSupported(*d, {Meters{0.5}, Meters{0.3}}, 0.4));
    auto invalid = endpoint(std::numeric_limits<double>::quiet_NaN(), 0.3);
    EXPECT_FALSE(measureWorkspaceDistance(*d, {invalid, endpoint(0.5, 0.3)}).result);
    auto p = diagnosticsProfile();
    p.homography = WorkspaceHomography{{1, 0, 0, 0, 1, 0, 1, 0, 1}};
    auto horizon = buildWorkspaceDiagnostics(p, {}, {}, 2, 2);
    ASSERT_TRUE(horizon.result);
    EXPECT_FALSE(workspaceDistanceDiskSupported(*horizon.result, {Meters{1}, Meters{0.3}}, 0));
}

TEST(WorkspaceDistanceTest, MissingSelectionJustificationAndHorizonFootprintsDoNotInventBounds)
{
    auto d = diagnostics(true);
    ASSERT_TRUE(d);
    auto a = endpoint(0.35, 0.25, 0), b = endpoint(0.65, 0.4, 0);
    a.selection.justification.clear();
    auto missing = measureWorkspaceDistance(*d, {a, b});
    ASSERT_TRUE(missing.result);
    EXPECT_FALSE(missing.result->plusMinusMeters);
    a.selection.justification = "   ";
    missing = measureWorkspaceDistance(*d, {a, b});
    ASSERT_TRUE(missing.result);
    EXPECT_FALSE(missing.result->plusMinusMeters);
    auto p = diagnosticsProfile();
    p.homography = WorkspaceHomography{
        {1.0 / 600, 1.0 / 5400, -5.0 / 27, 0, 1.0 / 450, -2.0 / 9, 0, -1.0 / 900, 10.0 / 9}};
    auto perspective = buildWorkspaceDiagnostics(p, {}, {}, 2, 2);
    ASSERT_TRUE(perspective.result);
    auto crossing = measureWorkspaceDistance(
        *perspective.result, {endpoint(0.5, 0.3, 1000, WorkspaceSelectionSpace::NativeUndistortedPixels), b});
    ASSERT_TRUE(crossing.result);
    EXPECT_FALSE(crossing.result->selectionRadiiMeters[0]);
    EXPECT_FALSE(crossing.result->plusMinusMeters);
}
TEST(WorkspaceDistanceTest, ImageFitHullConstraintsApplyToTheEntireWorldDisk)
{
    auto a = diagnosticsAcquisition();
    for (std::size_t i = 0; i < 4; ++i) {
        auto& p = a->references[i].point.image;
        p.u = 500 + (p.u - 500) * 0.5;
        p.v = 400 + (p.v - 400) * 0.5;
    }
    auto d = buildWorkspaceDiagnostics(a->profile, a, {}, 2, 2);
    ASSERT_TRUE(d.result) << d.error;
    EXPECT_TRUE(workspaceDistanceDiskSupported(*d.result, {Meters{0.5}, Meters{0.3}}, 0));
    EXPECT_FALSE(workspaceDistanceDiskSupported(*d.result, {Meters{0.5}, Meters{0.3}}, 0.16));
    EXPECT_FALSE(workspaceDistanceDiskSupported(*d.result, {Meters{0.1}, Meters{0.3}}, 0));
    auto vp = workspaceCameraViewport(1000, 800, 533.3, 777.7);
    ASSERT_TRUE(vp);
    const auto first = workspaceCameraDisplay(*vp, {0, 0});
    ASSERT_TRUE(first);
    EXPECT_FALSE(workspaceCameraPick(*vp, first->u - 0.01 * vp->width / vp->nativeWidth, first->v));
}
} // namespace
