#include "calibration/WorkspaceDiagnostics.hpp"
#include "WorkspaceDiagnosticsFixtures.hpp"
#include <chrono>
#include <gtest/gtest.h>
#include <random>

namespace
{
using namespace micro_robot;
using namespace micro_robot::test;
TEST(WorkspaceDiagnosticsTest, AffineScalesAndMissingEvidenceNeverBecomeZeroError)
{
    const auto d = buildWorkspaceDiagnostics(diagnosticsProfile());
    ASSERT_TRUE(d.result) << d.error;
    auto p = sampleWorkspaceDiagnostics(*d.result, {Meters{0.5}, Meters{0.3}});
    ASSERT_TRUE(p.xPixelsPerCentimeter);
    EXPECT_NEAR(*p.xPixelsPerCentimeter, 8, 1e-12);
    EXPECT_NEAR(*p.yPixelsPerCentimeter, 10, 1e-12);
    EXPECT_FALSE(p.fitCoverageKnown);
    EXPECT_FALSE(p.interpolatedAbsoluteMeters);
    EXPECT_FALSE(p.predictionRadiusMeters);
    EXPECT_FALSE(d.result->uncertaintyUnavailable.empty());
    EXPECT_FALSE(sampleWorkspaceDiagnostics(*d.result, {Meters{-0.1}, Meters{0.3}}).geometrySupported);
}
TEST(WorkspaceDiagnosticsTest, PerspectiveJacobianMatchesIndependentAnalyticProjection)
{
    // world->image: u=(600*x+100)/(1+0.5*y), v=(500*y+100)/(1+0.5*y).
    auto p = diagnosticsProfile();
    p.homography = WorkspaceHomography{
        {1.0 / 600, 1.0 / 5400, -5.0 / 27, 0, 1.0 / 450, -2.0 / 9, 0, -1.0 / 900, 10.0 / 9}};
    const auto d = buildWorkspaceDiagnostics(p);
    ASSERT_TRUE(d.result) << d.error;
    for (double y : {0.1, 0.3, 0.5}) {
        const double x = 0.5, w = 1 + 0.5 * y;
        auto s = sampleWorkspaceDiagnostics(*d.result, {Meters{x}, Meters{y}});
        ASSERT_TRUE(s.xPixelsPerCentimeter);
        EXPECT_NEAR(*s.xPixelsPerCentimeter, 6 / w, 1e-10);
        EXPECT_NEAR(*s.yPixelsPerCentimeter, std::hypot(-0.5 * (600 * x + 100), 450) / (w * w) / 100, 1e-10);
    }
    for (auto& value : p.homography->imageToWorld)
        value *= 1e-300;
    auto scaled = buildWorkspaceDiagnostics(p);
    ASSERT_TRUE(scaled.result) << scaled.error;
    EXPECT_NEAR(*sampleWorkspaceDiagnostics(*scaled.result, {Meters{0.5}, Meters{0.3}}).xPixelsPerCentimeter,
                6 / 1.15, 1e-10);
}
TEST(WorkspaceDiagnosticsTest, StudentQuantilesMatchPublishedReferenceValues)
{
    EXPECT_NEAR(*workspaceStudentCritical(0.975, 1), 12.7062047364, 1e-8);
    EXPECT_NEAR(*workspaceStudentCritical(0.975, 6), 2.4469118511, 1e-8);
    EXPECT_NEAR(*workspaceStudentCritical(0.9875, 6), 2.9686866842, 1e-7);
    EXPECT_NEAR(*workspaceStudentCritical(0.975, 4093), 1.9605437, 1e-5);
    EXPECT_FALSE(workspaceStudentCritical(1, 6));
    EXPECT_FALSE(workspaceStudentCritical(0.975, 0));
}
TEST(WorkspaceDiagnosticsTest, SurveyInterpolationAndConditionalPredictionHaveDistinctCoverage)
{
    auto a = diagnosticsAcquisition();
    auto d = buildWorkspaceDiagnostics(a->profile, a, diagnosticsEvidence(*a));
    ASSERT_TRUE(d.result) << d.error;
    ASSERT_TRUE(d.result->prediction) << d.result->uncertaintyUnavailable;
    auto s = sampleWorkspaceDiagnostics(*d.result, {Meters{0.5}, Meters{0.3}});
    ASSERT_TRUE(s.interpolatedAbsoluteMeters);
    EXPECT_NEAR(*s.interpolatedAbsoluteMeters, std::hypot(0.0025, 0.0007), 1e-12);
    EXPECT_NEAR(*s.interpolatedRelativePercent, 100 * *s.interpolatedAbsoluteMeters / std::hypot(1, 0.6),
                1e-12);
    ASSERT_TRUE(s.predictionRadiusMeters);
    EXPECT_GT(*s.predictionRadiusMeters, *s.interpolatedAbsoluteMeters);
    const double c =
        1 + 1.0 / 9 + 0.0025 * 0.0025 / (0.54 * 1.001 * 1.001) + 0.0007 * 0.0007 / (0.24 * 0.999 * 0.999);
    const auto& m = *d.result->prediction;
    EXPECT_NEAR(*s.predictionRadiusMeters,
                std::hypot(0.0025 / 1.001 + m.pointCritical * std::sqrt(c) * 0.0005,
                           0.0007 / 0.999 + m.pointCritical * std::sqrt(c) * 0.0005),
                1e-11);
    EXPECT_GT(*workspacePredictionRadius(*d.result, s.world, WorkspaceEndpointCoverage),
              *s.predictionRadiusMeters);
    EXPECT_FALSE(workspacePredictionRadius(*d.result, s.world, 0.99));
    auto outside = sampleWorkspaceDiagnostics(*d.result, {Meters{0.05}, Meters{0.3}});
    EXPECT_TRUE(outside.geometrySupported);
    EXPECT_FALSE(outside.interpolatedAbsoluteMeters);
    EXPECT_FALSE(outside.predictionRadiusMeters);
    EXPECT_FALSE(d.result->observations.front().surveyed);
    EXPECT_EQ(d.result->observations.size(), 13u);
}
TEST(WorkspaceDiagnosticsTest, MissingUnjustifiedAndPoorSurveyEvidenceIsUnavailable)
{
    auto a = diagnosticsAcquisition();
    auto e = diagnosticsEvidence(*a);
    e.independentSurveyConfirmed = false;
    auto d = buildWorkspaceDiagnostics(a->profile, a, e);
    ASSERT_TRUE(d.result);
    EXPECT_FALSE(d.result->prediction);
    EXPECT_TRUE(d.result->surveyHull.empty());
    e = diagnosticsEvidence(*a);
    e.gaussianAffineModelConfirmed = false;
    d = buildWorkspaceDiagnostics(a->profile, a, e);
    ASSERT_TRUE(d.result);
    EXPECT_FALSE(d.result->prediction);
    EXPECT_TRUE(sampleWorkspaceDiagnostics(*d.result, {Meters{0.5}, Meters{0.3}}).interpolatedAbsoluteMeters);
    e = diagnosticsEvidence(*a);
    e.additionalStandardUncertaintyMeters = 0;
    d = buildWorkspaceDiagnostics(a->profile, a, e);
    ASSERT_TRUE(d.result);
    EXPECT_FALSE(d.result->prediction);
    e = diagnosticsEvidence(*a);
    e.referenceIds.resize(5);
    d = buildWorkspaceDiagnostics(a->profile, a, e);
    ASSERT_TRUE(d.result);
    EXPECT_FALSE(d.result->prediction);
    e = diagnosticsEvidence(*a);
    e.referenceIds = {1};
    EXPECT_FALSE(buildWorkspaceDiagnostics(a->profile, a, e).result);
    e = diagnosticsEvidence(*a);
    e.referenceIds = {999};
    EXPECT_FALSE(buildWorkspaceDiagnostics(a->profile, a, e).result);
    e = diagnosticsEvidence(*a);
    e.referenceIds.push_back(e.referenceIds[0]);
    EXPECT_FALSE(buildWorkspaceDiagnostics(a->profile, a, e).result);
}
TEST(WorkspaceDiagnosticsTest, SignedOriginRelativeDenominatorAndIdentityAreExplicit)
{
    auto a = diagnosticsAcquisition();
    auto e = diagnosticsEvidence(*a);
    auto before = buildWorkspaceDiagnostics(a->profile, a, e);
    ASSERT_TRUE(before.result);
    a->profile.bounds = WorkspaceBounds{Meters{1}, Meters{0.6}, {Meters{-0.5}, Meters{-0.3}}};
    a->profile.homography->imageToWorld[2] -= 0.5;
    a->profile.homography->imageToWorld[5] -= 0.3;
    a->profile.frameId = "interior";
    for (auto& r : a->references) {
        r.point.world.x.value -= 0.5;
        r.point.world.y.value -= 0.3;
    }
    auto after = buildWorkspaceDiagnostics(a->profile, a, e);
    ASSERT_TRUE(after.result) << after.error;
    auto s = sampleWorkspaceDiagnostics(*after.result, {});
    EXPECT_NEAR(*s.xPixelsPerCentimeter, 8, 1e-12);
    EXPECT_NEAR(
        *s.interpolatedRelativePercent,
        *sampleWorkspaceDiagnostics(*before.result, {Meters{0.5}, Meters{0.3}}).interpolatedRelativePercent,
        1e-10);
    auto incompatible = a->profile;
    incompatible.frameId = "other";
    EXPECT_FALSE(buildWorkspaceDiagnostics(incompatible, a, e).result);
    EXPECT_FALSE(workspaceRelativeErrorPercent(0.001, WorkspaceBounds{Meters{1e-12}, Meters{1e-12}}));
    EXPECT_FALSE(buildWorkspaceDiagnostics(diagnosticsProfile(), {}, {}, 66, 25).result);
    EXPECT_FALSE(buildWorkspaceDiagnostics(diagnosticsProfile(), {}, {}, 1, 25).result);
    EXPECT_EQ(buildWorkspaceDiagnostics(diagnosticsProfile(), {}, {}, 65, 65).result->samples.size(), 4225u);
}
TEST(WorkspaceDiagnosticsTest, SyntheticIndependentNoiseIncreasesPredictiveScatter)
{
    auto quiet = diagnosticsAcquisition(), noisy = diagnosticsAcquisition(0.004);
    auto q = buildWorkspaceDiagnostics(quiet->profile, quiet, diagnosticsEvidence(*quiet));
    auto n = buildWorkspaceDiagnostics(noisy->profile, noisy, diagnosticsEvidence(*noisy));
    ASSERT_TRUE(q.result);
    ASSERT_TRUE(n.result) << n.error;
    ASSERT_TRUE(n.result->prediction) << n.result->uncertaintyUnavailable;
    EXPECT_GT(n.result->prediction->varianceX, q.result->prediction->varianceX);
    EXPECT_GT(*workspacePredictionRadius(*n.result, {Meters{0.5}, Meters{0.3}}),
              *workspacePredictionRadius(*q.result, {Meters{0.5}, Meters{0.3}}));
}
TEST(WorkspaceDiagnosticsTest, KnownFitHullMasksExtrapolationAndHorizon)
{
    auto a = diagnosticsAcquisition();
    for (std::size_t i = 0; i < 4; ++i) {
        auto& p = a->references[i].point;
        p.world.x.value = 0.1 + 0.8 * p.world.x.value;
        p.world.y.value = 0.05 + 0.8 * p.world.y.value;
        p.image = *unprojectWorkspacePoint(*a->profile.homography, p.world);
    }
    auto d = buildWorkspaceDiagnostics(a->profile, a);
    ASSERT_TRUE(d.result) << d.error;
    EXPECT_FALSE(sampleWorkspaceDiagnostics(*d.result, {Meters{0.01}, Meters{0.3}}).geometrySupported);
    auto p = diagnosticsProfile();
    p.homography = WorkspaceHomography{{1, 0, 0, 0, 1, 0, 1, 0, 1}};
    auto legacy = buildWorkspaceDiagnostics(p);
    ASSERT_TRUE(legacy.result) << legacy.error;
    EXPECT_FALSE(sampleWorkspaceDiagnostics(*legacy.result, {Meters{1}, Meters{0.3}}).geometrySupported);
}

TEST(WorkspaceDiagnosticsTest, CollinearSurveyAndNonfiniteFloorsAreUnavailable)
{
    auto a = diagnosticsAcquisition();
    a->references.resize(4);
    for (int i = 0; i < 6; ++i) {
        WorldPoint2D p{Meters{0.1 + 0.15 * i}, Meters{0.3}};
        a->references.push_back({static_cast<std::uint64_t>(5 + i),
                                 {*unprojectWorkspacePoint(*a->profile.homography, p), p},
                                 true,
                                 false,
                                 {}});
    }
    auto d = buildWorkspaceDiagnostics(a->profile, a, diagnosticsEvidence(*a));
    ASSERT_TRUE(d.result) << d.error;
    EXPECT_FALSE(d.result->prediction);
    EXPECT_TRUE(d.result->uncertaintyUnavailable.find("collinear") != std::string::npos);
    auto e = diagnosticsEvidence(*a);
    e.additionalStandardUncertaintyMeters = std::numeric_limits<double>::infinity();
    d = buildWorkspaceDiagnostics(a->profile, a, e);
    ASSERT_TRUE(d.result);
    EXPECT_FALSE(d.result->prediction);
}
TEST(WorkspaceDiagnosticsTest, ConditionalPredictionCoversSeededGaussianTrialsWithCorrelatedAxes)
{
    std::mt19937 random(249250);
    std::normal_distribution<double> normal(0, 0.002);
    int covered = 0;
    for (int trial = 0; trial < 300; ++trial) {
        auto a = diagnosticsAcquisition();
        for (auto& r : a->references)
            if (r.check) {
                const auto q = *projectWorkspacePoint(*a->profile.homography, r.point.image);
                const double z = normal(random), w = normal(random);
                r.point.world = {
                    Meters{q.x.value - (0.001 + 0.001 * q.x.value + z)},
                    Meters{q.y.value - (-0.001 + 0.002 * q.y.value + 0.7 * z + std::sqrt(0.51) * w)}};
            }
        auto e = diagnosticsEvidence(*a);
        e.additionalStandardUncertaintyMeters = 0.0001;
        const auto d = buildWorkspaceDiagnostics(a->profile, a, e, 2, 2);
        ASSERT_TRUE(d.result) << d.error;
        const auto radius = workspacePredictionRadius(*d.result, {Meters{0.5}, Meters{0.3}});
        ASSERT_TRUE(radius) << d.result->uncertaintyUnavailable;
        const double z = normal(random), w = normal(random);
        if (std::hypot(0.0015 + z, -0.0004 + 0.7 * z + std::sqrt(0.51) * w) <= *radius)
            ++covered;
    }
    // Deterministic model-specific calibration check, not a claim of physical coverage.
    EXPECT_GE(covered, 285);
    RecordProperty("covered_of_300", covered);
}
TEST(WorkspaceDiagnosticsTest, MaximumDatasetAndGridAreRetainedWithoutTruncation)
{
    auto a = std::make_shared<WorkspaceAcquisitionRecord>(
        WorkspaceAcquisitionRecord{"capacity", diagnosticsProfile(), {}, {}});
    a->profile.acquisitionId = a->id;
    std::uint64_t id = 0;
    for (bool check : {false, true})
        for (int y = 0; y < 64; ++y)
            for (int x = 0; x < 64; ++x) {
                WorldPoint2D p{Meters{check ? (x + 0.5) / 64.0 : x / 63.0},
                               Meters{0.6 * (check ? (y + 0.5) / 64.0 : y / 63.0)}};
                a->references.push_back(
                    {++id, {*unprojectWorkspacePoint(*a->profile.homography, p), p}, check, false, {}});
            }
    const auto start = std::chrono::steady_clock::now();
    const auto d = buildWorkspaceDiagnostics(a->profile, a, diagnosticsEvidence(*a), 65, 65);
    ASSERT_TRUE(d.result) << d.error;
    ASSERT_TRUE(d.result->prediction) << d.result->uncertaintyUnavailable;
    EXPECT_EQ(d.result->observations.size(), 8192u);
    EXPECT_EQ(d.result->samples.size(), 4225u);
    EXPECT_EQ(d.result->evidence.referenceIds.size(), 4096u);
    RecordProperty("elapsed_ms", std::chrono::duration_cast<std::chrono::milliseconds>(
                                     std::chrono::steady_clock::now() - start)
                                     .count());
}
} // namespace
