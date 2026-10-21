#include "calibration/OpenCvIntrinsicCalibrationSolver.hpp"
#include "IntrinsicFixtures.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <functional>
#include <limits>

namespace {
using namespace micro_robot;
using namespace micro_robot::test;
// Analytic projection independent of OpenCV: Rx/Ry board tilts followed by
// pinhole projection and Brown-Conrady distortion. No image files or random seed.
IntrinsicCalibrationInput syntheticInput(bool noisy = false)
{
    const auto expected = validIntrinsicCalibration();
    IntrinsicCalibrationInput input{"camera-a", 1000, 800, {}, {}};
    for (int i = 0; i < 14; ++i) {
        IntrinsicObservation view{static_cast<std::uint64_t>(i + 1), {}};
        const double ax = -0.45 + 0.15 * (i % 7);
        const double ay = -0.4 + 0.2 * (i % 5);
        for (const auto& p : calibrationObjectPoints(input.target)) {
            const double x0 = p.x - 0.1, y0 = p.y - 0.0625;
            const double x = std::cos(ay) * x0 + std::sin(ay) * std::sin(ax) * y0
                + 0.055 * (i % 3 - 1);
            const double y = std::cos(ax) * y0 + 0.04 * (i % 4 - 1.5);
            const double z = -std::sin(ay) * x0 + std::cos(ay) * std::sin(ax) * y0
                + 0.42 + 0.035 * (i % 5);
            const double xn = x / z, yn = y / z, r2 = xn * xn + yn * yn;
            const auto& d = expected.distortion;
            const double radial = 1 + d[0] * r2 + d[1] * r2 * r2 + d[4] * r2 * r2 * r2;
            const double xd = xn * radial + 2 * d[2] * xn * yn + d[3] * (r2 + 2 * xn * xn);
            const double yd = yn * radial + d[2] * (r2 + 2 * yn * yn) + 2 * d[3] * xn * yn;
            const double noise = noisy ? (i == 5 ? 2.0 : 0.05) : 0;
            const auto j = static_cast<double>(view.corners.size());
            view.corners.push_back({800 * xd + 500 + noise * std::sin(j * 2.1),
                                    810 * yd + 400 + noise * std::cos(j * 1.7)});
        }
        input.observations.push_back(std::move(view));
    }
    return input;
}
TEST(OpenCvIntrinsicCalibrationSolverTest, RecoversKnownIntrinsicsAndAllFiveDistortionCoefficients)
{
    OpenCvIntrinsicCalibrationSolver solver;
    const auto outcome = solver.solve(syntheticInput());
    ASSERT_TRUE(outcome.result) << outcome.error;
    const auto& r = *outcome.result;
    EXPECT_TRUE(outcome.error.empty());
    EXPECT_TRUE(validateIntrinsicCalibration(r).empty());
    const auto expected = validIntrinsicCalibration();
    for (std::size_t i = 0; i < 9; ++i) EXPECT_NEAR(r.cameraMatrix[i], expected.cameraMatrix[i], 0.01);
    ASSERT_EQ(r.distortion.size(), 5U);
    for (std::size_t i = 0; i < 5; ++i) EXPECT_NEAR(r.distortion[i], expected.distortion[i], 0.002);
    EXPECT_EQ(r.sourceId, "camera-a");
    EXPECT_EQ(r.width, 1000U);
    EXPECT_EQ(r.height, 800U);
    EXPECT_LT(r.rmsPixels, 0.001);
    ASSERT_EQ(r.perViewErrors.size(), 14U);
    for (std::size_t i = 0; i < 14; ++i) {
        EXPECT_EQ(r.perViewErrors[i].frameSequence, i + 1);
        EXPECT_EQ(r.perViewErrors[i].pointCount, 54U);
        EXPECT_LT(r.perViewErrors[i].rmsPixels, 0.001);
    }
}
TEST(OpenCvIntrinsicCalibrationSolverTest, ReportsNoisyViewAndWarningWithoutDiscardingFiniteModel)
{
    OpenCvIntrinsicCalibrationSolver solver;
    const auto outcome = solver.solve(syntheticInput(true));
    ASSERT_TRUE(outcome.result) << outcome.error;
    const auto& r = *outcome.result;
    EXPECT_GT(r.rmsPixels, 0.3);
    EXPECT_LT(r.rmsPixels, 1.0);
    EXPECT_GT(r.perViewErrors[5].rmsPixels, 1.5);
    EXPECT_TRUE(hasReprojectionWarning(r));
    EXPECT_NEAR(r.cameraMatrix[0], 800, 15);
    EXPECT_NEAR(r.cameraMatrix[4], 810, 15);
}
TEST(OpenCvIntrinsicCalibrationSolverTest, RejectsInvalidInputsWithoutReturningAModel)
{
    const std::vector<std::function<void(IntrinsicCalibrationInput&)>> corruptions{
        [](auto& r) { r.sourceId.clear(); }, [](auto& r) { r.width = 0; },
        [](auto& r) { r.width = 5000; }, [](auto& r) { r.observations.resize(2); },
        [](auto& r) { r.observations.resize(25); }, [](auto& r) { r.target.squareSizeMeters = 0; },
        [](auto& r) { r.observations[0].corners.pop_back(); },
        [](auto& r) { r.observations[0].frameSequence = 2; },
        [](auto& r) { r.observations[0].corners[0].u = -1; },
        [](auto& r) { r.observations[0].corners[0].v = std::numeric_limits<double>::quiet_NaN(); },
        [](auto& r) { for (auto& p : r.observations[0].corners) p = {20, 20}; }
    };
    OpenCvIntrinsicCalibrationSolver solver;
    for (std::size_t i = 0; i < corruptions.size(); ++i) {
        auto input = syntheticInput();
        corruptions[i](input);
        const auto outcome = solver.solve(input);
        EXPECT_FALSE(outcome.result) << i;
        EXPECT_FALSE(outcome.error.empty()) << i;
    }
    IntrinsicCalibrationConfig config;
    config.maximumAbsoluteDistortion = -1;
    EXPECT_FALSE(solver.solve(syntheticInput(), config).result);
}
} // namespace
