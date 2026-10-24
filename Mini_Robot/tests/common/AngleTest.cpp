#include "common/Angle.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>

namespace micro_robot {
namespace {
constexpr double pi = std::numbers::pi;

TEST(AngleTest, KeepsAnglesInsideTheHalfOpenRangeUnchanged)
{
    for (const double value : {0.0, -0.0, 1.0, -1.0, pi / 2, -pi / 2, pi, std::nextafter(-pi, 0.0)}) {
        EXPECT_EQ(normalizeAngle(Radians{value}).value, value) << value;
    }
}

TEST(AngleTest, MapsBothBoundaryRepresentationsToPositivePi)
{
    EXPECT_EQ(normalizeAngle(Radians{-pi}).value, pi);
    EXPECT_EQ(normalizeAngle(Radians{pi}).value, pi);
    EXPECT_NEAR(normalizeAngle(Radians{3 * pi}).value, pi, 1e-12);
    EXPECT_NEAR(normalizeAngle(Radians{-3 * pi}).value, pi, 1e-12);
}

TEST(AngleTest, WrapsWholeTurnsInBothDirections)
{
    for (int turns = -5; turns <= 5; ++turns) {
        for (const double angle : {0.0, 0.25, -0.25, 1.5, -1.5, 3.0, -3.0}) {
            const double normalized = normalizeAngle(Radians{angle + 2 * pi * turns}).value;
            EXPECT_NEAR(normalized, angle, 1e-12) << turns << ' ' << angle;
            EXPECT_GT(normalized, -pi);
            EXPECT_LE(normalized, pi);
        }
    }
}

TEST(AngleTest, PassesNonFiniteValuesThroughForExplicitRejection)
{
    EXPECT_TRUE(std::isnan(normalizeAngle(Radians{std::numeric_limits<double>::quiet_NaN()}).value));
    EXPECT_EQ(normalizeAngle(Radians{std::numeric_limits<double>::infinity()}).value,
              std::numeric_limits<double>::infinity());
}

} // namespace
} // namespace micro_robot
