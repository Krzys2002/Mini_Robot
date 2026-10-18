#include "control/DifferentialDrive.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>

namespace micro_robot {
namespace {

constexpr DifferentialDriveGeometry geometry{Meters{0.02}, Meters{0.10}};
constexpr double tolerance = 1e-12;

TEST(DifferentialDriveTest, StraightMotionAndReverseHaveEqualWheelSigns)
{
    for (const double speed : {0.0, 0.20, -0.20}) {
        const auto wheels = toWheelAngularVelocity({speed, 0.0}, geometry);
        ASSERT_TRUE(wheels);
        EXPECT_DOUBLE_EQ(wheels->leftRadiansPerSecond, speed / 0.02);
        EXPECT_DOUBLE_EQ(wheels->rightRadiansPerSecond, speed / 0.02);
    }
}

TEST(DifferentialDriveTest, PureRotationHasOppositeWheelSigns)
{
    for (const double omega : {2.0, -2.0}) {
        const auto wheels = toWheelAngularVelocity({0.0, omega}, geometry);
        ASSERT_TRUE(wheels);
        EXPECT_NEAR(wheels->leftRadiansPerSecond, -omega * 2.5, tolerance);
        EXPECT_NEAR(wheels->rightRadiansPerSecond, omega * 2.5, tolerance);
    }
}

TEST(DifferentialDriveTest, ArcAndPivotMatchAnalyticalWheelTargets)
{
    const auto arc = toWheelAngularVelocity({0.20, 2.0}, geometry);
    ASSERT_TRUE(arc);
    EXPECT_NEAR(arc->leftRadiansPerSecond, 5.0, tolerance);
    EXPECT_NEAR(arc->rightRadiansPerSecond, 15.0, tolerance);
    const auto pivot = toWheelAngularVelocity({0.10, 2.0}, geometry);
    ASSERT_TRUE(pivot);
    EXPECT_NEAR(pivot->leftRadiansPerSecond, 0.0, tolerance);
    EXPECT_NEAR(pivot->rightRadiansPerSecond, 10.0, tolerance);
}

TEST(DifferentialDriveTest, InverseMatchesKnownWheelsAndRoundTrips)
{
    const auto known = toBodyVelocity({-5.0, 15.0}, geometry);
    ASSERT_TRUE(known);
    EXPECT_NEAR(known->linearMetersPerSecond, 0.10, tolerance);
    EXPECT_NEAR(known->angularRadiansPerSecond, 4.0, tolerance);
    for (const double v : {-0.3, 0.0, 0.3}) {
        for (const double omega : {-3.0, 0.0, 3.0}) {
            const auto wheels = toWheelAngularVelocity({v, omega}, geometry);
            ASSERT_TRUE(wheels);
            const auto body = toBodyVelocity(*wheels, geometry);
            ASSERT_TRUE(body);
            EXPECT_NEAR(body->linearMetersPerSecond, v, tolerance);
            EXPECT_NEAR(body->angularRadiansPerSecond, omega, tolerance);
        }
    }
}

TEST(DifferentialDriveTest, GeometryChangesWheelTargetsInSIUnits)
{
    const auto wheels = toWheelAngularVelocity({0.2, 2.0}, {Meters{0.04}, Meters{0.20}});
    ASSERT_TRUE(wheels);
    EXPECT_NEAR(wheels->leftRadiansPerSecond, 0.0, tolerance);
    EXPECT_NEAR(wheels->rightRadiansPerSecond, 10.0, tolerance);
}

TEST(DifferentialDriveTest, UnicycleDerivativeUsesWorldHeadingAndBodyForwardSpeed)
{
    struct Case { double heading; double x; double y; };
    for (const auto expected : {Case{0.0, -0.2, 0.0},
             Case{std::numbers::pi / 2.0, 0.0, -0.2}, Case{std::numbers::pi, 0.2, 0.0},
             Case{-std::numbers::pi / 2.0, 0.0, 0.2}, Case{2.0 * std::numbers::pi, -0.2, 0.0}}) {
        const auto derivative = unicycleDerivative(
            {Meters{1.0}, Meters{-2.0}, Radians{expected.heading}}, {-0.2, -1.0});
        ASSERT_TRUE(derivative);
        EXPECT_NEAR(derivative->xMetersPerSecond, expected.x, tolerance);
        EXPECT_NEAR(derivative->yMetersPerSecond, expected.y, tolerance);
        EXPECT_DOUBLE_EQ(derivative->headingRadiansPerSecond, -1.0);
    }
    const auto rotation = unicycleDerivative({}, {0.0, 2.0});
    ASSERT_TRUE(rotation);
    EXPECT_DOUBLE_EQ(rotation->xMetersPerSecond, 0.0);
    EXPECT_DOUBLE_EQ(rotation->yMetersPerSecond, 0.0);
    EXPECT_DOUBLE_EQ(rotation->headingRadiansPerSecond, 2.0);
}

TEST(DifferentialDriveTest, BoundsScaleBothWheelsAndPreserveCurvatureAndSigns)
{
    for (const double sign : {-1.0, 1.0}) {
        const auto wheels = toBoundedWheelAngularVelocity({sign * 0.2, sign * 2.0}, geometry, 7.5);
        ASSERT_TRUE(wheels);
        EXPECT_NEAR(wheels->leftRadiansPerSecond, sign * 2.5, tolerance);
        EXPECT_NEAR(wheels->rightRadiansPerSecond, sign * 7.5, tolerance);
        EXPECT_LE(std::abs(wheels->rightRadiansPerSecond), 7.5);
        const auto body = toBodyVelocity(*wheels, geometry);
        ASSERT_TRUE(body);
        EXPECT_NEAR(body->linearMetersPerSecond, sign * 0.1, tolerance);
        EXPECT_NEAR(body->angularRadiansPerSecond, sign * 1.0, tolerance);
    }
    const auto rotation = toBoundedWheelAngularVelocity({0.0, -2.0}, geometry, 2.0);
    ASSERT_TRUE(rotation);
    EXPECT_DOUBLE_EQ(rotation->leftRadiansPerSecond, 2.0);
    EXPECT_DOUBLE_EQ(rotation->rightRadiansPerSecond, -2.0);
}

TEST(DifferentialDriveTest, BoundsLeaveFeasibleTargetsUnchangedAndSupportZeroLimit)
{
    for (const double limit : {15.0, 20.0}) {
        const auto wheels = toBoundedWheelAngularVelocity({0.2, 2.0}, geometry, limit);
        ASSERT_TRUE(wheels);
        EXPECT_NEAR(wheels->leftRadiansPerSecond, 5.0, tolerance);
        EXPECT_NEAR(wheels->rightRadiansPerSecond, 15.0, tolerance);
    }
    for (const VelocityCommand velocity : {VelocityCommand{}, VelocityCommand{0.2, 2.0}}) {
        const auto wheels = toBoundedWheelAngularVelocity(velocity, geometry, 0.0);
        ASSERT_TRUE(wheels);
        EXPECT_DOUBLE_EQ(wheels->leftRadiansPerSecond, 0.0);
        EXPECT_DOUBLE_EQ(wheels->rightRadiansPerSecond, 0.0);
    }
}

TEST(DifferentialDriveTest, RejectsInvalidGeometryAndLimits)
{
    for (const double invalid : {0.0, -0.1, std::numeric_limits<double>::infinity(),
             std::numeric_limits<double>::quiet_NaN()}) {
        for (const auto dimensions : {DifferentialDriveGeometry{Meters{invalid}, Meters{0.1}},
                 DifferentialDriveGeometry{Meters{0.02}, Meters{invalid}}}) {
            EXPECT_FALSE(toWheelAngularVelocity({}, dimensions));
            EXPECT_FALSE(toBodyVelocity({}, dimensions));
            EXPECT_FALSE(toBoundedWheelAngularVelocity({}, dimensions, 10.0));
        }
    }
    for (const double invalid : {-1.0, std::numeric_limits<double>::infinity(),
             std::numeric_limits<double>::quiet_NaN()}) {
        EXPECT_FALSE(toBoundedWheelAngularVelocity({}, geometry, invalid));
    }
}

TEST(DifferentialDriveTest, RejectsNonfiniteInputsAndOverflowWithoutMotionTargets)
{
    for (const double invalid : {std::numeric_limits<double>::infinity(),
             -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
        for (const VelocityCommand velocity : {VelocityCommand{invalid, 0.0}, VelocityCommand{0.0, invalid}}) {
            EXPECT_FALSE(toWheelAngularVelocity(velocity, geometry));
            EXPECT_FALSE(toBoundedWheelAngularVelocity(velocity, geometry, 0.0));
            EXPECT_FALSE(unicycleDerivative({}, velocity));
        }
        EXPECT_FALSE(toBodyVelocity({invalid, 0.0}, geometry));
        EXPECT_FALSE(toBodyVelocity({0.0, invalid}, geometry));
        EXPECT_FALSE(unicycleDerivative({Meters{invalid}, Meters{}, Radians{}}, {}));
        EXPECT_FALSE(unicycleDerivative({Meters{}, Meters{invalid}, Radians{}}, {}));
        EXPECT_FALSE(unicycleDerivative({Meters{}, Meters{}, Radians{invalid}}, {}));
    }
    const double huge = std::numeric_limits<double>::max();
    EXPECT_FALSE(toWheelAngularVelocity({huge, 0.0}, geometry));
    EXPECT_FALSE(toBoundedWheelAngularVelocity({huge, 0.0}, geometry, 10.0));
    EXPECT_FALSE(toBodyVelocity({huge, huge}, {Meters{2.0}, Meters{0.1}}));
    EXPECT_FALSE(toBodyVelocity({-huge, huge}, {Meters{1.0}, Meters{0.1}}));
}

} // namespace
} // namespace micro_robot
