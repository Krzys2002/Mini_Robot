// Analytic tests of the #160 actuator contract: profile validation, body-to-wheel signs,
// curvature-preserving saturation, the provisional actuator map, quantization with exact
// bound enforcement, polarity and channel assignment.
#include "DriveTestProfile.hpp"
#include "drive/DriveKinematics.hpp"
#include "drive/DriveProfile.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>

using namespace codecell;
using namespace codecell::drive;
using runtime::BodyVelocity;

namespace {
constexpr double kTolerance = 1e-12;
constexpr DriveProfile kProfile = fixtures::kTestDriveProfile;  // track 0.06 m, wheel 0.2 m/s, bound 0.4

DriveProfile unreversed()
{
    DriveProfile profile = kProfile;
    profile.leftReversed = false;
    profile.rightReversed = false;
    return profile;
}

std::int16_t leftDuty(const MotorDuty& duty) { return kLeftWheelChannel == MotorChannel::M1 ? duty.m1 : duty.m2; }
std::int16_t rightDuty(const MotorDuty& duty) { return kRightWheelChannel == MotorChannel::M1 ? duty.m1 : duty.m2; }

MotorDuty dutyFor(const BodyVelocity& body, const DriveProfile& profile)
{
    return motorDutyFor(saturate(wheelSpeedsFor(body, profile.trackWidthMetres), profile.maxWheelSpeedMetresPerSecond),
                        profile);
}
} // namespace

TEST(DriveProfileTest, DefaultIsInvalidAndTheTestProfileIsValid)
{
    EXPECT_NE(validate(DriveProfile{}), DriveProfileError::None) << "nothing physical is invented";
    EXPECT_EQ(validate(kProfile), DriveProfileError::None);
    EXPECT_EQ(dutyBoundCounts(kProfile), 102) << "0.4 of 255 counts, rounded down";
}

TEST(DriveProfileTest, EachRuleRejectsItsViolation)
{
    auto with = [](auto mutate) {
        DriveProfile profile = kProfile;
        mutate(profile);
        return validate(profile);
    };
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    EXPECT_EQ(with([&](auto& p) { p.trackWidthMetres = nan; }), DriveProfileError::NonFinite);
    EXPECT_EQ(with([&](auto& p) { p.maxWheelSpeedMetresPerSecond = inf; }), DriveProfileError::NonFinite);
    EXPECT_EQ(with([&](auto& p) { p.maxOutputFraction = -inf; }), DriveProfileError::NonFinite);
    EXPECT_EQ(with([](auto& p) { p.trackWidthMetres = 0.0; }), DriveProfileError::TrackWidthNotPositive);
    EXPECT_EQ(with([](auto& p) { p.maxWheelSpeedMetresPerSecond = -0.1; }), DriveProfileError::WheelSpeedNotPositive);
    EXPECT_EQ(with([](auto& p) { p.maxOutputFraction = 0.0; }), DriveProfileError::OutputBoundOutOfRange);
    EXPECT_EQ(with([](auto& p) { p.maxOutputFraction = 1.0 + 1e-6; }), DriveProfileError::OutputBoundOutOfRange);
    EXPECT_EQ(with([](auto& p) { p.maxOutputFraction = 0.5 / kDutyFullScale; }), DriveProfileError::OutputBoundOutOfRange)
        << "a bound below one duty count could never move";
    EXPECT_EQ(with([](auto& p) { p.maxLinearMetresPerSecond = 0.0; }), DriveProfileError::LinearLimitOutOfRange);
    EXPECT_EQ(with([](auto& p) { p.maxLinearMetresPerSecond = 0.21; }), DriveProfileError::LinearLimitOutOfRange);
    EXPECT_EQ(with([](auto& p) { p.maxAngularRadiansPerSecond = 0.0; }), DriveProfileError::AngularLimitOutOfRange);
    // On the spot, omega * track / 2 must stay within the wheel speed: 0.2 * 2 / 0.06 = 6.67 rad/s.
    EXPECT_EQ(with([](auto& p) { p.maxAngularRadiansPerSecond = 6.7; }), DriveProfileError::AngularLimitOutOfRange);
    EXPECT_EQ(with([](auto& p) { p.maxAngularRadiansPerSecond = 0.2 * 2.0 / 0.06; }), DriveProfileError::None)
        << "the exact boundary is reachable";
    EXPECT_EQ(with([](auto& p) { p.maxLinearMetresPerSecond = p.maxWheelSpeedMetresPerSecond; }),
              DriveProfileError::None);
    EXPECT_EQ(with([](auto& p) { p.maxOutputFraction = 1.0; }), DriveProfileError::None);
    EXPECT_EQ(with([](auto& p) { p.maxOutputFraction = 0.01; }), DriveProfileError::None) << "1 % is 2 counts";
}

// Forward, reverse, both rotations and zero: documented signs (robot-protocol.md,
// differential-drive.md) on wheel intent, and bounded duties on both channels.
TEST(DriveKinematicsTest, StraightRotationAndZeroHaveTheDocumentedSigns)
{
    struct Case {
        const char* name;
        BodyVelocity body;
        double left;
        double right;
    };
    const Case cases[] = {
        {"forward", {0.1, 0.0}, 0.1, 0.1},
        {"reverse", {-0.1, 0.0}, -0.1, -0.1},
        {"counter-clockwise", {0.0, 2.0}, -0.06, 0.06},
        {"clockwise", {0.0, -2.0}, 0.06, -0.06},
        {"zero", {0.0, 0.0}, 0.0, 0.0},
    };
    const DriveProfile profile = unreversed();
    for (const auto& c : cases) {
        SCOPED_TRACE(c.name);
        const auto wheels = wheelSpeedsFor(c.body, profile.trackWidthMetres);
        EXPECT_NEAR(wheels.leftMetresPerSecond, c.left, kTolerance);
        EXPECT_NEAR(wheels.rightMetresPerSecond, c.right, kTolerance);
        const auto duty = dutyFor(c.body, profile);
        EXPECT_EQ(leftDuty(duty) > 0, c.left > 0);
        EXPECT_EQ(leftDuty(duty) < 0, c.left < 0);
        EXPECT_EQ(rightDuty(duty) > 0, c.right > 0);
        EXPECT_EQ(rightDuty(duty) < 0, c.right < 0);
        EXPECT_LE(std::abs(duty.m1), dutyBoundCounts(profile));
        EXPECT_LE(std::abs(duty.m2), dutyBoundCounts(profile));
    }
}

TEST(DriveKinematicsTest, CombinedArcMatchesTheInverseKinematics)
{
    // v = 0.1 m/s, omega = 1 rad/s, b = 0.06 m: left 0.07, right 0.13 (counter-clockwise arc).
    const auto wheels = wheelSpeedsFor({0.1, 1.0}, kProfile.trackWidthMetres);
    EXPECT_NEAR(wheels.leftMetresPerSecond, 0.07, kTolerance);
    EXPECT_NEAR(wheels.rightMetresPerSecond, 0.13, kTolerance);
    const auto same = saturate(wheels, kProfile.maxWheelSpeedMetresPerSecond);
    EXPECT_EQ(same.leftMetresPerSecond, wheels.leftMetresPerSecond) << "within the limit nothing changes";
    EXPECT_EQ(same.rightMetresPerSecond, wheels.rightMetresPerSecond);
    // Provisional linear map: 0.13 / 0.2 of the 0.4 bound = 0.26 -> 66 counts (66.3).
    const auto duty = motorDutyFor(same, unreversed());
    EXPECT_EQ(leftDuty(duty), 36);   // 0.07 / 0.2 * 0.4 * 255 = 35.7
    EXPECT_EQ(rightDuty(duty), 66);
}

TEST(DriveKinematicsTest, SaturationScalesBothWheelsAndKeepsCurvature)
{
    // Body limits allow v = 0.15 and omega = 3 together, which needs 0.06 / 0.24 m/s.
    const auto raw = wheelSpeedsFor({0.15, 3.0}, kProfile.trackWidthMetres);
    const auto bounded = saturate(raw, kProfile.maxWheelSpeedMetresPerSecond);
    EXPECT_NEAR(bounded.rightMetresPerSecond, 0.2, kTolerance);
    EXPECT_NEAR(bounded.leftMetresPerSecond, 0.05, kTolerance);
    EXPECT_NEAR(bounded.leftMetresPerSecond / bounded.rightMetresPerSecond,
                raw.leftMetresPerSecond / raw.rightMetresPerSecond, kTolerance)
        << "same ratio, so the same path curvature";

    // Extreme finite values stay finite and bounded with their signs.
    for (const BodyVelocity body : {BodyVelocity{1e300, 0.0}, BodyVelocity{0.0, -1e300}, BodyVelocity{-1e300, 1e300},
                                    BodyVelocity{std::numeric_limits<double>::max(), 0.0}}) {
        const auto wheels = saturate(wheelSpeedsFor(body, kProfile.trackWidthMetres), kProfile.maxWheelSpeedMetresPerSecond);
        if (!std::isfinite(wheels.leftMetresPerSecond) || !std::isfinite(wheels.rightMetresPerSecond)) {
            // Overflow to infinity is detected by the drive and refused before any write.
            continue;
        }
        EXPECT_LE(std::abs(wheels.leftMetresPerSecond), kProfile.maxWheelSpeedMetresPerSecond * (1 + kTolerance));
        EXPECT_LE(std::abs(wheels.rightMetresPerSecond), kProfile.maxWheelSpeedMetresPerSecond * (1 + kTolerance));
    }
}

TEST(DriveKinematicsTest, QuantizationNeverExceedsTheApprovedBound)
{
    DriveProfile profile = unreversed();
    profile.maxOutputFraction = 0.41;  // 104.55 counts: rounding would give 105, the bound is 104.
    ASSERT_EQ(dutyBoundCounts(profile), 104);
    EXPECT_EQ(dutyCountsFor(outputFractionFor(profile.maxWheelSpeedMetresPerSecond, profile), false, profile), 104);
    EXPECT_EQ(dutyCountsFor(outputFractionFor(-profile.maxWheelSpeedMetresPerSecond, profile), false, profile), -104);
    for (int i = -2000; i <= 2000; ++i) {
        const double speed = i * 0.0005;  // -1 .. 1 m/s, five times the provisioned wheel speed.
        for (const bool reversed : {false, true}) {
            const auto counts = dutyCountsFor(outputFractionFor(speed, profile), reversed, profile);
            ASSERT_LE(std::abs(counts), 104) << speed;
        }
    }
    EXPECT_EQ(dutyCountsFor(outputFractionFor(0.0, profile), true, profile), 0);
    EXPECT_EQ(dutyCountsFor(outputFractionFor(-0.0, profile), false, profile), 0);
}

TEST(DriveKinematicsTest, PolarityAndChannelAssignmentApplyOnlyAtTheWrite)
{
    // kProfile reverses the right wheel: forward drives the left bridge M+ and the right bridge M-.
    const auto forward = dutyFor({0.1, 0.0}, kProfile);
    EXPECT_GT(leftDuty(forward), 0);
    EXPECT_LT(rightDuty(forward), 0);
    EXPECT_EQ(std::abs(leftDuty(forward)), std::abs(rightDuty(forward)));
    EXPECT_EQ(leftDuty(forward), 51) << "0.1 / 0.2 * 0.4 * 255 = 51";
    // Assignment from the profile constants: left is M1 and right is M2 (UNVERIFIED, #238).
    EXPECT_EQ(forward.m1, leftDuty(forward));
    EXPECT_EQ(forward.m2, rightDuty(forward));
    DriveProfile both = kProfile;
    both.leftReversed = true;
    const auto reversedForward = dutyFor({0.1, 0.0}, both);
    EXPECT_EQ(leftDuty(reversedForward), -51);
    EXPECT_EQ(rightDuty(reversedForward), -51);
}
