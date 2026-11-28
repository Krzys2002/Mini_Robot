// Tests of the #160 bounded drive output with a fake motor HAL and an explicit clock:
// rate limits from rest, the elapsed-time clamp, reversal through a zero write, the stop
// ramp budget (#156 B6), immediate disable, refusal of invalid input and exact bounds.
#include "DriveFakes.hpp"
#include "RuntimeFakes.hpp"
#include "drive/BoundedDriveOutput.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>

using namespace codecell;
using namespace codecell::drive;
using namespace codecell::drive::fakes;
using namespace std::chrono_literals;
using runtime::BodyVelocity;
using runtime::Instant;
using runtime::StopRampStep;

namespace {
constexpr DriveProfile kProfile = runtime::fakes::kTestDriveProfile;
constexpr milliseconds kPeriod = timing::kControlPeriod;
constexpr double kTolerance = 1e-12;
// Full scale over the provisional normal ramp time: 0.2 m/s per 100 ms = 2 m/s^2.
constexpr double kRate = 0.2 / 0.1;
constexpr double kStepSpeed = kRate * 0.010;  // Per 10 ms control step.
constexpr int kFullScaleSteps =
    static_cast<int>(kNormalFullScaleRampTime / kPeriod);  // 10 steps.
constexpr int kStopSteps = static_cast<int>(kStopFullScaleRampTime / kPeriod);

std::int16_t left(const MotorDuty& duty) { return kLeftWheelChannel == MotorChannel::M1 ? duty.m1 : duty.m2; }
std::int16_t right(const MotorDuty& duty) { return kRightWheelChannel == MotorChannel::M1 ? duty.m1 : duty.m2; }

struct Rig {
    FakeMotorDriver motors;
    BoundedDriveOutput drive{motors};
    Instant now{1s};

    explicit Rig(bool configure = true)
    {
        if (configure) {
            EXPECT_TRUE(drive.configure(kProfile, kPeriod));
        }
    }
    bool command(const BodyVelocity& v)
    {
        now += Instant{kPeriod};
        return drive.command(v, now);
    }
    StopRampStep ramp()
    {
        now += Instant{kPeriod};
        return drive.rampToStop(now);
    }
};
} // namespace

TEST(BoundedDriveOutputTest, UnconfiguredOrInvalidProfilesNeverWrite)
{
    Rig unconfigured{false};
    EXPECT_FALSE(unconfigured.command({0.1, 0.0}));
    EXPECT_EQ(unconfigured.ramp(), StopRampStep::Refused);
    EXPECT_FALSE(unconfigured.drive.configure(DriveProfile{}, kPeriod));
    DriveProfile bad = kProfile;
    bad.maxOutputFraction = 1.5;
    EXPECT_FALSE(unconfigured.drive.configure(bad, kPeriod));
    EXPECT_FALSE(unconfigured.drive.configure(kProfile, 0ms));
    EXPECT_EQ(unconfigured.motors.count(MotorEvent::Kind::Write), 0);
    EXPECT_EQ(unconfigured.motors.count(MotorEvent::Kind::Enable), 0) << "the bridges stay asleep";

    Rig rig;
    EXPECT_EQ(rig.motors.count(MotorEvent::Kind::Enable), 1);
    FakeMotorDriver failing;
    failing.failEnable = true;
    BoundedDriveOutput drive{failing};
    EXPECT_FALSE(drive.configure(kProfile, kPeriod)) << "a failed wake-up is reported";
}

TEST(BoundedDriveOutputTest, NonFiniteCommandsAreRefusedWithoutAWrite)
{
    Rig rig;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(rig.command({nan, 0.0}));
    EXPECT_FALSE(rig.command({0.0, inf}));
    // Finite but overflowing: the wheel conversion becomes infinite and is refused.
    EXPECT_FALSE(rig.command({std::numeric_limits<double>::max(), std::numeric_limits<double>::max()}));
    EXPECT_EQ(rig.motors.count(MotorEvent::Kind::Write), 0);
}

TEST(BoundedDriveOutputTest, NoStartupBoostAccelerationIsRateLimitedFromZero)
{
    Rig rig;
    for (int step = 1; step <= kFullScaleSteps + 3; ++step) {
        ASSERT_TRUE(rig.command({0.15, 3.0}));  // Saturated arc: right wheel at full scale.
        const auto intent = rig.drive.wheelIntent();
        const double expected = std::min(0.2, kStepSpeed * step);
        ASSERT_NEAR(intent.rightMetresPerSecond, expected, 1e-9) << step;
        ASSERT_LE(std::abs(rig.motors.lastWrite()->m1), dutyBoundCounts(kProfile));
        ASSERT_LE(std::abs(rig.motors.lastWrite()->m2), dutyBoundCounts(kProfile));
    }
    // The left wheel ramps to its own (curvature-preserving) target, 0.05 m/s, never above.
    EXPECT_NEAR(rig.drive.wheelIntent().leftMetresPerSecond, 0.05, kTolerance);
    // First write from rest is one step: 0.02 m/s -> 0.1 of the bound -> 10 counts. No boost.
    EXPECT_EQ(std::abs(right(rig.motors.writes().front())), 10);
}

TEST(BoundedDriveOutputTest, ElapsedTimeIsClampedToOneControlPeriod)
{
    Rig rig;
    ASSERT_TRUE(rig.command({0.1, 0.0}));
    const double first = rig.drive.wheelIntent().leftMetresPerSecond;
    ASSERT_TRUE(rig.drive.command({0.1, 0.0}, rig.now));  // Same instant: no extra step.
    EXPECT_DOUBLE_EQ(rig.drive.wheelIntent().leftMetresPerSecond, first);
    rig.now += 5s;  // A late write never earns a larger step.
    ASSERT_TRUE(rig.drive.command({0.1, 0.0}, rig.now));
    EXPECT_NEAR(rig.drive.wheelIntent().leftMetresPerSecond, first + kStepSpeed, kTolerance);
}

TEST(BoundedDriveOutputTest, ReversalDeceleratesThroughAZeroWriteThenAccelerates)
{
    Rig rig;
    for (int i = 0; i < kFullScaleSteps; ++i) ASSERT_TRUE(rig.command({0.15, 0.0}));
    ASSERT_NEAR(rig.drive.wheelIntent().leftMetresPerSecond, 0.15, kTolerance);
    rig.motors.events.clear();
    double previous = 0.15;
    bool sawZero = false;
    for (int i = 0; i < 2 * kFullScaleSteps; ++i) {
        ASSERT_TRUE(rig.command({-0.15, 0.0}));
        const double now = rig.drive.wheelIntent().leftMetresPerSecond;
        ASSERT_LE(std::abs(now - previous), kStepSpeed + kTolerance) << "no jump across zero";
        if (now == 0.0) sawZero = true;
        if (now < 0.0) {
            ASSERT_TRUE(sawZero) << "negative only after an exact zero";
        }
        previous = now;
    }
    EXPECT_TRUE(sawZero);
    EXPECT_NEAR(previous, -0.15, kTolerance);
    bool zeroWritten = false;
    for (const auto& duty : rig.motors.writes()) zeroWritten = zeroWritten || (duty.m1 == 0 && duty.m2 == 0);
    EXPECT_TRUE(zeroWritten) << "a zero duty reaches both bridges between the two directions";
}

TEST(BoundedDriveOutputTest, StopRampReachesZeroWithinTheB6BudgetAndNeverReverses)
{
    Rig rig;
    // 7 rad/s needs 0.21 m/s per wheel: the spin is saturated to full scale on both.
    for (int i = 0; i < 2 * kFullScaleSteps; ++i) ASSERT_TRUE(rig.command({0.0, 7.0}));
    ASSERT_NEAR(std::abs(rig.drive.wheelIntent().rightMetresPerSecond), 0.2, kTolerance);
    const auto startLeft = rig.drive.wheelIntent().leftMetresPerSecond;
    int steps = 0;
    StopRampStep result = StopRampStep::Ramping;
    double previous = std::abs(startLeft);
    while (result == StopRampStep::Ramping && steps < 100) {
        result = rig.ramp();
        ++steps;
        const double magnitude = std::abs(rig.drive.wheelIntent().leftMetresPerSecond);
        ASSERT_LE(magnitude, previous) << "the stop ramp only shrinks the output";
        ASSERT_FALSE(magnitude > 0.0 && std::signbit(rig.drive.wheelIntent().leftMetresPerSecond) != std::signbit(startLeft));
        previous = magnitude;
    }
    EXPECT_EQ(result, StopRampStep::AtRest);
    EXPECT_LE(steps, kStopSteps) << "full scale to zero within kStopRampMax (B6)";
    EXPECT_EQ(*rig.motors.lastWrite(), MotorDuty{}) << "the final ramp write is zero on both bridges";
    EXPECT_EQ(rig.ramp(), StopRampStep::AtRest) << "at rest stays at rest";
}

TEST(BoundedDriveOutputTest, DisableCoastsImmediatelyAndResetsTheRamps)
{
    Rig rig;
    for (int i = 0; i < kFullScaleSteps; ++i) ASSERT_TRUE(rig.command({0.15, 0.0}));
    ASSERT_NE(left(rig.drive.lastDuty()), 0);
    const auto writes = rig.motors.count(MotorEvent::Kind::Write);
    ASSERT_TRUE(rig.drive.disable());
    EXPECT_TRUE(rig.motors.lastIsCoast()) << "no ramp write before the coast";
    EXPECT_EQ(rig.motors.count(MotorEvent::Kind::Write), writes);
    EXPECT_EQ(rig.drive.lastDuty(), MotorDuty{});
    // Re-enabling starts from zero, not from the speed before the disable.
    ASSERT_TRUE(rig.command({0.15, 0.0}));
    EXPECT_NEAR(rig.drive.wheelIntent().leftMetresPerSecond, kStepSpeed, kTolerance);
    rig.motors.failCoast = true;
    EXPECT_FALSE(rig.drive.disable()) << "an unconfirmed disable is reported";
}

TEST(BoundedDriveOutputTest, EveryInputStaysWithinTheBoundOnBothChannels)
{
    Rig rig;
    const int bound = dutyBoundCounts(kProfile);
    const double values[] = {-1e9, -10.0, -0.2, -0.15, -0.01, 0.0, 0.01, 0.15, 0.2, 10.0, 1e9};
    for (const double v : values) {
        for (const double w : values) {
            for (int i = 0; i < 3; ++i) {
                if (!rig.command({v, w})) continue;
                const auto duty = *rig.motors.lastWrite();
                ASSERT_LE(std::abs(duty.m1), bound) << v << ", " << w;
                ASSERT_LE(std::abs(duty.m2), bound) << v << ", " << w;
            }
        }
    }
    EXPECT_GT(rig.motors.count(MotorEvent::Kind::Write), 0);
}

TEST(BoundedDriveOutputTest, RefusedMotorWriteIsReported)
{
    Rig rig;
    rig.motors.failWrite = true;
    EXPECT_FALSE(rig.command({0.1, 0.0}));
    EXPECT_EQ(rig.ramp(), StopRampStep::Refused);
}
