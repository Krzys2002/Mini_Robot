// Unit tests of the #160 per-wheel command-rate limiter: exact acceleration and
// deceleration boundaries, zero elapsed time and reversal through an exact zero.
#include "drive/WheelRateLimiter.hpp"
#include <gtest/gtest.h>
#include <cmath>

using namespace codecell::drive;

TEST(WheelRateLimiterTest, AccelerationAndDecelerationBoundariesAreExact)
{
    WheelRateLimiter limiter;
    const RampRates rates{2.0, 4.0};
    EXPECT_DOUBLE_EQ(limiter.step(1.0, 0.01, rates), 0.02) << "growth limited by acceleration";
    EXPECT_DOUBLE_EQ(limiter.step(0.025, 0.01, rates), 0.025) << "arrives exactly when within one step";
    EXPECT_DOUBLE_EQ(limiter.step(0.0, 0.001, rates), 0.021) << "shrink limited by deceleration";
    EXPECT_DOUBLE_EQ(limiter.step(0.021, 0.0, rates), 0.021) << "zero elapsed time changes nothing";
    limiter.reset();
    EXPECT_EQ(limiter.current(), 0.0);
}

TEST(WheelRateLimiterTest, ReversalStopsAtExactlyZeroBeforeChangingSign)
{
    WheelRateLimiter limiter;
    const RampRates rates{10.0, 10.0};
    limiter.step(0.05, 0.01, rates);
    ASSERT_DOUBLE_EQ(limiter.current(), 0.05);
    // A large step toward the opposite sign must not cross zero in one step.
    EXPECT_EQ(limiter.step(-0.05, 1.0, rates), 0.0);
    EXPECT_FALSE(std::signbit(limiter.current()));
    EXPECT_DOUBLE_EQ(limiter.step(-0.05, 1.0, rates), -0.05) << "the new sign starts on the next step";
}
