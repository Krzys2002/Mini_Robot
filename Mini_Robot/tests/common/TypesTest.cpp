#include "common/Types.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <concepts>
#include <cstdint>
#include <map>
#include <numbers>
#include <type_traits>
#include <unordered_map>

namespace micro_robot {
namespace {

static_assert(std::totally_ordered<RobotId>);
static_assert(!std::is_convertible_v<std::uint32_t, RobotId>);
static_assert(!std::is_convertible_v<RobotId, std::uint32_t>);
static_assert(!std::is_convertible_v<double, Meters>);
static_assert(!std::is_convertible_v<double, Radians>);
static_assert(!std::is_convertible_v<Meters, double>);
static_assert(!std::is_convertible_v<Radians, double>);
static_assert(!std::is_constructible_v<Radians, Meters>);
static_assert(!std::is_constructible_v<Meters, Radians>);
static_assert(MonotonicClock::is_steady);
static_assert(std::is_same_v<Timestamp::clock, std::chrono::steady_clock>);

TEST(RobotIdTest, ComparesByValue)
{
    constexpr RobotId first{7};
    constexpr RobotId same{7};
    constexpr RobotId next{8};

    EXPECT_EQ(first.value(), 7U);
    EXPECT_EQ(first, same);
    EXPECT_NE(first, next);
    EXPECT_LT(first, next);
}

TEST(RobotIdTest, WorksAsAnOrderedContainerKey)
{
    const std::map<RobotId, int> robots{{RobotId{8}, 80}, {RobotId{0}, 10}};

    EXPECT_EQ(robots.begin()->first, RobotId{0});
    EXPECT_EQ(robots.at(RobotId{8}), 80);
}

TEST(RobotIdTest, WorksAsAnUnorderedContainerKey)
{
    std::unordered_map<RobotId, int> robots;
    robots.emplace(RobotId{7}, 10);
    robots.insert_or_assign(RobotId{7}, 20);
    robots.emplace(RobotId{8}, 30);

    EXPECT_EQ(robots.size(), 2U);
    EXPECT_EQ(robots.at(RobotId{7}), 20);
    EXPECT_EQ(robots.at(RobotId{8}), 30);
}

TEST(Pose2DTest, DefaultsToWorldOriginAndZeroHeading)
{
    const Pose2D pose;

    EXPECT_DOUBLE_EQ(pose.x.value, 0.0);
    EXPECT_DOUBLE_EQ(pose.y.value, 0.0);
    EXPECT_DOUBLE_EQ(pose.heading.value, 0.0);
}

TEST(Pose2DTest, StoresMetricCoordinatesAndUnnormalizedRadians)
{
    const Pose2D pose{Meters{1.25}, Meters{-0.5}, Radians{3.0 * std::numbers::pi}};

    EXPECT_DOUBLE_EQ(pose.x.value, 1.25);
    EXPECT_DOUBLE_EQ(pose.y.value, -0.5);
    EXPECT_DOUBLE_EQ(pose.heading.value, 3.0 * std::numbers::pi);
}

TEST(TimestampTest, SupportsChronoOrderingAndElapsedTime)
{
    using namespace std::chrono_literals;
    const Timestamp start{std::chrono::duration_cast<Duration>(2s)};
    const Timestamp finish = start + 150ms;
    const Duration elapsed = finish - start;

    EXPECT_GT(finish, start);
    EXPECT_EQ(elapsed, 150ms);
    EXPECT_DOUBLE_EQ(std::chrono::duration<double>(elapsed).count(), 0.15);
}

} // namespace
} // namespace micro_robot
