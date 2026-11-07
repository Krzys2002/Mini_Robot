#include "FirmwareClock.hpp"
#include <gtest/gtest.h>
#include <limits>
using namespace codecell;
using namespace std::chrono_literals;
namespace {
class FakeClock final : public IFirmwareClock {
public:
    Milliseconds time{};
    Milliseconds now() const override { return time; }
};
TEST(FirmwareClockTest, InjectedClockMeasuresElapsedWithoutSleeping)
{
    FakeClock clock;
    clock.time = 100ms;
    const auto start = clock.now();
    clock.time += 25ms;
    EXPECT_EQ(elapsedSince(clock, start), 25ms);
    EXPECT_EQ(elapsedSince(clock, clock.now()), 0ms);
}
TEST(FirmwareClockTest, FutureAndNegativeReadingsAreUnavailable)
{
    FakeClock clock;
    clock.time = 100ms;
    EXPECT_FALSE(elapsedSince(clock, 101ms));
    EXPECT_FALSE(elapsedSince(clock, -1ms));
    clock.time = -1ms;
    EXPECT_FALSE(elapsedSince(clock, 0ms));
}
TEST(FirmwareClockTest, TimeDoesNotWrapAtUint32Milliseconds)
{
    FakeClock clock;
    const auto start = Milliseconds{static_cast<long long>(std::numeric_limits<std::uint32_t>::max())};
    clock.time = start + 1000ms;
    EXPECT_EQ(elapsedSince(clock, start), 1000ms);
}
}
