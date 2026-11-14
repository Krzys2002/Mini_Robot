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
TEST(FirmwareClockTest, MicrosecondsSinceBootTruncateToWholeMilliseconds)
{
    EXPECT_EQ(monotonicMilliseconds(0), 0ms);
    EXPECT_EQ(monotonicMilliseconds(999), 0ms);
    EXPECT_EQ(monotonicMilliseconds(1'000), 1ms);
    EXPECT_EQ(monotonicMilliseconds(1'999), 1ms);
    // One hour after boot: beyond the range of a 16-bit millisecond counter.
    EXPECT_EQ(monotonicMilliseconds(3'600'000'000), 3'600'000ms);
}
TEST(FirmwareClockTest, NegativeMicrosecondCountsAreUnavailable)
{
    EXPECT_FALSE(monotonicMilliseconds(-1));
    EXPECT_FALSE(monotonicMilliseconds(std::numeric_limits<std::int64_t>::min()));
}
TEST(FirmwareClockTest, MicrosecondConversionDoesNotWrapAtUint32Milliseconds)
{
    constexpr std::int64_t uint32MillisecondsInMicroseconds =
        static_cast<std::int64_t>(std::numeric_limits<std::uint32_t>::max()) * 1'000;
    EXPECT_EQ(monotonicMilliseconds(uint32MillisecondsInMicroseconds + 1'000'000),
              Milliseconds{static_cast<long long>(std::numeric_limits<std::uint32_t>::max())} + 1'000ms);
}
TEST(FirmwareClockTest, TimeDoesNotWrapAtUint32Milliseconds)
{
    FakeClock clock;
    const auto start = Milliseconds{static_cast<long long>(std::numeric_limits<std::uint32_t>::max())};
    clock.time = start + 1000ms;
    EXPECT_EQ(elapsedSince(clock, start), 1000ms);
}
}
