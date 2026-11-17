#include "LatencyHistogram.hpp"
#include <gtest/gtest.h>
#include <limits>
using codecell::Microseconds;
using codecell::timing::LatencyHistogram;
using namespace std::chrono_literals;
namespace {
TEST(LatencyHistogramTest, EmptyHistogramReportsZero)
{
    const LatencyHistogram h;
    EXPECT_EQ(h.count(), 0u);
    EXPECT_EQ(h.min(), 0us);
    EXPECT_EQ(h.max(), 0us);
    EXPECT_EQ(h.percentileUpperBound(0.99), 0us);
}
TEST(LatencyHistogramTest, SmallValuesAreExact)
{
    LatencyHistogram h;
    for (int value = 1; value <= 10; ++value) h.record(Microseconds{value});
    EXPECT_EQ(h.min(), 1us);
    EXPECT_EQ(h.max(), 10us);
    EXPECT_EQ(h.percentileUpperBound(0.5), 5us);
    EXPECT_EQ(h.percentileUpperBound(0.9), 9us);
    EXPECT_EQ(h.percentileUpperBound(1.0), 10us);
}
TEST(LatencyHistogramTest, PercentileNeverUnderstatesASampleAndNeverExceedsMax)
{
    LatencyHistogram h;
    constexpr int kSamples = 1000;
    for (int i = 1; i <= kSamples; ++i) h.record(Microseconds{i * 37});
    for (const double q : {0.5, 0.9, 0.99, 0.999}) {
        const auto rankValue = Microseconds{static_cast<long long>(q * kSamples) * 37};
        const auto bound = h.percentileUpperBound(q);
        EXPECT_GE(bound, rankValue) << q;
        // Relative bucket width is at most 1/16 above the exact range.
        EXPECT_LE(bound.count(), rankValue.count() + rankValue.count() / 16 + 1) << q;
        EXPECT_LE(bound, h.max());
    }
}
TEST(LatencyHistogramTest, BucketEdgesAreContiguousAndMonotonic)
{
    for (std::size_t i = 0; i + 2 < LatencyHistogram::kBucketCount; ++i) {
        const auto edge = LatencyHistogram::upperEdge(i);
        EXPECT_EQ(LatencyHistogram::bucketIndex(edge), i);
        EXPECT_EQ(LatencyHistogram::bucketIndex(edge + 1), i + 1);
    }
}
TEST(LatencyHistogramTest, NegativeSamplesAreCountedSeparately)
{
    LatencyHistogram h;
    h.record(-1us);
    h.record(5us);
    EXPECT_EQ(h.count(), 1u);
    EXPECT_EQ(h.negativeCount(), 1u);
    EXPECT_EQ(h.min(), 5us);
}
TEST(LatencyHistogramTest, HugeValuesSaturateInTheLastBucketWithExactMax)
{
    LatencyHistogram h;
    const auto huge = Microseconds{std::numeric_limits<Microseconds::rep>::max()};
    h.record(huge);
    EXPECT_EQ(h.max(), huge);
    EXPECT_EQ(h.percentileUpperBound(1.0), huge);
}
TEST(LatencyHistogramTest, ResetClearsEverything)
{
    LatencyHistogram h;
    h.record(3us);
    h.record(-3us);
    h.reset();
    EXPECT_EQ(h.count(), 0u);
    EXPECT_EQ(h.negativeCount(), 0u);
    EXPECT_EQ(h.max(), 0us);
}
} // namespace
