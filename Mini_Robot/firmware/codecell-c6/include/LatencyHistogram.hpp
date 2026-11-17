#pragma once
#include "FirmwareClock.hpp"
#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace codecell::timing {
// Fixed-memory latency histogram for instrumentation (#156). It never allocates,
// so it can be recorded from a measured task without perturbing the heap.
// Values below kExactLimit microseconds are exact; above it each octave has
// kSubBucketsPerOctave buckets (relative bucket width <= 1/16). Percentiles are
// reported as the inclusive upper edge of the bucket that holds the rank, never
// above the recorded maximum, so a reported bound never understates a sample.
// Not thread-safe: one writer, read after the writer has stopped.
class LatencyHistogram {
public:
    static constexpr std::uint64_t kExactLimit = 64;
    static constexpr std::uint64_t kSubBucketsPerOctave = 16;
    static constexpr std::uint64_t kOctaves = 26; // Covers up to 64 us << 26, about 71 minutes.
    static constexpr std::size_t kBucketCount = kExactLimit + kOctaves * kSubBucketsPerOctave;

    void record(Microseconds value)
    {
        if (value < Microseconds::zero()) {
            ++negative_;
            return;
        }
        const auto us = static_cast<std::uint64_t>(value.count());
        ++buckets_[bucketIndex(us)];
        ++count_;
        min_ = std::min(min_, us);
        max_ = std::max(max_, us);
    }
    void reset() { *this = LatencyHistogram{}; }

    [[nodiscard]] std::uint64_t count() const { return count_; }
    // Negative samples (for example a reading before its reference) are counted, not binned.
    [[nodiscard]] std::uint64_t negativeCount() const { return negative_; }
    [[nodiscard]] Microseconds min() const { return Microseconds{count_ ? toRep(min_) : 0}; }
    [[nodiscard]] Microseconds max() const { return Microseconds{toRep(max_)}; }

    // Conservative bound for the q-quantile, 0 < q <= 1 (q is clamped into that range).
    [[nodiscard]] Microseconds percentileUpperBound(double q) const
    {
        if (count_ == 0) return Microseconds::zero();
        q = std::clamp(q, std::numeric_limits<double>::min(), 1.0);
        auto rank = static_cast<std::uint64_t>(q * static_cast<double>(count_));
        if (static_cast<double>(rank) < q * static_cast<double>(count_)) ++rank; // ceil
        rank = std::clamp<std::uint64_t>(rank, 1, count_);
        std::uint64_t cumulative = 0;
        for (std::size_t i = 0; i < kBucketCount; ++i) {
            cumulative += buckets_[i];
            if (cumulative >= rank) return Microseconds{toRep(std::min(upperEdge(i), max_))};
        }
        return max();
    }

    [[nodiscard]] static constexpr std::size_t bucketIndex(std::uint64_t us)
    {
        if (us < kExactLimit) return static_cast<std::size_t>(us);
        const auto octave = static_cast<std::uint64_t>(std::bit_width(us)) - std::bit_width(kExactLimit);
        if (octave >= kOctaves) return kBucketCount - 1;
        const auto sub = (us >> (octave + 2)) - kSubBucketsPerOctave;
        return static_cast<std::size_t>(kExactLimit + octave * kSubBucketsPerOctave + sub);
    }
    // Largest value that maps to bucket `index` (the last bucket also holds overflow).
    [[nodiscard]] static constexpr std::uint64_t upperEdge(std::size_t index)
    {
        if (index < kExactLimit) return index;
        if (index >= kBucketCount - 1) return std::numeric_limits<std::uint64_t>::max();
        const auto octave = (index - kExactLimit) / kSubBucketsPerOctave;
        const auto sub = (index - kExactLimit) % kSubBucketsPerOctave;
        const auto width = std::uint64_t{4} << octave;
        return (kExactLimit << octave) + (sub + 1) * width - 1;
    }

private:
    static constexpr Microseconds::rep toRep(std::uint64_t value)
    {
        constexpr auto limit = static_cast<std::uint64_t>(std::numeric_limits<Microseconds::rep>::max());
        return static_cast<Microseconds::rep>(std::min(value, limit));
    }
    std::array<std::uint32_t, kBucketCount> buckets_{};
    std::uint64_t count_{};
    std::uint64_t negative_{};
    std::uint64_t min_{std::numeric_limits<std::uint64_t>::max()};
    std::uint64_t max_{};
};
} // namespace codecell::timing
