#pragma once
// Telemetry publication counters and spacing statistics (#164; #156 I11). Written only in the
// BLE host context, read by the lowest-priority reporter. Lock-free 32-bit atomics: counters
// wrap modulo 2^32 (use counterDelta() for windows); the interval extremes are per report
// window and are reset by takeWindow().
#include "telemetry/TelemetryScheduler.hpp"
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>

namespace codecell::telemetry {

enum class TelemetryCounter : std::uint8_t {
    Ticks,               // Schedule evaluations in the host context.
    TicksNotReady,       // Evaluations while no session was ready (nothing is sent).
    SessionStartFrames,  // Queued frames, by reason.
    StateChangeFrames,
    PeriodicFrames,
    NotifyFailures,      // The stack could not queue a frame: dropped and counted (#156 overload policy).
    EncodeFailures,      // A value equal to its sentinel: the frame was not sent.
    Count_,
};

struct TelemetryCounters {
    std::array<std::uint32_t, static_cast<std::size_t>(TelemetryCounter::Count_)> counts{};
    [[nodiscard]] std::uint32_t operator[](TelemetryCounter counter) const noexcept
    {
        return counts[static_cast<std::size_t>(counter)];
    }
    [[nodiscard]] std::uint32_t framesSent() const noexcept
    {
        return (*this)[TelemetryCounter::SessionStartFrames] + (*this)[TelemetryCounter::StateChangeFrames]
             + (*this)[TelemetryCounter::PeriodicFrames];
    }
};

// Gap statistics between consecutive queued frames of one session, in microseconds (values
// saturate just below 2^32). Empty fields (no sample in the window) are reported as none.
struct TelemetryWindow {
    std::optional<std::uint32_t> minIntervalUs{};
    std::optional<std::uint32_t> maxIntervalUs{};
    std::optional<std::uint32_t> periodicDeviationMaxUs{};  // max |gap - period| over periodic frames.
};

// Modular difference of two readings of a wrapping counter (newer - older).
[[nodiscard]] constexpr std::uint32_t counterDelta(std::uint32_t newer, std::uint32_t older) noexcept
{
    return static_cast<std::uint32_t>(newer - older);
}

class TelemetryDiagnostics {
public:
    void count(TelemetryCounter counter) noexcept
    {
        counters_[static_cast<std::size_t>(counter)].fetch_add(1, std::memory_order_relaxed);
    }

    void countFrame(FrameReason reason) noexcept
    {
        switch (reason) {
        case FrameReason::SessionStart: count(TelemetryCounter::SessionStartFrames); return;
        case FrameReason::StateChange: count(TelemetryCounter::StateChangeFrames); return;
        case FrameReason::Periodic: count(TelemetryCounter::PeriodicFrames); return;
        case FrameReason::None: return;
        }
    }

    // A queued frame `gap` after the previous queued frame of the same session.
    void recordGap(Instant gap, FrameReason reason, std::chrono::milliseconds period) noexcept
    {
        const auto us = clampToU32(gap);
        lowerTo(minIntervalUs_, us);
        raiseTo(maxIntervalUs_, us + 1);  // Maxima are stored plus one, so 0 can mean "none".
        if (reason == FrameReason::Periodic) {
            const auto deviation = gap - Instant{period};
            raiseTo(periodicDeviationMaxUs_, clampToU32(deviation < Instant::zero() ? -deviation : deviation) + 1);
        }
    }

    [[nodiscard]] TelemetryCounters counters() const noexcept
    {
        TelemetryCounters out;
        for (std::size_t i = 0; i < out.counts.size(); ++i) out.counts[i] = counters_[i].load(std::memory_order_relaxed);
        return out;
    }

    // Reporter: the extremes since the previous call, then a fresh window.
    [[nodiscard]] TelemetryWindow takeWindow() noexcept
    {
        TelemetryWindow out;
        out.minIntervalUs = present(minIntervalUs_.exchange(kNoMinimum, std::memory_order_relaxed), kNoMinimum);
        out.maxIntervalUs = maximum(maxIntervalUs_.exchange(kNoMaximum, std::memory_order_relaxed));
        out.periodicDeviationMaxUs = maximum(periodicDeviationMaxUs_.exchange(kNoMaximum, std::memory_order_relaxed));
        return out;
    }

private:
    static constexpr std::uint32_t kNoMinimum = std::numeric_limits<std::uint32_t>::max();
    static constexpr std::uint32_t kNoMaximum = 0;  // Maxima are stored plus one.

    [[nodiscard]] static constexpr std::uint32_t clampToU32(Instant value) noexcept
    {
        if (value.count() <= 0) return 0;
        constexpr auto limit = static_cast<Instant::rep>(std::numeric_limits<std::uint32_t>::max() - 1);
        return static_cast<std::uint32_t>(value.count() > limit ? limit : value.count());
    }
    [[nodiscard]] static std::optional<std::uint32_t> present(std::uint32_t value, std::uint32_t none) noexcept
    {
        return value == none ? std::nullopt : std::optional<std::uint32_t>{value};
    }
    [[nodiscard]] static std::optional<std::uint32_t> maximum(std::uint32_t storedPlusOne) noexcept
    {
        return storedPlusOne == kNoMaximum ? std::nullopt : std::optional<std::uint32_t>{storedPlusOne - 1};
    }
    // Compare-and-swap, so a sample is never lost to a concurrent takeWindow().
    static void raiseTo(std::atomic<std::uint32_t>& slot, std::uint32_t value) noexcept
    {
        auto current = slot.load(std::memory_order_relaxed);
        while (value > current && !slot.compare_exchange_weak(current, value, std::memory_order_relaxed)) {}
    }
    static void lowerTo(std::atomic<std::uint32_t>& slot, std::uint32_t value) noexcept
    {
        auto current = slot.load(std::memory_order_relaxed);
        while (value < current && !slot.compare_exchange_weak(current, value, std::memory_order_relaxed)) {}
    }

    std::array<std::atomic<std::uint32_t>, static_cast<std::size_t>(TelemetryCounter::Count_)> counters_{};
    std::atomic<std::uint32_t> minIntervalUs_{kNoMinimum};
    std::atomic<std::uint32_t> maxIntervalUs_{kNoMaximum};
    std::atomic<std::uint32_t> periodicDeviationMaxUs_{kNoMaximum};
};

[[nodiscard]] constexpr const char* toString(TelemetryCounter counter) noexcept
{
    switch (counter) {
    case TelemetryCounter::Ticks: return "ticks";
    case TelemetryCounter::TicksNotReady: return "ticks_not_ready";
    case TelemetryCounter::SessionStartFrames: return "session_start";
    case TelemetryCounter::StateChangeFrames: return "state_change";
    case TelemetryCounter::PeriodicFrames: return "periodic";
    case TelemetryCounter::NotifyFailures: return "notify_failures";
    case TelemetryCounter::EncodeFailures: return "encode_failures";
    case TelemetryCounter::Count_: break;
    }
    return "unknown";
}
} // namespace codecell::telemetry
