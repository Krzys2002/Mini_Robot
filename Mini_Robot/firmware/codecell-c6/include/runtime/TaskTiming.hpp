#pragma once
// Periodic task timing for diagnostics (#164; #156 I6 wake lateness, and step durations).
// The platform adapter (src/EspRuntimeAdapters.hpp) records into one TaskTiming per task from
// that task's own loop; the lowest-priority reporter reads it. Portable C++20, lock-free
// 32-bit atomics, no allocation. Recording costs two clock reads and a few atomic stores per
// step; it changes no schedule and no safety decision.
//
// Wake lateness is measured against the task's tick grid: each wake is expected one period
// of ticks after the previous expected wake. The grid's phase is the earliest wake seen so
// far, so the first samples may understate lateness by at most the phase error of the first
// wake, and the estimate only improves.
#include "runtime/RuntimeTypes.hpp"
#include <atomic>
#include <cstdint>
#include <limits>

namespace codecell::runtime {

struct TaskTimingSnapshot {
    std::uint32_t steps{0};              // Completed steps (wraps).
    std::uint32_t overruns{0};           // Wake-ups that were already due (the task fell behind).
    std::uint32_t latenessMaxUs{0};      // Since the previous takeWindow().
    std::uint32_t latenessWorstUs{0};    // Since boot.
    std::uint32_t stepMaxUs{0};          // Since the previous takeWindow().
    std::uint32_t stepWorstUs{0};        // Since boot.
};

class TaskTiming {
public:
    // Task context. `wakeUs`: clock at the return from the periodic wait; `tick`: the tick the
    // task was scheduled for; `tickUs`: the tick length.
    void woke(std::int64_t wakeUs, std::uint32_t tick, std::int64_t tickUs) noexcept
    {
        if (!started_) {
            started_ = true;
            idealUs_ = wakeUs;
        } else {
            idealUs_ += static_cast<std::int64_t>(static_cast<std::uint32_t>(tick - lastTick_)) * tickUs;
            if (wakeUs < idealUs_) idealUs_ = wakeUs;  // An earlier phase than assumed so far.
        }
        lastTick_ = tick;
        record(latenessMax_, latenessWorst_, wakeUs - idealUs_);
    }

    // Task context: the wait returned at once because the next wake was already due.
    void overrun() noexcept { overruns_.fetch_add(1, std::memory_order_relaxed); }

    // Task context: one step took `duration`.
    void stepCompleted(Instant duration) noexcept
    {
        steps_.fetch_add(1, std::memory_order_relaxed);
        record(stepMax_, stepWorst_, duration.count());
    }

    // Reporter (one caller): the counters, the window maxima since its previous call.
    [[nodiscard]] TaskTimingSnapshot takeWindow() noexcept
    {
        TaskTimingSnapshot out;
        out.steps = steps_.load(std::memory_order_relaxed);
        out.overruns = overruns_.load(std::memory_order_relaxed);
        out.latenessMaxUs = latenessMax_.exchange(0, std::memory_order_relaxed);
        out.latenessWorstUs = latenessWorst_.load(std::memory_order_relaxed);
        out.stepMaxUs = stepMax_.exchange(0, std::memory_order_relaxed);
        out.stepWorstUs = stepWorst_.load(std::memory_order_relaxed);
        return out;
    }

private:
    static void raiseTo(std::atomic<std::uint32_t>& slot, std::uint32_t value) noexcept
    {
        auto current = slot.load(std::memory_order_relaxed);
        while (value > current && !slot.compare_exchange_weak(current, value, std::memory_order_relaxed)) {}
    }
    static void record(std::atomic<std::uint32_t>& window, std::atomic<std::uint32_t>& worst, std::int64_t us) noexcept
    {
        constexpr std::int64_t limit = std::numeric_limits<std::uint32_t>::max();
        const auto value = static_cast<std::uint32_t>(us < 0 ? 0 : (us > limit ? limit : us));
        raiseTo(window, value);
        raiseTo(worst, value);
    }

    // Task context only.
    bool started_{false};
    std::int64_t idealUs_{0};
    std::uint32_t lastTick_{0};
    // Published.
    std::atomic<std::uint32_t> steps_{0};
    std::atomic<std::uint32_t> overruns_{0};
    std::atomic<std::uint32_t> latenessMax_{0};
    std::atomic<std::uint32_t> latenessWorst_{0};
    std::atomic<std::uint32_t> stepMax_{0};
    std::atomic<std::uint32_t> stepWorst_{0};
};
} // namespace codecell::runtime
