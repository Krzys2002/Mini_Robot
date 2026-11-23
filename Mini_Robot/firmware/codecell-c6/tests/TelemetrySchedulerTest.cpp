// Host tests of the #164 telemetry schedule (B10) on a fake microsecond clock: session start,
// periodic spacing on the tick grid under lateness, coalesced state changes, the rate bound,
// congestion without retry bursts, and the uptime and counter wrap points.
#include "TimingBudget.hpp"
#include "telemetry/TelemetryDiagnostics.hpp"
#include "telemetry/TelemetryScheduler.hpp"
#include <gtest/gtest.h>
#include <algorithm>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

using namespace codecell;
using namespace codecell::telemetry;
using namespace std::chrono_literals;
using runtime::MotionState;

namespace {
constexpr Instant kTick{timing::kTelemetryTickPeriod};
constexpr Instant kPeriod{timing::kTelemetryPeriod};
constexpr Instant kEligible{timing::kTelemetryPeriodicEligibility};  // 99 ms.
constexpr TelemetryKey kIdle{MotionState::Idle, false, false};
constexpr TelemetryKey kMoving{MotionState::Moving, false, false};
constexpr TelemetryKey kExpired{MotionState::Idle, true, false};
constexpr TelemetryKey kLatched{MotionState::EmergencyStop, false, false};

struct Sent {
    Instant at;
    FrameReason reason;
};

// Drives the scheduler on the tick grid. `lateness(k)` is how late tick k is delivered and
// `keyAt(t)` the state the runtime reports then; `queued(k)` says whether the stack takes it.
template <class Lateness, class Key, class Queued>
std::vector<Sent> simulate(Instant start, Instant duration, Lateness lateness, Key keyAt, Queued queued)
{
    TelemetryScheduler scheduler{timing::kTelemetryPeriod};
    scheduler.sessionStarted();
    std::vector<Sent> sent;
    for (std::int64_t k = 0; k * kTick < duration; ++k) {
        const Instant now = start + k * kTick + lateness(k);
        const auto key = keyAt(now);
        const auto reason = scheduler.due(now, key);
        if (reason == FrameReason::None) continue;
        const bool ok = queued(k);
        scheduler.attempted(now, key, ok);
        if (ok) sent.push_back({now, reason});
    }
    return sent;
}
auto noLateness = [](std::int64_t) { return Instant::zero(); };
auto alwaysQueued = [](std::int64_t) { return true; };
} // namespace

TEST(TelemetrySchedulerTest, FirstFrameOfASessionGoesOutAtTheFirstTick)
{
    TelemetryScheduler scheduler{timing::kTelemetryPeriod};
    EXPECT_EQ(scheduler.due(1s, kIdle), FrameReason::SessionStart);
    scheduler.attempted(1s, kIdle, true);
    EXPECT_EQ(scheduler.due(1s + kTick, kIdle), FrameReason::None);
    EXPECT_EQ(scheduler.lastAttempt(), Instant{1s});
}

TEST(TelemetrySchedulerTest, PeriodicFramesFollowTheEligibilityOnTheMillisecondChecks)
{
    // Checked every millisecond without lateness, a periodic frame goes out as soon as it is
    // eligible (99 ms), one millisecond inside the B10 deviation of the 100 ms period.
    const auto sent = simulate(1s, 10s, noLateness, [](Instant) { return kIdle; }, alwaysQueued);
    ASSERT_GE(sent.size(), 100U);
    EXPECT_EQ(sent.front().reason, FrameReason::SessionStart);
    for (std::size_t i = 1; i < sent.size(); ++i) {
        EXPECT_EQ(sent[i].reason, FrameReason::Periodic);
        EXPECT_EQ(sent[i].at - sent[i - 1].at, kEligible) << i;
    }
}

TEST(TelemetrySchedulerTest, SchedulingLatenessUpToTheAllowanceKeepsTheB10Deviation)
{
    // Every tick late by a different amount within the #156 scheduling allowance (2 ms).
    const auto lateness = [](std::int64_t k) { return Instant{(k * 7919) % (timing::kSchedulingAllowance.count() + 1)}; };
    const auto sent = simulate(1s, 20s, lateness, [](Instant) { return kIdle; }, alwaysQueued);
    ASSERT_GT(sent.size(), 150U);
    for (std::size_t i = 1; i < sent.size(); ++i) {
        const auto gap = sent[i].at - sent[i - 1].at;
        const auto deviation = gap > kPeriod ? gap - kPeriod : kPeriod - gap;
        EXPECT_LE(deviation, timing::kTelemetryPeriodDeviationMax) << i;  // B10: robot-side deviation <= 2 ms.
    }
}

// Review finding (#283): the scheduler admitted a state change 17.5 ms and a periodic frame
// 97.5 ms after the previous frame. The approved B10 values are >= 20 ms and 100 +/- 2 ms.
TEST(TelemetrySchedulerTest, BoundariesAreTheApprovedB10Values)
{
    static_assert(timing::kStateTelemetryMinSpacing == 20ms);
    static_assert(timing::kTelemetryPeriodDeviationMax == 2ms);
    static_assert(timing::kTelemetryPeriod == 100ms);
    TelemetryScheduler scheduler{timing::kTelemetryPeriod};
    scheduler.attempted(Instant::zero(), kIdle, true);
    EXPECT_EQ(scheduler.due(17'500us, kMoving), FrameReason::None) << "the review probe";
    EXPECT_EQ(scheduler.due(20ms - 1us, kMoving), FrameReason::None);
    EXPECT_EQ(scheduler.due(20ms, kMoving), FrameReason::StateChange);
    EXPECT_EQ(scheduler.due(97'500us, kIdle), FrameReason::None) << "the review probe";
    static_assert(timing::kTelemetryPeriodicEligibility >= 98ms, "never earlier than 100 - 2 ms");
    EXPECT_EQ(scheduler.due(kEligible - 1us, kIdle), FrameReason::None);
    EXPECT_EQ(scheduler.due(kEligible, kIdle), FrameReason::Periodic);
    EXPECT_EQ(scheduler.due(102ms, kIdle), FrameReason::Periodic);
}

// The model behind the rule (#283 review). Two sources evaluate the schedule in the host:
// - the telemetry task's check every millisecond, each late by any amount within the
//   scheduling allowance (events are processed in order, so lateness never reorders them);
// - ordinary write callbacks at arbitrary, off-grid instants. They publish a due periodic or
//   state-change frame, never the session's first frame. Some are sparse, and some come in
//   bursts that starve the task's checks entirely, as measured on the bench.
// The state changes at random. Every frame is at least 20 ms after the previous one, every
// periodic gap is 100 +/- 2 ms, at most 60 frames fall in any second, and a state change is
// sent no later than 20 ms after the previous frame plus one check and the lateness.
TEST(TelemetrySchedulerTest, TickAndWriteModelMeetsEveryB10BoundAtOnce)
{
    std::uint32_t seed = 0x2834u;
    const auto next = [&seed] {
        seed = seed * 1664525u + 1013904223u;
        return seed >> 8;
    };
    const auto allowance = timing::kSchedulingAllowance.count();
    const Instant stateDelayBound =
        Instant{timing::kStateTelemetryMinSpacing} + kTick + Instant{timing::kSchedulingAllowance};
    struct Evaluation {
        Instant at;
        bool fromWrite;
    };
    for (int pattern = 0; pattern < 160; ++pattern) {
        const Instant start = 1s + Instant{static_cast<std::int64_t>(next() % 5000)};
        const Instant length = 3s;
        // Bursts: [begin, end) during which the task's checks are starved.
        std::vector<std::pair<Instant, Instant>> bursts;
        for (Instant b = start + 50ms; b < start + length;) {
            const Instant gap{static_cast<std::int64_t>(20'000 + next() % 400'000)};
            const Instant duration{static_cast<std::int64_t>(5'000 + next() % 60'000)};
            bursts.push_back({b + gap, b + gap + duration});
            b += gap + duration;
        }
        const auto inBurst = [&](Instant t) {
            for (const auto& [b, e] : bursts)
                if (t >= b && t < e) return true;
            return false;
        };
        std::vector<Evaluation> events;
        Instant lastTick{0};
        for (std::int64_t k = 0; k * kTick < length; ++k) {
            const Instant grid = start + k * kTick;
            if (inBurst(grid)) continue;
            Instant late{0};
            switch (pattern % 4) {
            case 0: late = Instant{static_cast<std::int64_t>(next() % (allowance + 1))}; break;  // Random.
            case 1: late = Instant{k % 2 == 0 ? 0 : allowance}; break;                        // Alternating.
            case 2: late = Instant{(k * allowance / 7) % (allowance + 1)}; break;              // Ramps.
            case 3: late = Instant{(k / 20) % 2 == 0 ? allowance : 0}; break;                  // Steps.
            }
            lastTick = std::max(lastTick, grid + late);
            events.push_back({lastTick, false});
        }
        for (const auto& [b, e] : bursts)  // Back-to-back writes, one callback allowance or less apart.
            for (Instant t = b; t < e; t += Instant{static_cast<std::int64_t>(300 + next() % 500)})
                events.push_back({t, true});
        for (Instant t = start; t < start + length; t += Instant{static_cast<std::int64_t>(5'000 + next() % 150'000)})
            events.push_back({t + Instant{static_cast<std::int64_t>(next() % 1000)}, true});  // Sparse, off-grid.
        // The model window: ticks exist only inside it, so no evaluation may fall after it.
        std::erase_if(events, [&](const Evaluation& e) { return e.at >= start + length; });
        std::stable_sort(events.begin(), events.end(), [](const auto& a, const auto& b) { return a.at < b.at; });

        TelemetryScheduler scheduler{timing::kTelemetryPeriod};
        scheduler.sessionStarted();
        TelemetryKey key = kIdle;
        Instant nextChange = start + Instant{static_cast<std::int64_t>(next() % 80'000)};
        std::optional<TelemetryKey> sentKey;
        std::optional<Instant> pendingSince;  // First evaluation that saw an unsent state.
        std::optional<Instant> previous;
        std::vector<Instant> times;
        for (const auto& event : events) {
            while (event.at >= nextChange) {
                const TelemetryKey choices[] = {kIdle, kMoving, kExpired, kLatched};
                key = choices[next() % 4];
                nextChange += Instant{static_cast<std::int64_t>(1'000 + next() % 80'000)};
            }
            if (sentKey && key != *sentKey) {
                if (!pendingSince) pendingSince = event.at;
            } else {
                pendingSince.reset();
            }
            const auto reason = scheduler.due(event.at, key);
            if (reason == FrameReason::None) continue;
            if (event.fromWrite && reason == FrameReason::SessionStart) continue;
            scheduler.attempted(event.at, key, true);
            if (previous) {
                const auto gap = event.at - *previous;
                ASSERT_GE(gap, Instant{timing::kStateTelemetryMinSpacing}) << pattern;
                if (reason == FrameReason::Periodic) {
                    const auto deviation = gap > kPeriod ? gap - kPeriod : kPeriod - gap;
                    ASSERT_LE(deviation, timing::kTelemetryPeriodDeviationMax)
                        << "pattern " << pattern << " write=" << event.fromWrite << " gap_us=" << gap.count();
                }
                if (pendingSince) {
                    ASSERT_LE(event.at - std::max(*pendingSince, *previous), stateDelayBound)
                        << "state-change delay, pattern " << pattern;
                }
            }
            pendingSince.reset();
            sentKey = key;
            previous = event.at;
            times.push_back(event.at);
        }
        ASSERT_GE(times.size(), 25U) << pattern;
        for (std::size_t i = 0, j = 0; i < times.size(); ++i) {
            while (times[i] - times[j] >= 1s) ++j;
            ASSERT_LE(i - j + 1, static_cast<std::size_t>(timing::kTelemetryNotificationsPerSecondMax)) << pattern;
        }
    }
}

// Why the task checks every millisecond (#283 review): a frame sent from a write callback is
// off any coarse grid. With 5 ms checks, a frame at 22.5 ms made the 120 ms check refuse
// (97.5 ms elapsed) and the 125 ms one send a 102.5 ms gap, outside B10.
TEST(TelemetrySchedulerTest, AnOffGridFrameIsFollowedByAPeriodicFrameWithinB10)
{
    TelemetryScheduler scheduler{timing::kTelemetryPeriod};
    scheduler.attempted(Instant::zero(), kIdle, true);
    ASSERT_EQ(scheduler.due(22'500us, kMoving), FrameReason::StateChange);  // From a write.
    scheduler.attempted(22'500us, kMoving, true);
    std::optional<Instant> sent;
    for (Instant check = 23ms; !sent; check += kTick)  // The task's millisecond checks.
        if (scheduler.due(check, kMoving) == FrameReason::Periodic) sent = check;
    const auto gap = *sent - 22'500us;
    EXPECT_GE(gap, Instant{timing::kTelemetryPeriod} - Instant{timing::kTelemetryPeriodDeviationMax});
    EXPECT_LE(gap, Instant{timing::kTelemetryPeriod} + Instant{timing::kTelemetryPeriodDeviationMax});
}

TEST(TelemetrySchedulerTest, StateChangeIsSentAtTheNextTickOrOnceTheSpacingHasPassed)
{
    TelemetryScheduler scheduler{timing::kTelemetryPeriod};
    scheduler.attempted(1s, kIdle, true);  // Session start frame at 1 s.
    // A change 15 ms later waits for the 20 ms spacing.
    EXPECT_EQ(scheduler.due(1s + 15ms, kMoving), FrameReason::None);
    EXPECT_EQ(scheduler.due(1s + 20ms, kMoving), FrameReason::StateChange);
    scheduler.attempted(1s + 20ms, kMoving, true);
    // A change long after the previous frame goes out at the first check that sees it.
    EXPECT_EQ(scheduler.due(1s + 70ms, kExpired), FrameReason::StateChange);
}

TEST(TelemetrySchedulerTest, ChangesWithinTheSpacingCoalesceToTheLatestState)
{
    TelemetryScheduler scheduler{timing::kTelemetryPeriod};
    scheduler.attempted(1s, kIdle, true);
    // Moving and back to Idle before the spacing passes: nothing differs from what was sent.
    EXPECT_EQ(scheduler.due(1s + 5ms, kMoving), FrameReason::None);
    EXPECT_EQ(scheduler.due(1s + 20ms, kIdle), FrameReason::None);
    // E-stop after Moving: one frame with the latest state.
    EXPECT_EQ(scheduler.due(1s + 20ms, kLatched), FrameReason::StateChange);
}

TEST(TelemetrySchedulerTest, FlappingStateUnderAnyLatenessNeverExceedsTheB10Rate)
{
    // The state changes at every tick; ticks are late by 0..2.5 ms in an adversarial pattern.
    const auto lateness = [](std::int64_t k) { return k % 2 == 0 ? Instant::zero() : timing::kSchedulingAllowance; };
    const auto flapping = [](Instant now) { return (now / kTick) % 2 == 0 ? kIdle : kMoving; };
    const auto sent = simulate(1s, 10s, lateness, flapping, alwaysQueued);
    for (std::size_t i = 1; i < sent.size(); ++i)
        EXPECT_GE(sent[i].at - sent[i - 1].at, timing::kTelemetryMinFrameSpacing) << i;
    // Any one-second window holds at most 60 frames (B10).
    for (std::size_t i = 0; i < sent.size(); ++i) {
        std::size_t inWindow = 0;
        for (std::size_t j = i; j < sent.size() && sent[j].at - sent[i].at < 1s; ++j) ++inWindow;
        EXPECT_LE(inWindow, static_cast<std::size_t>(timing::kTelemetryNotificationsPerSecondMax));
    }
    // Nor are state changes starved: with millisecond checks a frame follows the previous one
    // after at most 20 ms plus one check and the lateness, so at least ~43 per second here.
    EXPECT_GE(sent.size(), 10U * 40U);
}

TEST(TelemetrySchedulerTest, CongestedStackGetsNoRetryBurstAndTheStateChangeIsTriedAgain)
{
    TelemetryScheduler scheduler{timing::kTelemetryPeriod};
    scheduler.attempted(1s, kIdle, true);
    // A periodic frame fails: the next periodic attempt is one period later, not the next tick.
    const Instant periodic = 1s + kPeriod;
    ASSERT_EQ(scheduler.due(periodic, kIdle), FrameReason::Periodic);
    scheduler.attempted(periodic, kIdle, false);
    EXPECT_EQ(scheduler.due(periodic + kTick, kIdle), FrameReason::None);
    EXPECT_EQ(scheduler.due(periodic + kEligible - 1us, kIdle), FrameReason::None);
    EXPECT_EQ(scheduler.due(periodic + kEligible, kIdle), FrameReason::Periodic);
    // A state change that fails is retried after the spacing, until it is queued.
    const Instant change = periodic + 30ms;
    ASSERT_EQ(scheduler.due(change, kLatched), FrameReason::StateChange);
    scheduler.attempted(change, kLatched, false);
    EXPECT_EQ(scheduler.due(change + 15ms, kLatched), FrameReason::None);
    EXPECT_EQ(scheduler.due(change + 20ms, kLatched), FrameReason::StateChange);
    scheduler.attempted(change + 20ms, kLatched, true);
    EXPECT_EQ(scheduler.due(change + 40ms, kLatched), FrameReason::None);
}

TEST(TelemetrySchedulerTest, AlwaysFailingStackIsAttemptedAtMostAtTheBoundedRate)
{
    std::size_t attempts = 0;
    TelemetryScheduler scheduler{timing::kTelemetryPeriod};
    scheduler.sessionStarted();
    for (Instant now = 1s; now < 11s; now += kTick) {
        const auto key = (now / kTick) % 3 == 0 ? kLatched : kIdle;
        if (scheduler.due(now, key) == FrameReason::None) continue;
        ++attempts;
        scheduler.attempted(now, key, false);
    }
    EXPECT_LE(attempts, 10U * 50U);  // At most one attempt per 20 ms: no queue, no burst.
}

TEST(TelemetrySchedulerTest, SpacingHoldsAcrossSessionsAndANewSessionStartsWithAFrame)
{
    TelemetryScheduler scheduler{timing::kTelemetryPeriod};
    scheduler.sessionStarted();
    scheduler.attempted(1s, kMoving, true);
    scheduler.sessionStarted();  // Reconnect 5 ms later.
    EXPECT_EQ(scheduler.due(1s + 5ms, kIdle), FrameReason::None);
    EXPECT_EQ(scheduler.due(1s + 20ms, kIdle), FrameReason::SessionStart);
    // Even an unchanged state is sent first in a new session.
    scheduler.attempted(1s + 20ms, kIdle, true);
    scheduler.sessionStarted();
    EXPECT_EQ(scheduler.due(2s, kIdle), FrameReason::SessionStart);
}

TEST(TelemetrySchedulerTest, UptimeIsMillisecondsSinceBootModulo2To32)
{
    constexpr std::int64_t kWrap = std::int64_t{1} << 32;
    EXPECT_EQ(uptimeMilliseconds(Instant::zero()), 0U);
    EXPECT_EQ(uptimeMilliseconds(Instant{123'456'789}), 123'456U);  // Truncated, never rounded up.
    EXPECT_EQ(uptimeMilliseconds(std::chrono::milliseconds{kWrap - 1}), std::numeric_limits<std::uint32_t>::max());
    EXPECT_EQ(uptimeMilliseconds(std::chrono::milliseconds{kWrap}), 0U);
    EXPECT_EQ(uptimeMilliseconds(std::chrono::milliseconds{kWrap + 5}), 5U);
}

TEST(TelemetrySchedulerTest, SchedulingDoesNotDependOnClockWrapPoints)
{
    // The uint32 microsecond and millisecond wrap points of the 64-bit robot clock.
    for (const Instant start : {Instant{(std::int64_t{1} << 32) - 3 * kPeriod.count()},
                                Instant{std::chrono::milliseconds{(std::int64_t{1} << 32) - 300}}}) {
        const auto sent = simulate(start, 1s, noLateness, [](Instant) { return kIdle; }, alwaysQueued);
        ASSERT_EQ(sent.size(), 11U);  // 0, 99, ..., 990 ms.
        for (std::size_t i = 1; i < sent.size(); ++i) EXPECT_EQ(sent[i].at - sent[i - 1].at, kEligible);
    }
}

TEST(TelemetryDiagnosticsTest, CountersWrapModuloAndWindowsReset)
{
    EXPECT_EQ(counterDelta(5, std::numeric_limits<std::uint32_t>::max() - 4), 10U);
    EXPECT_EQ(counterDelta(7, 7), 0U);

    TelemetryDiagnostics diagnostics;
    auto window = diagnostics.takeWindow();
    EXPECT_FALSE(window.minIntervalUs);
    EXPECT_FALSE(window.maxIntervalUs);
    EXPECT_FALSE(window.periodicDeviationMaxUs);
    diagnostics.recordGap(100'800us, FrameReason::Periodic, timing::kTelemetryPeriod);
    diagnostics.recordGap(99'000us, FrameReason::Periodic, timing::kTelemetryPeriod);
    diagnostics.recordGap(20'000us, FrameReason::StateChange, timing::kTelemetryPeriod);
    diagnostics.countFrame(FrameReason::Periodic);
    diagnostics.countFrame(FrameReason::StateChange);
    diagnostics.countFrame(FrameReason::None);
    window = diagnostics.takeWindow();
    EXPECT_EQ(window.minIntervalUs, 20'000U);
    EXPECT_EQ(window.maxIntervalUs, 100'800U);
    EXPECT_EQ(window.periodicDeviationMaxUs, 1'000U);  // |99 ms - 100 ms|, state changes excluded.
    EXPECT_EQ(diagnostics.counters().framesSent(), 2U);
    EXPECT_FALSE(diagnostics.takeWindow().maxIntervalUs);  // A fresh window.
}
