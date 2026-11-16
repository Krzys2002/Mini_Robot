// Host tests of the #164 telemetry schedule (B10) on a fake microsecond clock: session start,
// periodic spacing on the tick grid under lateness, coalesced state changes, the rate bound,
// congestion without retry bursts, and the uptime and counter wrap points.
#include "TimingBudget.hpp"
#include "telemetry/TelemetryDiagnostics.hpp"
#include "telemetry/TelemetryScheduler.hpp"
#include <gtest/gtest.h>
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

TEST(TelemetrySchedulerTest, PeriodicFramesFollowTheAdvertisedPeriodOnTheTickGrid)
{
    const auto sent = simulate(1s, 10s, noLateness, [](Instant) { return kIdle; }, alwaysQueued);
    ASSERT_EQ(sent.size(), 100U);  // 10 s at 100 ms: one session start, then 99 periodic frames.
    EXPECT_EQ(sent.front().reason, FrameReason::SessionStart);
    for (std::size_t i = 1; i < sent.size(); ++i) {
        EXPECT_EQ(sent[i].reason, FrameReason::Periodic);
        EXPECT_EQ(sent[i].at - sent[i - 1].at, kPeriod) << i;
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
        EXPECT_LE(deviation, timing::kSchedulingAllowance) << i;  // B10: robot-side deviation <= 2 ms.
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
    EXPECT_EQ(scheduler.due(98ms - 1us, kIdle), FrameReason::None);
    EXPECT_EQ(scheduler.due(98ms, kIdle), FrameReason::Periodic);
    EXPECT_EQ(scheduler.due(102ms, kIdle), FrameReason::Periodic);
}

// The model behind the rule (#283 review): ticks on the 5 ms grid, each late by any amount
// within the scheduling allowance, and the state changing at random. Every frame is at least
// 20 ms after the previous one, every periodic gap is 100 +/- 2 ms, at most 60 frames fall in
// any second, and a state change seen at a tick is sent no more than four ticks (20 ms) later.
TEST(TelemetrySchedulerTest, LatenessModelMeetsEveryB10BoundAtOnce)
{
    std::uint32_t seed = 0x2834u;
    const auto next = [&seed] {
        seed = seed * 1664525u + 1013904223u;
        return seed >> 8;
    };
    const auto allowance = timing::kSchedulingAllowance.count();
    for (int pattern = 0; pattern < 240; ++pattern) {
        TelemetryScheduler scheduler{timing::kTelemetryPeriod};
        scheduler.sessionStarted();
        TelemetryKey key = kIdle;
        std::optional<TelemetryKey> sentKey;
        std::optional<std::int64_t> pendingSince;  // First tick that saw an unsent state.
        std::optional<Instant> previous;
        std::vector<Instant> times;
        const Instant start = 1s + Instant{static_cast<std::int64_t>(next() % 5000)};
        for (std::int64_t k = 0; k < 2000; ++k) {
            Instant late{0};
            switch (pattern % 4) {
            case 0: late = Instant{static_cast<std::int64_t>(next() % (allowance + 1))}; break;  // Random.
            case 1: late = Instant{k % 2 == 0 ? 0 : allowance}; break;                        // Alternating.
            case 2: late = Instant{(k * allowance / 7) % (allowance + 1)}; break;              // Ramps.
            case 3: late = Instant{(k / 20) % 2 == 0 ? allowance : 0}; break;                  // Steps.
            }
            if (next() % 9 == 0) {
                const TelemetryKey choices[] = {kIdle, kMoving, kExpired, kLatched};
                key = choices[next() % 4];
            }
            const Instant now = start + k * kTick + late;
            if (sentKey && key != *sentKey) {
                if (!pendingSince) pendingSince = k;
            } else {
                pendingSince.reset();
            }
            const auto reason = scheduler.due(now, key);
            if (reason == FrameReason::None) continue;
            scheduler.attempted(now, key, true);
            if (previous) {
                const auto gap = now - *previous;
                ASSERT_GE(gap, Instant{timing::kStateTelemetryMinSpacing}) << pattern << " " << k;
                if (reason == FrameReason::Periodic) {
                    const auto deviation = gap > kPeriod ? gap - kPeriod : kPeriod - gap;
                    ASSERT_LE(deviation, timing::kTelemetryPeriodDeviationMax) << pattern << " " << k;
                }
            }
            if (pendingSince) ASSERT_LE((k - *pendingSince) * kTick, Instant{timing::kStateTelemetryMinSpacing})
                << "state-change delay, pattern " << pattern << " tick " << k;
            pendingSince.reset();
            sentKey = key;
            previous = now;
            times.push_back(now);
        }
        for (std::size_t i = 0, j = 0; i < times.size(); ++i) {
            while (times[i] - times[j] >= 1s) ++j;
            ASSERT_LE(i - j + 1, static_cast<std::size_t>(timing::kTelemetryNotificationsPerSecondMax)) << pattern;
        }
    }
}

TEST(TelemetrySchedulerTest, StateChangeIsSentAtTheNextTickOrOnceTheSpacingHasPassed)
{
    TelemetryScheduler scheduler{timing::kTelemetryPeriod};
    scheduler.attempted(1s, kIdle, true);  // Session start frame at 1 s.
    // A change 3 ticks later (15 ms) waits for the 20 ms spacing, i.e. the 4th tick.
    EXPECT_EQ(scheduler.due(1s + 3 * kTick, kMoving), FrameReason::None);
    EXPECT_EQ(scheduler.due(1s + 4 * kTick, kMoving), FrameReason::StateChange);
    scheduler.attempted(1s + 4 * kTick, kMoving, true);
    // A change long after the previous frame goes out at the first tick that sees it.
    EXPECT_EQ(scheduler.due(1s + 4 * kTick + 50ms, kExpired), FrameReason::StateChange);
}

TEST(TelemetrySchedulerTest, ChangesWithinTheSpacingCoalesceToTheLatestState)
{
    TelemetryScheduler scheduler{timing::kTelemetryPeriod};
    scheduler.attempted(1s, kIdle, true);
    // Moving and back to Idle before the spacing passes: nothing differs from what was sent.
    EXPECT_EQ(scheduler.due(1s + kTick, kMoving), FrameReason::None);
    EXPECT_EQ(scheduler.due(1s + 4 * kTick, kIdle), FrameReason::None);
    // E-stop after Moving: one frame with the latest state.
    EXPECT_EQ(scheduler.due(1s + 4 * kTick, kLatched), FrameReason::StateChange);
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
    // Nor are state changes starved. With the strict 20 ms bound, a late frame followed by an
    // on-time tick waits one more tick (25 ms), so this worst-case pattern still gets 40 per
    // second; each change is sent within 20 ms of the tick that saw it (LatenessModel test).
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
    EXPECT_EQ(scheduler.due(periodic + kPeriod - kTick, kIdle), FrameReason::None);
    EXPECT_EQ(scheduler.due(periodic + kPeriod, kIdle), FrameReason::Periodic);
    // A state change that fails is retried after the spacing, until it is queued.
    const Instant change = periodic + 30ms;
    ASSERT_EQ(scheduler.due(change, kLatched), FrameReason::StateChange);
    scheduler.attempted(change, kLatched, false);
    EXPECT_EQ(scheduler.due(change + 3 * kTick, kLatched), FrameReason::None);
    EXPECT_EQ(scheduler.due(change + 4 * kTick, kLatched), FrameReason::StateChange);
    scheduler.attempted(change + 4 * kTick, kLatched, true);
    EXPECT_EQ(scheduler.due(change + 8 * kTick, kLatched), FrameReason::None);
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
    EXPECT_EQ(scheduler.due(1s + kTick, kIdle), FrameReason::None);
    EXPECT_EQ(scheduler.due(1s + 4 * kTick, kIdle), FrameReason::SessionStart);
    // Even an unchanged state is sent first in a new session.
    scheduler.attempted(1s + 4 * kTick, kIdle, true);
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
        ASSERT_EQ(sent.size(), 10U);
        for (std::size_t i = 1; i < sent.size(); ++i) EXPECT_EQ(sent[i].at - sent[i - 1].at, kPeriod);
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
