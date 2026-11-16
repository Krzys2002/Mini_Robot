// Host tests of the #164 telemetry schedule (B10) on a fake microsecond clock: session start,
// periodic spacing on the tick grid under lateness, coalesced state changes, the rate bound,
// congestion without retry bursts, and the uptime and counter wrap points.
#include "TimingBudget.hpp"
#include "telemetry/TelemetryDiagnostics.hpp"
#include "telemetry/TelemetryScheduler.hpp"
#include <gtest/gtest.h>
#include <cstdint>
#include <limits>
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
    const auto lateness = [](std::int64_t k) { return k % 2 == 0 ? Instant::zero() : timing::kTelemetryTickTolerance; };
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
    EXPECT_GE(sent.size(), 10U * 45U);  // And state changes are not starved either.
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
