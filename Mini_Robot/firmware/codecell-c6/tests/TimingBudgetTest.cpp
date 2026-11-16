// Fake-clock boundary and timeline reviews of the #156 budget table. The model in
// StopBudgetModel.hpp is a specification oracle; these tests check that the
// proposed numbers compose for representative timelines, not that firmware exists.
#include "StopBudgetModel.hpp"
#include "TimingBudget.hpp"
#include <gtest/gtest.h>
#include <cstdint>
#include <limits>
#include <vector>

using namespace codecell::timing;
using namespace codecell::timing::model;
using namespace std::chrono_literals;

namespace {
constexpr Us kHorizon = 1s;
constexpr Us kCallbackLatencyWorst = kRobotStackQueueAllowance; // Parser flood ahead of a write.
Scenario worstLateness()
{
    Scenario s;
    s.supervisorLateness = [](std::int64_t) { return kSchedulingAllowance; };
    s.controlLateness = [](std::int64_t) { return kSchedulingAllowance; };
    return s;
}
// Every output-off time must be within `bound` of `reference` (when output ever moved).
void expectOffWithin(const Trace& trace, Us reference, Us bound)
{
    if (!trace.everNonzero) return;
    ASSERT_TRUE(trace.outputOff().has_value());
    EXPECT_LE(*trace.outputOff(), reference + bound);
}
} // namespace

TEST(TimingBudgetTest, ComposedBoundsMatchTheDocumentedTable)
{
    EXPECT_EQ(kExpiryDetectionBound, 7ms);
    EXPECT_EQ(lastValidCommandToOutputOff(kCommandValidityCeiling), 385ms);
    EXPECT_EQ(kRobotStackQueueAllowance, 19200us);
    EXPECT_EQ(kStopEnforcementLimit, 120ms);
    EXPECT_EQ(kTelemetryNotificationsPerSecondMax, 60);
}

TEST(TimingBudgetTest, WaitsRoundUpToWholeTicksUnlikeTruncation)
{
    EXPECT_EQ(ticksAtLeast(0us), 0u);
    EXPECT_EQ(ticksAtLeast(-5us), 0u);
    EXPECT_EQ(ticksAtLeast(1us), 1u);
    EXPECT_EQ(ticksAtLeast(1000us), 1u);
    EXPECT_EQ(ticksAtLeast(1001us), 2u); // pdMS_TO_TICKS-style truncation would give 1.
    EXPECT_EQ(ticksAtLeast(kExpiryDetectionBound), 7u);
}

TEST(TimingBudgetTest, ExpiryHappensAtTheDeadlineNotBefore)
{
    // Checks run every 5 ms from t = 0. A 50 ms window received at 0 has its deadline on a check.
    const auto onCheck = simulate({{0us, Input::Velocity, 50ms}}, {}, kHorizon);
    ASSERT_TRUE(onCheck.expiryDecision);
    EXPECT_EQ(*onCheck.expiryDecision, 50ms);
    // Received 1 us later, the check at 50 ms is 1 us early: expiry waits for the next check.
    const auto justAfter = simulate({{1us, Input::Velocity, 50ms}}, {}, kHorizon);
    ASSERT_TRUE(justAfter.expiryDecision);
    EXPECT_EQ(*justAfter.expiryDecision, 55ms);
    EXPECT_LE(*justAfter.expiryDecision, 1us + 50ms + kExpiryDetectionBound);
}

TEST(TimingBudgetTest, LastValidCommandIsBoundedForEveryPhaseValidityAndLateness)
{
    const std::vector<milliseconds> validities{1ms, 2ms, 7ms, 50ms, 125ms, 249ms, kCommandValidityCeiling};
    for (const auto& scenario : {Scenario{}, worstLateness()}) {
        for (const auto validity : validities) {
            for (Us phase = 0us; phase < Us{kSafetyCheckPeriod} * 2; phase += 250us) {
                const Us receipt = 3ms + phase;
                const auto trace = simulate({{receipt, Input::Velocity, validity}}, scenario, kHorizon);
                SCOPED_TRACE(testing::Message() << "validity=" << validity.count() << "ms phase=" << phase.count());
                ASSERT_TRUE(trace.expiryDecision);
                EXPECT_GE(*trace.expiryDecision, receipt + Us{validity});
                EXPECT_LE(*trace.expiryDecision, receipt + Us{validity} + kExpiryDetectionBound);
                expectOffWithin(trace, receipt, Us{kLastValidCommandToOutputOffBound});
                EXPECT_FALSE(trace.driveDisabled) << "a healthy control task stops by ramp";
            }
        }
    }
}

TEST(TimingBudgetTest, OneLostOrLateRefreshDoesNotExpireAnActiveCommand)
{
    // Desktop refresh every 100 ms with a 250 ms window; frame 3 is lost, frame 4 is 30 ms late.
    std::vector<Event> events;
    for (int i = 0; i < 8; ++i) {
        if (i == 3) continue;
        const Us late = i == 4 ? Us{kRequestedMaxConnectionInterval} : 0us;
        events.push_back({Us{i * kDesktopVelocityRefresh} + late, Input::Velocity, kCommandValidityCeiling});
    }
    const auto trace = simulate(events, worstLateness(), 750ms);
    EXPECT_FALSE(trace.expiryDecision);
    EXPECT_EQ(trace.state, State::Moving);
}

TEST(TimingBudgetTest, CommandLossWithoutDisconnectEventStillStops)
{
    // The desktop goes silent after t = 200 ms and no disconnect event is ever delivered.
    const std::vector<Event> events{{0ms, Input::Velocity, kCommandValidityCeiling},
                                    {100ms, Input::Velocity, kCommandValidityCeiling},
                                    {200ms, Input::Velocity, kCommandValidityCeiling}};
    const auto trace = simulate(events, worstLateness(), kHorizon);
    ASSERT_TRUE(trace.expiryDecision);
    expectOffWithin(trace, 200ms, Us{kLastValidCommandToOutputOffBound});
    EXPECT_EQ(trace.state, State::Idle);
}

TEST(TimingBudgetTest, StalledControlTaskIsDisabledByTheSupervisor)
{
    // Blocked sensor read or stalled control work from t = 40 ms; the command stays valid.
    Scenario scenario = worstLateness();
    scenario.controlStalls = {{40ms, kHorizon}};
    const auto trace = simulate({{0ms, Input::Velocity, kCommandValidityCeiling}}, scenario, kHorizon);
    ASSERT_TRUE(trace.driveDisabled);
    EXPECT_LE(*trace.driveDisabled, 40ms + Us{kStalledControlToOutputOffBound});
    EXPECT_EQ(trace.state, State::Fault);
}

TEST(TimingBudgetTest, StaleNonzeroOutputAfterExpiryIsEnforced)
{
    // The control task keeps running but replays its old output (stale buffer) after expiry.
    Scenario scenario = worstLateness();
    scenario.controlReplaysStaleOutput = true;
    const auto trace = simulate({{0ms, Input::Velocity, kCommandValidityCeiling}}, scenario, kHorizon);
    ASSERT_TRUE(trace.expiryDecision);
    ASSERT_TRUE(trace.driveDisabled);
    EXPECT_GE(*trace.driveDisabled, *trace.expiryDecision + Us{kStopEnforcementLimit});
    EXPECT_LE(*trace.driveDisabled, Us{kLastValidCommandToOutputOffBound});
    EXPECT_EQ(trace.state, State::Fault);
}

TEST(TimingBudgetTest, ControlStallDuringTheStopRampIsDisabled)
{
    // Expiry at 250 ms; the control task stalls during the ramp and never reaches zero.
    Scenario scenario = worstLateness();
    scenario.controlStalls = {{270ms, kHorizon}};
    const auto trace = simulate({{0ms, Input::Velocity, kCommandValidityCeiling}}, scenario, kHorizon);
    ASSERT_TRUE(trace.expiryDecision);
    ASSERT_TRUE(trace.driveDisabled);
    EXPECT_FALSE(trace.outputZero);
    EXPECT_LE(*trace.driveDisabled, Us{kLastValidCommandToOutputOffBound});
    EXPECT_EQ(trace.state, State::Fault);
}

TEST(TimingBudgetTest, EmergencyStopBehindAParserFloodDisablesWithinItsBound)
{
    // Stack receipt at 100 ms; every inbound buffer ahead of it holds an ordinary write.
    const Us stackReceipt = 100ms;
    const std::vector<Event> events{{0ms, Input::Velocity, kCommandValidityCeiling},
                                    {stackReceipt + kCallbackLatencyWorst, Input::EmergencyStop},
                                    {stackReceipt + kCallbackLatencyWorst + 1ms, Input::Velocity, 100ms}};
    const auto trace = simulate(events, worstLateness(), kHorizon);
    ASSERT_TRUE(trace.driveDisabled);
    EXPECT_LE(*trace.driveDisabled, stackReceipt + Us{kEstopStackReceiptToDisableBound});
    EXPECT_EQ(trace.state, State::EmergencyStop);
    EXPECT_TRUE(trace.rejectedMotionWhileLatched);
    EXPECT_FALSE(trace.expiryDecision) << "the latch dropped the setpoint; nothing left to expire";
}

TEST(TimingBudgetTest, NotificationPressureLatenessKeepsEveryBound)
{
    // Telemetry/result pressure is modeled as maximum allowed lateness on every wake-up,
    // with commands arriving in connection-event bursts (several in one event).
    std::vector<Event> events;
    for (int burst = 0; burst < 5; ++burst) {
        for (int i = 0; i < 4; ++i) {
            events.push_back({Us{burst * kRequestedMaxConnectionInterval} + Us{i * 400}, Input::Velocity, 100ms});
        }
    }
    const Us lastReceipt = events.back().at;
    const auto trace = simulate(events, worstLateness(), kHorizon);
    ASSERT_TRUE(trace.expiryDecision);
    EXPECT_LE(*trace.expiryDecision, lastReceipt + 100ms + kExpiryDetectionBound);
    expectOffWithin(trace, lastReceipt, Us{kLastValidCommandToOutputOffBound});
}

TEST(TimingBudgetTest, StopAndDisconnectRampWithoutWaitingForExpiry)
{
    for (const auto input : {Input::Stop, Input::Disconnect}) {
        const std::vector<Event> events{{0ms, Input::Velocity, kCommandValidityCeiling}, {50ms, input}};
        const auto trace = simulate(events, worstLateness(), kHorizon);
        EXPECT_FALSE(trace.expiryDecision);
        ASSERT_TRUE(trace.outputZero);
        EXPECT_LE(*trace.outputZero, 50ms + Us{kControlPeriod} + kSchedulingAllowance + Us{kStopRampMax});
        EXPECT_EQ(trace.state, State::Idle);
    }
}

TEST(TimingBudgetTest, ResultsDoNotDependOnClockWraparoundPoints)
{
    // 64-bit microseconds since boot: the uint32 microsecond (about 71.6 min) and uint32
    // millisecond (about 49.7 days, protocol uptimeMs) wrap points are ordinary values.
    const std::vector<Event> events{{0ms, Input::Velocity, kCommandValidityCeiling},
                                    {100ms, Input::Velocity, kCommandValidityCeiling}};
    const auto reference = simulate(events, worstLateness(), kHorizon);
    constexpr auto kU32 = static_cast<std::int64_t>(std::numeric_limits<std::uint32_t>::max());
    for (const Us offset : {Us{kU32 - 50'000}, Us{std::chrono::milliseconds{kU32 - 50}}}) {
        Scenario shifted = worstLateness();
        shifted.clockOffset = offset;
        const auto trace = simulate(events, shifted, kHorizon);
        EXPECT_EQ(trace.expiryDecision, reference.expiryDecision);
        EXPECT_EQ(trace.outputZero, reference.outputZero);
        EXPECT_EQ(trace.state, reference.state);
    }
}
