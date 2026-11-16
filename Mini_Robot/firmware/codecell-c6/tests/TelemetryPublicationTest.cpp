// Host tests of #164 telemetry publication through the BLE peripheral controller, with the
// real #158-#162 runtime behind it and the fake radio link of BleTestRig.hpp: v1 bytes and
// flags, the shared transmit counter with CommandResults (including its wrap), periodic and
// state-change timing, unknown measurements, the Read value, congestion and allocation.
#include "AllocationCounter.hpp"
#include "BleTestRig.hpp"
#include "protocol/V1Codec.hpp"
#include "telemetry/TelemetryScheduler.hpp"
#include <gtest/gtest.h>
#include <cstdint>
#include <optional>
#include <vector>

using namespace codecell;
using namespace codecell::ble;
using namespace codecell::ble::testrig;
using telemetry::TelemetryCounter;

namespace {
constexpr Instant kTick{timing::kTelemetryTickPeriod};
static_assert(timing::kTelemetryTickPeriod == timing::kSafetyCheckPeriod, "Rig::run ticks telemetry every safety step");

struct Telemetry {
    std::uint16_t sequence{};
    std::uint8_t state{};
    std::uint8_t flags{};
    std::uint16_t lastAccepted{};
    std::uint32_t uptime{};
    std::uint16_t battery{};
    std::int16_t linear{};
    std::int16_t angular{};
};

bool isTelemetry(const Notification& n)
{
    return n.length == kTelemetryLength && n.bytes[kTypeAt] == static_cast<std::uint8_t>(MessageType::Telemetry);
}
std::uint16_t sequenceOf(const Notification& n)
{
    return readU16(std::span<const std::uint8_t>{n.bytes.data(), n.length}, kSequenceAt);
}
Telemetry parseTelemetry(const Notification& n)
{
    EXPECT_TRUE(isTelemetry(n));
    EXPECT_EQ(n.bytes[kVersionAt], kProtocolVersion);
    const std::span<const std::uint8_t> b{n.bytes.data(), n.length};
    const auto u32 = static_cast<std::uint32_t>(b[kTelemetryUptimeAt]) | (static_cast<std::uint32_t>(b[kTelemetryUptimeAt + 1]) << 8)
                   | (static_cast<std::uint32_t>(b[kTelemetryUptimeAt + 2]) << 16)
                   | (static_cast<std::uint32_t>(b[kTelemetryUptimeAt + 3]) << 24);
    return {readU16(b, kSequenceAt), b[kTelemetryStateAt], b[kTelemetryFlagsAt], readU16(b, kTelemetryLastAcceptedAt),
            u32, readU16(b, kTelemetryBatteryAt), readI16(b, kTelemetryLinearAt), readI16(b, kTelemetryAngularAt)};
}
std::vector<Telemetry> telemetryFrames(const FakeLink& link)
{
    std::vector<Telemetry> out;
    for (const auto& n : link.notifications)
        if (isTelemetry(n)) out.push_back(parseTelemetry(n));
    return out;
}
std::uint32_t telemetryCount(const Rig& rig, TelemetryCounter c) { return rig.ble->telemetryCounters()[c]; }
std::uint32_t uptimeOf(Instant t) { return telemetry::uptimeMilliseconds(t); }
} // namespace

TEST(TelemetryPublicationTest, NoTelemetryBeforeReadinessThenTheFirstFrameAtTheFirstTick)
{
    Rig rig;
    rig.ble->onTelemetryTick();
    rig.ble->onConnected(kOwner);
    rig.subscribe(kOwner, Characteristic::CommandResult);
    rig.ble->onTelemetryTick();  // Only one CCCD: not ready, nothing is sent.
    EXPECT_TRUE(rig.link.notifications.empty());
    EXPECT_EQ(telemetryCount(rig, TelemetryCounter::TicksNotReady), 2U);
    rig.subscribe(kOwner, Characteristic::Telemetry);
    EXPECT_FALSE(rig.link.telemetryValue);  // Empty until the session's first frame.
    rig.ble->onTelemetryTick();
    ASSERT_EQ(rig.link.notifications.size(), 1U);
    const auto t = parseTelemetry(rig.link.notifications[0]);
    EXPECT_EQ(t.sequence, 0);
    EXPECT_EQ(t.state, kStateIdle);
    EXPECT_EQ(t.flags, 0);  // No accepted command yet; nothing expired; never batteryLow.
    EXPECT_EQ(t.lastAccepted, kNoAcceptedSequence);
    EXPECT_EQ(t.uptime, uptimeOf(rig.clock.time));
    EXPECT_EQ(t.battery, kUnknownBattery);
    EXPECT_EQ(t.linear, kUnknownVelocity);
    EXPECT_EQ(t.angular, kUnknownVelocity);
    ASSERT_TRUE(rig.link.telemetryValue);
    EXPECT_EQ(rig.link.telemetryValue->bytes, rig.link.notifications[0].bytes);
    EXPECT_EQ(telemetryCount(rig, TelemetryCounter::SessionStartFrames), 1U);
}

TEST(TelemetryPublicationTest, FrameIsTheV1EncodingOfTheRuntimeStateAndSessionSequence)
{
    Rig rig;
    rig.makeMoving(258);
    rig.ble->onTelemetryTick();
    ASSERT_TRUE(isTelemetry(rig.link.notifications.back()));
    protocol::v1::TelemetryValues expected;
    expected.state = MotionState::Moving;
    expected.lastAcceptedSequence = 258;
    expected.uptimeMilliseconds = uptimeOf(rig.clock.time);
    const auto bytes = encodeTelemetry(1, expected);  // Result 0 for the Velocity, then this frame.
    ASSERT_TRUE(bytes);
    const auto& n = rig.link.notifications.back();
    EXPECT_TRUE(std::equal(bytes->begin(), bytes->end(), n.bytes.begin()));
    EXPECT_EQ(parseTelemetry(n).flags, kFlagHasAcceptedCommand);
}

TEST(TelemetryPublicationTest, ResultsAndTelemetryShareOneGapFreeCounter)
{
    Rig rig;
    rig.makeReady();
    rig.telemetryTicks = true;
    for (std::uint16_t i = 0; i < 200; ++i) {
        rig.command(kOwner, headerFrame(MessageType::Stop, i));
        rig.run(Instant{(i % 7 + 1) * 3000});
    }
    std::size_t telemetry = 0;
    for (std::size_t i = 0; i < rig.link.notifications.size(); ++i) {
        EXPECT_EQ(sequenceOf(rig.link.notifications[i]), static_cast<std::uint16_t>(i)) << i;
        if (isTelemetry(rig.link.notifications[i])) ++telemetry;
    }
    EXPECT_EQ(rig.link.notifications.size() - telemetry, 200U);  // Still exactly one result per frame.
    EXPECT_GT(telemetry, 5U);
}

TEST(TelemetryPublicationTest, SharedCounterWrapsFrom65535To0AcrossBothFrameTypes)
{
    Rig rig;
    rig.makeReady();
    rig.ble->onTelemetryTick();  // Sequence 0.
    rig.link.notifications.reserve(70000);
    for (int i = 0; i < 65534; ++i) rig.command(kOwner, headerFrame(MessageType::Stop, static_cast<std::uint16_t>(i)));
    ASSERT_EQ(sequenceOf(rig.link.notifications.back()), 65534);
    rig.run(timing::kTelemetryPeriod);
    rig.ble->onTelemetryTick();  // Periodic: sequence 65535.
    rig.command(kOwner, headerFrame(MessageType::Stop, 1));  // Result: sequence 0.
    rig.run(timing::kTelemetryPeriod);
    rig.ble->onTelemetryTick();  // Sequence 1.
    const auto& n = rig.link.notifications;
    ASSERT_GE(n.size(), 3U);
    EXPECT_TRUE(isTelemetry(n[n.size() - 3]));
    EXPECT_EQ(sequenceOf(n[n.size() - 3]), 65535);
    EXPECT_FALSE(isTelemetry(n[n.size() - 2]));
    EXPECT_EQ(sequenceOf(n[n.size() - 2]), 0);
    EXPECT_EQ(sequenceOf(n[n.size() - 1]), 1);
    EXPECT_TRUE(isNewer(0, 65535) && isNewer(1, 0));  // The desktop's newer-than accepts each.
}

TEST(TelemetryPublicationTest, PeriodicTelemetryFollowsTheAdvertisedPeriodWhileReady)
{
    Rig rig;
    rig.makeReady();
    rig.telemetryTicks = true;
    rig.run(2s);
    const auto frames = telemetryFrames(rig.link);
    ASSERT_GE(frames.size(), 20U);
    for (std::size_t i = 1; i < frames.size(); ++i)
        EXPECT_EQ(frames[i].uptime - frames[i - 1].uptime, timing::kTelemetryPeriod.count()) << i;
    EXPECT_EQ(telemetryCount(rig, TelemetryCounter::PeriodicFrames), frames.size() - 1);
}

TEST(TelemetryPublicationTest, EmergencyStopResultComesFirstThenTelemetryWithinTheSpacing)
{
    Rig rig;
    rig.makeMoving();
    rig.telemetryTicks = true;
    rig.run(30ms);
    const auto before = rig.link.notifications.size();
    const Instant latchedAt = rig.clock.time;
    rig.estop(kOwner, headerFrame(MessageType::EmergencyStop, 1));
    EXPECT_EQ(rig.last().code, ResultCode::Accepted);
    rig.run(timing::kStateTelemetryMinSpacing + kTick);
    std::optional<Telemetry> latched;
    for (std::size_t i = before + 1; i < rig.link.notifications.size() && !latched; ++i)
        if (isTelemetry(rig.link.notifications[i])) latched = parseTelemetry(rig.link.notifications[i]);
    ASSERT_TRUE(latched) << "a state change is published within the B10 spacing";
    EXPECT_EQ(latched->state, kStateEmergencyStop);
    EXPECT_LE(Instant{std::chrono::milliseconds{latched->uptime}} - latchedAt,
              Instant{timing::kStateTelemetryMinSpacing} + kTick);
    EXPECT_GE(telemetryCount(rig, TelemetryCounter::StateChangeFrames), 1U);
}

TEST(TelemetryPublicationTest, ExpiryIsPublishedWithCommandExpiredUntilTheNextAcceptedStop)
{
    Rig rig;
    rig.makeMoving();
    rig.telemetryTicks = true;
    rig.run(Instant{std::chrono::milliseconds{kValidity}} + 50ms);
    ASSERT_EQ(rig.status().state, MotionState::Idle);
    const auto frames = telemetryFrames(rig.link);
    ASSERT_FALSE(frames.empty());
    EXPECT_EQ(frames.back().state, kStateIdle);
    EXPECT_EQ(frames.back().flags & kFlagCommandExpired, kFlagCommandExpired);
    rig.command(kOwner, headerFrame(MessageType::Stop, 1));
    rig.run(timing::kStateTelemetryMinSpacing + kTick);
    EXPECT_EQ(telemetryFrames(rig.link).back().flags & kFlagCommandExpired, 0);
}

TEST(TelemetryPublicationTest, MeasuredVelocityAndBatteryStayUnknownWhileTheDriveIsCommanded)
{
    Rig rig;
    rig.makeMoving();
    rig.telemetryTicks = true;
    for (int i = 0; i < 4; ++i) {
        rig.command(kOwner, velocityFrame(static_cast<std::uint16_t>(1 + i), kForward, kValidity));
        rig.run(100ms);
        ASSERT_EQ(rig.status().state, MotionState::Moving);
        ASSERT_TRUE(rig.drive.enabled);
    }
    const auto frames = telemetryFrames(rig.link);
    ASSERT_GE(frames.size(), 3U);
    for (const auto& t : frames) {
        EXPECT_EQ(t.state, kStateMoving);
        EXPECT_EQ(t.linear, kUnknownVelocity) << "commanded velocity is not a measurement";
        EXPECT_EQ(t.angular, kUnknownVelocity);
        EXPECT_EQ(t.battery, kUnknownBattery);
        EXPECT_EQ(t.flags & kFlagBatteryLow, 0);
    }
}

TEST(TelemetryPublicationTest, TelemetryNeverExtendsTheCommandDeadline)
{
    Rig rig;
    rig.makeMoving();  // Accepted 30 ms ago with a 200 ms window.
    rig.telemetryTicks = true;
    rig.run(Instant{std::chrono::milliseconds{kValidity}} - 30ms - 10ms);
    EXPECT_EQ(rig.status().state, MotionState::Moving);
    rig.run(10ms + timing::kSafetyCheckPeriod);
    EXPECT_EQ(rig.status().state, MotionState::Idle);
    EXPECT_TRUE(rig.status().commandExpired);
}

TEST(TelemetryPublicationTest, NotifyFailureDropsTheFrameCountsItAndTheSessionContinues)
{
    Rig rig;
    rig.makeReady();
    rig.telemetryTicks = true;
    rig.run(20ms);
    const auto readValue = rig.link.telemetryValue;
    ASSERT_TRUE(readValue);
    rig.link.failTelemetry = true;
    rig.run(1s);
    const auto failures = telemetryCount(rig, TelemetryCounter::NotifyFailures);
    EXPECT_GE(failures, 9U);
    EXPECT_LE(failures, 11U);  // One per period: no retry burst while the state is unchanged.
    EXPECT_EQ(rig.ble->state(), LinkState::Ready);
    EXPECT_TRUE(rig.status().sessionReady);
    rig.command(kOwner, headerFrame(MessageType::Stop, 0));
    EXPECT_EQ(rig.last().code, ResultCode::Accepted);  // Results still flow.
    EXPECT_EQ(rig.link.telemetryValue->bytes, readValue->bytes) << "the Read value stays the last queued frame";
    rig.link.failTelemetry = false;
    rig.run(150ms);
    // The dropped frames spent their sequences: the desktop sees them as lost (a gap).
    const auto& n = rig.link.notifications;
    std::uint32_t gaps = 0;
    for (std::size_t i = 1; i < n.size(); ++i) gaps += static_cast<std::uint16_t>(sequenceOf(n[i]) - sequenceOf(n[i - 1]) - 1);
    EXPECT_EQ(gaps, failures);
    EXPECT_TRUE(isTelemetry(n.back()));
}

TEST(TelemetryPublicationTest, CongestedLinkForTenSecondsStaysBoundedAllocatesNothingAndKeepsTheSession)
{
    Rig rig;
    rig.makeReady();
    rig.telemetryTicks = true;
    rig.link.failTelemetry = true;
    rig.link.notifications.reserve(4096);
    rig.drive.events.reserve(8192);
    std::vector<std::vector<std::uint8_t>> frames;  // Built first: the test's own frames allocate.
    for (std::uint16_t i = 0; i < 1000; ++i)
        frames.push_back(headerFrame(i % 2 == 0 ? MessageType::EmergencyStop : MessageType::ClearEmergencyStop, i));
    std::size_t allocations = 0;
    {
        codecell::testing::AllocationCount counter;
        for (std::uint16_t i = 0; i < 1000; ++i) {
            // A state that changes all the time: E-stop and clear every 10 ms.
            if (i % 2 == 0) rig.estop(kOwner, frames[i]);
            else rig.command(kOwner, frames[i]);
            rig.run(10ms);
        }
        allocations = counter.count();
    }
    EXPECT_EQ(allocations, 0U);
    EXPECT_EQ(rig.ble->state(), LinkState::Ready);
    // At most one attempt per tolerated spacing: <= 57 per second (B10 caps 60).
    EXPECT_LE(static_cast<std::size_t>(rig.link.telemetryAttempts), 10U * 57U);
    EXPECT_EQ(rig.bleCount(BleCounter::ResultsSent), 1000U);
}

TEST(TelemetryPublicationTest, ReadValueIsEmptyOutsideASessionAndEachSessionRestartsAtZero)
{
    Rig rig;
    rig.makeReady();
    rig.ble->onTelemetryTick();
    ASSERT_TRUE(rig.link.telemetryValue);
    rig.subscribe(kOwner, Characteristic::Telemetry, false);  // Ends the session.
    EXPECT_FALSE(rig.link.telemetryValue);
    rig.run(50ms);
    rig.ble->onTelemetryTick();
    EXPECT_EQ(telemetryFrames(rig.link).size(), 1U) << "no telemetry without a ready session";
    rig.subscribe(kOwner, Characteristic::Telemetry, true);
    rig.ble->onTelemetryTick();
    ASSERT_TRUE(isTelemetry(rig.link.notifications.back()));
    EXPECT_EQ(sequenceOf(rig.link.notifications.back()), 0);
    rig.ble->onDisconnected(kOwner, kRemoteUserTerminated);
    EXPECT_FALSE(rig.link.telemetryValue);
    rig.makeReady();
    EXPECT_FALSE(rig.link.telemetryValue);
    rig.run(50ms);
    rig.ble->onTelemetryTick();
    EXPECT_EQ(sequenceOf(rig.link.notifications.back()), 0);
}

TEST(TelemetryPublicationTest, ValidatedBatteryAndALowFlagChangeArePublished)
{
    Rig rig;
    rig.makeReady();
    rig.inputs.override = telemetry::TelemetryMeasurements{3912, false, std::nullopt, std::nullopt};
    rig.ble->onTelemetryTick();
    EXPECT_EQ(telemetryFrames(rig.link).back().battery, 3912);
    rig.inputs.override = telemetry::TelemetryMeasurements{3400, true, std::nullopt, std::nullopt};
    rig.run(timing::kStateTelemetryMinSpacing);
    rig.ble->onTelemetryTick();
    const auto t = telemetryFrames(rig.link).back();
    EXPECT_EQ(t.battery, 3400);
    EXPECT_EQ(t.flags & kFlagBatteryLow, kFlagBatteryLow);
    EXPECT_EQ(telemetryCount(rig, TelemetryCounter::StateChangeFrames), 1U);
}

TEST(TelemetryPublicationTest, AValueEqualToItsSentinelIsNotSentAndSpendsNoSequence)
{
    Rig rig;
    rig.makeReady();
    rig.inputs.override = telemetry::TelemetryMeasurements{kUnknownBattery, false, std::nullopt, std::nullopt};
    rig.ble->onTelemetryTick();
    EXPECT_TRUE(rig.link.notifications.empty());
    EXPECT_EQ(telemetryCount(rig, TelemetryCounter::EncodeFailures), 1U);
    rig.command(kOwner, headerFrame(MessageType::Stop, 0));
    EXPECT_EQ(rig.last().sequence, 0);
}

TEST(TelemetryPublicationTest, UptimeInTheFrameWrapsAt2To32Milliseconds)
{
    Rig rig;
    rig.makeReady();
    rig.clock.time = std::chrono::milliseconds{(std::int64_t{1} << 32) + 7};
    rig.ble->onTelemetryTick();
    ASSERT_TRUE(isTelemetry(rig.link.notifications.back()));
    EXPECT_EQ(parseTelemetry(rig.link.notifications.back()).uptime, 7U);
}

TEST(TelemetryPublicationTest, SpacingStatisticsAreReportedPerWindow)
{
    Rig rig;
    rig.makeReady();
    rig.telemetryTicks = true;
    rig.run(1s);
    const auto window = rig.ble->takeTelemetryWindow();
    ASSERT_TRUE(window.maxIntervalUs && window.periodicDeviationMaxUs);
    EXPECT_EQ(*window.maxIntervalUs, 100'000U);
    EXPECT_EQ(*window.periodicDeviationMaxUs, 0U);  // A fake clock has no lateness.
    EXPECT_FALSE(rig.ble->takeTelemetryWindow().maxIntervalUs);
}
