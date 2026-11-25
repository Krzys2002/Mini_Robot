// Host attack tests of the restricted-lab-v1 profile on the production controller (#165).
// They complement the #163 lifecycle tests with the cases the profile leaves to #165:
// the identifiable profile (F10), cross-session replay as a documented residual risk
// (AT-04/R1/R2), replacement of the controller after a disconnect (AT-06), and floods of
// unauthorized or malformed traffic that must never move the robot nor delay the E-stop
// (AT-07/AT-08). Radio-level behaviour (pairing requests, bond storage, two real centrals)
// is hardware-in-the-loop and not covered here.
#include "BleTestRig.hpp"
#include "diagnostics/StatusReport.hpp"
#include <gtest/gtest.h>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

using namespace codecell;
using namespace codecell::ble;
using namespace codecell::ble::testrig;

namespace {
// The bytes a passive observer records from the operator's session (R4).
const std::vector<std::uint8_t> recordedVelocity = velocityFrame(258, kForward, kValidity);
const std::vector<std::uint8_t> recordedClear = headerFrame(MessageType::ClearEmergencyStop, 259);
} // namespace

// ---- F10: the profile is identifiable and has no alternative -----------------------------

TEST(RestrictedLabProfileTest, TheOnlyProfileIsRestrictedLabV1AndIsPrintedInDiagnostics)
{
    EXPECT_EQ(kSecurityProfile, SecurityProfile::RestrictedLabV1);
    EXPECT_EQ(kSecurityProfileId, "restricted-lab-v1");
    EXPECT_STREQ(toString(kSecurityProfile), "restricted-lab-v1");
    const auto identity = diagnostics::identityLine({}).view();
    EXPECT_NE(identity.find(" profile=restricted-lab-v1 "), std::string_view::npos) << identity;
    for (const std::string_view claim : {"authenticated=yes", "secure", "encrypted=yes", "profile=secure"})
        EXPECT_EQ(identity.find(claim), std::string_view::npos) << claim;
}

TEST(RestrictedLabProfileTest, RestrictedLabSecurityIsLabelledAndNeverReportedAsEstablished)
{
    Rig rig;
    rig.makeReady();
    EXPECT_TRUE(rig.ble->status().securitySession);
    EXPECT_TRUE(rig.ble->status().securityBypassed);
    const auto line = diagnostics::bleLine(rig.ble->status(), rig.ble->diagnostics()).view();
    EXPECT_NE(line.find("security=bypassed"), std::string_view::npos) << line;
    EXPECT_EQ(line.find("security=established"), std::string_view::npos) << line;
    const std::string_view warning{kRestrictedLabSecurityWarning};
    EXPECT_NE(warning.find("not implemented"), std::string_view::npos);
    EXPECT_NE(warning.find("restricted-lab"), std::string_view::npos);
}

// ---- AT-04: replay across sessions is accepted as a new first frame and stays bounded ----

TEST(RestrictedLabEnforcementTest, RecordedVelocityIsAcceptedAsANewSessionsFirstFrameAndExpires)
{
    Rig rig;
    rig.makeReady();
    rig.command(kOwner, recordedVelocity);
    EXPECT_EQ(rig.last().code, ResultCode::Accepted);
    rig.command(kOwner, recordedVelocity);
    EXPECT_EQ(rig.last().code, ResultCode::StaleSequence) << "an equal sequence is stale inside one session";
    rig.ble->onDisconnected(kOwner, kRemoteUserTerminated);
    EXPECT_FALSE(rig.drive.enabled);

    rig.makeReady(kSecond);
    rig.command(kSecond, recordedVelocity);
    EXPECT_EQ(rig.last().to, kSecond);
    EXPECT_EQ(rig.last().code, ResultCode::Accepted) << "documented residual risk R1: no replay protection across sessions";
    rig.run(30ms);
    EXPECT_EQ(rig.status().state, MotionState::Moving);
    rig.run(std::chrono::milliseconds{kValidity} + 100ms);
    EXPECT_FALSE(rig.drive.enabled) << "motion ends at validForMs";
    EXPECT_NE(rig.status().state, MotionState::Moving);
    rig.command(kSecond, recordedVelocity);
    EXPECT_EQ(rig.last().code, ResultCode::StaleSequence);
}

TEST(RestrictedLabEnforcementTest, RecordedClearIsAcceptedInANewSessionAndNeverStartsMotion)
{
    Rig rig;
    rig.makeMoving();
    rig.estop(kOwner, headerFrame(MessageType::EmergencyStop, 1));
    rig.ble->onDisconnected(kOwner, kRemoteUserTerminated);
    rig.makeReady(kSecond);
    EXPECT_EQ(rig.status().state, MotionState::EmergencyStop);
    rig.command(kSecond, recordedClear);
    EXPECT_EQ(rig.last().code, ResultCode::Accepted) << "documented residual risk R2";
    EXPECT_EQ(rig.status().state, MotionState::Idle);
    const auto commands = rig.drive.commandCount();
    rig.run(300ms);
    EXPECT_EQ(rig.drive.commandCount(), commands);
    EXPECT_FALSE(rig.drive.enabled);
}

// ---- AT-06: replacement of the controller ------------------------------------------------

TEST(RestrictedLabEnforcementTest, DisconnectDuringMotionStopsAndANewCentralNeedsAFreshVelocityOnly)
{
    Rig rig;
    rig.makeMoving();
    rig.ble->onDisconnected(kOwner, kRemoteUserTerminated);
    rig.run(200ms);
    EXPECT_FALSE(rig.drive.enabled);
    EXPECT_NE(rig.status().state, MotionState::EmergencyStop) << "a disconnect creates no latch";
    EXPECT_EQ(rig.link.advertisingStarts, 2);
    rig.makeReady(kSecond);
    rig.run(100ms);
    EXPECT_FALSE(rig.drive.enabled) << "nothing resumes by itself";
    rig.command(kSecond, velocityFrame(0, kForward, kValidity));
    EXPECT_EQ(rig.last().code, ResultCode::Accepted);
}

TEST(RestrictedLabEnforcementTest, LatchedDisconnectKeepsTheLatchUntilAnExplicitClear)
{
    Rig rig;
    rig.makeMoving();
    rig.estop(kOwner, headerFrame(MessageType::EmergencyStop, 1));
    rig.ble->onDisconnected(kOwner, kRemoteUserTerminated);
    rig.makeReady(kSecond);
    rig.command(kSecond, velocityFrame(0, kForward, kValidity));
    EXPECT_EQ(rig.last().code, ResultCode::EmergencyStopLatched);
    EXPECT_FALSE(rig.drive.enabled);
    rig.command(kSecond, headerFrame(MessageType::ClearEmergencyStop, 1));
    EXPECT_EQ(rig.last().code, ResultCode::Accepted);
    rig.command(kSecond, velocityFrame(2, kForward, kValidity));
    EXPECT_EQ(rig.last().code, ResultCode::Accepted);
}

// ---- AT-16: a Clear in flight is discarded with the session ------------------------------

TEST(RestrictedLabEnforcementTest, NotificationLossDiscardsAPendingClearAndKeepsTheLatch)
{
    Rig rig;
    rig.makeMoving();
    rig.estop(kOwner, headerFrame(MessageType::EmergencyStop, 1));
    rig.command(kOwner, headerFrame(MessageType::ClearEmergencyStop, 2));
    rig.command(kOwner, velocityFrame(3, kForward, kValidity));
    rig.subscribe(kOwner, Characteristic::Telemetry, false);
    rig.run(300ms);
    EXPECT_FALSE(rig.drive.enabled);
    EXPECT_EQ(rig.ble->state(), LinkState::Connected);
    EXPECT_FALSE(rig.link.advertising);
    rig.subscribe(kOwner, Characteristic::Telemetry, true);
    rig.run(100ms);
    EXPECT_FALSE(rig.drive.enabled) << "re-enabling the CCCD does not resume anything";
}

// ---- AT-07/AT-08: storms never move the robot and never delay the E-stop -----------------

TEST(RestrictedLabEnforcementTest, ConnectionStormsAndUnauthorizedFramesNeverMoveTheRobot)
{
    Rig rig;
    for (int round = 0; round < 200; ++round) {
        const auto central = static_cast<ConnectionHandle>(10 + round % 7);
        rig.ble->onConnected(central);
        // Not ready (no CCCDs), then a partial subscription: motion and clear must not act.
        rig.command(central, velocityFrame(static_cast<std::uint16_t>(round), kForward, kValidity));
        rig.command(central, headerFrame(MessageType::ClearEmergencyStop, static_cast<std::uint16_t>(round)));
        rig.subscribe(central, Characteristic::CommandResult);
        rig.command(central, velocityFrame(static_cast<std::uint16_t>(round + 1), kForward, kValidity));
        rig.ble->onMtuChanged(central, 23);
        rig.ble->onLinkParametersUpdated(central, {6, 0, 100});
        rig.run(10ms);
        EXPECT_FALSE(rig.drive.enabled) << round;
        EXPECT_EQ(rig.drive.commandCount(), 0) << round;
        rig.ble->onDisconnected(central, kRemoteUserTerminated);
    }
    EXPECT_EQ(rig.status().state, MotionState::Idle);
    EXPECT_FALSE(rig.status().sessionReady);
    EXPECT_EQ(rig.bleCount(BleCounter::SessionsReady), 0U);
}

TEST(RestrictedLabEnforcementTest, ForeignCentralStormBesideTheOwnerLeavesTheSessionAndTheEmergencyStopIntact)
{
    Rig rig;
    rig.makeMoving();
    const auto owner = rig.bleCount(BleCounter::SessionsReady);
    for (int round = 0; round < 200; ++round) {
        rig.ble->onConnected(kSecond);
        rig.subscribe(kSecond, Characteristic::CommandResult);
        rig.subscribe(kSecond, Characteristic::Telemetry);
        rig.command(kSecond, headerFrame(MessageType::Stop, static_cast<std::uint16_t>(round)));
        rig.command(kSecond, headerFrame(MessageType::ClearEmergencyStop, static_cast<std::uint16_t>(round)));
        rig.ble->onDisconnected(kSecond, kRemoteUserTerminated);
    }
    EXPECT_EQ(rig.bleCount(BleCounter::SessionsReady), owner);
    EXPECT_EQ(rig.bleCount(BleCounter::RejectedCentrals), 200U);
    EXPECT_EQ(rig.link.terminated.size(), 200U);
    EXPECT_EQ(rig.ble->state(), LinkState::Ready);
    EXPECT_EQ(rig.status().state, MotionState::Moving);
    rig.estop(kSecond, {});
    EXPECT_EQ(rig.status().state, MotionState::EmergencyStop);
    EXPECT_FALSE(rig.drive.enabled);
}

TEST(RestrictedLabEnforcementTest, MalformedAndOversizedFloodThenAnEmergencyStopStillLatchesImmediately)
{
    Rig rig;
    rig.makeMoving();
    const auto answeredBefore = rig.bleCount(BleCounter::UnansweredCommandWrites) + rig.bleCount(BleCounter::ResultsSent);
    std::vector<std::vector<std::uint8_t>> frames{{}, {0x00}, {0xFF, 0xFF, 0xFF, 0xFF}, {kProtocolVersion},
                                                  std::vector<std::uint8_t>(kMaxAttPayload + 40, 0xA5),
                                                  std::vector<std::uint8_t>(kVelocityLength + 1, 0x01)};
    for (int i = 0; i < 300; ++i) {
        for (const auto& frame : frames) rig.command(kOwner, frame);
        rig.command(kOwner, velocityFrame(static_cast<std::uint16_t>(1000 + i), kForward, kValidity));
    }
    EXPECT_LE(rig.bleCount(BleCounter::UnansweredCommandWrites) + rig.bleCount(BleCounter::ResultsSent) - answeredBefore,
              300U * (frames.size() + 1U));
    rig.estop(kOwner, {});
    EXPECT_EQ(rig.status().state, MotionState::EmergencyStop) << "no queueing or authorization precedes the latch";
    EXPECT_FALSE(rig.drive.enabled);
    rig.run(300ms);
    EXPECT_FALSE(rig.drive.enabled);
}
