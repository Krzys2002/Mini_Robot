// Host tests of the #162 robot-local safety invariant: the independent command watchdog,
// session readiness, the E-stop latch, fault recovery and the safety transition table.
// Everything runs on a fake clock with explicit task interleavings. Where it matters, the
// outputs are checked at the drive or motor fake (what was written), not only status flags.
// None of this is HIL evidence: physical output timing belongs to #238/#166.
#include "DriveFakes.hpp"
#include "RuntimeFakes.hpp"
#include "drive/BoundedDriveOutput.hpp"
#include "protocol/V1CommandSession.hpp"
#include "runtime/CommandProcessor.hpp"
#include "runtime/FirmwareRuntime.hpp"
#include <gtest/gtest.h>
#include <array>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <vector>

using namespace codecell;
using namespace codecell::runtime;
using namespace codecell::runtime::fakes;
using namespace std::chrono_literals;
namespace timing = codecell::timing;

namespace {
constexpr BodyVelocity kForward{0.05, 0.0};
constexpr BodyVelocity kTurn{0.0, 0.5};
constexpr milliseconds kValid = 100ms;
constexpr Instant kBoot = 1s;
constexpr Instant kTick{timing::kSafetyCheckPeriod};

// The real runtime over the body-level drive fake.
struct Rig {
    FakeClock clock;
    FakePlatform platform;
    FakeDrive drive;
    FakeConfiguration configuration;
    std::optional<FirmwareRuntime> runtime;

    explicit Rig(Instant start = kBoot)
    {
        clock.time = start;
        drive.section = &platform.section;
        runtime.emplace(RuntimeDependencies{clock, platform, drive, configuration, nullptr});
        EXPECT_EQ(runtime->start(), StartResult::Running);
    }
    IMotionCommandSink& commands() { return runtime->commands(); }
    RuntimeStatus status() const { return runtime->status(); }
    void control() { runtime->controlStep().step(); }
    void supervise() { runtime->safetyStep().step(); }
    // One 5 ms supervisor period; the control task runs every second period (10 ms).
    void tick(bool withControl = true)
    {
        clock.advance(kTick);
        supervise();
        if (withControl && (clock.time / kTick) % 2 == 0) control();
    }
    void run(Instant duration, bool withControl = true)
    {
        const Instant end = clock.time + duration;
        while (clock.time < end) tick(withControl);
    }
    CommandOutcome velocity(SessionToken session, const BodyVelocity& v = kForward, milliseconds validFor = kValid,
                            std::optional<Instant> receipt = {})
    {
        return commands().velocity(session, v, validFor, receipt.value_or(clock.nowMicroseconds()));
    }
};

// The real runtime over the real bounded drive and the motor HAL fake: the duty writes,
// coasts and sleeps are the outputs.
struct OutputRig {
    FakeClock clock;
    FakePlatform platform;
    drive::fakes::FakeMotorDriver motors;
    drive::BoundedDriveOutput drive{motors};
    FakeConfiguration configuration;
    std::optional<FirmwareRuntime> runtime;
    SessionToken session{};

    OutputRig()
    {
        clock.time = kBoot;
        runtime.emplace(RuntimeDependencies{clock, platform, drive, configuration, nullptr});
        EXPECT_EQ(runtime->start(), StartResult::Running);
        motors.section = &platform.section;
        session = runtime->commands().openSession();
    }
    CommandOutcome velocity(const BodyVelocity& v, milliseconds validFor)
    {
        return runtime->commands().velocity(session, v, validFor, clock.nowMicroseconds());
    }
    // Output off: the last duty handed to the bridges is zero, and the drive was disabled
    // (coast, possibly followed by sleep) after it.
    [[nodiscard]] bool outputOff() const
    {
        if (drive.lastDuty() != drive::MotorDuty{}) return false;
        return !motors.events.empty() && motors.events.back().kind != drive::fakes::MotorEvent::Kind::Write;
    }
};
} // namespace

// ---- Deadlines on the 64-bit robot clock ------------------------------------------------

TEST(SafetyDeadlineTest, DeadlineArithmeticNeverOverflows)
{
    constexpr Instant kMax = Instant::max();
    EXPECT_EQ(deadlineAfter(1s, 250ms), Instant{1s + 250ms});
    EXPECT_EQ(deadlineAfter(kMax - Instant{250ms}, 250ms), kMax);
    EXPECT_FALSE(deadlineAfter(kMax - Instant{250ms} + 1us, 250ms)) << "not representable: refused, not wrapped";
    EXPECT_FALSE(deadlineAfter(kMax, 1us));
    EXPECT_FALSE(deadlineAfter(1s, -1us));
    EXPECT_EQ(deadlineAfter(-5s, 250ms), Instant{-5s + 250ms}) << "an earlier start only shortens the window";
}

// Fake-clock boundary on the real runtime: before, at and after the deadline, with the
// deadline placed across the uint32 microsecond and millisecond wrap points and near the
// end of the int64 clock. Only the 64-bit microsecond clock is used, so nothing changes.
TEST(SafetyDeadlineTest, ExpiryHappensExactlyAtTheDeadlineAcrossClockWrapPoints)
{
    const std::array<Instant, 4> kPoints{kBoot, Instant{std::int64_t{1} << 32},
                                         Instant{(std::int64_t{1} << 32) * 1000},
                                         Instant::max() - Instant{10s}};
    for (const Instant point : kPoints) {
        for (const milliseconds validFor : {1ms, 31ms, 100ms, timing::kCommandValidityCeiling}) {
            SCOPED_TRACE(testing::Message() << "point=" << point.count() << "us validFor=" << validFor.count());
            const Instant receipt = point - Instant{validFor} / 2;
            Rig rig{receipt};
            const auto session = rig.commands().openSession();
            ASSERT_EQ(rig.velocity(session, kForward, validFor), CommandOutcome::Accepted);
            const Instant deadline = receipt + Instant{validFor};

            rig.clock.time = deadline - 1us;  // Before: still fresh, and the control step drives.
            rig.control();
            rig.supervise();
            EXPECT_EQ(rig.status().state, MotionState::Moving);
            EXPECT_TRUE(rig.drive.enabled);
            const int writes = rig.drive.commandCount();

            rig.clock.time = deadline;  // At: the gate refuses the snapshot, the supervisor expires it.
            rig.control();
            EXPECT_EQ(rig.drive.commandCount(), writes) << "no write at now == deadline";
            rig.supervise();
            EXPECT_EQ(rig.status().state, MotionState::Idle);
            EXPECT_TRUE(rig.status().commandExpired);

            rig.clock.time = deadline + 1us;  // After: nothing comes back.
            rig.control();
            rig.supervise();
            EXPECT_EQ(rig.drive.commandCount(), writes);
            EXPECT_EQ(rig.runtime->diagnostics()[Counter::CommandExpired], 1u);
        }
    }
}

TEST(SafetyDeadlineTest, UnrepresentableDeadlineIsRefusedWithoutMotion)
{
    Rig rig{Instant::max() - Instant{100ms}};
    const auto session = rig.commands().openSession();
    EXPECT_EQ(rig.velocity(session, kForward, 250ms), CommandOutcome::RejectedInvalid);
    EXPECT_EQ(rig.status().state, MotionState::Idle);
    EXPECT_EQ(rig.velocity(session, kForward, 50ms), CommandOutcome::Accepted) << "a representable one is fine";
}

TEST(SafetyDeadlineTest, FutureReceiptCannotExtendValidityPastNowPlusTheWindow)
{
    Rig rig;
    const auto session = rig.commands().openSession();
    const Instant now = rig.clock.time;
    ASSERT_EQ(rig.velocity(session, kForward, kValid, now + 10s), CommandOutcome::Accepted);
    rig.clock.time = now + Instant{kValid} - 1us;
    rig.control();
    rig.supervise();
    EXPECT_EQ(rig.status().state, MotionState::Moving);
    rig.clock.time = now + Instant{kValid};
    rig.supervise();
    EXPECT_EQ(rig.status().state, MotionState::Idle) << "the receipt was not trusted beyond the gate's clock";
}

TEST(SafetyDeadlineTest, CommandAlreadyPastItsDeadlineSupersedesAndExpiresAtOnce)
{
    Rig rig;
    const auto session = rig.commands().openSession();
    ASSERT_EQ(rig.velocity(session, kForward, 200ms), CommandOutcome::Accepted);
    rig.run(20ms);
    ASSERT_TRUE(rig.drive.enabled);
    const int writes = rig.drive.commandCount();
    // A Velocity received 150 ms ago (a delayed callback) with a 100 ms window.
    const Instant receipt = rig.clock.time - Instant{150ms};
    ASSERT_EQ(rig.velocity(session, kTurn, kValid, receipt), CommandOutcome::Accepted);
    EXPECT_EQ(rig.status().state, MotionState::Idle);
    EXPECT_TRUE(rig.status().commandExpired);
    EXPECT_TRUE(rig.status().stopRamp) << "the old command does not continue: normal stop";
    rig.run(200ms);
    EXPECT_EQ(rig.drive.commandCount(), writes) << "neither the old nor the expired command is driven";
    EXPECT_FALSE(rig.drive.enabled);
}

// ---- Sessions ---------------------------------------------------------------------------

TEST(SafetySessionTest, VelocityAndClearNeedTheCurrentReadySession)
{
    Rig rig;
    EXPECT_EQ(rig.velocity(SessionToken{}), CommandOutcome::RejectedSessionNotReady) << "no session yet";
    EXPECT_EQ(rig.commands().clearEmergencyStop(SessionToken{}), CommandOutcome::RejectedSessionNotReady);
    EXPECT_FALSE(rig.status().sessionReady);
    EXPECT_EQ(rig.commands().stop(), CommandOutcome::Accepted) << "Stop takes effect before readiness";
    EXPECT_EQ(rig.commands().emergencyStop(), CommandOutcome::Accepted) << "so does EmergencyStop";
    EXPECT_EQ(rig.status().state, MotionState::EmergencyStop);

    const auto first = rig.commands().openSession();
    EXPECT_TRUE(rig.status().sessionReady);
    EXPECT_EQ(rig.commands().clearEmergencyStop(first), CommandOutcome::Accepted);
    const auto second = rig.commands().openSession();  // Replacement (reconnect).
    EXPECT_NE(first, second);
    EXPECT_EQ(rig.velocity(first), CommandOutcome::RejectedSessionNotReady) << "a replaced session is never current";
    EXPECT_EQ(rig.commands().clearEmergencyStop(first), CommandOutcome::RejectedSessionNotReady);
    EXPECT_EQ(rig.velocity(second), CommandOutcome::Accepted);
    rig.commands().sessionEnded(second, SessionEndReason::Disconnected);
    EXPECT_FALSE(rig.status().sessionReady);
    EXPECT_EQ(rig.velocity(second), CommandOutcome::RejectedSessionNotReady) << "an ended session is never current";
    rig.run(100ms);
    EXPECT_EQ(rig.drive.commandCount(), 0) << "nothing above ever reached the outputs";
}

// Every way a session stops being ready, plus replacement, with intent both active (being
// driven) and pending (admitted but not yet read by the control task).
TEST(SafetySessionTest, SessionEndAndReplacementInvalidateActiveAndPendingIntent)
{
    struct Case {
        const char* name;
        std::function<void(Rig&, SessionToken)> event;
    };
    const std::vector<Case> cases{
        {"Disconnect", [](Rig& r, SessionToken s) { r.commands().sessionEnded(s, SessionEndReason::Disconnected); }},
        {"CccdDisable",
         [](Rig& r, SessionToken s) { r.commands().sessionEnded(s, SessionEndReason::NotificationsDisabled); }},
        {"ResultOverload",
         [](Rig& r, SessionToken s) { r.commands().sessionEnded(s, SessionEndReason::ResultQueueExhausted); }},
        {"Replacement", [](Rig& r, SessionToken) { (void)r.commands().openSession(); }},
    };
    for (const bool pending : {false, true}) {
        for (const auto& c : cases) {
            SCOPED_TRACE(testing::Message() << c.name << " pending=" << pending);
            Rig rig;
            const auto session = rig.commands().openSession();
            ASSERT_EQ(rig.velocity(session, kForward, timing::kCommandValidityCeiling), CommandOutcome::Accepted);
            if (!pending) rig.run(20ms);
            const int writes = rig.drive.commandCount();
            c.event(rig, session);
            EXPECT_EQ(rig.status().state, MotionState::Idle);
            rig.run(timing::kCommandValidityCeiling);
            EXPECT_EQ(rig.drive.commandCount(), writes) << "no write for the old session after the event";
            EXPECT_FALSE(rig.drive.enabled);
            EXPECT_EQ(rig.velocity(session), CommandOutcome::RejectedSessionNotReady);
            EXPECT_EQ(rig.status().fault, FaultCode::None) << "a normal stop, not a fault";
        }
    }
}

TEST(SafetySessionTest, StaleSessionEndDoesNotStopTheCurrentSession)
{
    Rig rig;
    const auto old = rig.commands().openSession();
    const auto current = rig.commands().openSession();
    ASSERT_EQ(rig.velocity(current, kForward, timing::kCommandValidityCeiling), CommandOutcome::Accepted);
    rig.run(20ms);
    rig.commands().sessionEnded(old, SessionEndReason::Disconnected);  // A late event of the old session.
    EXPECT_EQ(rig.status().state, MotionState::Moving);
    EXPECT_TRUE(rig.status().sessionReady);
    EXPECT_EQ(rig.runtime->diagnostics()[Counter::SessionEnded], 0u) << "a stale end is not counted";
    const int writes = rig.drive.commandCount();
    rig.run(20ms);
    EXPECT_GT(rig.drive.commandCount(), writes);
}

TEST(SafetySessionTest, LatchSurvivesDisconnectReconnectAndReplacement)
{
    Rig rig;
    auto session = rig.commands().openSession();
    ASSERT_EQ(rig.velocity(session), CommandOutcome::Accepted);
    rig.run(20ms);
    ASSERT_EQ(rig.commands().emergencyStop(), CommandOutcome::Accepted);
    rig.commands().sessionEnded(session, SessionEndReason::Disconnected);
    session = rig.commands().openSession();  // Reconnect.
    EXPECT_EQ(rig.status().state, MotionState::EmergencyStop) << "the RAM latch persists across reconnect";
    session = rig.commands().openSession();  // Replacement.
    EXPECT_EQ(rig.status().state, MotionState::EmergencyStop);
    EXPECT_EQ(rig.velocity(session), CommandOutcome::RejectedEmergencyStopLatched);
    const int writes = rig.drive.commandCount();
    ASSERT_EQ(rig.commands().clearEmergencyStop(session), CommandOutcome::Accepted);
    EXPECT_EQ(rig.status().state, MotionState::Idle);
    EXPECT_FALSE(rig.status().outputEnabled);
    rig.run(300ms);
    EXPECT_EQ(rig.drive.commandCount(), writes) << "clear yields zero idle: no old motion is replayed";
    ASSERT_EQ(rig.velocity(session, kTurn), CommandOutcome::Accepted) << "a new Velocity is required";
    rig.run(10ms);
    EXPECT_DOUBLE_EQ(rig.drive.lastCommand()->angularRadiansPerSecond, kTurn.angularRadiansPerSecond);
}

// ---- E-stop and clear races, independent enforcement -----------------------------------

TEST(SafetyRaceTest, EmergencyStopAndClearBetweenCopyAndWriteNeverReviveTheCopy)
{
    for (const bool alreadyMoving : {false, true}) {
        SCOPED_TRACE(alreadyMoving ? "moving" : "first write");
        Rig rig;
        const auto session = rig.commands().openSession();
        ASSERT_EQ(rig.velocity(session, kForward, timing::kCommandValidityCeiling), CommandOutcome::Accepted);
        if (alreadyMoving) rig.control();
        const int writes = rig.drive.commandCount();
        rig.clock.onNextRead = [&] {  // After the mailbox copy, before the write.
            (void)rig.commands().emergencyStop();
            ASSERT_EQ(rig.commands().clearEmergencyStop(session), CommandOutcome::Accepted);
        };
        rig.control();
        EXPECT_EQ(rig.drive.commandCount(), writes);
        EXPECT_FALSE(rig.drive.enabled);
        EXPECT_EQ(rig.status().state, MotionState::Idle);
        rig.run(timing::kCommandValidityCeiling);
        EXPECT_EQ(rig.drive.commandCount(), writes) << "the copy is from an older generation";
    }
}

TEST(SafetyRaceTest, EmergencyStopDisablesBeforeAnyResultOrStatusIsProduced)
{
    Rig rig;
    const auto session = rig.commands().openSession();
    ASSERT_EQ(rig.velocity(session, kForward, timing::kCommandValidityCeiling), CommandOutcome::Accepted);
    rig.run(20ms);
    ASSERT_TRUE(rig.drive.enabled);
    // Ordinary work is backed up: a flood of newer setpoints and a stalled control task.
    for (int i = 0; i < 100; ++i) ASSERT_EQ(rig.velocity(session, kTurn), CommandOutcome::Accepted);
    struct View final : protocol::v1::IRuntimeStatusView {
        Rig& rig;
        mutable std::optional<bool> enabledAtFirstRead;
        explicit View(Rig& r) : rig(r) {}
        RuntimeStatus status() const noexcept override
        {
            if (!enabledAtFirstRead) enabledAtFirstRead = rig.drive.enabled;
            return rig.status();
        }
    } view{rig};
    protocol::v1::CommandSession wire{rig.commands(), view, protocol::v1::CommandLimits{150, 3000, 250}};
    wire.open();
    const std::vector<std::uint8_t> malformed{0xFF};
    const auto before = rig.clock.time;
    const auto result = wire.onEmergencyStopWrite(malformed);
    ASSERT_TRUE(view.enabledAtFirstRead);
    EXPECT_FALSE(*view.enabledAtFirstRead) << "disabled before the result was built";
    EXPECT_EQ(result[protocol::v1::kResultStateAt], static_cast<std::uint8_t>(MotionState::EmergencyStop));
    EXPECT_EQ(rig.clock.time, before) << "no wait on the fake clock (B8 is the in-call disable)";
    rig.run(timing::kCommandValidityCeiling, /*withControl=*/true);
    EXPECT_FALSE(rig.drive.enabled) << "the flood behind it never drives";
}

// B5 at the outputs: from receipt of the last valid Velocity, the commanded duty reaches
// zero or the drive is disabled within the bound whether the control task runs, stalls,
// or is paused intermittently; no disconnect event is needed.
TEST(SafetyRaceTest, LastValidCommandIsBoundedAtTheOutputsWithControlRunningStalledOrPaused)
{
    enum class Control { Running, StalledAfterFirstWrite, PausedEveryOtherCycle };
    for (const auto mode : {Control::Running, Control::StalledAfterFirstWrite, Control::PausedEveryOtherCycle}) {
        for (const milliseconds validFor : {1ms, 20ms, 100ms, timing::kCommandValidityCeiling}) {
            SCOPED_TRACE(testing::Message() << "mode=" << static_cast<int>(mode) << " validFor=" << validFor.count());
            OutputRig rig;
            ASSERT_EQ(rig.velocity({drive::fixtures::kTestDriveProfile.maxLinearMetresPerSecond, 0.0}, validFor),
                      CommandOutcome::Accepted);
            const Instant receipt = rig.clock.time;
            rig.runtime->controlStep().step();  // First write at receipt.
            int cycle = 0;
            while (!rig.outputOff() && rig.clock.time - receipt < 2s) {
                rig.clock.advance(kTick);
                rig.runtime->safetyStep().step();
                if ((rig.clock.time / kTick) % 2 != 0) continue;
                ++cycle;
                const bool runs = mode == Control::Running || (mode == Control::PausedEveryOtherCycle && cycle % 2 == 0);
                if (runs) rig.runtime->controlStep().step();
            }
            EXPECT_TRUE(rig.outputOff());
            EXPECT_LE(rig.clock.time - receipt, Instant{timing::kLastValidCommandToOutputOffBound});
            const auto status = rig.runtime->status();
            switch (mode) {
            case Control::Running:
                EXPECT_EQ(status.fault, FaultCode::None) << "a normal bounded ramp, not a fault";
                EXPECT_FALSE(rig.motors.asleep);
                break;
            case Control::StalledAfterFirstWrite:
                EXPECT_EQ(status.fault, FaultCode::ControlStalled) << "immediate disable, then power-down";
                EXPECT_TRUE(rig.motors.asleep);
                break;
            case Control::PausedEveryOtherCycle:
                // Half the ramp steps: a long ramp misses the B6 limit and is enforced instead.
                EXPECT_TRUE(status.fault == FaultCode::None || status.fault == FaultCode::StopRampOverrun);
                EXPECT_EQ(rig.motors.asleep, status.fault == FaultCode::StopRampOverrun);
                break;
            }
        }
    }
}

// E-stop electrical disable with the ordinary control path stalled mid-acceleration: the
// coast is inside the call and no write follows (B8, separately from the B6 ramp above).
TEST(SafetyRaceTest, EmergencyStopWithAStalledControlPathCoastsInsideTheCall)
{
    OutputRig rig;
    ASSERT_EQ(rig.velocity({drive::fixtures::kTestDriveProfile.maxLinearMetresPerSecond, 0.0},
                           timing::kCommandValidityCeiling),
              CommandOutcome::Accepted);
    for (int i = 0; i < 3; ++i) {
        rig.clock.advance(Instant{timing::kControlPeriod});
        rig.runtime->safetyStep().step();
        rig.runtime->controlStep().step();
    }
    ASSERT_NE(rig.drive.lastDuty(), drive::MotorDuty{});
    rig.clock.advance(Instant{timing::kControlStallLimit} / 2);  // The control task is stalled now.
    const auto writes = rig.motors.count(drive::fakes::MotorEvent::Kind::Write);
    ASSERT_EQ(rig.runtime->commands().emergencyStop(), CommandOutcome::Accepted);
    EXPECT_TRUE(rig.motors.lastIsCoast()) << "coasting before emergencyStop() returned";
    EXPECT_EQ(rig.drive.lastDuty(), drive::MotorDuty{});
    for (int i = 0; i < 40; ++i) {
        rig.clock.advance(kTick);
        rig.runtime->safetyStep().step();
        if (i % 2 == 0) rig.runtime->controlStep().step();  // The control task resumes.
    }
    EXPECT_EQ(rig.motors.count(drive::fakes::MotorEvent::Kind::Write), writes);
    EXPECT_EQ(rig.runtime->status().state, MotionState::EmergencyStop);
}

// ---- Task failures, watchdog, shutdown and recovery ------------------------------------

TEST(SafetyFaultTest, OnlyTheSupervisorIsOnTheTaskWatchdog)
{
    Rig rig;
    ASSERT_EQ(rig.platform.tasks.size(), 2u);
    for (const auto& task : rig.platform.tasks)
        EXPECT_EQ(task.spec.watchdog, std::string{task.spec.name} == kSafetyTaskName) << task.spec.name;
    static_assert(timing::kTaskWatchdogTimeout < timing::kCommandValidityCeiling);
}

TEST(SafetyFaultTest, MaintenanceReleasesTheWatchdogOnlyOnceTheDriveIsOff)
{
    Rig rig;
    ASSERT_TRUE(rig.runtime->enterMaintenance());
    EXPECT_EQ(rig.platform.watchdogReleases, 1);
    Rig failing;
    failing.drive.failDisable = true;
    EXPECT_FALSE(failing.runtime->enterMaintenance());
    EXPECT_EQ(failing.platform.watchdogReleases, 0) << "an unconfirmed disable keeps the watchdog";
    EXPECT_TRUE(failing.drive.poweredDown) << "and powers the drive down";
}

TEST(SafetyFaultTest, DeadControlTaskAtRestIsATaskFailure)
{
    Rig healthy;
    healthy.run(10s);
    EXPECT_EQ(healthy.status().fault, FaultCode::None) << "running tasks never fault at rest";

    Rig rig;
    const auto session = rig.commands().openSession();
    const Instant last = rig.clock.time;
    while (rig.status().state != MotionState::Fault && rig.clock.time - last < 1s) rig.tick(/*withControl=*/false);
    EXPECT_EQ(rig.status().fault, FaultCode::ControlStalled);
    EXPECT_GT(rig.clock.time - last, Instant{timing::kControlStallLimit});
    EXPECT_LE(rig.clock.time - last, Instant{timing::kControlStallLimit} + kTick);
    EXPECT_TRUE(rig.drive.poweredDown);
    EXPECT_EQ(rig.velocity(session), CommandOutcome::RejectedFault) << "no Accepted that can never be executed";
}

TEST(SafetyFaultTest, EveryFaultDisablesThenPowersDownAndNeverRecoversWithoutAReset)
{
    Rig rig;
    auto session = rig.commands().openSession();
    ASSERT_EQ(rig.velocity(session), CommandOutcome::Accepted);
    rig.run(20ms);
    rig.drive.failCommand = true;
    rig.run(10ms);
    ASSERT_EQ(rig.status().fault, FaultCode::DriveWriteFailed);
    const int disable = rig.drive.firstIndexOf(DriveEvent::Kind::Disable);
    const int powerDown = rig.drive.firstIndexOf(DriveEvent::Kind::PowerDown);
    ASSERT_GE(powerDown, 0);
    EXPECT_LT(disable, powerDown);
    rig.drive.failCommand = false;
    const int writes = rig.drive.commandCount();
    session = rig.commands().openSession();  // Reconnect.
    EXPECT_EQ(rig.velocity(session), CommandOutcome::RejectedFault);
    EXPECT_EQ(rig.commands().clearEmergencyStop(session), CommandOutcome::RejectedFault);
    EXPECT_EQ(rig.runtime->enterMaintenance(), true);
    rig.run(1s);
    EXPECT_EQ(rig.status().state, MotionState::Fault);
    EXPECT_EQ(rig.drive.commandCount(), writes);
}

TEST(SafetyFaultTest, ShutdownEndsMotionAndTheSessionAndPowersDownInsideTheCall)
{
    Rig rig;
    const auto session = rig.commands().openSession();
    ASSERT_EQ(rig.velocity(session, kForward, timing::kCommandValidityCeiling), CommandOutcome::Accepted);
    rig.run(20ms);
    ASSERT_TRUE(rig.drive.enabled);
    const auto events = rig.drive.events.size();
    rig.runtime->shutdown();
    ASSERT_EQ(rig.drive.events.size(), events + 2);
    EXPECT_EQ(rig.drive.events[events].kind, DriveEvent::Kind::Disable);
    EXPECT_EQ(rig.drive.events[events + 1].kind, DriveEvent::Kind::PowerDown);
    const auto status = rig.status();
    EXPECT_FALSE(status.ready);
    EXPECT_FALSE(status.sessionReady);
    EXPECT_FALSE(status.outputEnabled);
    EXPECT_EQ(rig.velocity(session), CommandOutcome::RejectedNotReady);
    EXPECT_EQ(rig.velocity(rig.commands().openSession()), CommandOutcome::RejectedNotReady);
    rig.run(100ms);
    EXPECT_FALSE(rig.drive.enabled);
}

// The hook the platform runs inside esp_restart() is the runtime's shutdown.
TEST(SafetyFaultTest, RegisteredRestartHookShutsDownADrivingRuntime)
{
    Rig rig;
    ASSERT_NE(rig.platform.shutdownHook, nullptr);
    const auto session = rig.commands().openSession();
    ASSERT_EQ(rig.velocity(session, kForward, timing::kCommandValidityCeiling), CommandOutcome::Accepted);
    rig.run(20ms);
    ASSERT_TRUE(rig.drive.enabled);
    rig.platform.shutdownHook->beforeRestart();
    EXPECT_FALSE(rig.drive.enabled);
    EXPECT_TRUE(rig.drive.poweredDown);
    EXPECT_FALSE(rig.status().ready);
    EXPECT_EQ(rig.velocity(session), CommandOutcome::RejectedNotReady);
}

// A reset (any cause: task watchdog, brownout, panic, power cycle) is a new boot with fresh
// RAM. It returns to zero output and never restores the previous intent, session or latch.
TEST(SafetyFaultTest, ResetReturnsToZeroOutputIdleAndRestoresNothing)
{
    SessionToken previous{};
    {
        Rig before;
        previous = before.commands().openSession();
        ASSERT_EQ(before.velocity(previous, kForward, timing::kCommandValidityCeiling), CommandOutcome::Accepted);
        before.run(20ms);
        ASSERT_EQ(before.commands().emergencyStop(), CommandOutcome::Accepted);
    }  // Reset: everything above was RAM.
    Rig after;
    ASSERT_FALSE(after.drive.events.empty());
    EXPECT_EQ(after.drive.events.front().kind, DriveEvent::Kind::Disable) << "outputs off before anything else";
    const auto status = after.status();
    EXPECT_EQ(status.state, MotionState::Idle) << "v1: the E-stop latch is RAM-only and clears on reset";
    EXPECT_FALSE(status.sessionReady);
    EXPECT_FALSE(status.outputEnabled);
    EXPECT_FALSE(status.commandExpired);
    EXPECT_EQ(after.velocity(previous), CommandOutcome::RejectedSessionNotReady)
        << "a token from before the reset is not a session of this boot";
    after.run(1s);
    EXPECT_EQ(after.drive.commandCount(), 0) << "nothing is restored";
    const auto session = after.commands().openSession();
    ASSERT_EQ(after.velocity(session), CommandOutcome::Accepted) << "only new intent in a new session moves";
}

// ---- Command processor: no old command stays buffered ---------------------------------

TEST(SafetyMailboxTest, MotionEndingTransitionsLeaveNoBufferedCommand)
{
    FakeClock clock;
    clock.time = kBoot;
    FakeDrive drive;
    RuntimeDiagnostics diagnostics;
    ActuatorGate gate{drive, diagnostics, clock};
    SetpointMailbox mailbox;
    CommandProcessor commands{gate, mailbox, diagnostics};
    commands.setVelocityLimits(1.0, 1.0);
    gate.markReady(clock.time);
    auto session = commands.openSession();
    const auto admit = [&] { return commands.velocity(session, kForward, kValid, clock.time); };

    ASSERT_EQ(admit(), CommandOutcome::Accepted);
    ASSERT_TRUE(mailbox.latest());
    ASSERT_EQ(commands.clearEmergencyStop(session), CommandOutcome::Accepted);
    EXPECT_TRUE(mailbox.latest()) << "a no-op clear while moving keeps the setpoint";
    (void)commands.stop();
    EXPECT_FALSE(mailbox.latest()) << "Stop";

    ASSERT_EQ(admit(), CommandOutcome::Accepted);
    (void)commands.emergencyStop();
    EXPECT_FALSE(mailbox.latest()) << "EmergencyStop";
    ASSERT_EQ(commands.clearEmergencyStop(session), CommandOutcome::Accepted);
    EXPECT_FALSE(mailbox.latest()) << "clear does not resurrect anything";

    ASSERT_EQ(admit(), CommandOutcome::Accepted);
    const auto old = session;
    session = commands.openSession();
    EXPECT_FALSE(mailbox.latest()) << "replacement";

    ASSERT_EQ(admit(), CommandOutcome::Accepted);
    commands.sessionEnded(old, SessionEndReason::Disconnected);
    EXPECT_TRUE(mailbox.latest()) << "a stale end does not touch the current session";
    commands.sessionEnded(session, SessionEndReason::NotificationsDisabled);
    EXPECT_FALSE(mailbox.latest()) << "session end";
}

// ---- The transition table ---------------------------------------------------------------

namespace {
enum class Start { Idle, Moving, EmergencyStop, Fault };
enum Event : std::size_t {
    Velocity,
    VelocityStaleSession,
    Stop,
    EmergencyStop,
    Clear,
    ClearStaleSession,
    Expiry,
    OpenSession,
    EndSession,
    EndStaleSession,
    Fault,
    Maintenance,
    Shutdown,
    EventCount,
};
constexpr std::array<const char*, EventCount> kEventNames{
    "Velocity", "VelocityStaleSession", "Stop", "EmergencyStop", "Clear", "ClearStaleSession", "Expiry",
    "OpenSession", "EndSession", "EndStaleSession", "Fault", "Maintenance", "Shutdown"};
constexpr MotionState I = MotionState::Idle;
constexpr MotionState M = MotionState::Moving;
constexpr MotionState E = MotionState::EmergencyStop;
constexpr MotionState F = MotionState::Fault;
// Resulting state for every (start, event). Columns follow `Event`.
constexpr std::array<std::array<MotionState, EventCount>, 4> kTable{{
    /* Idle   */ {M, I, I, E, I, I, I, I, I, I, F, I, I},
    /* Moving */ {M, M, I, E, M, M, I, I, I, M, F, I, I},
    /* E-stop */ {E, E, E, E, I, E, E, E, E, E, F, E, E},
    /* Fault  */ {F, F, F, F, F, F, F, F, F, F, F, F, F},
}};
// Events after which a Moving start may still drive its original setpoint.
constexpr std::array<bool, EventCount> kKeepsOldIntent{false, true,  false, false, true,  true, false,
                                                       false, false, true,  false, false, false};
} // namespace

TEST(SafetyTransitionTest, EveryStateAndEventFollowsTheTable)
{
    constexpr Instant kReceipt = 1s;
    for (std::size_t row = 0; row < kTable.size(); ++row) {
        for (std::size_t event = 0; event < EventCount; ++event) {
            SCOPED_TRACE(testing::Message() << "start=" << row << " event=" << kEventNames[event]);
            SafetyState safety;
            safety.markReady();
            const auto stale = safety.openSession();
            const auto session = safety.openSession();
            const auto start = static_cast<Start>(row);
            if (start == Start::Moving) {
                ASSERT_EQ(safety.admitVelocity(session, kReceipt, kReceipt, kValid, kValid, true),
                          CommandOutcome::Accepted);
            }
            if (start == Start::EmergencyStop) safety.latchEmergencyStop();
            if (start == Start::Fault) safety.enterFault(FaultCode::DriveWriteFailed);
            const auto originalGeneration = safety.generation();
            const Instant now = kReceipt + 1ms;

            switch (static_cast<Event>(event)) {
            case Velocity: (void)safety.admitVelocity(session, now, now, kValid, kValid, true); break;
            case VelocityStaleSession: (void)safety.admitVelocity(stale, now, now, kValid, kValid, true); break;
            case Stop: safety.stop(); break;
            case EmergencyStop: safety.latchEmergencyStop(); break;
            case Clear: (void)safety.clearEmergencyStop(session); break;
            case ClearStaleSession: (void)safety.clearEmergencyStop(stale); break;
            case Expiry: (void)safety.expireIfDue(kReceipt + Instant{kValid}); break;
            case OpenSession: (void)safety.openSession(); break;
            case EndSession: (void)safety.endSession(session); break;
            case EndStaleSession: (void)safety.endSession(stale); break;
            case Fault: safety.enterFault(FaultCode::ControlStalled); break;
            case Maintenance: safety.enterMaintenance(); break;
            case Shutdown: safety.shutDown(); break;
            case EventCount: break;
            }

            EXPECT_EQ(safety.state(), kTable[row][event]);
            // Nonzero output is possible only while Moving, ready, in a session, before the deadline.
            const bool drivable = safety.permits(safety.generation(), now);
            EXPECT_EQ(drivable, safety.state() == MotionState::Moving);
            if (safety.state() != MotionState::Moving) {
                EXPECT_FALSE(safety.deadline());
            }
            if (start == Start::Moving) {
                EXPECT_EQ(safety.permits(originalGeneration, now), kKeepsOldIntent[event])
                    << "only no-op events keep the original setpoint drivable";
            }
            if (start == Start::Fault) {
                EXPECT_EQ(safety.fault(), FaultCode::DriveWriteFailed) << "first fault kept";
            }
        }
    }
}

TEST(SafetyTransitionTest, NotReadyStatesNeverAdmitOrPermitMotion)
{
    const std::vector<std::pair<const char*, std::function<void(SafetyState&)>>> cases{
        {"StartupIncomplete", [](SafetyState&) {}},
        {"Maintenance", [](SafetyState& s) { s.enterMaintenance(); s.markReady(); }},
        {"ShutDown", [](SafetyState& s) { s.shutDown(); s.markReady(); }},
        {"MaintenanceAfterReady", [](SafetyState& s) { s.markReady(); s.enterMaintenance(); }},
        {"ShutDownAfterReady", [](SafetyState& s) { s.markReady(); s.shutDown(); }},
    };
    for (const auto& [name, setup] : cases) {
        SCOPED_TRACE(name);
        SafetyState safety;
        setup(safety);
        const auto session = safety.openSession();
        EXPECT_EQ(safety.admitVelocity(session, 0us, 0us, kValid, kValid, true), CommandOutcome::RejectedNotReady);
        EXPECT_FALSE(safety.permits(safety.generation(), 0us));
        EXPECT_FALSE(safety.ready());
    }
}

// ---- Telemetry flags on the wire --------------------------------------------------------

namespace {
std::vector<std::uint8_t> velocityFrame(std::uint16_t sequence, std::int16_t linear, std::uint16_t validFor)
{
    using namespace protocol::v1;
    std::vector<std::uint8_t> b(kVelocityLength, 0);
    b[kVersionAt] = kProtocolVersion;
    b[kTypeAt] = static_cast<std::uint8_t>(MessageType::Velocity);
    const auto put = [&b](std::size_t at, std::uint16_t v) {
        b[at] = static_cast<std::uint8_t>(v & 0xFFU);
        b[at + 1] = static_cast<std::uint8_t>(v >> 8);
    };
    put(kSequenceAt, sequence);
    put(kVelocityLinearAt, static_cast<std::uint16_t>(linear));
    put(kVelocityValidForAt, validFor);
    return b;
}
std::vector<std::uint8_t> headerFrame(protocol::v1::MessageType type, std::uint16_t sequence)
{
    return {protocol::v1::kProtocolVersion, static_cast<std::uint8_t>(type),
            static_cast<std::uint8_t>(sequence & 0xFFU), static_cast<std::uint8_t>(sequence >> 8)};
}

struct WireRig {
    Rig rig;
    struct View final : protocol::v1::IRuntimeStatusView {
        const Rig& r;
        explicit View(const Rig& rig) : r(rig) {}
        RuntimeStatus status() const noexcept override { return r.status(); }
    } view{rig};
    protocol::v1::CommandSession session{rig.commands(), view, protocol::v1::CommandLimits{150, 3000, 250}};

    WireRig() { session.open(); }
    protocol::v1::ResultCode write(const std::vector<std::uint8_t>& frame)
    {
        const auto bytes = session.onCommandWrite(frame, rig.clock.nowMicroseconds());
        return static_cast<protocol::v1::ResultCode>(bytes[protocol::v1::kResultCodeAt]);
    }
    struct Flags {
        MotionState state;
        bool expired;
        bool accepted;
    };
    Flags telemetry()
    {
        using namespace protocol::v1;
        TelemetryValues values;
        values.state = MotionState::Fault;  // Ignored: the session reports the runtime's state.
        values.commandExpired = false;      // Ignored as well.
        const auto bytes = session.telemetry(values);
        EXPECT_TRUE(bytes);
        const auto flags = (*bytes)[kTelemetryFlagsAt];
        return {static_cast<MotionState>((*bytes)[kTelemetryStateAt]), (flags & kFlagCommandExpired) != 0,
                (flags & kFlagHasAcceptedCommand) != 0};
    }
};
} // namespace

TEST(SafetyTelemetryTest, TelemetryReflectsEveryTransitionWithTheV1Flags)
{
    using protocol::v1::MessageType;
    using protocol::v1::ResultCode;
    WireRig w;
    auto t = w.telemetry();
    EXPECT_EQ(t.state, MotionState::Idle);
    EXPECT_FALSE(t.expired);
    EXPECT_FALSE(t.accepted);

    ASSERT_EQ(w.write(velocityFrame(1, 100, 50)), ResultCode::Accepted);
    t = w.telemetry();
    EXPECT_EQ(t.state, MotionState::Moving);
    EXPECT_TRUE(t.accepted);
    w.rig.run(50ms);  // The watchdog ends motion.
    t = w.telemetry();
    EXPECT_EQ(t.state, MotionState::Idle);
    EXPECT_TRUE(t.expired) << "set when the watchdog ends motion";
    ASSERT_EQ(w.write(velocityFrame(2, 100, 50)), ResultCode::Accepted);
    EXPECT_FALSE(w.telemetry().expired) << "cleared by the next accepted Velocity";
    w.rig.run(50ms);
    EXPECT_TRUE(w.telemetry().expired);
    // Rejected and duplicate traffic and telemetry neither clear nor refresh anything.
    EXPECT_EQ(w.write(velocityFrame(2, 100, 50)), ResultCode::StaleSequence);
    EXPECT_EQ(w.write(velocityFrame(3, 151, 50)), ResultCode::OutOfRange);
    EXPECT_TRUE(w.telemetry().expired);
    ASSERT_EQ(w.write(headerFrame(MessageType::EmergencyStop, 4)), ResultCode::Accepted);
    t = w.telemetry();
    EXPECT_EQ(t.state, MotionState::EmergencyStop);
    EXPECT_TRUE(t.expired) << "E-stop does not clear it";
    ASSERT_EQ(w.write(headerFrame(MessageType::Stop, 5)), ResultCode::Accepted);
    t = w.telemetry();
    EXPECT_EQ(t.state, MotionState::EmergencyStop) << "Stop has no effect while latched...";
    EXPECT_FALSE(t.expired) << "...but it is an accepted Stop";
    ASSERT_EQ(w.write(headerFrame(MessageType::ClearEmergencyStop, 6)), ResultCode::Accepted);
    EXPECT_EQ(w.telemetry().state, MotionState::Idle);

    ASSERT_EQ(w.write(velocityFrame(7, 100, 50)), ResultCode::Accepted);
    w.rig.run(50ms);
    ASSERT_TRUE(w.telemetry().expired);
    w.session.end(SessionEndReason::Disconnected);
    w.session.open();  // Reconnect: a new session reports on its own commands.
    t = w.telemetry();
    EXPECT_FALSE(t.expired);
    EXPECT_FALSE(t.accepted);

    w.rig.drive.failCommand = true;
    ASSERT_EQ(w.write(velocityFrame(1, 100, 50)), ResultCode::Accepted);
    w.rig.run(10ms);
    EXPECT_EQ(w.telemetry().state, MotionState::Fault);
}

TEST(SafetyTelemetryTest, RejectedDuplicateAndDiagnosticTrafficNeverExtendTheDeadline)
{
    using protocol::v1::MessageType;
    using protocol::v1::ResultCode;
    WireRig w;
    constexpr std::uint16_t kWindowMs = 100;
    ASSERT_EQ(w.write(velocityFrame(10, 100, kWindowMs)), ResultCode::Accepted);
    const Instant deadline = w.rig.clock.time + Instant{milliseconds{kWindowMs}};
    const std::vector<std::vector<std::uint8_t>> rejected{
        velocityFrame(10, 100, kWindowMs),                // Duplicate.
        velocityFrame(9, 100, kWindowMs),                 // Older.
        velocityFrame(11, 151, kWindowMs),                // Out of range.
        velocityFrame(11, 100, 0),                        // Invalid validity.
        velocityFrame(11, 100, 251),                      // Above the ceiling.
        headerFrame(MessageType::ClearEmergencyStop, 10), // Stale clear.
        {0x7B, 0x22, 0x76, 0x22},                         // JSON.
        {},
    };
    while (w.rig.clock.time < deadline) {
        EXPECT_EQ(w.rig.status().state, MotionState::Moving);
        for (const auto& frame : rejected) EXPECT_NE(w.write(frame), ResultCode::Accepted);
        (void)w.telemetry();
        (void)w.rig.runtime->diagnostics();
        w.rig.tick();
    }
    EXPECT_EQ(w.rig.clock.time, deadline);
    EXPECT_EQ(w.rig.status().state, MotionState::Idle) << "expired exactly at the original deadline";
    EXPECT_EQ(w.session.lastAcceptedSequence(), 10);
}
