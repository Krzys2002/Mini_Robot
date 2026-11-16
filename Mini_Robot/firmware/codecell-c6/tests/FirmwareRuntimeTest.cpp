// Host tests of the #158 runtime orchestration: injected start-up failures, the single
// command authority, stale snapshots, concurrent stop/new-command handoffs and mailbox
// pressure. Interleavings are chosen explicitly on a fake clock; nothing touches hardware.
#include "RuntimeFakes.hpp"
#include "runtime/FirmwareRuntime.hpp"
#include <gtest/gtest.h>
#include <ostream>
#include <string>

using namespace codecell::runtime;
using namespace codecell::runtime::fakes;
using namespace std::chrono_literals;
namespace timing = codecell::timing;

namespace {
constexpr BodyVelocity kForward{0.05, 0.0};
constexpr BodyVelocity kTurn{0.0, 0.5};
constexpr milliseconds kValid = 100ms;

struct Rig {
    FakeClock clock;
    FakePlatform platform;
    FakeDrive drive;
    FakeConfiguration configuration;
    FakeInertialSensor sensor;
    std::optional<FirmwareRuntime> runtime;

    explicit Rig(bool withSensor = false)
    {
        clock.time = 1s;
        drive.section = &platform.section;
        runtime.emplace(RuntimeDependencies{clock, platform, drive, configuration, withSensor ? &sensor : nullptr});
    }
    FirmwareRuntime& rt() { return *runtime; }
    IMotionCommandSink& commands() { return rt().commands(); }
    void control() { rt().controlStep().step(); }
    void supervise() { rt().safetyStep().step(); }
    // Runs both tasks on their nominal periods for `duration`.
    void run(Instant duration)
    {
        const Instant end = clock.time + duration;
        while (clock.time < end) {
            clock.advance(Instant{timing::kSafetyCheckPeriod});
            supervise();
            if ((clock.time / Instant{timing::kSafetyCheckPeriod}) % 2 == 0) control();
        }
    }
    // The ready session the commands belong to; opened on first use, as #163 will on CCCDs.
    SessionToken session{};
    SessionToken currentSession()
    {
        if (session.value == 0) session = commands().openSession();
        return session;
    }
    void endSession(SessionEndReason reason)
    {
        commands().sessionEnded(session, reason);
        session = {};
    }
    CommandOutcome clear() { return commands().clearEmergencyStop(currentSession()); }
    CommandOutcome velocity(const BodyVelocity& v, milliseconds validFor = kValid)
    {
        return commands().velocity(currentSession(), v, validFor, clock.nowMicroseconds());
    }
};

bool everCommandedNonzero(const FakeDrive& drive)
{
    return drive.commandCount() > 0;
}
} // namespace

TEST(FirmwareRuntimeTest, BootsDisabledAndRejectsMotionUntilStarted)
{
    Rig rig;
    EXPECT_EQ(rig.velocity(kForward), CommandOutcome::RejectedNotReady);
    EXPECT_FALSE(rig.rt().status().ready);
    ASSERT_EQ(rig.rt().start(), StartResult::Running);
    ASSERT_FALSE(rig.drive.events.empty());
    EXPECT_EQ(rig.drive.events.front().kind, DriveEvent::Kind::Disable) << "outputs off before anything else";
    const auto status = rig.rt().status();
    EXPECT_EQ(status.state, MotionState::Idle);
    EXPECT_EQ(status.fault, FaultCode::None);
    EXPECT_TRUE(status.ready);
    EXPECT_FALSE(status.outputEnabled);
    rig.run(100ms);
    EXPECT_FALSE(everCommandedNonzero(rig.drive)) << "idle boot never commands the drive";
    EXPECT_EQ(rig.rt().start(), StartResult::Running) << "start is idempotent";
    EXPECT_EQ(rig.platform.tasks.size(), 2u);
}

TEST(FirmwareRuntimeTest, TasksFollowTheBudgetPrioritiesPeriodsAndOrder)
{
    Rig rig;
    ASSERT_EQ(rig.rt().start(), StartResult::Running);
    ASSERT_EQ(rig.platform.tasks.size(), 2u);
    const auto& safety = rig.platform.tasks[0].spec;
    const auto& control = rig.platform.tasks[1].spec;
    EXPECT_EQ(std::string{safety.name}, kSafetyTaskName) << "the supervisor starts before control";
    EXPECT_EQ(safety.priority, timing::kSafetyPriority);
    EXPECT_EQ(safety.period, timing::kSafetyCheckPeriod);
    EXPECT_EQ(std::string{control.name}, kControlTaskName);
    EXPECT_EQ(control.priority, timing::kControlPriority);
    EXPECT_EQ(control.period, timing::kControlPeriod);
    EXPECT_GT(safety.stackBytes, 0u);
    EXPECT_GT(control.stackBytes, 0u);
}

struct InjectedFailure {
    const char* name;
    std::function<void(Rig&)> inject;
    FaultCode expected;
    std::size_t tasksStarted;
};

// Keeps CTest names readable (GoogleTest would otherwise print the object bytes).
void PrintTo(const InjectedFailure& failure, std::ostream* out)
{
    *out << failure.name;
}

class StartupFailureTest : public testing::TestWithParam<InjectedFailure> {};

TEST_P(StartupFailureTest, LeavesDrivesOffReportsTheFaultAndNeverMoves)
{
    const auto& failure = GetParam();
    Rig rig;
    failure.inject(rig);
    EXPECT_EQ(rig.rt().start(), StartResult::Faulted);
    const auto status = rig.rt().status();
    EXPECT_EQ(status.state, MotionState::Fault);
    EXPECT_EQ(status.fault, failure.expected) << toString(status.fault);
    EXPECT_FALSE(status.ready);
    EXPECT_FALSE(status.outputEnabled);
    EXPECT_EQ(rig.platform.tasks.size(), failure.tasksStarted);

    // Normal motion is never attempted: commands are refused and any task that did start
    // keeps the outputs off.
    EXPECT_EQ(rig.velocity(kForward), CommandOutcome::RejectedFault);
    EXPECT_EQ(rig.clear(), CommandOutcome::RejectedFault);
    for (int i = 0; i < 20; ++i) {
        rig.clock.advance(Instant{timing::kSafetyCheckPeriod});
        for (const auto& task : rig.platform.tasks) task.body->step();
    }
    EXPECT_FALSE(everCommandedNonzero(rig.drive));
    EXPECT_EQ(rig.rt().start(), StartResult::Faulted) << "a failed start is not retried into motion";
}

INSTANTIATE_TEST_SUITE_P(
    InjectedInitialization, StartupFailureTest,
    testing::Values(
        InjectedFailure{"DriveDisable", [](Rig& r) { r.drive.failDisable = true; }, FaultCode::DriveDisableFailed, 0},
        InjectedFailure{"ShutdownHookRegistration", [](Rig& r) { r.platform.failShutdownHook = true; },
                        FaultCode::ShutdownHookUnavailable, 0},
        InjectedFailure{"ConfigMissing", [](Rig& r) { r.configuration.value.reset(); },
                        FaultCode::ConfigurationUnavailable, 0},
        InjectedFailure{"ConfigInvalid",
                        [](Rig& r) { r.configuration.value->controlTask.priority = timing::kNimbleHostPriority + 1; },
                        FaultCode::ConfigurationInvalid, 0},
        InjectedFailure{"CriticalSectionAllocation", [](Rig& r) { r.platform.failCriticalSection = true; },
                        FaultCode::SynchronizationUnavailable, 0},
        InjectedFailure{"SupervisorTask", [](Rig& r) { r.platform.failTaskNamed = kSafetyTaskName; },
                        FaultCode::SupervisorTaskStartFailed, 0},
        InjectedFailure{"ControlTask", [](Rig& r) { r.platform.failTaskNamed = kControlTaskName; },
                        FaultCode::ControlTaskStartFailed, 1}),
    [](const testing::TestParamInfo<InjectedFailure>& info) { return std::string{info.param.name}; });

// Review finding (#281): a failed shutdown-hook registration was only printed, and the image
// still became motion-ready without its deliberate-restart protection.
TEST(FirmwareRuntimeTest, FailedShutdownHookRegistrationStopsStartupBeforeTheDriveIsConfigured)
{
    Rig rig;
    rig.platform.failShutdownHook = true;
    ASSERT_EQ(rig.rt().start(), StartResult::Faulted);
    EXPECT_FALSE(rig.rt().shutdownHookRegistered());
    EXPECT_EQ(rig.rt().status().fault, FaultCode::ShutdownHookUnavailable);
    EXPECT_FALSE(rig.drive.configured) << "the bridges are never woken";
    EXPECT_FALSE(rig.rt().config().has_value());
    EXPECT_TRUE(rig.platform.tasks.empty());
    EXPECT_FALSE(rig.drive.enabled);
    EXPECT_TRUE(rig.drive.poweredDown) << "the fault powers the drive down";
    EXPECT_EQ(rig.velocity(kForward), CommandOutcome::RejectedFault);
    EXPECT_EQ(rig.clear(), CommandOutcome::RejectedFault);
    EXPECT_FALSE(everCommandedNonzero(rig.drive));
}

TEST(FirmwareRuntimeTest, ShutdownHookIsRegisteredBeforeAnyOtherStartupStep)
{
    // Every later step can fail and the hook is already in place, including a boot disable
    // that could not be confirmed, where the drive state is least certain.
    for (const bool failDisable : {true, false}) {
        Rig rig;
        rig.drive.failDisable = failDisable;
        rig.drive.failConfigure = !failDisable;
        ASSERT_EQ(rig.rt().start(), StartResult::Faulted);
        EXPECT_EQ(rig.rt().status().fault,
                  failDisable ? FaultCode::DriveDisableFailed : FaultCode::DriveConfigurationRejected);
        EXPECT_TRUE(rig.rt().shutdownHookRegistered()) << failDisable;
        ASSERT_NE(rig.platform.shutdownHook, nullptr) << failDisable;
        rig.platform.shutdownHook->beforeRestart();
        EXPECT_TRUE(rig.drive.poweredDown) << failDisable;
    }

    Rig ok;
    ASSERT_EQ(ok.rt().start(), StartResult::Running);
    EXPECT_TRUE(ok.rt().shutdownHookRegistered());
    ASSERT_NE(ok.platform.shutdownHook, nullptr);
}

// Review finding (#283): the telemetry task was started after the runtime and BLE, and a
// failure only logged a warning while Velocity was still accepted.
TEST(FirmwareRuntimeTest, TelemetryTaskIsAMandatoryStartupStep)
{
    struct Step final : IPeriodicStep {
        void step() noexcept override {}
    } telemetry;
    const PeriodicTask task{{"mr_telemetry", timing::kTelemetryPriority, kDefaultTaskStackBytes,
                             timing::kTelemetryTickPeriod, false},
                            &telemetry};
    {
        FakeClock clock;
        FakePlatform platform;
        FakeDrive drive;
        FakeConfiguration configuration;
        drive.section = &platform.section;
        FirmwareRuntime runtime{{clock, platform, drive, configuration, nullptr, task}};
        ASSERT_EQ(runtime.start(), StartResult::Running);
        EXPECT_TRUE(runtime.telemetryTaskStarted());
        ASSERT_EQ(platform.tasks.size(), 3U);
        EXPECT_EQ(std::string{platform.tasks[2].spec.name}, "mr_telemetry") << "after the supervisor and control";
        EXPECT_EQ(platform.tasks[2].spec.priority, timing::kTelemetryPriority);
    }
    FakeClock clock;
    FakePlatform platform;
    FakeDrive drive;
    FakeConfiguration configuration;
    drive.section = &platform.section;
    platform.failTaskNamed = "mr_telemetry";
    FirmwareRuntime runtime{{clock, platform, drive, configuration, nullptr, task}};
    EXPECT_EQ(runtime.start(), StartResult::Faulted);
    EXPECT_FALSE(runtime.telemetryTaskStarted());
    const auto status = runtime.status();
    EXPECT_EQ(status.fault, FaultCode::TelemetryTaskStartFailed);
    EXPECT_FALSE(status.ready);
    EXPECT_FALSE(status.outputEnabled);
    const auto session = runtime.commands().openSession();
    EXPECT_EQ(runtime.commands().velocity(session, kForward, 100ms, clock.nowMicroseconds()), CommandOutcome::RejectedFault);
    EXPECT_FALSE(everCommandedNonzero(drive));
}

TEST(FirmwareRuntimeTest, InvalidConfigurationReportsWhichRuleFailed)
{
    Rig rig;
    rig.configuration.value->commandValidityCeiling = timing::kCommandValidityCeiling + 1ms;
    EXPECT_EQ(rig.rt().start(), StartResult::Faulted);
    EXPECT_EQ(rig.rt().configError(), ConfigError::ValidityCeilingOutOfRange);
    EXPECT_FALSE(rig.rt().config().has_value()) << "an invalid configuration is never applied";
}

TEST(FirmwareRuntimeTest, AcceptedVelocityDrivesOnlyThroughTheControlTaskAndGate)
{
    Rig rig;
    ASSERT_EQ(rig.rt().start(), StartResult::Running);
    rig.drive.calledOutsideSection = false;  // The boot disable precedes the section (single-threaded).
    ASSERT_EQ(rig.velocity(kForward), CommandOutcome::Accepted);
    EXPECT_FALSE(everCommandedNonzero(rig.drive)) << "the command path never writes the drive";
    EXPECT_EQ(rig.rt().status().state, MotionState::Moving);
    rig.control();
    ASSERT_TRUE(rig.drive.lastCommand());
    EXPECT_DOUBLE_EQ(rig.drive.lastCommand()->linearMetresPerSecond, kForward.linearMetresPerSecond);
    EXPECT_TRUE(rig.rt().status().outputEnabled);
    EXPECT_FALSE(rig.drive.calledOutsideSection) << "every drive call is serialized by the gate";
    EXPECT_LE(rig.platform.section.maxDepth, 1) << "the critical section is never entered recursively";
    EXPECT_EQ(rig.platform.section.depth, 0);
}

TEST(FirmwareRuntimeTest, NonFiniteOrNonPositiveCommandsAreRejected)
{
    Rig rig;
    ASSERT_EQ(rig.rt().start(), StartResult::Running);
    EXPECT_EQ(rig.velocity({std::numeric_limits<double>::quiet_NaN(), 0.0}), CommandOutcome::RejectedInvalid);
    EXPECT_EQ(rig.velocity({0.0, std::numeric_limits<double>::infinity()}), CommandOutcome::RejectedInvalid);
    EXPECT_EQ(rig.velocity(kForward, 0ms), CommandOutcome::RejectedInvalid);
    EXPECT_EQ(rig.rt().diagnostics()[Counter::CommandRejected], 3u);
    rig.run(50ms);
    EXPECT_FALSE(everCommandedNonzero(rig.drive));
}

TEST(FirmwareRuntimeTest, ValidityIsCappedByTheConfiguredCeiling)
{
    Rig rig;
    ASSERT_EQ(rig.rt().start(), StartResult::Running);
    const Instant receipt = rig.clock.time;
    ASSERT_EQ(rig.velocity(kForward, 60000ms), CommandOutcome::Accepted);
    while (rig.rt().status().state == MotionState::Moving && rig.clock.time - receipt < 1s)
        rig.run(Instant{timing::kSafetyCheckPeriod});
    EXPECT_EQ(rig.rt().status().state, MotionState::Idle);
    EXPECT_FALSE(rig.drive.enabled);
    EXPECT_GE(rig.clock.time - receipt, Instant{timing::kCommandValidityCeiling});
    EXPECT_LE(rig.clock.time - receipt,
              Instant{timing::kCommandValidityCeiling} + Instant{timing::kExpiryDetectionBound});
    EXPECT_EQ(rig.rt().diagnostics()[Counter::CommandExpired], 1u);
}

TEST(FirmwareRuntimeTest, StaleSnapshotPastItsDeadlineCannotDriveEvenWithoutTheSupervisor)
{
    Rig rig;
    ASSERT_EQ(rig.rt().start(), StartResult::Running);
    ASSERT_EQ(rig.velocity(kForward, 20ms), CommandOutcome::Accepted);
    rig.control();
    ASSERT_TRUE(rig.drive.enabled);
    const int writes = rig.drive.commandCount();
    rig.clock.advance(20ms);  // Exactly at the deadline: now >= deadline is expired.
    rig.control();            // The supervisor has not run yet.
    EXPECT_EQ(rig.drive.commandCount(), writes);
    EXPECT_FALSE(rig.drive.enabled);
    EXPECT_EQ(rig.rt().diagnostics()[Counter::StaleSetpointBlocked], 1u);
}

TEST(FirmwareRuntimeTest, StalledControlTaskIsDisabledByTheSupervisorWithinB7)
{
    Rig rig;
    ASSERT_EQ(rig.rt().start(), StartResult::Running);
    ASSERT_EQ(rig.velocity(kForward, timing::kCommandValidityCeiling), CommandOutcome::Accepted);
    rig.control();
    ASSERT_TRUE(rig.drive.enabled);
    const Instant lastUpdate = rig.clock.time;
    // Control stops (for example a blocked sensor read); only the supervisor runs.
    while (rig.drive.enabled && rig.clock.time - lastUpdate < 1s) {
        rig.clock.advance(Instant{timing::kSafetyCheckPeriod});
        rig.supervise();
    }
    EXPECT_FALSE(rig.drive.enabled);
    EXPECT_LE(rig.clock.time - lastUpdate, Instant{timing::kStalledControlToOutputOffBound});
    EXPECT_EQ(rig.rt().status().fault, FaultCode::ControlStalled);
    EXPECT_EQ(rig.velocity(kForward), CommandOutcome::RejectedFault);
}

TEST(FirmwareRuntimeTest, BlockedSensorReadCannotReenableOutputsAfterTheStallFault)
{
    Rig rig{true};
    ASSERT_EQ(rig.rt().start(), StartResult::Running);
    ASSERT_EQ(rig.velocity(kForward, timing::kCommandValidityCeiling), CommandOutcome::Accepted);
    rig.control();
    ASSERT_TRUE(rig.drive.enabled);
    const int writes = rig.drive.commandCount();
    // The next read blocks for 60 ms; the higher-priority supervisor keeps running.
    rig.sensor.whileReading = [&rig] {
        rig.sensor.whileReading = nullptr;
        for (int i = 0; i < 12; ++i) {
            rig.clock.advance(Instant{timing::kSafetyCheckPeriod});
            rig.supervise();
        }
    };
    rig.control();  // Resumes with the setpoint it is about to copy: must stay off.
    EXPECT_EQ(rig.drive.commandCount(), writes);
    EXPECT_FALSE(rig.drive.enabled);
    EXPECT_EQ(rig.rt().status().fault, FaultCode::ControlStalled);
}

TEST(FirmwareRuntimeTest, OptionalSensorIsLocalAndItsAbsenceNeverBlocksMotion)
{
    Rig rig{true};
    ASSERT_EQ(rig.rt().start(), StartResult::Running);
    rig.sensor.next.reset();  // Unavailable sensor.
    ASSERT_EQ(rig.velocity(kForward), CommandOutcome::Accepted);
    rig.control();
    EXPECT_TRUE(rig.drive.enabled);
    EXPECT_EQ(rig.rt().diagnostics()[Counter::SensorUnavailable], 1u);
    EXPECT_EQ(rig.rt().status().fault, FaultCode::None);

    Rig noSensor;
    ASSERT_EQ(noSensor.rt().start(), StartResult::Running);
    ASSERT_EQ(noSensor.velocity(kForward), CommandOutcome::Accepted);
    noSensor.control();
    EXPECT_TRUE(noSensor.drive.enabled);
}

// Every stop-like event injected after the control task copied the setpoint but before
// it wrote the drive (the preemption window) must win: that copy never drives again.
TEST(FirmwareRuntimeTest, StopLikeEventBetweenSnapshotCopyAndWriteAlwaysWins)
{
    struct Case {
        const char* name;
        std::function<void(Rig&)> event;
        MotionState after;
    };
    const std::vector<Case> cases{
        {"Stop", [](Rig& r) { (void)r.commands().stop(); }, MotionState::Idle},
        {"EmergencyStop", [](Rig& r) { (void)r.commands().emergencyStop(); }, MotionState::EmergencyStop},
        {"Disconnect", [](Rig& r) { r.endSession(SessionEndReason::Disconnected); }, MotionState::Idle},
        {"NotificationsOff",
         [](Rig& r) { r.endSession(SessionEndReason::NotificationsDisabled); }, MotionState::Idle},
        {"ResultOverload",
         [](Rig& r) { r.endSession(SessionEndReason::ResultQueueExhausted); }, MotionState::Idle},
    };
    for (const bool alreadyMoving : {false, true}) {
        for (const auto& c : cases) {
            SCOPED_TRACE(testing::Message() << c.name << " alreadyMoving=" << alreadyMoving);
            Rig rig;
            ASSERT_EQ(rig.rt().start(), StartResult::Running);
            ASSERT_EQ(rig.velocity(kForward), CommandOutcome::Accepted);
            if (alreadyMoving) rig.control();
            const int writes = rig.drive.commandCount();
            rig.clock.onNextRead = [&] { c.event(rig); };  // Fires after the mailbox copy.
            rig.control();
            EXPECT_EQ(rig.drive.commandCount(), writes);
            EXPECT_FALSE(rig.drive.enabled);
            EXPECT_EQ(rig.rt().status().state, c.after);
            rig.run(50ms);
            EXPECT_EQ(rig.drive.commandCount(), writes) << "no later cycle replays the old setpoint";
        }
    }
}

TEST(FirmwareRuntimeTest, NewCommandAfterAStopRacingAnOldSnapshotDrivesOnlyTheNewOne)
{
    Rig rig;
    ASSERT_EQ(rig.rt().start(), StartResult::Running);
    ASSERT_EQ(rig.velocity(kForward), CommandOutcome::Accepted);
    rig.control();
    // While control holds a copy of kForward: Stop, then a new Velocity kTurn.
    rig.clock.onNextRead = [&] {
        (void)rig.commands().stop();
        ASSERT_EQ(rig.velocity(kTurn), CommandOutcome::Accepted);
    };
    rig.control();
    EXPECT_FALSE(rig.drive.enabled) << "the copy of kForward is from an older generation";
    rig.control();
    ASSERT_TRUE(rig.drive.lastCommand());
    EXPECT_DOUBLE_EQ(rig.drive.lastCommand()->angularRadiansPerSecond, kTurn.angularRadiansPerSecond);
    EXPECT_DOUBLE_EQ(rig.drive.lastCommand()->linearMetresPerSecond, 0.0);
}

TEST(FirmwareRuntimeTest, ClearingTheLatchDoesNotResumeTheSetpointLeftInTheMailbox)
{
    Rig rig;
    ASSERT_EQ(rig.rt().start(), StartResult::Running);
    ASSERT_EQ(rig.velocity(kForward), CommandOutcome::Accepted);
    rig.control();
    ASSERT_EQ(rig.commands().emergencyStop(), CommandOutcome::Accepted);
    EXPECT_FALSE(rig.drive.enabled) << "disabled before emergencyStop() returns";
    EXPECT_EQ(rig.velocity(kTurn), CommandOutcome::RejectedEmergencyStopLatched);
    EXPECT_EQ(rig.commands().stop(), CommandOutcome::Accepted);
    rig.endSession(SessionEndReason::Disconnected);
    EXPECT_EQ(rig.rt().status().state, MotionState::EmergencyStop) << "the latch survives a session end";
    const int writes = rig.drive.commandCount();
    ASSERT_EQ(rig.clear(), CommandOutcome::Accepted);
    EXPECT_EQ(rig.rt().status().state, MotionState::Idle);
    rig.run(50ms);
    EXPECT_EQ(rig.drive.commandCount(), writes) << "a new Velocity is required after clear";
    ASSERT_EQ(rig.velocity(kTurn), CommandOutcome::Accepted);
    rig.control();
    EXPECT_TRUE(rig.drive.enabled);
    EXPECT_EQ(rig.clear(), CommandOutcome::Accepted);
    rig.control();
    EXPECT_TRUE(rig.drive.enabled) << "clear is a no-op when not latched";
}

TEST(FirmwareRuntimeTest, CommandFloodBetweenControlCyclesKeepsOnlyTheNewestSetpoint)
{
    Rig rig;
    ASSERT_EQ(rig.rt().start(), StartResult::Running);
    constexpr int kFrames = 1000;
    constexpr double kStep = 0.0001;  // m/s per frame; the last frame stays within maxLinear.
    for (int i = 1; i <= kFrames; ++i)
        ASSERT_EQ(rig.velocity({kStep * i, 0.0}), CommandOutcome::Accepted);
    rig.control();
    EXPECT_EQ(rig.drive.commandCount(), 1);
    EXPECT_DOUBLE_EQ(rig.drive.lastCommand()->linearMetresPerSecond, kStep * kFrames);
    const auto diagnostics = rig.rt().diagnostics();
    EXPECT_EQ(diagnostics[Counter::VelocityAccepted], static_cast<std::uint32_t>(kFrames));
    EXPECT_EQ(diagnostics[Counter::MailboxOverwrite], static_cast<std::uint32_t>(kFrames - 1));
}

TEST(FirmwareRuntimeTest, ResultQueueExhaustionStopsAndIsCounted)
{
    Rig rig;
    ASSERT_EQ(rig.rt().start(), StartResult::Running);
    ASSERT_EQ(rig.velocity(kForward), CommandOutcome::Accepted);
    rig.control();
    rig.endSession(SessionEndReason::ResultQueueExhausted);
    EXPECT_EQ(rig.rt().status().state, MotionState::Idle);
    EXPECT_TRUE(rig.rt().status().stopRamp) << "a session end is a normal stop (B6 ramp)";
    rig.control();
    EXPECT_FALSE(rig.drive.enabled);
    EXPECT_EQ(rig.rt().diagnostics()[Counter::SessionOverload], 1u);
    EXPECT_EQ(rig.rt().diagnostics()[Counter::SessionEnded], 1u);
}

TEST(FirmwareRuntimeTest, RefusedDriveWriteFaultsAndDisables)
{
    Rig rig;
    ASSERT_EQ(rig.rt().start(), StartResult::Running);
    rig.drive.failCommand = true;
    ASSERT_EQ(rig.velocity(kForward), CommandOutcome::Accepted);
    rig.control();
    EXPECT_EQ(rig.rt().status().fault, FaultCode::DriveWriteFailed);
    ASSERT_GE(rig.drive.events.size(), 2u);
    EXPECT_EQ(rig.drive.events[rig.drive.events.size() - 2].kind, DriveEvent::Kind::Disable);
    EXPECT_EQ(rig.drive.events.back().kind, DriveEvent::Kind::PowerDown) << "disable first, then power down";
    rig.drive.failCommand = false;
    EXPECT_EQ(rig.velocity(kForward), CommandOutcome::RejectedFault);
    const int writes = rig.drive.commandCount();
    rig.run(50ms);
    EXPECT_EQ(rig.drive.commandCount(), writes);
}

TEST(FirmwareRuntimeTest, FailedDisableAfterStartFaultsTheRuntime)
{
    Rig rig;
    ASSERT_EQ(rig.rt().start(), StartResult::Running);
    rig.drive.failDisable = true;
    rig.control();  // Idle: the gate disables, which now fails.
    EXPECT_EQ(rig.rt().status().fault, FaultCode::DriveDisableFailed);
    EXPECT_EQ(rig.velocity(kForward), CommandOutcome::RejectedFault);
}
