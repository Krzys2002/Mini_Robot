// Runtime tests of the #160 actuator contract: the real bounded drive behind the #158
// gate, with a fake motor HAL, fake clock and hand-interleaved tasks. They check that
// E-stop, faults and maintenance disable without waiting for any ramp, that normal Stop
// and expiry ramp to zero within the #156 B6 budget, that a ramp which does not finish is
// enforced, and that invalid body commands never reach the actuator path.
#include "DriveFakes.hpp"
#include "RuntimeFakes.hpp"
#include "drive/BoundedDriveOutput.hpp"
#include "runtime/FirmwareRuntime.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <optional>

using namespace codecell;
using namespace codecell::runtime;
using namespace codecell::runtime::fakes;
using namespace codecell::drive::fakes;
using namespace std::chrono_literals;
using drive::MotorDuty;

namespace {
constexpr BodyVelocity kFullForward{kTestDriveProfile.maxLinearMetresPerSecond, 0.0};
constexpr milliseconds kValid = 200ms;
constexpr int kFullScaleSteps = static_cast<int>(drive::kNormalFullScaleRampTime / timing::kControlPeriod);

int magnitude(const MotorDuty& duty) { return std::max(std::abs(duty.m1), std::abs(duty.m2)); }

struct Rig {
    FakeClock clock;
    FakePlatform platform;
    FakeMotorDriver motors;
    drive::BoundedDriveOutput drive{motors};
    FakeConfiguration configuration;
    std::optional<FirmwareRuntime> runtime;

    Rig()
    {
        clock.time = 1s;
        runtime.emplace(RuntimeDependencies{clock, platform, drive, configuration, nullptr});
        EXPECT_EQ(runtime->start(), StartResult::Running);
        motors.section = &platform.section;
    }
    IMotionCommandSink& commands() { return runtime->commands(); }
    RuntimeStatus status() { return runtime->status(); }
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
    // One 5 ms supervisor period; the control task runs every second period (10 ms).
    void tick()
    {
        clock.advance(Instant{timing::kSafetyCheckPeriod});
        runtime->safetyStep().step();
        if ((clock.time / Instant{timing::kSafetyCheckPeriod}) % 2 == 0) runtime->controlStep().step();
    }
    void run(Instant duration)
    {
        const Instant end = clock.time + duration;
        while (clock.time < end) tick();
    }
    // Drives the full-scale forward command until the ramp has settled.
    void accelerateFully()
    {
        ASSERT_EQ(velocity(kFullForward), CommandOutcome::Accepted);
        for (int i = 0; i < 2 * (kFullScaleSteps + 1); ++i) tick();
        ASSERT_EQ(magnitude(drive.lastDuty()),
                  static_cast<int>(std::lround(0.15 / 0.2 * kTestDriveProfile.maxOutputFraction * drive::kDutyFullScale)));
    }
};
} // namespace

TEST(ActuatorRuntimeTest, ForwardRampsUpWithTheProvisionedSignsAndBound)
{
    Rig rig;
    rig.accelerateFully();
    const auto duty = rig.drive.lastDuty();
    // kTestDriveProfile reverses the right wheel (M1, #317): forward is M2 positive, M1 negative.
    EXPECT_GT(duty.m2, 0);
    EXPECT_LT(duty.m1, 0);
    EXPECT_EQ(duty.m1, -duty.m2);
    EXPECT_FALSE(rig.motors.calledOutsideSection) << "every HAL write is serialized by the gate";
    for (const auto& write : rig.motors.writes()) EXPECT_LE(magnitude(write), drive::dutyBoundCounts(kTestDriveProfile));
}

TEST(ActuatorRuntimeTest, EmergencyStopDisablesInsideTheCallEvenWhileAccelerating)
{
    Rig rig;
    ASSERT_EQ(rig.velocity(kFullForward), CommandOutcome::Accepted);
    rig.run(30ms);  // Mid-acceleration: output nonzero and still rising.
    ASSERT_GT(magnitude(rig.drive.lastDuty()), 0);
    const int writes = rig.motors.count(MotorEvent::Kind::Write);
    ASSERT_EQ(rig.commands().emergencyStop(), CommandOutcome::Accepted);
    EXPECT_TRUE(rig.motors.lastIsCoast()) << "coasting before emergencyStop() returns, no ramp step";
    EXPECT_EQ(rig.drive.lastDuty(), MotorDuty{});
    rig.run(200ms);
    EXPECT_EQ(rig.motors.count(MotorEvent::Kind::Write), writes) << "no write at all after the E-stop";
    EXPECT_EQ(rig.status().state, MotionState::EmergencyStop);
}

TEST(ActuatorRuntimeTest, EmergencyStopAndFaultEndARunningStopRampAtOnce)
{
    for (const bool emergency : {true, false}) {
        SCOPED_TRACE(emergency ? "E-stop" : "maintenance");
        Rig rig;
        rig.accelerateFully();
        ASSERT_EQ(rig.commands().stop(), CommandOutcome::Accepted);
        rig.run(20ms);  // Two ramp steps: output still well above zero.
        ASSERT_TRUE(rig.status().stopRamp);
        ASSERT_GT(magnitude(rig.drive.lastDuty()), 0);
        const int writes = rig.motors.count(MotorEvent::Kind::Write);
        if (emergency) {
            ASSERT_EQ(rig.commands().emergencyStop(), CommandOutcome::Accepted);
        } else {
            ASSERT_TRUE(rig.runtime->enterMaintenance());
        }
        EXPECT_TRUE(rig.motors.lastIsCoast());
        EXPECT_FALSE(rig.status().stopRamp);
        EXPECT_FALSE(rig.status().outputEnabled);
        rig.run(200ms);
        EXPECT_EQ(rig.motors.count(MotorEvent::Kind::Write), writes) << "the ramp never resumes";
    }
}

TEST(ActuatorRuntimeTest, ControlStallDuringTheStopRampFaultsAndDisables)
{
    Rig rig;
    rig.accelerateFully();
    ASSERT_EQ(rig.commands().stop(), CommandOutcome::Accepted);
    rig.run(10ms);
    ASSERT_TRUE(rig.status().stopRamp);
    const Instant lastControl = rig.clock.time;
    while (!rig.motors.asleep && rig.clock.time - lastControl < 1s) {
        rig.clock.advance(Instant{timing::kSafetyCheckPeriod});
        rig.runtime->safetyStep().step();  // Control is stalled; only the supervisor runs.
    }
    EXPECT_TRUE(rig.motors.endsCoastThenSleep()) << "a fault coasts, then powers the bridges down";
    EXPECT_LE(rig.clock.time - lastControl, Instant{timing::kStalledControlToOutputOffBound});
    EXPECT_EQ(rig.status().fault, FaultCode::ControlStalled);
}

// Review finding: a short-validity Velocity accepted just before every supervisor period
// expires at each check. Expiry handling must not hide the heartbeat check, or a stalled
// control task leaves the last duty applied indefinitely.
TEST(ActuatorRuntimeTest, ControlStallIsEnforcedWhileShortCommandsKeepExpiring)
{
    constexpr milliseconds kShortValidity = 1ms;
    Rig rig;
    rig.accelerateFully();
    const Instant lastControl = rig.clock.time;
    // Commands are accepted until the stall fault (#162 then refuses them as RobotFault).
    while (rig.status().fault == FaultCode::None && rig.clock.time - lastControl < 1s) {
        rig.clock.advance(Instant{timing::kSafetyCheckPeriod - kShortValidity});
        ASSERT_EQ(rig.velocity(kFullForward, kShortValidity), CommandOutcome::Accepted);
        rig.clock.advance(Instant{kShortValidity});
        rig.runtime->safetyStep().step();  // Control is stalled; only the supervisor runs.
    }
    EXPECT_TRUE(rig.motors.endsCoastThenSleep()) << "a fault coasts, then powers the bridges down";
    EXPECT_FALSE(rig.status().outputEnabled);
    EXPECT_EQ(rig.status().fault, FaultCode::ControlStalled);
    EXPECT_LE(rig.clock.time - lastControl, Instant{timing::kStalledControlToOutputOffBound});
}

// The same pattern with control running but a ramp that never reaches zero: the stop
// enforcement must still fire although every check also sees an expiry.
TEST(ActuatorRuntimeTest, StopRampEnforcementIsNotHiddenByRepeatedExpiry)
{
    constexpr milliseconds kShortValidity = 1ms;
    FakeClock clock;
    clock.time = 1s;
    FakePlatform platform;
    FakeDrive drive;
    FakeConfiguration configuration;
    FirmwareRuntime runtime{{clock, platform, drive, configuration, nullptr}};
    ASSERT_EQ(runtime.start(), StartResult::Running);
    drive.rampNeverEnds = true;
    const auto session = runtime.commands().openSession();
    ASSERT_EQ(runtime.commands().velocity(session, kFullForward, kValid, clock.nowMicroseconds()),
              CommandOutcome::Accepted);
    clock.advance(10ms);
    runtime.controlStep().step();
    ASSERT_TRUE(drive.enabled);
    const Instant start = clock.time;
    while (drive.enabled && clock.time - start < 1s) {
        clock.advance(Instant{timing::kSafetyCheckPeriod - kShortValidity});
        ASSERT_EQ(runtime.commands().velocity(session, kFullForward, kShortValidity, clock.nowMicroseconds()),
                  CommandOutcome::Accepted);
        clock.advance(Instant{kShortValidity});
        runtime.safetyStep().step();
        if ((clock.time / Instant{timing::kSafetyCheckPeriod}) % 2 == 0) runtime.controlStep().step();
    }
    EXPECT_FALSE(drive.enabled);
    EXPECT_EQ(runtime.status().fault, FaultCode::StopRampOverrun);
    EXPECT_LE(clock.time - start, Instant{timing::kStopEnforcementLimit} + Instant{timing::kSafetyCheckPeriod});
}

// Normal Stop and expiry follow the separate bounded deceleration contract (#156 B6):
// the commanded output only shrinks and is zero, then disabled, within the budget.
TEST(ActuatorRuntimeTest, StopAndExpiryRampToZeroWithinTheB6Budget)
{
    for (const bool expiry : {false, true}) {
        SCOPED_TRACE(expiry ? "expiry" : "Stop");
        Rig rig;
        rig.accelerateFully();
        Instant decision{};
        if (expiry) {
            ASSERT_EQ(rig.velocity(kFullForward, 20ms), CommandOutcome::Accepted);
            while (rig.status().state == MotionState::Moving) rig.tick();
            decision = rig.clock.time;  // The supervisor expired it on this tick.
            EXPECT_EQ(rig.runtime->diagnostics()[Counter::CommandExpired], 1u);
        } else {
            decision = rig.clock.time;
            ASSERT_EQ(rig.commands().stop(), CommandOutcome::Accepted);
        }
        EXPECT_EQ(rig.status().state, MotionState::Idle);
        EXPECT_TRUE(rig.status().stopRamp);
        int previous = magnitude(rig.drive.lastDuty());
        while (!rig.motors.lastIsCoast() && rig.clock.time - decision < 1s) {
            rig.tick();
            const int now = magnitude(rig.drive.lastDuty());
            ASSERT_LE(now, previous) << "a normal stop never increases the output";
            previous = now;
        }
        EXPECT_TRUE(rig.motors.lastIsCoast());
        EXPECT_EQ(*rig.motors.lastWrite(), MotorDuty{}) << "the last ramp write before the disable is zero";
        // Ramp of 10 control steps plus at most one period to the first step (B6: zero <= 112 ms).
        EXPECT_LE(rig.clock.time - decision, Instant{timing::kStopRampMax + timing::kControlPeriod});
        // From 0.15 m/s at 0.02 m/s per step: 8 ramp steps, the first possibly on the decision tick.
        const int rampSteps = static_cast<int>(std::ceil(kFullForward.linearMetresPerSecond
                                                         / (kTestDriveProfile.maxWheelSpeedMetresPerSecond
                                                            / kFullScaleSteps)));
        EXPECT_GE(rig.clock.time - decision, Instant{timing::kControlPeriod} * (rampSteps - 1))
            << "the output was ramped, not cut";
        EXPECT_EQ(rig.status().fault, FaultCode::None);
        EXPECT_FALSE(rig.status().stopRamp);
    }
}

TEST(ActuatorRuntimeTest, NewVelocityDuringTheStopRampContinuesWithoutAJump)
{
    Rig rig;
    rig.accelerateFully();
    ASSERT_EQ(rig.commands().stop(), CommandOutcome::Accepted);
    rig.run(40ms);
    const int ramped = magnitude(rig.drive.lastDuty());
    ASSERT_GT(ramped, 0);
    ASSERT_EQ(rig.velocity(kFullForward), CommandOutcome::Accepted);
    rig.run(10ms);
    EXPECT_FALSE(rig.status().stopRamp);
    const int counts = std::abs(rig.drive.lastDuty().m1);
    EXPECT_GT(counts, ramped);
    // One normal acceleration step: 0.02 m/s -> 0.1 of the 0.4 bound -> 10.2 counts.
    EXPECT_LE(counts - ramped, 11) << "the new command accelerates from the ramped output";
}

TEST(ActuatorRuntimeTest, OutOfRangeBodyCommandsAreRejectedBeforeTheActuatorPath)
{
    Rig rig;
    const double maxLinear = kTestDriveProfile.maxLinearMetresPerSecond;
    const double maxAngular = kTestDriveProfile.maxAngularRadiansPerSecond;
    constexpr double kAbove = 1e-6;
    EXPECT_EQ(rig.velocity({maxLinear + kAbove, 0.0}), CommandOutcome::RejectedOutOfRange);
    EXPECT_EQ(rig.velocity({-maxLinear - kAbove, 0.0}), CommandOutcome::RejectedOutOfRange);
    EXPECT_EQ(rig.velocity({0.0, maxAngular + kAbove}), CommandOutcome::RejectedOutOfRange);
    EXPECT_EQ(rig.velocity({0.0, -maxAngular - kAbove}), CommandOutcome::RejectedOutOfRange);
    EXPECT_EQ(rig.runtime->diagnostics()[Counter::CommandRejected], 4u);
    EXPECT_EQ(rig.status().state, MotionState::Idle);
    rig.run(50ms);
    EXPECT_EQ(rig.motors.count(MotorEvent::Kind::Write), 0) << "no rejected command reaches the drive";
    EXPECT_EQ(rig.velocity({-maxLinear, -maxAngular}), CommandOutcome::Accepted) << "the limits are inclusive";
    // Range comes after state (v1 order): a latched robot reports the latch.
    ASSERT_EQ(rig.commands().emergencyStop(), CommandOutcome::Accepted);
    EXPECT_EQ(rig.velocity({maxLinear * 2, 0.0}), CommandOutcome::RejectedEmergencyStopLatched);
}

TEST(ActuatorRuntimeTest, RefusedHalWriteFaultsAndCoasts)
{
    Rig rig;
    ASSERT_EQ(rig.velocity(kFullForward), CommandOutcome::Accepted);
    rig.run(20ms);
    rig.motors.failWrite = true;
    rig.run(10ms);
    EXPECT_EQ(rig.status().fault, FaultCode::DriveWriteFailed);
    EXPECT_TRUE(rig.motors.endsCoastThenSleep());
}

// Enforcement of B6 with an adapter whose ramp never reaches zero (body-level fake).
TEST(ActuatorRuntimeTest, StopRampThatDoesNotReachZeroIsEnforcedByTheSupervisor)
{
    FakeClock clock;
    clock.time = 1s;
    FakePlatform platform;
    FakeDrive drive;
    FakeConfiguration configuration;
    FirmwareRuntime runtime{{clock, platform, drive, configuration, nullptr}};
    ASSERT_EQ(runtime.start(), StartResult::Running);
    drive.rampNeverEnds = true;
    const auto session = runtime.commands().openSession();
    ASSERT_EQ(runtime.commands().velocity(session, kFullForward, kValid, clock.nowMicroseconds()),
              CommandOutcome::Accepted);
    clock.advance(10ms);
    runtime.controlStep().step();
    ASSERT_TRUE(drive.enabled);
    const Instant decision = clock.time;
    ASSERT_EQ(runtime.commands().stop(), CommandOutcome::Accepted);
    while (drive.enabled && clock.time - decision < 1s) {
        clock.advance(Instant{timing::kSafetyCheckPeriod});
        runtime.safetyStep().step();
        if ((clock.time / Instant{timing::kSafetyCheckPeriod}) % 2 == 0) runtime.controlStep().step();
    }
    EXPECT_FALSE(drive.enabled);
    EXPECT_EQ(runtime.status().fault, FaultCode::StopRampOverrun);
    EXPECT_GE(clock.time - decision, Instant{timing::kStopEnforcementLimit});
    EXPECT_LE(clock.time - decision, Instant{timing::kStopEnforcementLimit} + Instant{timing::kSafetyCheckPeriod});
    EXPECT_GT(drive.rampStepCount(), 0);
}

TEST(ActuatorRuntimeTest, RejectedDriveProfileFaultsBeforeAnyTaskStarts)
{
    FakeClock clock;
    FakePlatform platform;
    FakeDrive drive;
    drive.failConfigure = true;
    FakeConfiguration configuration;
    FirmwareRuntime runtime{{clock, platform, drive, configuration, nullptr}};
    EXPECT_EQ(runtime.start(), StartResult::Faulted);
    EXPECT_EQ(runtime.status().fault, FaultCode::DriveConfigurationRejected);
    EXPECT_TRUE(platform.tasks.empty());
    EXPECT_FALSE(drive.enabled);

    // A configuration without a valid drive profile never reaches the adapter.
    FakeDrive second;
    FakeConfiguration noProfile;
    noProfile.value = RuntimeConfig{};
    FirmwareRuntime other{{clock, platform, second, noProfile, nullptr}};
    EXPECT_EQ(other.start(), StartResult::Faulted);
    EXPECT_EQ(other.configError(), ConfigError::DriveProfileInvalid);
    EXPECT_FALSE(second.configured);
}
