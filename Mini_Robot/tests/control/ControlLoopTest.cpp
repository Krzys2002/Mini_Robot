#include "control/ControlLoop.hpp"
#include "simulation/SimulationTransport.hpp"
#include "tracking/RobotTracker.hpp"

#include <gtest/gtest.h>
#include <limits>
#include <numbers>
#include <stdexcept>

namespace {
using namespace micro_robot;
using namespace std::chrono_literals;
constexpr RobotId id{1}, other{2};
const ControlLoopConfig config{20ms, 100ms, 80ms, .2, 1.0};
const Pose2D target{Meters{1}, Meters{2}, Radians{-3.13}};
struct Policy final : IController {
    std::optional<VelocityCommand> output{VelocityCommand{.1, .5}};
    std::function<void()> during;
    std::vector<ControlInput> inputs;
    std::optional<VelocityCommand> calculate(const ControlInput& input) override
    {
        inputs.push_back(input);
        if (during) during();
        return output;
    }
};
struct ControlLoopTest : testing::Test {
    Timestamp now{1s};
    WorldModel world;
    SimulationTransport transport;
    RobotManager robots{transport, world};
    RobotTracker tracker{world, {100ms, 500ms}};
    Policy policy;
    ControlLoop loop{id, world, robots, policy, config, [this] { return now; }};
    ControlLoopTest()
    {
        world.setWorkspaceFrame({Meters{2}, Meters{2}}, "control-loop-test");
        EXPECT_TRUE(robots.addRobot(id, Timestamp{}));
        EXPECT_TRUE(robots.connect(id));
        transport.advanceTo(now);
        observe();
        EXPECT_TRUE(loop.setTarget(target));
    }
    void observe(Pose2D pose = {Meters{.25}, Meters{.5}, Radians{3.13}})
    { EXPECT_EQ(tracker.observe(id, pose, now, now, .8), TrackerUpdate::Applied); }
    void run()
    {
        ASSERT_TRUE(loop.start());
        ASSERT_TRUE(loop.poll());
        transport.advanceTo(now);
    }
};
TEST_F(ControlLoopTest, LatestMeasuredStateTargetErrorsAndBoundedOutputUseManagerAndSimulation)
{
    policy.output = VelocityCommand{.8, -2};
    run();
    ASSERT_EQ(policy.inputs.size(), 1u);
    EXPECT_EQ(policy.inputs[0].robot.poseUpdatedAt, now);
    EXPECT_EQ(policy.inputs[0].tracking.age, 0ms);
    EXPECT_DOUBLE_EQ(policy.inputs[0].target.x.value, 1);
    const auto cycle = *loop.lastCycle();
    EXPECT_EQ(cycle.scheduledAt, now);
    EXPECT_EQ(cycle.startedAt, now);
    EXPECT_EQ(cycle.finishedAt, now);
    EXPECT_EQ(cycle.observedAt, now);
    EXPECT_NEAR(cycle.error->xMeters, .75, 1e-12);
    EXPECT_NEAR(cycle.error->yMeters, 1.5, 1e-12);
    EXPECT_NEAR(cycle.error->headingRadians, 2 * std::numbers::pi - 6.26, 1e-12);
    EXPECT_DOUBLE_EQ(cycle.bounded->linearMetersPerSecond, .2);
    EXPECT_DOUBLE_EQ(cycle.bounded->angularRadiansPerSecond, -.5);
    EXPECT_EQ(cycle.submission, CommandSubmission::Submitted);
    EXPECT_EQ(loop.operationalState(), ControlOperationalState::Running);
    ASSERT_TRUE(transport.lastSentCommand(id));
    EXPECT_DOUBLE_EQ(std::get<VelocityCommand>(transport.lastSentCommand(id)->payload).linearMetersPerSecond, .2);
    EXPECT_FALSE(transport.lastSentCommand(other));
    EXPECT_FALSE(loop.setTarget({}));
    now += 20ms;
    observe({Meters{.5}, Meters{1}, Radians{0}});
    ASSERT_TRUE(loop.poll());
    EXPECT_DOUBLE_EQ(policy.inputs.back().robot.pose->x.value, .5);
}
TEST_F(ControlLoopTest, PhaseSchedulingSkipsMissedCyclesAndNeverCatchesUpInABurst)
{
    run();
    now += 19ms;
    EXPECT_FALSE(loop.poll());
    now += 46ms;
    observe();
    ASSERT_TRUE(loop.poll());
    EXPECT_EQ(policy.inputs.size(), 2u);
    EXPECT_EQ(loop.lastCycle()->lateness, 45ms);
    EXPECT_EQ(loop.lastCycle()->skippedPeriods, 2u);
    EXPECT_FALSE(loop.poll());
    now += 14ms;
    EXPECT_FALSE(loop.poll());
    now += 1ms;
    EXPECT_TRUE(loop.poll());
    EXPECT_EQ(loop.lastCycle()->scheduledAt, Timestamp{1080ms});
}
TEST_F(ControlLoopTest, AgeLimitInhibitsWithStopAndNeverRestartsOnFreshMeasurementsAlone)
{
    run();
    now += 80ms;
    ASSERT_TRUE(loop.poll());
    EXPECT_EQ(loop.state(), ControlLoopState::Inhibited);
    EXPECT_EQ(loop.operationalState(), ControlOperationalState::LostTracking);
    EXPECT_EQ(loop.lastCycle()->result, ControlCycleResult::TrackingUnavailable);
    EXPECT_EQ(loop.lastCycle()->stopSubmission, CommandSubmission::Submitted);
    EXPECT_EQ(policy.inputs.size(), 1u);
    transport.advanceTo(now);
    EXPECT_TRUE(std::holds_alternative<StopCommand>(transport.lastSentCommand(id)->payload));
    now += 1ms;
    observe();
    EXPECT_FALSE(loop.poll());
    EXPECT_TRUE(loop.start());
    EXPECT_TRUE(loop.poll());
}
TEST_F(ControlLoopTest, ObservationAgeMustBeStrictlyBelowConfiguredMaximum)
{
    now += 79ms;
    EXPECT_TRUE(loop.start());
    EXPECT_TRUE(loop.poll());
    EXPECT_EQ(loop.operationalState(), ControlOperationalState::Running);
}

TEST_F(ControlLoopTest, StaleLostInvalidatedAndUnknownTrackingNeverEnterController)
{
    for (const auto age : {100ms, 500ms}) {
        now = Timestamp{1s} + age;
        EXPECT_FALSE(loop.start());
    }
    now += 1ms;
    observe();
    tracker.invalidate(now);
    EXPECT_FALSE(loop.start());
    tracker.clearMeasurements(now);
    EXPECT_FALSE(loop.start());
    EXPECT_TRUE(policy.inputs.empty());
}
TEST_F(ControlLoopTest, SafetyIsCheckedEvenBeforeNextDeadline)
{
    run();
    now += 1ms;
    tracker.invalidate(now);
    EXPECT_TRUE(loop.poll());
    EXPECT_EQ(policy.inputs.size(), 1u);
    EXPECT_EQ(loop.state(), ControlLoopState::Inhibited);
}
TEST_F(ControlLoopTest, DisconnectReconnectRequiresExplicitRestart)
{
    run();
    now += 1ms;
    transport.simulateDisconnect(id);
    transport.advanceTo(now);
    EXPECT_TRUE(loop.poll());
    EXPECT_EQ(loop.lastCycle()->result, ControlCycleResult::SessionUnavailable);
    EXPECT_EQ(loop.operationalState(), ControlOperationalState::LostConnection);
    EXPECT_EQ(loop.lastCycle()->stopSubmission, CommandSubmission::NotConnected);
    EXPECT_TRUE(robots.connect(id));
    now += 1ms;
    transport.advanceTo(now);
    observe();
    EXPECT_FALSE(loop.poll());
    EXPECT_EQ(policy.inputs.size(), 1u);
    EXPECT_TRUE(loop.start());
    EXPECT_TRUE(loop.poll());
}
TEST_F(ControlLoopTest, EmergencyStopAndFaultPreventPolicyExecution)
{
    ASSERT_TRUE(robots.requestEmergencyStop(id));
    EXPECT_FALSE(loop.start());
    EXPECT_EQ(loop.state(), ControlLoopState::Stopped);
    EXPECT_EQ(loop.operationalState(), ControlOperationalState::EStop);
    now += 1ms; observe();
    EXPECT_FALSE(loop.start()); // Fresh tracking cannot clear the E-stop latch.
    EXPECT_EQ(loop.operationalState(), ControlOperationalState::EStop);
    EXPECT_TRUE(policy.inputs.empty());
}
TEST_F(ControlLoopTest, FaultSessionIsReportedSeparatelyFromLostConnectionAndEStop)
{
    transport.simulateFault(id);
    now += 1ms;
    transport.advanceTo(now);
    EXPECT_FALSE(loop.start());
    EXPECT_EQ(loop.operationalState(), ControlOperationalState::Fault);
}
TEST_F(ControlLoopTest, ExpiryDuringCalculationDropsOutputBeforeSubmission)
{
    policy.during = [this] { now += 80ms; };
    ASSERT_TRUE(loop.start());
    EXPECT_TRUE(loop.poll());
    EXPECT_EQ(loop.lastCycle()->result, ControlCycleResult::TrackingUnavailable);
    EXPECT_FALSE(loop.lastCycle()->submission);
    transport.advanceTo(now);
    EXPECT_TRUE(std::holds_alternative<StopCommand>(transport.lastSentCommand(id)->payload));
}
TEST_F(ControlLoopTest, SessionChangesDuringCalculationDropOutput)
{
    policy.during = [this] { EXPECT_TRUE(robots.requestEmergencyStop(id)); };
    ASSERT_TRUE(loop.start());
    EXPECT_TRUE(loop.poll());
    EXPECT_EQ(loop.lastCycle()->result, ControlCycleResult::SessionUnavailable);
    EXPECT_FALSE(loop.lastCycle()->submission);
}
TEST_F(ControlLoopTest, ReentrantStopDuringCalculationCannotSubmitCalculatedMotion)
{
    policy.during = [this] { loop.stop(); };
    ASSERT_TRUE(loop.start());
    EXPECT_TRUE(loop.poll());
    ASSERT_TRUE(loop.lastCycle());
    EXPECT_EQ(loop.lastCycle()->result, ControlCycleResult::Stopped);
    EXPECT_FALSE(loop.lastCycle()->submission);
    transport.advanceTo(now);
    ASSERT_TRUE(transport.lastSentCommand(id));
    EXPECT_TRUE(std::holds_alternative<StopCommand>(transport.lastSentCommand(id)->payload));
}
TEST_F(ControlLoopTest, InvalidOutputAndUnavailableOrThrowingControllerFailClosed)
{
    policy.output = VelocityCommand{std::numeric_limits<double>::quiet_NaN(), 0};
    ASSERT_TRUE(loop.start());
    ASSERT_TRUE(loop.poll());
    EXPECT_EQ(loop.lastCycle()->result, ControlCycleResult::InvalidOutput);
    EXPECT_EQ(loop.operationalState(), ControlOperationalState::Fault);
    EXPECT_FALSE(loop.lastCycle()->submission);
    EXPECT_TRUE(loop.hasLatchedControllerFault());
    EXPECT_FALSE(loop.start()); // Start is not an implicit fault acknowledgement.
    policy.output = VelocityCommand{.1, 0};
    ASSERT_TRUE(loop.acknowledgeControllerFault());
    ASSERT_TRUE(loop.start());
    ASSERT_TRUE(loop.poll());

    policy.output.reset();
    loop.stop();
    ASSERT_TRUE(loop.start());
    ASSERT_TRUE(loop.poll());
    EXPECT_EQ(loop.lastCycle()->result, ControlCycleResult::ControllerUnavailable);
    EXPECT_FALSE(loop.start());
    EXPECT_TRUE(loop.hasLatchedControllerFault());
    policy.output = VelocityCommand{.1, 0};
    ASSERT_TRUE(loop.acknowledgeControllerFault());
    policy.during = [] { throw std::runtime_error("policy failure"); };
    ASSERT_TRUE(loop.start());
    ASSERT_TRUE(loop.poll());
    EXPECT_EQ(loop.lastCycle()->result, ControlCycleResult::ControllerFailure);
    EXPECT_FALSE(loop.start());
}
TEST_F(ControlLoopTest, ClockReversalStopsWithoutCallingController)
{
    run();
    now -= 1ms;
    ASSERT_TRUE(loop.poll());
    EXPECT_EQ(loop.lastCycle()->result, ControlCycleResult::InvalidClock);
    EXPECT_EQ(policy.inputs.size(), 1u);
    now += 2ms;
    observe();
    EXPECT_TRUE(loop.acknowledgeControllerFault());
}

TEST_F(ControlLoopTest, ReconnectBarrierIsPausedUntilFreshlyTimestampedExplicitRestart)
{
    run();
    now += 1ms;
    transport.simulateDisconnect(id);
    transport.advanceTo(now);
    EXPECT_TRUE(loop.poll());
    ASSERT_TRUE(robots.connect(id));
    transport.advanceTo(now + 1ms); // Connection barrier is newer than the loop clock.
    observe();
    ASSERT_TRUE(loop.start());
    ASSERT_TRUE(loop.poll());
    EXPECT_EQ(loop.lastCycle()->submission, CommandSubmission::StaleSession);
    EXPECT_EQ(loop.operationalState(), ControlOperationalState::Paused);
    now += 2ms;
    observe();
    ASSERT_TRUE(loop.start());
    ASSERT_TRUE(loop.poll());
    EXPECT_EQ(loop.operationalState(), ControlOperationalState::Running);
}
TEST_F(ControlLoopTest, WorldFrameReplacementDoesNotReuseAnOldTarget)
{
    run();
    now += 20ms;
    world.setWorkspaceFrame(WorkspaceBounds{Meters{1}, Meters{1}}, "replacement");
    observe();
    EXPECT_TRUE(loop.poll());
    EXPECT_EQ(loop.lastCycle()->result, ControlCycleResult::FrameChanged);
    EXPECT_FALSE(loop.start());
    EXPECT_TRUE(loop.setTarget(target));
    EXPECT_TRUE(loop.start());
}
TEST_F(ControlLoopTest, ExplicitStopIsIdempotentAndPolicyCanBeReplacedWithoutManagerChanges)
{
    run();
    loop.stop();
    EXPECT_EQ(loop.state(), ControlLoopState::Stopped);
    EXPECT_EQ(loop.operationalState(), ControlOperationalState::Paused);
    EXPECT_EQ(loop.lastCycle()->result, ControlCycleResult::Stopped);
    loop.stop();
    EXPECT_FALSE(loop.poll());
    Policy alternate;
    alternate.output = VelocityCommand{-.1, 0};
    ControlLoop replacement{id, world, robots, alternate, config, [this] { return now; }};
    ASSERT_TRUE(replacement.setTarget(target));
    ASSERT_TRUE(replacement.start());
    ASSERT_TRUE(replacement.poll());
    EXPECT_EQ(alternate.inputs.size(), 1u);
}
TEST_F(ControlLoopTest, ExplicitPauseRequiresStartToResume)
{
    EXPECT_EQ(loop.operationalState(), ControlOperationalState::Ready);
    ASSERT_TRUE(loop.start());
    EXPECT_EQ(loop.operationalState(), ControlOperationalState::Running);
    loop.stop();
    EXPECT_EQ(loop.operationalState(), ControlOperationalState::Paused);
    EXPECT_FALSE(loop.poll());
    EXPECT_TRUE(loop.start());
    EXPECT_EQ(loop.operationalState(), ControlOperationalState::Running);
}

TEST_F(ControlLoopTest, MissingTargetInvalidTargetConfigurationAndUntrackedRobotCannotArm)
{
    ControlLoop empty{id, world, robots, policy, config, [this] { return now; }};
    EXPECT_FALSE(empty.start());
    EXPECT_EQ(empty.state(), ControlLoopState::Stopped);
    EXPECT_FALSE(empty.setTarget({Meters{std::numeric_limits<double>::quiet_NaN()}, Meters{}, Radians{}}));
    EXPECT_FALSE(empty.start());
    auto invalid = config;
    invalid.period = 0ms;
    EXPECT_THROW((ControlLoop{id, world, robots, policy, invalid}), std::invalid_argument);
    invalid = config; invalid.commandValidity = invalid.period;
    EXPECT_THROW((ControlLoop{id, world, robots, policy, invalid}), std::invalid_argument);
    invalid = config; invalid.maximumLinearMetersPerSecond = std::numeric_limits<double>::infinity();
    EXPECT_THROW((ControlLoop{id, world, robots, policy, invalid}), std::invalid_argument);
    invalid = config; invalid.maximumObservationAge = Duration::zero();
    EXPECT_THROW((ControlLoop{id, world, robots, policy, invalid}), std::invalid_argument);
}
TEST_F(ControlLoopTest, DomainSequencesCorrelateVelocityAndStopWithoutClaimingWireOrder)
{
    std::uint64_t sequence = 41;
    ControlLoop correlated{id, world, robots, policy, config, [this] { return now; }, [&] { return sequence++; }};
    ASSERT_TRUE(correlated.setTarget(target)); ASSERT_TRUE(correlated.start()); ASSERT_TRUE(correlated.poll());
    EXPECT_EQ(correlated.lastCycle()->commandSequence, 41u);
    transport.advanceTo(now);
    EXPECT_EQ(transport.lastSentCommand(id)->metadata.sequence, 41u);
    correlated.stop();
    EXPECT_EQ(correlated.lastCycle()->stopSequence, 42u);
    now += 1ms; transport.advanceTo(now);
    EXPECT_EQ(transport.lastSentCommand(id)->metadata.sequence, 42u);
    EXPECT_TRUE(std::holds_alternative<StopCommand>(transport.lastSentCommand(id)->payload));
}
TEST(ControlLoopTestStandalone, TargetRequiresAnInstalledWorkspaceFrame)
{
    WorldModel world;
    SimulationTransport transport;
    RobotManager robots{transport, world};
    Policy policy;
    ControlLoop loop{RobotId{1}, world, robots, policy, config, [] { return Timestamp{1s}; }};
    EXPECT_FALSE(loop.setTarget(target));
    world.setWorkspaceFrame({Meters{2}, Meters{2}}, "test-frame");
    EXPECT_TRUE(loop.setTarget(target));
}
} // namespace
