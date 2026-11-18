#include "control/GoalPointController.hpp"
#include "control/ControlLoop.hpp"
#include "control/DifferentialDrive.hpp"
#include "common/Angle.hpp"
#include "simulation/SimulationTransport.hpp"
#include "tracking/RobotTracker.hpp"

#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include <deque>
#include <limits>
#include <numbers>
#include <stdexcept>

namespace {
using namespace micro_robot;
using namespace std::chrono_literals;
constexpr double pi = std::numbers::pi;
const GoalPointControllerConfig config{1.0, 3.0, 2.0, .25, 1.5, Meters{.005}, Radians{.01}, Radians{pi / 3.0}, true, Meters{.01}};
Pose2D pose(double x, double y, double heading) { return {Meters{x}, Meters{y}, Radians{heading}}; }

TEST(GoalPointControllerTest, WorldAndRobotErrorsUseMetersAndShortestSignedRadians)
{
    GoalPointController controller{config};
    const auto result = controller.evaluate(pose(0, 0, pi / 2), pose(1, 0, pi));
    ASSERT_TRUE(result);
    EXPECT_DOUBLE_EQ(result->error.worldXmeters, 1);
    EXPECT_DOUBLE_EQ(result->error.worldYmeters, 0);
    EXPECT_NEAR(result->error.forwardMeters, 0, 1e-12);
    EXPECT_NEAR(result->error.leftMeters, -1, 1e-12);
    EXPECT_NEAR(result->error.distanceMeters, 1, 1e-12);
    EXPECT_NEAR(result->error.bearingRadians, -pi / 2, 1e-12);
    EXPECT_NEAR(result->error.finalHeadingRadians, pi / 2, 1e-12);
}
TEST(GoalPointControllerTest, AlignedApproachIsProportionalAndSaturatesAtConfiguredBounds)
{
    GoalPointController controller{config};
    const auto near = controller.evaluate(pose(0, 0, 0), pose(.1, 0, 0));
    ASSERT_TRUE(near);
    EXPECT_EQ(near->state, GoalControlState::Approaching);
    EXPECT_NEAR(near->command.linearMetersPerSecond, .1, 1e-12);
    EXPECT_DOUBLE_EQ(near->command.angularRadiansPerSecond, 0);
    const auto far = controller.evaluate(pose(0, 0, 0), pose(10, 0, 0));
    ASSERT_TRUE(far);
    EXPECT_DOUBLE_EQ(far->command.linearMetersPerSecond, config.maximumLinearMetersPerSecond);
    const auto side = controller.evaluate(pose(0, 0, 0), pose(0, -1, 0));
    ASSERT_TRUE(side);
    EXPECT_EQ(side->state, GoalControlState::TurningToPoint);
    EXPECT_DOUBLE_EQ(side->command.linearMetersPerSecond, 0);
    EXPECT_DOUBLE_EQ(side->command.angularRadiansPerSecond, -config.maximumAngularRadiansPerSecond);
}
TEST(GoalPointControllerTest, ForwardSpeedIsReducedWhileTurningAndNeverReverses)
{
    GoalPointController controller{config};
    const auto result = controller.evaluate(pose(0, 0, 0), pose(.1, .1, 0));
    ASSERT_TRUE(result);
    EXPECT_EQ(result->state, GoalControlState::Approaching);
    EXPECT_NEAR(result->command.linearMetersPerSecond,
        std::hypot(.1, .1) * (1.0 - (pi / 4) / (pi / 3)), 1e-12);
    EXPECT_GT(result->command.angularRadiansPerSecond, 0);
    EXPECT_LE(result->command.angularRadiansPerSecond, config.maximumAngularRadiansPerSecond);
}
TEST(GoalPointControllerTest, TurnBoundaryBringsForwardSpeedContinuouslyToZero)
{
    GoalPointController controller{config};
    constexpr double epsilon = 1e-6;
    const double angle = config.turnInPlaceAngle.value;
    const auto below = controller.evaluate({}, pose(std::cos(angle - epsilon), std::sin(angle - epsilon), 0));
    const auto at = controller.evaluate({}, pose(std::cos(angle), std::sin(angle), 0));
    ASSERT_TRUE(below); ASSERT_TRUE(at);
    EXPECT_EQ(below->state, GoalControlState::Approaching);
    EXPECT_EQ(at->state, GoalControlState::TurningToPoint);
    EXPECT_GT(below->command.linearMetersPerSecond, 0.0);
    EXPECT_LT(below->command.linearMetersPerSecond, 1e-5);
    EXPECT_DOUBLE_EQ(at->command.linearMetersPerSecond, 0.0);
}
TEST(GoalPointControllerTest, BehindRobotTurnsInPlaceWithDeterministicCounterclockwiseTie)
{
    GoalPointController controller{config};
    const auto behind = controller.evaluate(pose(0, 0, 0), pose(-1, 0, 0));
    ASSERT_TRUE(behind);
    EXPECT_EQ(behind->state, GoalControlState::TurningToPoint);
    EXPECT_DOUBLE_EQ(behind->command.linearMetersPerSecond, 0);
    EXPECT_GT(behind->command.angularRadiansPerSecond, 0);
    const auto rightBehind = controller.evaluate(pose(0, 0, 0), pose(-1, -.1, 0));
    ASSERT_TRUE(rightBehind);
    EXPECT_DOUBLE_EQ(rightBehind->command.linearMetersPerSecond, 0);
    EXPECT_LT(rightBehind->command.angularRadiansPerSecond, 0);
}
TEST(GoalPointControllerTest, ArrivalBoundariesAreInclusiveAndRepeatedEvaluationStaysStopped)
{
    GoalPointController controller{config};
    const auto goal = pose(config.positionTolerance.value, 0, config.headingTolerance.value);
    for (int i = 0; i < 5; ++i) {
        const auto result = controller.evaluate(pose(0, 0, 0), goal);
        ASSERT_TRUE(result);
        EXPECT_EQ(result->state, GoalControlState::Arrived);
        EXPECT_DOUBLE_EQ(result->command.linearMetersPerSecond, 0);
        EXPECT_DOUBLE_EQ(result->command.angularRadiansPerSecond, 0);
    }
    const auto outside = controller.evaluate(pose(0, 0, 0), pose(
        std::nextafter(config.positionTolerance.value, 1.0), 0, 0));
    ASSERT_TRUE(outside);
    EXPECT_EQ(outside->state, GoalControlState::Approaching);
    const auto headingOutside = controller.evaluate(pose(0, 0, 0), pose(0, 0,
        std::nextafter(config.headingTolerance.value, 1.0)));
    ASSERT_TRUE(headingOutside);
    EXPECT_EQ(headingOutside->state, GoalControlState::AligningHeading);
}
TEST(GoalPointControllerTest, PoseGoalAlignsInPlaceWhilePointGoalIgnoresFinalHeading)
{
    GoalPointController controller{config};
    const auto result = controller.evaluate(pose(1, 2, 0), pose(1, 2, -.2));
    ASSERT_TRUE(result);
    EXPECT_EQ(result->state, GoalControlState::AligningHeading);
    EXPECT_DOUBLE_EQ(result->command.linearMetersPerSecond, 0);
    EXPECT_NEAR(result->command.angularRadiansPerSecond, -.4, 1e-12);
    auto pointConfig = config;
    pointConfig.alignFinalHeading = false;
    GoalPointController point{pointConfig};
    EXPECT_EQ(point.evaluate(pose(1, 2, 0), pose(1, 2, pi))->state, GoalControlState::Arrived);
}
TEST(GoalPointControllerTest, HeadingAlignmentUsesPositionExitHysteresis)
{
    GoalPointController controller{config};
    ControlInput input{RobotState{RobotId{1}, Timestamp{}, pose(0, 0, 0), Timestamp{}, {}},
        {}, pose(.004, 0, .2), Timestamp{}};
    input.tracking.status = TrackingStatus::Fresh;
    ASSERT_TRUE(controller.calculate(input));
    ASSERT_EQ(controller.controlState(), GoalControlState::AligningHeading);

    input.target = pose(.007, 0, .2);
    const auto held = controller.calculate(input);
    ASSERT_TRUE(held);
    EXPECT_EQ(controller.controlState(), GoalControlState::AligningHeading);
    EXPECT_DOUBLE_EQ(held->linearMetersPerSecond, 0.0);

    input.target = pose(.011, 0, .2);
    ASSERT_TRUE(controller.calculate(input));
    EXPECT_EQ(controller.controlState(), GoalControlState::Approaching);

    controller.reset();
    input.target = pose(.007, 0, .2);
    ASSERT_TRUE(controller.calculate(input));
    EXPECT_EQ(controller.controlState(), GoalControlState::Approaching);
}
TEST(GoalPointControllerTest, HeadingWrapAndEquivalentAngleRepresentationsHaveNoSpuriousJump)
{
    GoalPointController controller{config};
    const auto left = controller.evaluate(pose(0, 0, pi - .001), pose(0, 0, -pi + .001));
    ASSERT_TRUE(left);
    // Tighten tolerance to inspect commands, rather than stopping inside it.
    auto tightConfig = config;
    tightConfig.headingTolerance = Radians{.00001};
    GoalPointController tight{tightConfig};
    const auto before = tight.evaluate(pose(0, 0, pi - .001), pose(0, 0, pi));
    const auto after = tight.evaluate(pose(0, 0, -pi + .001), pose(0, 0, pi));
    ASSERT_TRUE(before); ASSERT_TRUE(after);
    EXPECT_NEAR(before->command.angularRadiansPerSecond, .002, 1e-12);
    EXPECT_NEAR(after->command.angularRadiansPerSecond, -.002, 1e-12);
    EXPECT_LT(std::abs(before->command.angularRadiansPerSecond - after->command.angularRadiansPerSecond), .01);
    const auto a = controller.evaluate(pose(0, 0, pi), pose(-1, 0, 0));
    const auto b = controller.evaluate(pose(0, 0, -pi), pose(-1, 0, 0));
    EXPECT_DOUBLE_EQ(a->command.linearMetersPerSecond, b->command.linearMetersPerSecond);
    EXPECT_DOUBLE_EQ(a->command.angularRadiansPerSecond, b->command.angularRadiansPerSecond);
}
TEST(GoalPointControllerTest, InvalidMeasurementsMissingPoseAndNonrepresentableErrorsAreUnavailable)
{
    GoalPointController controller{config};
    for (const double invalid : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
        EXPECT_FALSE(controller.evaluate(pose(invalid, 0, 0), {}));
        EXPECT_FALSE(controller.evaluate({}, pose(0, invalid, 0)));
        EXPECT_FALSE(controller.evaluate(pose(0, 0, invalid), {}));
    }
    const double largest = std::numeric_limits<double>::max();
    EXPECT_FALSE(controller.evaluate(pose(-largest, 0, 0), pose(largest, 0, 0)));
    ControlInput input{RobotState{RobotId{1}, Timestamp{}, {}, {}, {}}, {}, {}, Timestamp{}};
    EXPECT_FALSE(controller.calculate(input));
    input.robot.pose = pose(0, 0, 0);
    input.target = pose(1, 0, 0);
    input.tracking.status = TrackingStatus::Fresh;
    ASSERT_TRUE(controller.calculate(input));
    EXPECT_EQ(controller.calculate(input)->linearMetersPerSecond, config.maximumLinearMetersPerSecond);
}
TEST(GoalPointControllerTest, ControllerBoundaryRejectsUnknownStaleAndLostTracking)
{
    GoalPointController controller{config};
    ControlInput input{RobotState{RobotId{1}, Timestamp{}, pose(0, 0, 0), Timestamp{}, {}},
        {}, pose(1, 0, 0), Timestamp{}};
    for (const auto status : {TrackingStatus::Unknown, TrackingStatus::Stale, TrackingStatus::Lost}) {
        input.tracking.status = status;
        EXPECT_FALSE(controller.calculate(input));
    }
}
TEST(GoalPointControllerTest, InvalidConfigurationIsRejected)
{
    using Field = double GoalPointControllerConfig::*;
    const std::array<Field, 5> fields{&GoalPointControllerConfig::positionGainPerSecond,
        &GoalPointControllerConfig::bearingGainPerSecond, &GoalPointControllerConfig::finalHeadingGainPerSecond,
        &GoalPointControllerConfig::maximumLinearMetersPerSecond, &GoalPointControllerConfig::maximumAngularRadiansPerSecond};
    for (const auto field : fields) {
        for (const double invalid : {0.0, -1.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
            auto c = config; c.*field = invalid;
            EXPECT_THROW((GoalPointController{c}), std::invalid_argument);
        }
    }
    auto c = config; c.bearingGainPerSecond = c.positionGainPerSecond;
    EXPECT_THROW((GoalPointController{c}), std::invalid_argument);
    for (const double invalid : {0.0, -1.0, std::numeric_limits<double>::infinity()}) {
        c = config; c.positionTolerance = Meters{invalid};
        EXPECT_THROW((GoalPointController{c}), std::invalid_argument);
        c = config; c.headingTolerance = Radians{invalid};
        EXPECT_THROW((GoalPointController{c}), std::invalid_argument);
        c = config; c.turnInPlaceAngle = Radians{invalid};
        EXPECT_THROW((GoalPointController{c}), std::invalid_argument);
    }
    c = config; c.turnInPlaceAngle = Radians{pi};
    EXPECT_THROW((GoalPointController{c}), std::invalid_argument);
    c = config; c.positionExitTolerance = c.positionTolerance;
    EXPECT_THROW((GoalPointController{c}), std::invalid_argument);
    c = config; c.positionExitTolerance = Meters{std::numeric_limits<double>::infinity()};
    EXPECT_THROW((GoalPointController{c}), std::invalid_argument);
    c = config; c.headingTolerance = Radians{2 * pi};
    EXPECT_THROW((GoalPointController{c}), std::invalid_argument);
}
TEST(GoalPointControllerTest, ConfiguredGainsAndLimitsRemainFiniteEvenAtExtremeFiniteValues)
{
    auto c = config;
    c.positionGainPerSecond = std::numeric_limits<double>::max() / 2;
    c.bearingGainPerSecond = std::numeric_limits<double>::max();
    c.finalHeadingGainPerSecond = std::numeric_limits<double>::max();
    c.maximumAngularRadiansPerSecond = std::numeric_limits<double>::denorm_min();
    GoalPointController controller{c};
    const auto straight = controller.evaluate({}, pose(1, 0, 0));
    ASSERT_TRUE(straight);
    EXPECT_DOUBLE_EQ(straight->command.angularRadiansPerSecond, 0);
    EXPECT_LE(straight->command.linearMetersPerSecond, c.maximumLinearMetersPerSecond);
    const auto turn = controller.evaluate({}, pose(-1, 0, 0));
    ASSERT_TRUE(turn);
    EXPECT_EQ(turn->command.angularRadiansPerSecond, c.maximumAngularRadiansPerSecond);
}

TEST(GoalPointControllerTest, NoisyTrackingAndDelayedCommandsHoldHeadingAlignment)
{
    const RobotId id{1};
    Timestamp now{1s};
    constexpr auto period = 20ms;
    constexpr double dt = .02;
    constexpr std::size_t delayCycles = 3;
    WorldModel world;
    world.setWorkspaceFrame({Meters{2}, Meters{2}}, "noisy-delay-test");
    SimulationTransport transport;
    RobotManager robots{transport, world};
    RobotTracker tracker{world, {100ms, 500ms}};
    ASSERT_TRUE(robots.addRobot(id, Timestamp{}));
    ASSERT_TRUE(robots.connect(id));
    transport.advanceTo(now);

    Pose2D actual = pose(-.04, 0, 0);
    const Pose2D target = pose(0, 0, .2);
    auto measured = [&](int cycle) {
        const double phase = static_cast<double>(cycle);
        return pose(actual.x.value + .0005 * std::sin(phase * .71),
            actual.y.value + .0005 * std::cos(phase * .53),
            actual.heading.value + .002 * std::sin(phase * .37));
    };
    ASSERT_EQ(tracker.observe(id, measured(0), now, now), TrackerUpdate::Applied);
    GoalPointController controller{config};
    ControlLoop loop{id, world, robots, controller,
        {period, 100ms, 80ms, config.maximumLinearMetersPerSecond,
         config.maximumAngularRadiansPerSecond}, [&] { return now; }};
    ASSERT_TRUE(loop.setTarget(target));
    ASSERT_TRUE(loop.start());
    std::deque<VelocityCommand> delayedCommands;
    bool observedAlignment = false;
    bool reached = false;

    for (int cycle = 0; cycle < 1200; ++cycle) {
        ASSERT_TRUE(loop.poll());
        const auto sample = *loop.lastCycle();
        const auto phase = controller.controlState();
        if (phase == GoalControlState::AligningHeading) {
            observedAlignment = true;
            EXPECT_DOUBLE_EQ(sample.requested->linearMetersPerSecond, 0.0);
            if (sample.error->distanceMeters <= config.positionExitTolerance.value
                && std::abs(sample.error->headingRadians) > config.headingTolerance.value)
                EXPECT_EQ(phase, GoalControlState::AligningHeading);
        }
        if (phase == GoalControlState::Arrived) {
            reached = true;
            break;
        }
        ASSERT_TRUE(sample.bounded);
        delayedCommands.push_back(*sample.bounded);
        if (delayedCommands.size() > delayCycles) {
            const auto applied = delayedCommands.front();
            delayedCommands.pop_front();
            actual.x.value += dt * applied.linearMetersPerSecond * std::cos(actual.heading.value);
            actual.y.value += dt * applied.linearMetersPerSecond * std::sin(actual.heading.value);
            actual.heading = normalizeAngle(Radians{actual.heading.value
                + dt * applied.angularRadiansPerSecond});
        }
        now += period;
        ASSERT_EQ(tracker.observe(id, measured(cycle + 1), now, now), TrackerUpdate::Applied);
        transport.advanceTo(now);
    }
    EXPECT_TRUE(observedAlignment);
    EXPECT_TRUE(reached);
}

struct Scenario { Pose2D start, target; bool finalHeading; };
class GoalPointConvergenceTest : public testing::TestWithParam<Scenario> {};
TEST_P(GoalPointConvergenceTest, ReachesAndSettlesThroughSchedulerManagerAndSimulatedTransport)
{
    const auto scenario = GetParam();
    auto c = config; c.alignFinalHeading = scenario.finalHeading;
    GoalPointController controller{c};
    const RobotId id{1};
    Timestamp now{1s};
    constexpr auto period = 20ms;
    constexpr double dt = .02;
    constexpr int maximumCycles = 4000;
    WorldModel world;
    world.setWorkspaceFrame({Meters{2}, Meters{2}}, "goal-controller-test");
    SimulationTransport transport;
    RobotManager robots{transport, world};
    RobotTracker tracker{world, {100ms, 500ms}};
    ASSERT_TRUE(robots.addRobot(id, Timestamp{}));
    ASSERT_TRUE(robots.connect(id));
    transport.advanceTo(now);
    Pose2D measured = scenario.start;
    ASSERT_EQ(tracker.observe(id, measured, now, now), TrackerUpdate::Applied);
    ControlLoop loop{id, world, robots, controller,
        {period, 100ms, 80ms, c.maximumLinearMetersPerSecond, c.maximumAngularRadiansPerSecond}, [&] { return now; }};
    ASSERT_TRUE(loop.setTarget(scenario.target));
    ASSERT_TRUE(loop.start());
    int settled = 0;
    for (int step = 0; step < maximumCycles && settled < 5; ++step) {
        ASSERT_TRUE(loop.poll());
        ASSERT_EQ(loop.lastCycle()->result, ControlCycleResult::Submitted);
        transport.advanceTo(now);
        const auto sent = transport.lastSentCommand(id);
        ASSERT_TRUE(sent);
        ASSERT_TRUE(std::holds_alternative<VelocityCommand>(sent->payload));
        const auto command = std::get<VelocityCommand>(sent->payload);
        EXPECT_GE(command.linearMetersPerSecond, 0);
        EXPECT_LE(command.linearMetersPerSecond, c.maximumLinearMetersPerSecond);
        EXPECT_LE(std::abs(command.angularRadiansPerSecond), c.maximumAngularRadiansPerSecond);
        const auto result = controller.evaluate(measured, scenario.target);
        ASSERT_TRUE(result);
        if (result->state == GoalControlState::Arrived) {
            ++settled;
            EXPECT_DOUBLE_EQ(command.linearMetersPerSecond, 0);
            EXPECT_DOUBLE_EQ(command.angularRadiansPerSecond, 0);
        } else settled = 0;
        // Ideal no-slip point robot, Euler integration of the existing pure
        // kinematic model. No physical response, vision noise or radio delay.
        const auto derivative = unicycleDerivative(measured, command);
        ASSERT_TRUE(derivative);
        measured.x.value += dt * derivative->xMetersPerSecond;
        measured.y.value += dt * derivative->yMetersPerSecond;
        measured.heading = normalizeAngle(Radians{measured.heading.value + dt * derivative->headingRadiansPerSecond});
        now += period;
        ASSERT_EQ(tracker.observe(id, measured, now, now), TrackerUpdate::Applied);
    }
    EXPECT_EQ(settled, 5) << "final x=" << measured.x.value << " y=" << measured.y.value << " heading=" << measured.heading.value;
    const auto final = controller.evaluate(measured, scenario.target);
    ASSERT_TRUE(final);
    EXPECT_LE(final->error.distanceMeters, c.positionTolerance.value);
    if (scenario.finalHeading) EXPECT_LE(std::abs(final->error.finalHeadingRadians), c.headingTolerance.value);
    loop.stop();
}
INSTANTIATE_TEST_SUITE_P(RepresentativeStarts, GoalPointConvergenceTest, testing::Values(
    Scenario{pose(0, 0, 0), pose(1, 0, 0), true},
    Scenario{pose(0, 0, pi), pose(1, 0, 0), true},
    Scenario{pose(0, 0, 0), pose(-1, 0, pi), true},
    Scenario{pose(0, 0, 0), pose(0, 1, -pi / 2), true},
    Scenario{pose(-1, -.5, -2), pose(.7, .4, 2.8), true},
    Scenario{pose(.5, .5, pi - .01), pose(-.5, -.2, -pi + .01), true},
    Scenario{pose(.001, -.001, 0), pose(0, 0, -2), true},
    Scenario{pose(0, 0, -pi / 2), pose(.5, -.5, pi), false},
    Scenario{pose(1, 1, 2), pose(1, 1, 2), true}
));
} // namespace
