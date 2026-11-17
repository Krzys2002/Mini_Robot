#include "app/QtControlLoopScheduler.hpp"
#include "simulation/SimulationTransport.hpp"
#include "tracking/RobotTracker.hpp"

#include <QCoreApplication>
#include <QEventLoop>
#include <QSignalSpy>
#include <QTimer>
#include <gtest/gtest.h>

namespace {
using namespace micro_robot;
using namespace std::chrono_literals;
class PromptPolicy final : public IController {
public:
    int calls{};
    std::optional<VelocityCommand> calculate(const ControlInput&) override
    { ++calls; return VelocityCommand{.1, 0}; }
};
TEST(ControlLoopSchedulerTest, TimerRunsAlongsideQueuedGuiEventsAndStopsCleanly)
{
    const RobotId id{1};
    WorldModel world;
    SimulationTransport transport;
    RobotManager robots{transport, world};
    ASSERT_TRUE(robots.addRobot(id, Timestamp{}));
    ASSERT_TRUE(robots.connect(id));
    transport.advanceTo(MonotonicClock::now());
    RobotTracker tracker{world, {5s, 10s}};
    const auto now = MonotonicClock::now();
    ASSERT_EQ(tracker.observe(id, {}, now, now), TrackerUpdate::Applied);
    PromptPolicy policy;
    ControlLoop loop{id, world, robots, policy, {5ms, 100ms, 5s, .2, 1}};
    ASSERT_TRUE(loop.setTarget({Meters{1}, Meters{}, Radians{}}));
    QEventLoop events;
    int heartbeats = 0;
    QTimer heartbeat;
    heartbeat.setInterval(1ms);
    QObject::connect(&heartbeat, &QTimer::timeout, &events, [&] { ++heartbeats; });
    {
        QtControlLoopScheduler scheduler{loop};
        QSignalSpy cycles{&scheduler, &QtControlLoopScheduler::cycleCompleted};
        QObject::connect(&scheduler, &QtControlLoopScheduler::cycleCompleted, &events, [&] {
            if (policy.calls >= 3) events.quit();
        });
        QTimer deadline;
        deadline.setSingleShot(true);
        QObject::connect(&deadline, &QTimer::timeout, &events, &QEventLoop::quit);
        deadline.start(2s);
        heartbeat.start();
        ASSERT_TRUE(scheduler.start());
        events.exec();
        EXPECT_GE(cycles.count(), 3);
        EXPECT_GE(heartbeats, 1);
        EXPECT_EQ(loop.lastCycle()->result, ControlCycleResult::Submitted);
    }
    EXPECT_EQ(loop.state(), ControlLoopState::Stopped);
    const auto calls = policy.calls;
    QCoreApplication::processEvents();
    EXPECT_EQ(policy.calls, calls);
}
} // namespace
