#include "app/RobotManager.hpp"
#include "simulation/SimulationTransport.hpp"

#include <gtest/gtest.h>
#include <functional>
#include <limits>
#include <type_traits>
#include <vector>

namespace {
using namespace micro_robot;
using namespace std::chrono_literals;
constexpr RobotId one{1}, two{2}, unknown{99};
class ManualTransport final : public IRobotTransport {
public:
    std::weak_ptr<IRobotTransportEvents> sink;
    std::map<RobotId, ConnectionState> states;
    std::vector<RobotId> connects, disconnects;
    std::vector<std::pair<RobotId, RobotCommand>> commands;
    void setEventSink(std::weak_ptr<IRobotTransportEvents> value) override { sink = std::move(value); }
    ConnectionState connectionState(RobotId id) const override
    { const auto it = states.find(id); return it == states.end() ? ConnectionState::Disconnected : it->second; }
    void connect(RobotId id) override { connects.push_back(id); }
    void disconnect(RobotId id) override { disconnects.push_back(id); }
    void sendCommand(RobotId id, const RobotCommand& command) override { commands.emplace_back(id, command); }
    void state(RobotId id, ConnectionState value, Timestamp at)
    { states.insert_or_assign(id, value); if (auto locked = sink.lock()) locked->onConnectionStateChanged(id, value, at); }
    void telemetry(RobotTelemetry value)
    { if (auto locked = sink.lock()) locked->onTelemetry(value); }
    void error(TransportError value)
    { if (auto locked = sink.lock()) locked->onTransportError(value); }
};
class Observer final : public IRobotTransportEvents {
public:
    std::vector<RobotId> states;
    std::vector<RobotTelemetry> samples;
    std::vector<TransportError> errors;
    std::function<void(RobotId, ConnectionState)> onState;
    std::function<void(const RobotTelemetry&)> onSample;
    void onConnectionStateChanged(RobotId id, ConnectionState state, Timestamp) override
    { states.push_back(id); if (onState) onState(id, state); }
    void onTelemetry(const RobotTelemetry& sample) override
    { samples.push_back(sample); if (onSample) onSample(sample); }
    void onTransportError(const TransportError& error) override { errors.push_back(error); }
};
RobotTelemetry sample(RobotId id, Timestamp at = Timestamp{2s})
{ return {id, at, 3.7, 0.1, -0.2, {currentTransportModelVersion, 17}}; }
struct Fixture {
    ManualTransport transport;
    WorldModel world;
    RobotManager robots{transport, world};
    std::shared_ptr<Observer> observer{std::make_shared<Observer>()};
    Fixture() { robots.setEventSink(observer); }
    void connected(RobotId id, Timestamp at = Timestamp{1s})
    {
        ASSERT_TRUE(robots.addRobot(id, Timestamp{}));
        ASSERT_TRUE(robots.connect(id));
        transport.state(id, ConnectionState::Connecting, at);
        transport.state(id, ConnectionState::Connected, at);
    }
};
TEST(RobotManagerTest, ExplicitRegistrationPreservesWorldPoseAndRejectsUnknownRequests)
{
    Fixture f;
    static_assert(!std::is_copy_constructible_v<RobotManager>);
    ASSERT_TRUE(f.world.registerRobot(one, Timestamp{}));
    ASSERT_EQ(f.world.updatePose(one, {Meters{1}, Meters{2}, Radians{3}}, Timestamp{}), PoseUpdateResult::Applied);
    ASSERT_TRUE(f.robots.addRobot(one, Timestamp{5s}));
    EXPECT_FALSE(f.robots.addRobot(one, Timestamp{6s}));
    EXPECT_DOUBLE_EQ(f.world.robot(one)->pose->x.value, 1);
    EXPECT_FALSE(f.robots.connect(unknown));
    EXPECT_FALSE(f.robots.disconnect(unknown));
    EXPECT_FALSE(f.robots.removeRobot(unknown));
    EXPECT_EQ(f.robots.stop(unknown, Timestamp{}, 1s), CommandSubmission::UnknownRobot);
    EXPECT_EQ(f.robots.connectionState(unknown), ConnectionState::Disconnected);
    EXPECT_TRUE(f.transport.connects.empty());
    EXPECT_TRUE(f.transport.commands.empty());
    f.transport.states[two] = ConnectionState::Connected;
    EXPECT_FALSE(f.robots.addRobot(two, Timestamp{})); // Never silently adopt a live endpoint.
}
TEST(RobotManagerTest, DistinctSessionsRouteCommandsStopTelemetryAndErrorsByRobotId)
{
    Fixture f; f.connected(two); f.connected(one);
    const RobotCommand command{VelocityCommand{0.2, -0.5}, Timestamp{1s}, 2s, {1, 42}};
    EXPECT_EQ(f.robots.sendCommand(two, command), CommandSubmission::Submitted);
    EXPECT_EQ(f.robots.stop(one, Timestamp{1s}, 2s), CommandSubmission::Submitted);
    ASSERT_EQ(f.transport.commands.size(), 2U);
    EXPECT_EQ(f.transport.commands[0].first, two);
    EXPECT_EQ(f.transport.commands[0].second.metadata.sequence, 42U);
    EXPECT_DOUBLE_EQ(std::get<VelocityCommand>(f.transport.commands[0].second.payload).linearMetersPerSecond, 0.2);
    EXPECT_EQ(f.transport.commands[1].first, one);
    EXPECT_TRUE(std::holds_alternative<StopCommand>(f.transport.commands[1].second.payload));
    f.transport.telemetry(sample(two));
    EXPECT_FALSE(f.robots.session(one)->telemetry);
    ASSERT_TRUE(f.robots.session(two)->telemetry);
    EXPECT_EQ(f.robots.session(two)->telemetry->metadata.sequence, 17U);
    f.transport.error({one, TransportOperation::SendCommand, TransportErrorCode::BackendFailure, "write failed", Timestamp{3s}});
    EXPECT_TRUE(f.robots.session(one)->lastError);
    EXPECT_FALSE(f.robots.session(two)->lastError);
    EXPECT_FALSE(f.world.robot(two)->pose); // Communication cannot invent vision state.
    auto copy = f.robots.sessions();
    EXPECT_EQ(copy.begin()->first, one);
    copy.clear();
    EXPECT_EQ(f.robots.sessions().size(), 2U);
}
TEST(RobotManagerTest, RejectsUnregisteredDisconnectedOldAndIncompatibleTelemetry)
{
    Fixture f; f.connected(one);
    f.transport.telemetry(sample(unknown));
    f.transport.telemetry(sample(one, Timestamp{})); // Before connection.
    EXPECT_TRUE(f.observer->samples.empty());
    f.transport.telemetry(sample(one));
    f.transport.telemetry(sample(one)); // Duplicate receipt timestamp.
    f.transport.telemetry(sample(one, Timestamp{1500ms}));
    EXPECT_EQ(f.observer->samples.size(), 1U);
    auto bad = sample(one, Timestamp{3s}); bad.metadata.modelVersion = 2;
    f.transport.telemetry(bad);
    EXPECT_EQ(f.observer->errors.back().code, TransportErrorCode::UnsupportedVersion);
    bad.metadata.modelVersion = 1; bad.linearMetersPerSecond = std::numeric_limits<double>::quiet_NaN();
    f.transport.telemetry(bad);
    EXPECT_EQ(f.observer->errors.back().code, TransportErrorCode::InvalidTelemetry);
    EXPECT_EQ(f.observer->samples.size(), 1U);
    EXPECT_EQ(f.robots.session(one)->telemetry->receivedAt, Timestamp{2s});
    ASSERT_TRUE(f.robots.disconnect(one));
    f.transport.telemetry(sample(one, Timestamp{4s}));
    EXPECT_FALSE(f.robots.session(one)->telemetry);
    EXPECT_EQ(f.robots.stop(one, Timestamp{4s}, 1s), CommandSubmission::NotConnected);
    EXPECT_FALSE(f.robots.connect(one)); // Await terminal callback before reconnect.
}
TEST(RobotManagerTest, UnknownMeasurementFieldsAndZeroVoltageStayExplicit)
{
    Fixture f; f.connected(one);
    f.transport.telemetry({one, Timestamp{2s}, std::nullopt, std::nullopt, std::nullopt});
    ASSERT_TRUE(f.robots.session(one)->telemetry);
    EXPECT_FALSE(f.robots.session(one)->telemetry->linearMetersPerSecond);
    f.transport.telemetry({one, Timestamp{3s}, 0.0, 0.0, 0.0});
    EXPECT_EQ(f.robots.session(one)->telemetry->batteryVoltageVolts, 0.0);
}
TEST(RobotManagerTest, RemovalRetainsActiveOwnershipUntilTerminalAndKeepsWorldRecords)
{
    Fixture f; f.connected(one); f.connected(two);
    EXPECT_TRUE(f.robots.removeRobot(one));
    ASSERT_TRUE(f.robots.session(one));
    EXPECT_TRUE(f.robots.session(one)->removing);
    EXPECT_FALSE(f.robots.addRobot(one, Timestamp{2s}));
    EXPECT_EQ(f.robots.stop(one, Timestamp{2s}, 1s), CommandSubmission::NotConnected);
    f.transport.state(one, ConnectionState::Disconnecting, Timestamp{2s});
    EXPECT_TRUE(f.robots.session(one));
    f.transport.state(one, ConnectionState::Disconnected, Timestamp{3s});
    EXPECT_FALSE(f.robots.session(one));
    EXPECT_EQ(f.robots.connectionState(two), ConnectionState::Connected);
    f.transport.telemetry(sample(one, Timestamp{4s}));
    EXPECT_TRUE(f.observer->samples.empty());
    EXPECT_TRUE(f.world.robot(one));
    EXPECT_TRUE(f.robots.addRobot(one, Timestamp{4s}));
    EXPECT_FALSE(f.robots.session(one)->telemetry);
}
TEST(RobotManagerTest, UnrequestedConnectionsAreCancelledWithoutRepeatingDisconnectOnProgress)
{
    Fixture f;
    ASSERT_TRUE(f.robots.addRobot(one, Timestamp{}));
    f.transport.state(one, ConnectionState::Connecting, Timestamp{1s});
    EXPECT_EQ(f.transport.disconnects.size(), 1U);
    EXPECT_EQ(f.robots.stop(one, Timestamp{1s}, 1s), CommandSubmission::NotConnected);
    f.transport.state(one, ConnectionState::Connected, Timestamp{2s});
    EXPECT_EQ(f.transport.disconnects.size(), 2U);
    f.transport.state(one, ConnectionState::Disconnecting, Timestamp{3s});
    EXPECT_EQ(f.transport.disconnects.size(), 2U);
    f.transport.state(one, ConnectionState::Disconnected, Timestamp{4s});
    EXPECT_TRUE(f.robots.isStopped());
}
TEST(RobotManagerTest, PendingConnectCanBeRemovedWithoutRequiringAnUnemittedTerminalEvent)
{
    Fixture f;
    ASSERT_TRUE(f.robots.addRobot(one, Timestamp{}));
    ASSERT_TRUE(f.robots.connect(one));
    EXPECT_TRUE(f.robots.removeRobot(one));
    EXPECT_FALSE(f.robots.session(one));
    EXPECT_EQ(f.transport.disconnects, std::vector<RobotId>{one});
}
TEST(RobotManagerTest, ShutdownDrainsAllSessionsAndRejectsNewWork)
{
    Fixture f; f.connected(one); f.connected(two);
    f.robots.shutdown(); f.robots.shutdown();
    EXPECT_EQ(f.transport.disconnects.size(), 2U);
    EXPECT_TRUE(f.transport.commands.empty()); // Teardown never promises a delivered Stop.
    EXPECT_FALSE(f.robots.isStopped());
    EXPECT_EQ(f.robots.stop(one, Timestamp{2s}, 1s), CommandSubmission::ShuttingDown);
    EXPECT_FALSE(f.robots.addRobot(unknown, Timestamp{}));
    EXPECT_FALSE(f.robots.connect(one));
    f.transport.state(one, ConnectionState::Disconnected, Timestamp{2s});
    EXPECT_FALSE(f.robots.isStopped());
    f.transport.state(two, ConnectionState::Fault, Timestamp{2s});
    EXPECT_TRUE(f.robots.isStopped());
}
TEST(RobotManagerTest, ObserverCanRemoveSessionsDuringStateOrTelemetryCallbacks)
{
    Fixture f;
    ASSERT_TRUE(f.robots.addRobot(one, Timestamp{}));
    ASSERT_TRUE(f.robots.connect(one));
    f.observer->onState = [&](RobotId id, ConnectionState state) {
        if (state == ConnectionState::Connected) EXPECT_TRUE(f.robots.removeRobot(id));
    };
    f.transport.state(one, ConnectionState::Connected, Timestamp{1s});
    EXPECT_TRUE(f.robots.session(one)->removing);
    f.transport.state(one, ConnectionState::Disconnected, Timestamp{2s});
    f.observer->onState = {};
    f.connected(two);
    f.observer->onSample = [&](const RobotTelemetry& value) { EXPECT_TRUE(f.robots.removeRobot(value.robotId)); };
    f.transport.telemetry(sample(two));
    EXPECT_TRUE(f.robots.session(two)->removing);
}
TEST(RobotManagerTest, ExpiredOrDetachedObserverDoesNotDetachBackendStateTracking)
{
    Fixture f; f.connected(one);
    f.observer.reset();
    f.transport.telemetry(sample(one));
    ASSERT_TRUE(f.robots.session(one)->telemetry);
    auto replacement = std::make_shared<Observer>();
    f.robots.setEventSink(replacement);
    EXPECT_TRUE(replacement->samples.empty()); // No implicit replay.
    f.transport.telemetry(sample(one, Timestamp{3s}));
    ASSERT_EQ(replacement->samples.size(), 1U);
    f.robots.setEventSink({});
    f.transport.state(one, ConnectionState::Fault, Timestamp{4s});
    EXPECT_EQ(f.robots.connectionState(one), ConnectionState::Fault);
    EXPECT_TRUE(replacement->states.empty());
}
TEST(RobotManagerTest, DestructionDetachesEvenARetainedBackendSink)
{
    ManualTransport transport; WorldModel world;
    auto observer = std::make_shared<Observer>();
    std::shared_ptr<IRobotTransportEvents> retained;
    {
        RobotManager manager{transport, world}; manager.setEventSink(observer);
        ASSERT_TRUE(manager.addRobot(one, Timestamp{}));
        ASSERT_TRUE(manager.connect(one));
        retained = transport.sink.lock();
    }
    EXPECT_TRUE(transport.sink.expired());
    EXPECT_EQ(transport.disconnects, std::vector<RobotId>{one});
    retained->onConnectionStateChanged(one, ConnectionState::Connected, Timestamp{1s});
    retained->onTelemetry(sample(one));
    retained->onTransportError({one, TransportOperation::Connect, TransportErrorCode::BackendFailure, "late", Timestamp{1s}});
    EXPECT_TRUE(observer->states.empty());
    EXPECT_TRUE(observer->samples.empty());
    EXPECT_TRUE(observer->errors.empty());
}
TEST(RobotManagerTest, SimulationUsesSameServiceAndKeepsValidationInAdapter)
{
    SimulationTransport transport; WorldModel world; RobotManager robots{transport, world};
    auto observer = std::make_shared<Observer>(); robots.setEventSink(observer);
    ASSERT_TRUE(robots.addRobot(one, Timestamp{})); ASSERT_TRUE(robots.addRobot(two, Timestamp{}));
    ASSERT_TRUE(robots.connect(one)); ASSERT_TRUE(robots.connect(two)); transport.advanceTo(Timestamp{1s});
    ASSERT_EQ(robots.connectionState(one), ConnectionState::Connected);
    const RobotCommand command{VelocityCommand{0.2, 0.4}, Timestamp{1s}, 5s};
    EXPECT_EQ(robots.sendCommand(two, command), CommandSubmission::Submitted);
    EXPECT_EQ(robots.stop(one, Timestamp{1s}, 5s), CommandSubmission::Submitted);
    transport.advanceTo(Timestamp{2s});
    EXPECT_EQ(robots.session(two)->telemetry->linearMetersPerSecond, 0.2);
    EXPECT_EQ(robots.session(one)->telemetry->linearMetersPerSecond, 0.0);
    auto invalid = command; invalid.metadata.modelVersion = 2;
    EXPECT_EQ(robots.sendCommand(one, invalid), CommandSubmission::Submitted);
    transport.advanceTo(Timestamp{3s});
    EXPECT_EQ(robots.session(one)->lastError->code, TransportErrorCode::UnsupportedVersion);
    auto expired = command; expired.validFor = 1ms;
    EXPECT_EQ(robots.sendCommand(one, expired), CommandSubmission::Submitted);
    transport.advanceTo(Timestamp{4s});
    EXPECT_EQ(robots.session(one)->lastError->code, TransportErrorCode::InvalidCommand);
    EXPECT_FALSE(robots.session(two)->lastError);
    robots.shutdown(); transport.advanceTo(Timestamp{5s});
    EXPECT_TRUE(robots.isStopped());
    EXPECT_FALSE(robots.session(one)->telemetry);
    EXPECT_FALSE(robots.session(two)->telemetry);
}
TEST(RobotManagerTest, DisconnectAndFaultClearOnlyThatSessionAndAllowExplicitRetry)
{
    Fixture f; f.connected(one); f.connected(two);
    f.transport.telemetry(sample(one)); f.transport.telemetry(sample(two));
    f.transport.state(one, ConnectionState::Fault, Timestamp{3s});
    EXPECT_FALSE(f.robots.session(one)->telemetry);
    EXPECT_TRUE(f.robots.session(two)->telemetry);
    ASSERT_TRUE(f.robots.connect(one));
    f.transport.state(one, ConnectionState::Connected, Timestamp{4s});
    f.transport.error({one, TransportOperation::ReceiveTelemetry, TransportErrorCode::BackendFailure,
                       "old session error", Timestamp{3s}});
    EXPECT_FALSE(f.robots.session(one)->lastError);
    f.transport.state(one, ConnectionState::Disconnected, Timestamp{2s}); // Old state cannot overwrite new session.
    EXPECT_EQ(f.robots.connectionState(one), ConnectionState::Connected);
    EXPECT_FALSE(f.robots.session(one)->telemetry);
    f.transport.telemetry(sample(one, Timestamp{3500ms}));
    EXPECT_FALSE(f.robots.session(one)->telemetry);
}
TEST(RobotManagerTest, UnsupportedEmergencyStopIsExplicitAndNeverSubstitutesOrdinaryStop)
{
    Fixture f; f.connected(one);
    EXPECT_FALSE(f.robots.supportsEmergencyStop());
    EXPECT_FALSE(f.robots.requestEmergencyStop(one));
    EXPECT_FALSE(f.robots.session(one)->emergencyStopRequested);
    EXPECT_TRUE(f.transport.commands.empty());
}
TEST(RobotManagerTest, EmergencyLatchPreemptsPendingMotionPersistsAcrossReconnectAndIsolatesRobots)
{
    SimulationTransport transport; WorldModel world; RobotManager robots{transport, world};
    ASSERT_TRUE(robots.addRobot(one, Timestamp{})); ASSERT_TRUE(robots.addRobot(two, Timestamp{}));
    ASSERT_TRUE(robots.connect(one)); ASSERT_TRUE(robots.connect(two)); transport.advanceTo(Timestamp{1s});
    RobotCommand motion{VelocityCommand{0.1, 0.2}, Timestamp{1s}, 10s};
    ASSERT_EQ(robots.sendCommand(one, motion), CommandSubmission::Submitted);
    ASSERT_EQ(robots.sendCommand(two, motion), CommandSubmission::Submitted);
    EXPECT_FALSE(robots.requestEmergencyStop(unknown));
    ASSERT_TRUE(robots.requestEmergencyStop(one)); transport.advanceTo(Timestamp{2s});
    EXPECT_FALSE(transport.lastSentCommand(one));
    EXPECT_EQ(robots.session(one)->telemetry->operatingState, RobotOperatingState::EmergencyStop);
    EXPECT_EQ(robots.session(two)->telemetry->operatingState, RobotOperatingState::Moving);
    EXPECT_NE(robots.sendCommand(one, motion), CommandSubmission::Submitted);
    EXPECT_EQ(robots.stop(one, Timestamp{2s}, 10s), CommandSubmission::Submitted);
    // Even a caller bypassing RobotManager cannot bypass the simulation latch.
    transport.sendCommand(one, motion); transport.advanceTo(Timestamp{3s});
    EXPECT_TRUE(std::holds_alternative<StopCommand>(transport.lastSentCommand(one)->payload));
    transport.sendCommand(one, motion); transport.advanceTo(Timestamp{4s});
    EXPECT_TRUE(std::holds_alternative<StopCommand>(transport.lastSentCommand(one)->payload));
    ASSERT_TRUE(robots.disconnect(one)); transport.advanceTo(Timestamp{5s});
    ASSERT_TRUE(robots.connect(one)); transport.advanceTo(Timestamp{6s});
    EXPECT_TRUE(robots.session(one)->emergencyStopRequested);
    EXPECT_EQ(robots.session(one)->telemetry->operatingState, RobotOperatingState::EmergencyStop);
    EXPECT_NE(robots.sendCommand(one, motion), CommandSubmission::Submitted);
    robots.shutdown(); transport.advanceTo(Timestamp{7s});
    EXPECT_FALSE(robots.requestEmergencyStop(one));
}
} // namespace
