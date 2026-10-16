#include "simulation/FakeCameraSource.hpp"
#include "simulation/SimulationTransport.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace micro_robot {
namespace {

using namespace std::chrono_literals;

class CameraEvents final : public ICameraEvents {
public:
    void onFrame(FramePtr frame) override { frames.push_back(std::move(frame)); }
    void onCameraStateChanged(CameraState state, Timestamp at) override
    {
        states.emplace_back(state, at);
        if (onState) {
            onState(state);
        }
    }
    void onCameraError(const CameraError& error) override { errors.push_back(error); }

    std::vector<FramePtr> frames;
    std::vector<std::pair<CameraState, Timestamp>> states;
    std::vector<CameraError> errors;
    std::function<void(CameraState)> onState;
};

class TransportEvents final : public IRobotTransportEvents {
public:
    void onConnectionStateChanged(RobotId id, ConnectionState state, Timestamp at) override
    {
        states.push_back({id, state, at});
        if (onState) {
            onState(id, state);
        }
    }
    void onTelemetry(const RobotTelemetry& value) override { telemetry.push_back(value); }
    void onTransportError(const TransportError& value) override { errors.push_back(value); }

    struct StateEvent {
        RobotId id;
        ConnectionState state;
        Timestamp at;
    };
    std::vector<StateEvent> states;
    std::vector<RobotTelemetry> telemetry;
    std::vector<TransportError> errors;
    std::function<void(RobotId, ConnectionState)> onState;
};

class CapturingLogSink final : public ILogSink {
public:
    void write(const LogRecord& record) override { records.push_back(record); }
    std::vector<LogRecord> records;
};

bool containsLog(const std::vector<LogRecord>& records, LogLevel level, const std::string& text)
{
    return std::any_of(records.begin(), records.end(), [&](const LogRecord& record) {
        return record.level == level && record.message.find(text) != std::string::npos;
    });
}

TEST(FakeCameraTest, GeneratesRepeatableLatestFramesAndRetainsStorage)
{
    FakeCameraConfig config;
    config.frameInterval = 10ms;
    config.patternSeed = 5;
    FakeCameraSource first{config};
    FakeCameraSource second{config};
    auto firstEvents = std::make_shared<CameraEvents>();
    auto secondEvents = std::make_shared<CameraEvents>();
    first.setEventSink(firstEvents);
    second.setEventSink(secondEvents);
    ICameraSource& source = first;
    source.start();
    second.start();
    EXPECT_TRUE(firstEvents->states.empty());
    EXPECT_TRUE(firstEvents->frames.empty());
    first.advanceTo(Timestamp{1s});
    second.advanceTo(Timestamp{1s});
    first.advanceTo(Timestamp{1s + 35ms});
    second.advanceTo(Timestamp{1s + 35ms});
    first.advanceTo(Timestamp{1s + 35ms});
    ASSERT_EQ(firstEvents->frames.size(), 2U);
    ASSERT_EQ(secondEvents->frames.size(), 2U);
    EXPECT_EQ(firstEvents->frames[0]->sequence, 0U);
    EXPECT_EQ(firstEvents->frames[1]->sequence, 3U);
    EXPECT_EQ(firstEvents->frames[1]->capturedAt, Timestamp{1s + 35ms});
    EXPECT_EQ(firstEvents->frames[1]->width, 8U);
    EXPECT_EQ(firstEvents->frames[1]->height, 6U);
    EXPECT_EQ(firstEvents->frames[1]->strideBytes, 8U);
    EXPECT_EQ(firstEvents->frames[1]->pixelFormat, PixelFormat::Gray8);
    EXPECT_EQ(firstEvents->frames[0]->pixels->size(), 48U);
    EXPECT_EQ(firstEvents->frames[0]->pixels->front(), std::byte{5});
    EXPECT_EQ(firstEvents->frames[1]->pixels->front(), std::byte{8});
    EXPECT_EQ(*firstEvents->frames[0]->pixels, *secondEvents->frames[0]->pixels);
    EXPECT_EQ(*firstEvents->frames[1]->pixels, *secondEvents->frames[1]->pixels);
    EXPECT_NE(firstEvents->frames[0]->pixels, firstEvents->frames[1]->pixels);

    source.stop();
    first.advanceTo(Timestamp{2s});
    EXPECT_EQ(source.state(), CameraState::Stopped);
    EXPECT_EQ(firstEvents->frames.size(), 2U);
    source.start();
    first.advanceTo(Timestamp{3s});
    ASSERT_EQ(firstEvents->frames.size(), 3U);
    EXPECT_EQ(firstEvents->frames.back()->sequence, 4U);
    // Previously retained buffers have not been reused or mutated.
    EXPECT_EQ(firstEvents->frames.front()->pixels->front(), std::byte{5});
}

TEST(FakeCameraTest, RequestsAreIdempotentAndPendingStartCanBeCancelled)
{
    FakeCameraSource source;
    auto events = std::make_shared<CameraEvents>();
    source.setEventSink(events);
    source.start();
    source.stop();
    source.advanceTo(Timestamp{1s});
    EXPECT_EQ(source.state(), CameraState::Stopped);
    EXPECT_TRUE(events->frames.empty());
    source.start();
    source.start();
    source.advanceTo(Timestamp{2s});
    source.start();
    source.advanceTo(Timestamp{2s});
    EXPECT_EQ(events->states.size(), 1U);
    EXPECT_EQ(events->frames.size(), 1U);
    source.stop();
    source.stop();
    source.advanceTo(Timestamp{3s});
    EXPECT_EQ(events->states.size(), 2U);
    EXPECT_EQ(events->frames.size(), 1U);
}

TEST(FakeCameraTest, CallbackCanStopBeforeFrameDeliveryAndSubscriptionsAreWeak)
{
    FakeCameraSource source;
    auto events = std::make_shared<CameraEvents>();
    source.setEventSink(events);
    events->onState = [&](CameraState state) {
        if (state == CameraState::Streaming) {
            source.stop();
        }
    };
    source.start();
    source.advanceTo(Timestamp{1s});
    EXPECT_EQ(source.state(), CameraState::Stopped);
    EXPECT_TRUE(events->frames.empty());
    EXPECT_EQ(events.use_count(), 1);
    events.reset();
    source.start();
    EXPECT_NO_THROW(source.advanceTo(Timestamp{2s}));
    source.setEventSink({});
    EXPECT_NO_THROW(source.advanceTo(Timestamp{3s}));
}

TEST(FakeCameraTest, RejectsInvalidCadenceAndBackwardOrRecursiveTime)
{
    FakeCameraConfig config;
    config.frameInterval = 0ms;
    EXPECT_THROW(FakeCameraSource{config}, std::invalid_argument);
    FakeCameraSource source;
    source.advanceTo(Timestamp{2s});
    EXPECT_THROW(source.advanceTo(Timestamp{1s}), std::invalid_argument);
    auto events = std::make_shared<CameraEvents>();
    source.setEventSink(events);
    events->onState = [&](CameraState) {
        EXPECT_THROW(source.advanceTo(Timestamp{3s}), std::logic_error);
    };
    source.start();
    source.advanceTo(Timestamp{3s});
}

TEST(SimulationTransportTest, ConnectionsAreDeferredIdempotentAndCancellable)
{
    SimulationTransport simulation;
    IRobotTransport& transport = simulation;
    auto events = std::make_shared<TransportEvents>();
    transport.setEventSink(events);
    const RobotId id{7};
    transport.connect(id);
    transport.connect(id);
    EXPECT_EQ(transport.connectionState(id), ConnectionState::Disconnected);
    EXPECT_TRUE(events->states.empty());
    simulation.advanceTo(Timestamp{1s});
    EXPECT_EQ(transport.connectionState(id), ConnectionState::Connected);
    EXPECT_EQ(transport.connectionState(RobotId{8}), ConnectionState::Disconnected);
    simulation.advanceTo(Timestamp{1s});
    transport.connect(id);
    simulation.advanceTo(Timestamp{1s});
    EXPECT_EQ(events->states.size(), 1U);
    EXPECT_EQ(events->telemetry.size(), 1U);
    transport.disconnect(id);
    transport.disconnect(id);
    simulation.advanceTo(Timestamp{2s});
    EXPECT_EQ(transport.connectionState(id), ConnectionState::Disconnected);
    EXPECT_EQ(events->states.size(), 2U);
    EXPECT_EQ(events->telemetry.size(), 1U);
    transport.connect(RobotId{8});
    transport.disconnect(RobotId{8});
    simulation.advanceTo(Timestamp{3s});
    EXPECT_EQ(events->states.size(), 2U);
}

TEST(SimulationTransportTest, CopiesCommandsAndProducesDeterministicTelemetry)
{
    SimulationTransport first;
    SimulationTransport second;
    auto firstEvents = std::make_shared<TransportEvents>();
    auto secondEvents = std::make_shared<TransportEvents>();
    first.setEventSink(firstEvents);
    second.setEventSink(secondEvents);
    const RobotId id{7};
    RobotCommand command{VelocityCommand{0.2, -0.5}, Timestamp{1s}, 100ms};
    for (IRobotTransport* transport : {&first, &second}) {
        transport->connect(id);
        transport->sendCommand(id, command);
    }
    command.payload = StopCommand{};
    EXPECT_FALSE(first.lastSentCommand(id));
    first.advanceTo(Timestamp{1s});
    second.advanceTo(Timestamp{1s});
    ASSERT_TRUE(first.lastSentCommand(id));
    EXPECT_TRUE(std::holds_alternative<VelocityCommand>(first.lastSentCommand(id)->payload));
    ASSERT_EQ(firstEvents->telemetry.size(), 1U);
    ASSERT_EQ(secondEvents->telemetry.size(), 1U);
    EXPECT_EQ(firstEvents->telemetry[0].robotId, id);
    EXPECT_EQ(firstEvents->telemetry[0].receivedAt, Timestamp{1s});
    EXPECT_EQ(firstEvents->telemetry[0].batteryVoltageVolts, 3.7);
    EXPECT_EQ(firstEvents->telemetry[0].linearMetersPerSecond, 0.2);
    EXPECT_EQ(firstEvents->telemetry[0].angularRadiansPerSecond, -0.5);
    EXPECT_EQ(firstEvents->telemetry[0].linearMetersPerSecond,
              secondEvents->telemetry[0].linearMetersPerSecond);
    first.advanceTo(Timestamp{1s + 100ms});
    EXPECT_EQ(firstEvents->telemetry.back().linearMetersPerSecond, 0.0);
    first.sendCommand(id, {StopCommand{}, Timestamp{2s}, 100ms});
    first.advanceTo(Timestamp{2s});
    EXPECT_TRUE(std::holds_alternative<StopCommand>(first.lastSentCommand(id)->payload));
    EXPECT_EQ(firstEvents->telemetry.back().angularRadiansPerSecond, 0.0);
}

TEST(SimulationTransportTest, TelemetryCadenceAndBatteryAreConfigurable)
{
    SimulationTransport transport{{100ms, 4.1}};
    auto events = std::make_shared<TransportEvents>();
    transport.setEventSink(events);
    const RobotId id{2};
    transport.connect(id);
    transport.advanceTo(Timestamp{1s});
    ASSERT_EQ(events->telemetry.size(), 1U);
    EXPECT_EQ(events->telemetry[0].batteryVoltageVolts, 4.1);
    transport.advanceTo(Timestamp{1s + 99ms});
    EXPECT_EQ(events->telemetry.size(), 1U);
    transport.advanceTo(Timestamp{1s + 100ms});
    transport.advanceTo(Timestamp{1s + 150ms});
    ASSERT_EQ(events->telemetry.size(), 2U);
    EXPECT_EQ(events->telemetry[1].receivedAt, Timestamp{1s + 100ms});

    // A new session reports immediately instead of continuing the old cadence.
    transport.disconnect(id);
    transport.advanceTo(Timestamp{1s + 160ms});
    transport.connect(id);
    transport.advanceTo(Timestamp{1s + 170ms});
    ASSERT_EQ(events->telemetry.size(), 3U);
    EXPECT_EQ(events->telemetry[2].receivedAt, Timestamp{1s + 170ms});

    // Extreme timestamps do not overflow the elapsed-time check.
    SimulationTransport extremes{{1s}};
    extremes.setEventSink(events);
    extremes.connect(id);
    extremes.advanceTo(Timestamp::min());
    extremes.advanceTo(Timestamp::max());
    EXPECT_EQ(events->telemetry.back().receivedAt, Timestamp::max());
}

TEST(SimulationTransportTest, RejectsInvalidConfiguration)
{
    EXPECT_THROW(SimulationTransport({-1ms}), std::invalid_argument);
    EXPECT_THROW(SimulationTransport({0ms, 0.0}), std::invalid_argument);
    EXPECT_THROW(SimulationTransport({0ms, -3.7}), std::invalid_argument);
    EXPECT_THROW(SimulationTransport({0ms, std::numeric_limits<double>::quiet_NaN()}),
                 std::invalid_argument);
    EXPECT_NO_THROW(SimulationTransport({0ms, 3.7}));
}

TEST(SimulationTransportTest, LogsSimulatedCommunicationDeterministically)
{
    auto run = [] {
        Logger logger{LogLevel::Debug, [] { return WallTime{}; }};
        auto sink = std::make_shared<CapturingLogSink>();
        logger.addSink(sink);
        SimulationTransport transport{{}, logger.channel("simulation")};
        const RobotId id{3};
        transport.connect(id);
        transport.sendCommand(id, {VelocityCommand{0.2, -0.5}, Timestamp{1s}, 100ms});
        transport.advanceTo(Timestamp{1s});
        transport.sendCommand(id, {VelocityCommand{0.1, 0.0}, Timestamp{0s}, 100ms});
        transport.advanceTo(Timestamp{2s});
        transport.disconnect(id);
        transport.advanceTo(Timestamp{3s});
        transport.sendCommand(id, {StopCommand{}, Timestamp{3s}, 100ms});
        transport.advanceTo(Timestamp{3s});
        return sink->records;
    };
    const auto records = run();
    EXPECT_TRUE(containsLog(records, LogLevel::Debug, "Simulated connect requested"));
    EXPECT_TRUE(containsLog(records, LogLevel::Info, "Simulated connection state: Connected"));
    EXPECT_TRUE(containsLog(records, LogLevel::Debug,
        "Simulated command accepted: velocity linear=0.200 m/s angular=-0.500 rad/s"));
    EXPECT_TRUE(containsLog(records, LogLevel::Debug,
        "Simulated telemetry: battery=3.700 V linear=0.200 m/s angular=-0.500 rad/s"));
    EXPECT_TRUE(containsLog(records, LogLevel::Warning, "Simulated command rejected (InvalidCommand)"));
    EXPECT_TRUE(containsLog(records, LogLevel::Debug, "Simulated disconnect requested"));
    EXPECT_TRUE(containsLog(records, LogLevel::Info, "Simulated connection state: Disconnected"));
    EXPECT_TRUE(containsLog(records, LogLevel::Warning, "Simulated command rejected (NotConnected)"));
    for (const auto& record : records) {
        EXPECT_EQ(record.module, "simulation");
        EXPECT_EQ(record.robotId, RobotId{3});
    }

    const auto repeated = run();
    ASSERT_EQ(repeated.size(), records.size());
    for (std::size_t i = 0; i < records.size(); ++i) {
        EXPECT_EQ(repeated[i].level, records[i].level);
        EXPECT_EQ(repeated[i].message, records[i].message);
    }
}

TEST(SimulationTransportTest, PreservesReservedMetadataWithoutSequenceOrdering)
{
    SimulationTransport transport;
    const RobotId id{7};
    transport.connect(id);
    RobotCommand command{VelocityCommand{0.2, 0.5}, Timestamp{1s}, 1s};
    command.metadata.sequence = std::numeric_limits<std::uint64_t>::max();
    transport.sendCommand(id, command);
    command.metadata.sequence.reset();
    transport.advanceTo(Timestamp{1s});
    ASSERT_TRUE(transport.lastSentCommand(id));
    EXPECT_EQ(transport.lastSentCommand(id)->metadata.modelVersion, currentTransportModelVersion);
    EXPECT_EQ(transport.lastSentCommand(id)->metadata.sequence,
              std::numeric_limits<std::uint64_t>::max());
    // Reservation alone does not establish wire ordering or deduplication.
    command.metadata.sequence = 0;
    transport.sendCommand(id, command);
    transport.advanceTo(Timestamp{1s});
    EXPECT_EQ(transport.lastSentCommand(id)->metadata.sequence, 0u);
    command.payload = StopCommand{};
    transport.sendCommand(id, command);
    transport.advanceTo(Timestamp{1s});
    EXPECT_TRUE(std::holds_alternative<StopCommand>(transport.lastSentCommand(id)->payload));
}

TEST(SimulationTransportTest, UnsupportedVersionsReportDeferredErrorsWithoutReplacingAcceptedCommand)
{
    SimulationTransport transport;
    auto events = std::make_shared<TransportEvents>();
    transport.setEventSink(events);
    const RobotId id{7};
    transport.connect(id);
    transport.sendCommand(id, {VelocityCommand{0.2, 0.5}, Timestamp{1s}, 1s});
    transport.advanceTo(Timestamp{1s});
    const std::vector<std::uint16_t> versions{0, 2, std::numeric_limits<std::uint16_t>::max()};
    for (const auto version : versions) {
        RobotCommand command{StopCommand{}, Timestamp{1s}, 1s};
        command.metadata.modelVersion = version;
        const auto previousErrors = events->errors.size();
        transport.sendCommand(id, command);
        EXPECT_EQ(events->errors.size(), previousErrors);
        transport.advanceTo(Timestamp{1s});
        ASSERT_EQ(events->errors.size(), previousErrors + 1);
        EXPECT_EQ(events->errors.back().code, TransportErrorCode::UnsupportedVersion);
        EXPECT_EQ(events->errors.back().operation, TransportOperation::SendCommand);
        EXPECT_EQ(events->errors.back().robotId, id);
        EXPECT_EQ(events->errors.back().occurredAt, Timestamp{1s});
        ASSERT_TRUE(transport.lastSentCommand(id));
        EXPECT_TRUE(std::holds_alternative<VelocityCommand>(transport.lastSentCommand(id)->payload));
    }
    transport.sendCommand(id, {StopCommand{}, Timestamp{1s}, 1s});
    transport.advanceTo(Timestamp{1s});
    EXPECT_TRUE(std::holds_alternative<StopCommand>(transport.lastSentCommand(id)->payload));
}

TEST(SimulationTransportTest, TelemetryMetadataDoesNotImplyCommandAcknowledgement)
{
    SimulationTransport transport;
    auto events = std::make_shared<TransportEvents>();
    transport.setEventSink(events);
    const RobotId id{7};
    transport.connect(id);
    transport.sendCommand(id, {StopCommand{}, Timestamp{1s}, 1s,
                              {currentTransportModelVersion, 42}});
    transport.advanceTo(Timestamp{1s});
    ASSERT_EQ(events->telemetry.size(), 1u);
    EXPECT_EQ(events->telemetry.back().metadata.modelVersion, currentTransportModelVersion);
    EXPECT_FALSE(events->telemetry.back().metadata.sequence);
}

TEST(SimulationTransportTest, FreshnessHasExclusiveDeadlineAndHandlesExtremeTimestamps)
{
    SimulationTransport transport;
    auto events = std::make_shared<TransportEvents>();
    transport.setEventSink(events);
    const RobotId id{7};
    transport.connect(id);
    transport.sendCommand(id, {VelocityCommand{0.2, 0.5}, Timestamp::min(), Duration::max()});
    transport.advanceTo(Timestamp::min());
    ASSERT_TRUE(transport.lastSentCommand(id));
    EXPECT_EQ(events->telemetry.back().linearMetersPerSecond, 0.2);
    // No signed duration subtraction overflow on expiry or new validation.
    transport.sendCommand(id, {StopCommand{}, Timestamp::min(), Duration::max()});
    transport.advanceTo(Timestamp::max());
    ASSERT_EQ(events->errors.size(), 1u);
    EXPECT_EQ(events->errors.back().code, TransportErrorCode::InvalidCommand);
    EXPECT_EQ(events->telemetry.back().linearMetersPerSecond, 0.0);

    SimulationTransport boundary;
    boundary.setEventSink(events);
    boundary.connect(id);
    const RobotCommand command{VelocityCommand{0.3, 0.5}, Timestamp{1s}, 100ms};
    boundary.sendCommand(id, command);
    boundary.advanceTo(Timestamp{1s + 100ms} - Duration{1});
    ASSERT_TRUE(boundary.lastSentCommand(id));
    EXPECT_EQ(events->telemetry.back().linearMetersPerSecond, 0.3);
    boundary.sendCommand(id, command);
    boundary.advanceTo(Timestamp{1s + 100ms});
    EXPECT_EQ(events->errors.size(), 2u);
    EXPECT_EQ(events->telemetry.back().linearMetersPerSecond, 0.0);
}

TEST(SimulationTransportTest, LatestPendingCommandWinsAndDisconnectClearsIt)
{
    SimulationTransport transport;
    const RobotId id{7};
    transport.connect(id);
    transport.sendCommand(id, {VelocityCommand{0.2, 0.5}, Timestamp{1s}, 1s});
    transport.sendCommand(id, {StopCommand{}, Timestamp{1s}, 1s});
    transport.advanceTo(Timestamp{1s});
    ASSERT_TRUE(transport.lastSentCommand(id));
    EXPECT_TRUE(std::holds_alternative<StopCommand>(transport.lastSentCommand(id)->payload));
    transport.sendCommand(id, {VelocityCommand{0.3, 0.5}, Timestamp{2s}, 1s});
    transport.disconnect(id);
    transport.advanceTo(Timestamp{2s});
    EXPECT_FALSE(transport.lastSentCommand(id));
    transport.connect(id);
    transport.advanceTo(Timestamp{3s});
    EXPECT_FALSE(transport.lastSentCommand(id));
}

TEST(SimulationTransportTest, RejectsDisconnectedStaleFutureAndNonfiniteCommands)
{
    SimulationTransport transport;
    auto events = std::make_shared<TransportEvents>();
    transport.setEventSink(events);
    const RobotId id{7};
    transport.sendCommand(id, {StopCommand{}, Timestamp{1s}, 1s});
    EXPECT_TRUE(events->errors.empty());
    transport.advanceTo(Timestamp{1s});
    ASSERT_EQ(events->errors.size(), 1U);
    EXPECT_EQ(events->errors.back().code, TransportErrorCode::NotConnected);
    transport.connect(id);
    transport.advanceTo(Timestamp{2s});
    const std::vector<RobotCommand> invalid{
        {StopCommand{}, Timestamp{1s}, 1s},
        {StopCommand{}, Timestamp{3s}, 1s},
        {StopCommand{}, Timestamp{2s}, 0ms},
        {StopCommand{}, Timestamp{2s}, -1ms},
        {VelocityCommand{std::numeric_limits<double>::infinity(), 0.0}, Timestamp{2s}, 1s},
        {VelocityCommand{0.0, std::numeric_limits<double>::quiet_NaN()}, Timestamp{2s}, 1s}
    };
    for (const auto& command : invalid) {
        transport.sendCommand(id, command);
        transport.advanceTo(Timestamp{2s});
        EXPECT_EQ(events->errors.back().code, TransportErrorCode::InvalidCommand);
    }
    EXPECT_EQ(events->errors.size(), 1 + invalid.size());
    EXPECT_FALSE(transport.lastSentCommand(id));
}

TEST(SimulationTransportTest, OrdersRobotsAndHandlesCallbacksAndExpiredSinks)
{
    SimulationTransport transport;
    auto events = std::make_shared<TransportEvents>();
    transport.setEventSink(events);
    transport.connect(RobotId{9});
    transport.connect(RobotId{2});
    transport.advanceTo(Timestamp{1s});
    ASSERT_EQ(events->telemetry.size(), 2U);
    EXPECT_EQ(events->telemetry[0].robotId, RobotId{2});
    EXPECT_EQ(events->telemetry[1].robotId, RobotId{9});
    events->onState = [&](RobotId id, ConnectionState state) {
        if (state == ConnectionState::Connected) {
            transport.disconnect(id);
            EXPECT_THROW(transport.advanceTo(Timestamp{2s}), std::logic_error);
        }
    };
    transport.connect(RobotId{3});
    transport.advanceTo(Timestamp{2s});
    EXPECT_EQ(transport.connectionState(RobotId{3}), ConnectionState::Disconnected);
    for (const auto& telemetry : events->telemetry) {
        EXPECT_NE(telemetry.robotId, RobotId{3});
    }
    EXPECT_EQ(events.use_count(), 1);
    events.reset();
    EXPECT_NO_THROW(transport.advanceTo(Timestamp{3s}));
    transport.setEventSink({});
    EXPECT_THROW(transport.advanceTo(Timestamp{2s}), std::invalid_argument);
}

// Different test-only implementations establish that the consumer has no
// dependency on fake/simulation classes. The harness alone owns the time driver.
class EventOnlyCamera final : public ICameraSource {
public:
    void setEventSink(std::weak_ptr<ICameraEvents> sink) override { sink_ = std::move(sink); }
    CameraInfo info() const override { return {"event-only", "Event-only test camera"}; }
    CameraState state() const override { return state_; }
    void start() override { active_ = true; }
    void stop() override { active_ = false; }
    void advanceTo(Timestamp at)
    {
        const auto desired = active_ ? CameraState::Streaming : CameraState::Stopped;
        if (auto sink = sink_.lock()) {
            if (desired != state_) {
                state_ = desired;
                sink->onCameraStateChanged(state_, at);
            }
            if (active_) {
                sink->onFrame(std::make_shared<const Frame>(Frame{
                    0, at, 1, 1, 1, PixelFormat::Gray8,
                    std::make_shared<const std::vector<std::byte>>(1, std::byte{0})
                }));
            }
        }
    }
private:
    std::weak_ptr<ICameraEvents> sink_;
    CameraState state_{CameraState::Stopped};
    bool active_{};
};

class EventOnlyTransport final : public IRobotTransport {
public:
    void setEventSink(std::weak_ptr<IRobotTransportEvents> sink) override { sink_ = std::move(sink); }
    ConnectionState connectionState(RobotId id) const override
    {
        return connected_ && *connected_ == id ? ConnectionState::Connected
                                              : ConnectionState::Disconnected;
    }
    void connect(RobotId id) override { desired_ = id; }
    void disconnect(RobotId) override { desired_.reset(); }
    void sendCommand(RobotId, const RobotCommand& command) override { command_ = command; }
    void advanceTo(Timestamp at)
    {
        if (auto sink = sink_.lock()) {
            if (connected_ && !desired_) {
                const auto id = *connected_;
                connected_.reset();
                sink->onConnectionStateChanged(id, ConnectionState::Disconnected, at);
            }
            if (!connected_ && desired_) {
                connected_ = desired_;
                sink->onConnectionStateChanged(*connected_, ConnectionState::Connected, at);
            }
            if (connected_ && command_) {
                sink->onTelemetry({*connected_, at, 3.3, 0.0, 0.0});
                command_.reset();
            }
        }
    }
private:
    std::weak_ptr<IRobotTransportEvents> sink_;
    std::optional<RobotId> desired_;
    std::optional<RobotId> connected_;
    std::optional<RobotCommand> command_;
};

// These consumer functions intentionally accept only the public abstractions.
void startCameraConsumer(ICameraSource& source, const std::shared_ptr<CameraEvents>& events)
{
    source.setEventSink(events);
    EXPECT_FALSE(source.info().sourceId.empty());
    source.start();
}

void connectTransportConsumer(IRobotTransport& transport,
                              const std::shared_ptr<TransportEvents>& events)
{
    transport.setEventSink(events);
    transport.connect(RobotId{7});
    transport.sendCommand(RobotId{7}, {StopCommand{}, Timestamp{1s}, 1s});
}

template <typename Backend>
void exerciseCameraConsumer()
{
    auto backend = std::make_unique<Backend>();
    auto* driver = backend.get(); // Harness-only non-owning reference.
    std::unique_ptr<ICameraSource> source = std::move(backend);
    auto events = std::make_shared<CameraEvents>();
    startCameraConsumer(*source, events);
    EXPECT_TRUE(events->frames.empty());
    driver->advanceTo(Timestamp{1s});
    EXPECT_EQ(source->state(), CameraState::Streaming);
    ASSERT_FALSE(events->frames.empty());
    EXPECT_TRUE(events->frames.back()->pixels);
    source->stop();
    driver->advanceTo(Timestamp{2s});
    EXPECT_EQ(source->state(), CameraState::Stopped);
    source.reset();
    EXPECT_TRUE(events->frames.front()->pixels);
}

template <typename Backend>
void exerciseTransportConsumer()
{
    auto backend = std::make_unique<Backend>();
    auto* driver = backend.get();
    std::unique_ptr<IRobotTransport> transport = std::move(backend);
    auto events = std::make_shared<TransportEvents>();
    connectTransportConsumer(*transport, events);
    EXPECT_TRUE(events->telemetry.empty());
    driver->advanceTo(Timestamp{1s});
    EXPECT_EQ(transport->connectionState(RobotId{7}), ConnectionState::Connected);
    ASSERT_FALSE(events->telemetry.empty());
    EXPECT_EQ(events->telemetry.back().robotId, RobotId{7});
    transport->disconnect(RobotId{7});
    driver->advanceTo(Timestamp{2s});
    EXPECT_EQ(transport->connectionState(RobotId{7}), ConnectionState::Disconnected);
}

TEST(BackendSubstitutionTest, CameraConsumerUsesTwoDifferentImplementations)
{
    exerciseCameraConsumer<FakeCameraSource>();
    exerciseCameraConsumer<EventOnlyCamera>();
}

TEST(BackendSubstitutionTest, TransportConsumerUsesTwoDifferentImplementations)
{
    exerciseTransportConsumer<SimulationTransport>();
    exerciseTransportConsumer<EventOnlyTransport>();
}

} // namespace
} // namespace micro_robot
