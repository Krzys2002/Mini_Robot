#include "camera/ICameraSource.hpp"
#include "transport/IRobotTransport.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <map>
#include <memory>
#include <optional>
#include <type_traits>
#include <utility>
#include <vector>

namespace micro_robot {
namespace {

using namespace std::chrono_literals;

static_assert(std::is_abstract_v<ICameraSource>);
static_assert(std::is_abstract_v<IRobotTransport>);
static_assert(std::has_virtual_destructor_v<ICameraSource>);
static_assert(std::has_virtual_destructor_v<IRobotTransport>);
static_assert(std::has_virtual_destructor_v<ICameraEvents>);
static_assert(std::has_virtual_destructor_v<IRobotTransportEvents>);
static_assert(std::is_const_v<FramePtr::element_type>);

class RecordingEvents final : public ICameraEvents, public IRobotTransportEvents {
public:
    void onFrame(FramePtr frame) override { lastFrame = std::move(frame); }
    void onCameraStateChanged(CameraState state, Timestamp at) override
    {
        cameraStates.emplace_back(state, at);
    }
    void onCameraError(const CameraError& error) override { cameraError = error; }
    void onConnectionStateChanged(RobotId id, ConnectionState state, Timestamp at) override
    {
        robotId = id;
        connection = state;
        changedAt = at;
    }
    void onTelemetry(const RobotTelemetry& value) override { telemetry = value; }
    void onTransportError(const TransportError& error) override { transportError = error; }

    FramePtr lastFrame;
    std::vector<std::pair<CameraState, Timestamp>> cameraStates;
    std::optional<CameraError> cameraError;
    std::optional<RobotId> robotId;
    std::optional<ConnectionState> connection;
    std::optional<Timestamp> changedAt;
    std::optional<RobotTelemetry> telemetry;
    std::optional<TransportError> transportError;
};

// Test-only probes record requests. Explicit publish methods stand in for later
// asynchronous completion; these are not production camera/transport backends.
class CameraProbe final : public ICameraSource {
public:
    explicit CameraProbe(bool& destroyed) : destroyed_(destroyed) {}
    ~CameraProbe() override { destroyed_ = true; }

    void setEventSink(std::weak_ptr<ICameraEvents> sink) override { sink_ = std::move(sink); }
    CameraInfo info() const override { return {"test-source", "Test camera"}; }
    CameraState state() const override { return state_; }
    void start() override { startRequested = true; }
    void stop() override { stopRequested = true; }

    bool publishState(CameraState state, Timestamp at)
    {
        state_ = state;
        if (auto sink = sink_.lock()) {
            sink->onCameraStateChanged(state, at);
            return true;
        }
        return false;
    }
    void publishFrame(FramePtr frame)
    {
        if (auto sink = sink_.lock()) {
            sink->onFrame(std::move(frame));
        }
    }
    void publishError(const CameraError& error)
    {
        if (auto sink = sink_.lock()) {
            sink->onCameraError(error);
        }
    }

    bool startRequested{};
    bool stopRequested{};

private:
    bool& destroyed_;
    CameraState state_{CameraState::Stopped};
    std::weak_ptr<ICameraEvents> sink_;
};

class TransportProbe final : public IRobotTransport {
public:
    explicit TransportProbe(bool& destroyed) : destroyed_(destroyed) {}
    ~TransportProbe() override { destroyed_ = true; }

    void setEventSink(std::weak_ptr<IRobotTransportEvents> sink) override
    {
        sink_ = std::move(sink);
    }
    ConnectionState connectionState(RobotId id) const override
    {
        const auto found = states_.find(id);
        return found == states_.end() ? ConnectionState::Disconnected : found->second;
    }
    void connect(RobotId id) override { connectRequested = id; }
    void disconnect(RobotId id) override { disconnectRequested = id; }
    void sendCommand(RobotId id, const RobotCommand& command) override
    {
        submittedCommand.emplace(id, command);
    }

    bool publishState(RobotId id, ConnectionState state, Timestamp at)
    {
        states_.insert_or_assign(id, state);
        if (auto sink = sink_.lock()) {
            sink->onConnectionStateChanged(id, state, at);
            return true;
        }
        return false;
    }
    void publishTelemetry(const RobotTelemetry& telemetry)
    {
        if (auto sink = sink_.lock()) {
            sink->onTelemetry(telemetry);
        }
    }
    void publishError(const TransportError& error)
    {
        if (auto sink = sink_.lock()) {
            sink->onTransportError(error);
        }
    }

    std::optional<RobotId> connectRequested;
    std::optional<RobotId> disconnectRequested;
    std::optional<std::pair<RobotId, RobotCommand>> submittedCommand;

private:
    bool& destroyed_;
    std::weak_ptr<IRobotTransportEvents> sink_;
    std::map<RobotId, ConnectionState> states_;
};

TEST(CameraInterfaceTest, RequestsAndEventsUseTheAbstractBoundary)
{
    bool destroyed = false;
    auto probe = std::make_unique<CameraProbe>(destroyed);
    auto& completion = *probe;
    std::unique_ptr<ICameraSource> source = std::move(probe);
    auto events = std::make_shared<RecordingEvents>();
    source->setEventSink(events);

    EXPECT_EQ(source->info().sourceId, "test-source");
    EXPECT_EQ(source->state(), CameraState::Stopped);
    source->start();
    EXPECT_TRUE(completion.startRequested);
    EXPECT_TRUE(events->cameraStates.empty());
    const Timestamp at{2s};
    completion.publishState(CameraState::Streaming, at);
    EXPECT_EQ(source->state(), CameraState::Streaming);
    ASSERT_EQ(events->cameraStates.size(), 1U);
    EXPECT_EQ(events->cameraStates.front(), std::make_pair(CameraState::Streaming, at));

    source->stop();
    EXPECT_TRUE(completion.stopRequested);
    completion.publishState(CameraState::Stopped, at + 1s);
    source.reset();
    EXPECT_TRUE(destroyed);
}

TEST(CameraInterfaceTest, FrameStorageOutlivesTheProducer)
{
    bool destroyed = false;
    auto source = std::make_unique<CameraProbe>(destroyed);
    auto events = std::make_shared<RecordingEvents>();
    source->setEventSink(events);
    source->publishState(CameraState::Streaming, Timestamp{1s});

    const Timestamp capturedAt{2s};
    auto frame = std::make_shared<Frame>(Frame{
        42, capturedAt, 2, 1, 2, PixelFormat::Gray8,
        std::make_shared<const std::vector<std::byte>>(
            std::vector<std::byte>{std::byte{0x11}, std::byte{0x22}})
    });
    source->publishFrame(frame);
    frame.reset();
    source->publishState(CameraState::Stopped, Timestamp{3s});
    source.reset();

    ASSERT_TRUE(events->lastFrame);
    EXPECT_EQ(events->lastFrame->sequence, 42U);
    EXPECT_EQ(events->lastFrame->capturedAt, capturedAt);
    ASSERT_TRUE(events->lastFrame->pixels);
    EXPECT_EQ(events->lastFrame->pixels->at(1), std::byte{0x22});
}

TEST(CameraInterfaceTest, WeakSubscriptionCanExpireOrBeDetached)
{
    bool destroyed = false;
    CameraProbe source{destroyed};
    auto events = std::make_shared<RecordingEvents>();
    source.setEventSink(events);
    EXPECT_EQ(events.use_count(), 1);
    events.reset();
    EXPECT_FALSE(source.publishState(CameraState::Stopped, Timestamp{1s}));

    events = std::make_shared<RecordingEvents>();
    source.setEventSink(events);
    const CameraError error{CameraErrorCode::Unavailable, "Source unavailable", Timestamp{2s}};
    source.publishError(error);
    ASSERT_TRUE(events->cameraError);
    EXPECT_EQ(events->cameraError->code, CameraErrorCode::Unavailable);
    EXPECT_EQ(events->cameraError->occurredAt, error.occurredAt);
    source.setEventSink({});
    EXPECT_FALSE(source.publishState(CameraState::Stopped, Timestamp{3s}));
    EXPECT_TRUE(events->cameraStates.empty());
}

TEST(TransportInterfaceTest, ConnectionsArePerRobotAndRequestsAreDeferred)
{
    bool destroyed = false;
    auto probe = std::make_unique<TransportProbe>(destroyed);
    auto& completion = *probe;
    std::unique_ptr<IRobotTransport> transport = std::move(probe);
    auto events = std::make_shared<RecordingEvents>();
    transport->setEventSink(events);
    const RobotId id{7};
    transport->connect(id);
    EXPECT_EQ(completion.connectRequested, id);
    EXPECT_FALSE(events->connection);
    const Timestamp at{2s};
    completion.publishState(id, ConnectionState::Connected, at);
    EXPECT_EQ(transport->connectionState(id), ConnectionState::Connected);
    EXPECT_EQ(transport->connectionState(RobotId{8}), ConnectionState::Disconnected);
    EXPECT_EQ(events->robotId, id);
    EXPECT_EQ(events->changedAt, at);
    transport->disconnect(id);
    EXPECT_EQ(completion.disconnectRequested, id);
    completion.publishState(id, ConnectionState::Disconnected, at + 1s);
    transport.reset();
    EXPECT_TRUE(destroyed);
}

TEST(TransportInterfaceTest, CommandsAndTelemetryAreOwnedDomainValues)
{
    bool destroyed = false;
    TransportProbe probe{destroyed};
    IRobotTransport& transport = probe;
    auto events = std::make_shared<RecordingEvents>();
    transport.setEventSink(events);
    const RobotId id{7};
    const Timestamp at{2s};
    RobotCommand command{VelocityCommand{0.2, -0.5}, at, 100ms};
    transport.sendCommand(id, command);
    command.payload = StopCommand{};
    ASSERT_TRUE(probe.submittedCommand);
    EXPECT_EQ(probe.submittedCommand->first, id);
    const auto& copied = probe.submittedCommand->second;
    ASSERT_TRUE(std::holds_alternative<VelocityCommand>(copied.payload));
    EXPECT_DOUBLE_EQ(std::get<VelocityCommand>(copied.payload).linearMetersPerSecond, 0.2);
    EXPECT_EQ(copied.issuedAt, at);
    EXPECT_EQ(copied.validFor, 100ms);
    transport.sendCommand(id, command);
    EXPECT_TRUE(std::holds_alternative<StopCommand>(probe.submittedCommand->second.payload));

    RobotTelemetry telemetry{id, at + 20ms, 3.7, std::nullopt, std::nullopt};
    probe.publishTelemetry(telemetry);
    telemetry.batteryVoltageVolts = 0.0;
    ASSERT_TRUE(events->telemetry);
    EXPECT_EQ(events->telemetry->robotId, id);
    EXPECT_EQ(events->telemetry->receivedAt, at + 20ms);
    ASSERT_TRUE(events->telemetry->batteryVoltageVolts);
    EXPECT_DOUBLE_EQ(*events->telemetry->batteryVoltageVolts, 3.7);
    EXPECT_FALSE(events->telemetry->linearMetersPerSecond);
}

TEST(TransportInterfaceTest, ErrorsAndWeakSubscriptionsUseTheAbstractBoundary)
{
    bool destroyed = false;
    TransportProbe transport{destroyed};
    auto events = std::make_shared<RecordingEvents>();
    transport.setEventSink(events);
    EXPECT_EQ(events.use_count(), 1);
    const TransportError error{RobotId{7}, TransportOperation::SendCommand,
        TransportErrorCode::NotConnected, "Robot disconnected", Timestamp{2s}};
    transport.publishError(error);
    ASSERT_TRUE(events->transportError);
    EXPECT_EQ(events->transportError->robotId, RobotId{7});
    EXPECT_EQ(events->transportError->operation, TransportOperation::SendCommand);
    EXPECT_EQ(events->transportError->code, TransportErrorCode::NotConnected);
    transport.setEventSink({});
    EXPECT_FALSE(transport.publishState(RobotId{7}, ConnectionState::Disconnected, Timestamp{3s}));
    transport.setEventSink(events);
    events.reset();
    EXPECT_FALSE(transport.publishState(RobotId{7}, ConnectionState::Disconnected, Timestamp{4s}));
}

} // namespace
} // namespace micro_robot
