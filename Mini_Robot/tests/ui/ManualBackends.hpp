#pragma once

#include "camera/ICameraSource.hpp"
#include "transport/IRobotTransport.hpp"

#include <map>
#include <memory>
#include <utility>
#include <vector>

namespace micro_robot::test {

// Test-only alternative backends. Requests are recorded; tests publish events
// explicitly to exercise asynchronous states omitted by the immediate fakes.
class ManualCamera final : public ICameraSource {
public:
    void setEventSink(std::weak_ptr<ICameraEvents> sink) override { sink_ = std::move(sink); }
    CameraInfo info() const override { return {"manual-camera", "Manual test camera"}; }
    CameraState state() const override { return state_; }
    void start() override { ++startRequests; }
    void stop() override { ++stopRequests; }

    void publishState(CameraState state, Timestamp at)
    {
        state_ = state;
        if (auto sink = sink_.lock()) sink->onCameraStateChanged(state, at);
    }
    void publishFrame(FramePtr frame)
    {
        if (auto sink = sink_.lock()) sink->onFrame(std::move(frame));
    }
    void publishError(const CameraError& error)
    {
        if (auto sink = sink_.lock()) sink->onCameraError(error);
    }

    int startRequests{};
    int stopRequests{};

private:
    CameraState state_{CameraState::Stopped};
    std::weak_ptr<ICameraEvents> sink_;
};

class ManualTransport final : public IRobotTransport {
public:
    void setEventSink(std::weak_ptr<IRobotTransportEvents> sink) override { sink_ = std::move(sink); }
    ConnectionState connectionState(RobotId id) const override
    {
        const auto found = states_.find(id);
        return found == states_.end() ? ConnectionState::Disconnected : found->second;
    }
    void connect(RobotId id) override { connectRequests.push_back(id); }
    void disconnect(RobotId id) override { disconnectRequests.push_back(id); }
    void sendCommand(RobotId id, const RobotCommand& command) override
    {
        commands.emplace_back(id, command);
    }
    void publishState(RobotId id, ConnectionState state, Timestamp at)
    {
        states_.insert_or_assign(id, state);
        if (auto sink = sink_.lock()) sink->onConnectionStateChanged(id, state, at);
    }
    void publishTelemetry(const RobotTelemetry& telemetry)
    {
        if (auto sink = sink_.lock()) sink->onTelemetry(telemetry);
    }

    std::vector<RobotId> connectRequests;
    std::vector<RobotId> disconnectRequests;
    std::vector<std::pair<RobotId, RobotCommand>> commands;

private:
    std::weak_ptr<IRobotTransportEvents> sink_;
    std::map<RobotId, ConnectionState> states_;
};

} // namespace micro_robot::test
