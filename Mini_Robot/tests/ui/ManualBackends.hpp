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
    explicit ManualCamera(CameraInfo info = {"manual-camera", "Manual test camera"})
        : info_(std::move(info)) {}
    ~ManualCamera() override { *alive = false; }
    ManualCamera(const ManualCamera&) = delete;
    ManualCamera& operator=(const ManualCamera&) = delete;

    void setEventSink(std::weak_ptr<ICameraEvents> sink) override { sink_ = std::move(sink); }
    CameraInfo info() const override { return info_; }
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

    [[nodiscard]] bool hasSink() const { return !sink_.expired(); }

    int startRequests{};
    int stopRequests{};
    // Copy before handing ownership away to observe destruction afterwards.
    std::shared_ptr<bool> alive{std::make_shared<bool>(true)};

private:
    CameraInfo info_;
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
