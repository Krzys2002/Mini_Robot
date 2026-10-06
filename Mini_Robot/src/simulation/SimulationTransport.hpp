#pragma once

#include "transport/IRobotTransport.hpp"

#include <map>
#include <optional>

namespace micro_robot {

// Deterministic domain-level transport; no radio, protocol, or robot physics.
// All requests are deferred until advanceTo() on the owning thread.
class SimulationTransport final : public IRobotTransport {
public:
    SimulationTransport() = default;
    SimulationTransport(const SimulationTransport&) = delete;
    SimulationTransport& operator=(const SimulationTransport&) = delete;
    SimulationTransport(SimulationTransport&&) = delete;
    SimulationTransport& operator=(SimulationTransport&&) = delete;

    void setEventSink(std::weak_ptr<IRobotTransportEvents> sink) override;
    [[nodiscard]] ConnectionState connectionState(RobotId id) const override;
    void connect(RobotId id) override;
    void disconnect(RobotId id) override;
    void sendCommand(RobotId id, const RobotCommand& command) override;

    // Nondecreasing virtual time, no recursive calls. Telemetry is emitted once
    // per new timestamp per connected robot, in RobotId order. One pending command
    // per robot: newer submissions supersede older ones before dispatch.
    void advanceTo(Timestamp now);
    [[nodiscard]] std::optional<RobotCommand> lastSentCommand(RobotId id) const;

private:
    struct Session {
        bool desiredConnected{};
        ConnectionState state{ConnectionState::Disconnected};
        std::optional<RobotCommand> pendingCommand;
        std::optional<RobotCommand> lastSentCommand;
        std::optional<Timestamp> lastTelemetryAt;
    };

    std::weak_ptr<IRobotTransportEvents> sink_;
    std::map<RobotId, Session> sessions_;
    std::optional<Timestamp> lastAdvance_;
    bool advancing_{};
};

} // namespace micro_robot
