#pragma once

#include "infrastructure/logging/Logger.hpp"
#include "transport/IRobotTransport.hpp"

#include <map>
#include <optional>

namespace micro_robot {

struct SimulationTransportConfig {
    // Minimum spacing between telemetry samples of one connected robot. Zero emits
    // one sample per advancement with a new timestamp. Must not be negative.
    Duration telemetryInterval{Duration::zero()};
    // Constant battery voltage reported by every simulated robot. Must be finite
    // and positive.
    double batteryVoltageVolts{3.7};
};

// Deterministic domain-level transport; no radio, protocol, or robot physics.
// All requests are deferred until advanceTo() on the owning thread.
// Optional logging: requests and accepted commands/telemetry at Debug, connection
// state changes at Info, and rejected commands at Warning. The logger must outlive
// the transport. Throws std::invalid_argument for an invalid configuration.
class SimulationTransport final : public IRobotTransport {
public:
    explicit SimulationTransport(SimulationTransportConfig config = {},
                                 std::optional<LogChannel> log = std::nullopt);
    SimulationTransport(const SimulationTransport&) = delete;
    SimulationTransport& operator=(const SimulationTransport&) = delete;
    SimulationTransport(SimulationTransport&&) = delete;
    SimulationTransport& operator=(SimulationTransport&&) = delete;

    void setEventSink(std::weak_ptr<IRobotTransportEvents> sink) override;
    [[nodiscard]] ConnectionState connectionState(RobotId id) const override;
    void connect(RobotId id) override;
    void disconnect(RobotId id) override;
    void sendCommand(RobotId id, const RobotCommand& command) override;
    [[nodiscard]] bool supportsEmergencyStop() const noexcept override { return true; }
    void requestEmergencyStop(RobotId id) override;

    // Nondecreasing virtual time, no recursive calls. Telemetry is emitted at most
    // once per new timestamp per connected robot, in RobotId order: on the first
    // advancement of a session, then once telemetryInterval has elapsed since the
    // previous sample. One pending command
    // per robot: newer submissions supersede older ones before dispatch.
    void advanceTo(Timestamp now);
    [[nodiscard]] std::optional<RobotCommand> lastSentCommand(RobotId id) const;

private:
    struct Session {
        bool desiredConnected{};
        bool emergencyStopRequested{};
        ConnectionState state{ConnectionState::Disconnected};
        std::optional<RobotCommand> pendingCommand;
        std::optional<RobotCommand> lastSentCommand;
        std::optional<Timestamp> lastTelemetryAt;
    };

    SimulationTransportConfig config_;
    std::optional<LogChannel> log_;
    std::weak_ptr<IRobotTransportEvents> sink_;
    std::map<RobotId, Session> sessions_;
    std::optional<Timestamp> lastAdvance_;
    bool advancing_{};
};

} // namespace micro_robot
