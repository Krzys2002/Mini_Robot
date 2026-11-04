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
    // Simulated advertised limits, matching the v1 millisecond/scaled-SI contract.
    double maximumLinearMetersPerSecond{32.767};
    double maximumAngularRadiansPerSecond{32.767};
    std::chrono::milliseconds maximumCommandValidity{500};
};

enum class SimulationRobotState { Idle, Moving };
struct SimulationRobotStatus {
    SimulationRobotState state{SimulationRobotState::Idle};
    bool commandExpired{};
    std::optional<std::uint16_t> lastAcceptedSequence;
};

// Deterministic domain-level transport; no radio or robot physics.
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
    void cancelPendingMotion(RobotId id) override;
    [[nodiscard]] bool supportsEmergencyStop() const noexcept override { return true; }
    void requestEmergencyStop(RobotId id) override;
    [[nodiscard]] bool supportsEmergencyClear() const noexcept override { return true; }
    bool requestEmergencyClear(RobotId id, std::uint64_t request) override;

    // Harness-only scenarios, deferred to the next advancement. They cancel
    // connection intent and pending/active motion; no automatic reconnect.
    // Repeated injections coalesce; callbacks must obey the same owning-thread rules.
    void simulateDisconnect(RobotId id);
    void simulateFault(RobotId id);
    // Diagnostic simulated device state, not an authoritative pose or a new
    // domain/UI contract. Empty outside a connected session.
    [[nodiscard]] std::optional<SimulationRobotStatus> robotStatus(RobotId id) const;

    // Nondecreasing virtual time, no recursive calls. Periodic telemetry is emitted
    // at most once per new timestamp per robot, in RobotId order: on the first
    // advancement of a session, then once telemetryInterval has elapsed since the
    // previous sample. State/watchdog changes bypass cadence. One pending command
    // per robot: newer submissions supersede older ones, except a queued Stop.
    void advanceTo(Timestamp now);
    [[nodiscard]] std::optional<RobotCommand> lastSentCommand(RobotId id) const;

private:
    struct Session {
        bool desiredConnected{};
        bool emergencyStopRequested{};
        std::optional<std::uint64_t> clearRequest;
        ConnectionState state{ConnectionState::Disconnected};
        std::optional<RobotCommand> pendingCommand;
        std::optional<RobotCommand> lastSentCommand;
        std::optional<Timestamp> lastTelemetryAt;
        std::optional<ConnectionState> pendingFailure;
        SimulationRobotStatus status;
        std::uint16_t transmitSequence{};
        std::uint64_t telemetrySequence{};
        std::optional<Timestamp> motionStartedAt;
        Duration motionValidity{};
        VelocityCommand velocity;
        bool statusChanged{};
    };

    SimulationTransportConfig config_;
    std::optional<LogChannel> log_;
    std::weak_ptr<IRobotTransportEvents> sink_;
    std::map<RobotId, Session> sessions_;
    std::optional<Timestamp> lastAdvance_;
    bool advancing_{};
};

} // namespace micro_robot
