#pragma once

#include "infrastructure/logging/Logger.hpp"
#include "transport/IRobotTransport.hpp"
#include "world/WorldModel.hpp"

#include <map>
#include <memory>
#include <optional>

namespace micro_robot {
struct RobotSession {
    RobotId robotId;
    ConnectionState connection{ConnectionState::Disconnected};
    std::optional<Timestamp> connectionChangedAt{};
    std::optional<RobotTelemetry> telemetry{};
    std::optional<TransportError> lastError{};
    bool connectRequested{};
    bool removing{};
};
enum class CommandSubmission { Submitted, UnknownRobot, NotConnected, ShuttingDown };

// Application service, owning-thread only; no Qt, radio handles or pose estimates.
// Exclusively coordinates one injected transport's sessions/event subscription.
// Transport, world and optional logger outlive this service. Backend selection and
// endpoint identity verification remain in composition/adapters. Observers are weak,
// prompt/nonthrowing and may request operations, but must not destroy the service
// from a callback. Snapshot/query results are detached values.
class RobotManager final {
public:
    RobotManager(IRobotTransport& transport, WorldModel& world,
                 std::optional<LogChannel> log = std::nullopt);
    ~RobotManager();
    RobotManager(const RobotManager&) = delete;
    RobotManager& operator=(const RobotManager&) = delete;
    RobotManager(RobotManager&&) = delete;
    RobotManager& operator=(RobotManager&&) = delete;

    void setEventSink(std::weak_ptr<IRobotTransportEvents> sink);
    // Explicit registration only; unknown callbacks never create robots. Existing
    // world poses/registration are preserved. Backend must be quiescent for this ID.
    [[nodiscard]] bool addRobot(RobotId id, Timestamp registeredAt);
    [[nodiscard]] bool connect(RobotId id);
    [[nodiscard]] bool disconnect(RobotId id);
    // Deferred removal while active/pending: retain subscription until a terminal
    // callback. World observations are deliberately retained, not unregistered.
    [[nodiscard]] bool removeRobot(RobotId id);
    [[nodiscard]] std::optional<RobotSession> session(RobotId id) const;
    [[nodiscard]] std::map<RobotId, RobotSession> sessions() const;
    [[nodiscard]] ConnectionState connectionState(RobotId id) const;
    // Submitted means handed to adapter, never delivered/executed. Adapter owns
    // command validation, dispatch-time freshness, wire sequence and watchdogs.
    [[nodiscard]] CommandSubmission sendCommand(RobotId id, const RobotCommand& command);
    [[nodiscard]] CommandSubmission stop(RobotId id, Timestamp issuedAt, Duration validFor);
    // Idempotent, nonblocking. Drain the backend/event loop until isStopped()
    // before destroying the transport, outside an event callback. Destruction
    // requests shutdown and safely detaches, but cannot await radio teardown.
    void shutdown();
    [[nodiscard]] bool isStopped() const;
private:
    class Events;
    void receiveState(RobotId id, ConnectionState state, Timestamp at);
    void receiveTelemetry(const RobotTelemetry& sample);
    void receiveError(const TransportError& error);
    IRobotTransport& transport_;
    WorldModel& world_;
    std::optional<LogChannel> log_;
    std::map<RobotId, RobotSession> sessions_;
    std::shared_ptr<Events> events_;
    std::weak_ptr<IRobotTransportEvents> sink_;
    bool shuttingDown_{};
};
} // namespace micro_robot
