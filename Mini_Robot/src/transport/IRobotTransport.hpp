#pragma once

#include "transport/TransportTypes.hpp"

#include <memory>

namespace micro_robot {

class IRobotTransportEvents {
public:
    virtual ~IRobotTransportEvents() = default;
    virtual void onConnectionStateChanged(RobotId id, ConnectionState state,
                                          Timestamp changedAt) = 0;
    virtual void onTelemetry(const RobotTelemetry& telemetry) = 0;
    virtual void onTransportError(const TransportError& error) = 0;
};

// Calls and callbacks are serialized on the backend's owning thread. Requests
// return promptly; events are deferred, never invoked inline from a request.
// Future workers marshal results to that thread and Qt adapters queue GUI updates.
// No radio addressing, service UUIDs, encoding, or platform handles belong here.
class IRobotTransport {
public:
    // Owner disconnects active robots and asynchronously awaits terminal states
    // before destruction on the owning thread, outside an event callback.
    // Implementations cancel pending work and leave no callbacks outstanding.
    virtual ~IRobotTransport() = default;

    // Retain only a weak reference; lock for each dispatch. Empty detaches.
    // Callbacks must be short/non-throwing. Referenced event values are valid only
    // during the callback; copy them before queuing to another thread.
    virtual void setEventSink(std::weak_ptr<IRobotTransportEvents> sink) = 0;

    // Unknown IDs are Disconnected. Backend-specific ID/address mapping is private.
    [[nodiscard]] virtual ConnectionState connectionState(RobotId id) const = 0;
    // Repeated requests are idempotent; disconnect cancels a pending connect.
    virtual void connect(RobotId id) = 0;
    virtual void disconnect(RobotId id) = 0;

    // Copy the command before returning if retained for asynchronous dispatch.
    // Return does not mean delivery or execution. Invalid/stale commands or a
    // disconnected destination report an error event. Backend validates finite
    // setpoints and positive validFor, and checks freshness again before dispatch.
    // Protocol sequence numbers, acknowledgments, and encoding remain internal.
    virtual void sendCommand(RobotId id, const RobotCommand& command) = 0;
};

} // namespace micro_robot
