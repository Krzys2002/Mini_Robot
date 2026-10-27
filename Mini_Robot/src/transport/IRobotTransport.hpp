#pragma once

#include "transport/TransportTypes.hpp"

#include <memory>

namespace micro_robot {

class IRobotTransportEvents {
public:
    virtual ~IRobotTransportEvents() = default;
    // connectionState(id) already reflects this state when the callback runs.
    virtual void onConnectionStateChanged(RobotId id, ConnectionState state,
                                          Timestamp changedAt) = 0;
    // Publish only supported domain versions. Adapters report UnsupportedVersion
    // with ReceiveTelemetry rather than handing unknown values to consumers.
    virtual void onTelemetry(const RobotTelemetry& telemetry) = 0;
    // Diagnostic only; an error does not implicitly change connection state.
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
    // Fault is quiescent; connect may attempt recovery. Terminal failures emit
    // Fault as well as an error; no pending work remains for that robot. Disconnect
    // on a quiescent Fault may preserve Fault. Callback requests/sink replacement
    // take effect on subsequent dispatches without nested callbacks.
    // Backends that locate robots through IRobotDiscovery fail connect with
    // Unavailable (and Fault) when the ID was not discovered or is ambiguous.
    virtual void connect(RobotId id) = 0;
    virtual void disconnect(RobotId id) = 0;

    // Bounded pending intent: newest Velocity replaces older unsent Velocity;
    // Stop replaces any unsent intent and cannot be overwritten by Velocity
    // before dispatch. Stop is an ordinary command, not an emergency-stop latch.
    // Copy the command before returning if retained for asynchronous dispatch.
    // Return does not mean delivery or execution. Invalid/stale commands or a
    // disconnected destination report an error event. Backend validates finite
    // setpoints, supported domain modelVersion, and positive validFor, and checks
    // freshness again before dispatch. Unsupported versions report
    // UnsupportedVersion without accepting the command. Metadata is copied too;
    // reserved sequence values do not imply acknowledgement/deduplication/order.
    // Wire versions, sequence allocation, acknowledgments, and encoding belong
    // to adapters; callers need not assign a sequence to submit a domain command.
    virtual void sendCommand(RobotId id, const RobotCommand& command) = 0;
};

} // namespace micro_robot
