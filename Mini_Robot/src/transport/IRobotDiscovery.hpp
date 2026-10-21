#pragma once

#include "common/RobotId.hpp"
#include "common/Time.hpp"

#include <memory>
#include <string>
#include <vector>

namespace micro_robot {

// One reachable robot as seen by a radio scan. robotId is the identity the robot
// advertises; it is a hint until a transport session verifies it on connect.
// No radio address, platform peripheral identifier, or service UUID belongs here:
// the backend maps RobotId to its own peer identity privately.
struct DiscoveredRobot {
    RobotId robotId;
    std::string displayName; // Advertised name, display hint only; may be empty.
    Timestamp lastSeenAt; // Desktop receipt time of the latest advertisement.
    // More than one peer claims robotId. The backend does not choose one; a
    // connect request for this ID fails until the conflict disappears.
    bool ambiguous{};
};

enum class RobotScanState { Idle, Scanning };

enum class RobotDiscoveryErrorCode {
    Unavailable, // No usable adapter or service, e.g. powered off or not running.
    PermissionDenied, // The operating system refused radio access.
    BackendFailure,
};

struct RobotDiscoveryError {
    RobotDiscoveryErrorCode code;
    std::string message;
    Timestamp occurredAt;
};

class IRobotDiscoveryEvents {
public:
    virtual ~IRobotDiscoveryEvents() = default;
    // scanState() already reflects this state when the callback runs.
    virtual void onScanStateChanged(RobotScanState state, Timestamp changedAt) = 0;
    // Complete value snapshot ordered by RobotId, one entry per RobotId. An empty
    // list is a valid result.
    virtual void onRobotsChanged(std::vector<DiscoveredRobot> robots) = 0;
    // Diagnostic. A failure that ends a scan also publishes Idle; a refused start
    // reports only the error and leaves the state Idle.
    virtual void onDiscoveryError(const RobotDiscoveryError& error) = 0;
};

// Thread-affine robot inventory, independent of BLE, Zigbee, and platform APIs.
// Discovery never connects: IRobotTransport::connect(RobotId) does, using the
// backend's private mapping from the most recent scan. Calls and callbacks are
// serialized on the owning thread. Requests return promptly; events are deferred,
// never invoked inline from a request. Recipients must be prompt/non-throwing and
// must not destroy the backend in a callback.
class IRobotDiscovery {
public:
    // Destroy on the owning thread, outside callbacks. Cancel pending delivery
    // and stop any scan in progress.
    virtual ~IRobotDiscovery() = default;
    // Weak subscription; empty detaches. No inline replay when changing sinks.
    virtual void setEventSink(std::weak_ptr<IRobotDiscoveryEvents> sink) = 0;
    [[nodiscard]] virtual RobotScanState scanState() const = 0;
    // Latest snapshot. Kept after a scan ends; the first snapshot of a new scan
    // replaces it.
    [[nodiscard]] virtual std::vector<DiscoveredRobot> robots() const = 0;
    // Idempotent. A backend may bound the scan duration and publish Idle itself.
    virtual void startScan() = 0;
    // Idempotent; cancels a pending start.
    virtual void stopScan() = 0;
};

} // namespace micro_robot
