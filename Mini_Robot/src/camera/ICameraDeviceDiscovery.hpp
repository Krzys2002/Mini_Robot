#pragma once

#include <memory>
#include <string>
#include <vector>

namespace micro_robot {

// Device inventory only, not a capture source. id is opaque and may be empty
// when unavailable; never synthesize identity from a display name/list index.
// An adapter preserves its backend's ID, without promising cross-OS stability.
struct CameraDevice {
    std::string id;
    std::string displayName;
};

class ICameraDeviceEvents {
public:
    virtual ~ICameraDeviceEvents() = default;
    // Complete value snapshot; an empty list is a successful inventory result.
    virtual void onCameraDevicesChanged(std::vector<CameraDevice> devices) = 0;
};

// Thread-affine discovery boundary independent of Qt/native camera APIs.
// Calls and callbacks are serialized on the owning thread. refresh() returns
// promptly and reports a complete snapshot asynchronously, never inline. Device
// changes also publish complete snapshots; requests may coalesce. Recipients
// must be prompt/non-throwing and must not destroy the backend in a callback.
class ICameraDeviceDiscovery {
public:
    // Destroy on the owning thread, outside callbacks. Cancel pending delivery.
    virtual ~ICameraDeviceDiscovery() = default;
    // Weak subscription; empty detaches. No inline replay when changing sinks.
    virtual void setEventSink(std::weak_ptr<ICameraDeviceEvents> sink) = 0;
    [[nodiscard]] virtual std::vector<CameraDevice> devices() const = 0;
    virtual void refresh() = 0;
};

} // namespace micro_robot
