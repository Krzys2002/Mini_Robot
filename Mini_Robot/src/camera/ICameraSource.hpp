#pragma once

#include "camera/CameraTypes.hpp"

#include <memory>

namespace micro_robot {

class ICameraEvents {
public:
    virtual ~ICameraEvents() = default;
    virtual void onFrame(FramePtr frame) = 0;
    virtual void onCameraStateChanged(CameraState state, Timestamp changedAt) = 0;
    virtual void onCameraError(const CameraError& error) = 0;
};

// Thread-affine asynchronous contract; no QObject, capture library, or device API.
// Calls and event dispatch are serialized on the source's owning thread. Worker
// results must be marshalled there. Request methods return promptly and never
// invoke events inline. Completion/failure is reported through state/error events.
// Event handlers must return promptly and must not throw.
class ICameraSource {
public:
    // Owner first requests stop and awaits Stopped (or terminal Disconnected/Fault).
    // Implementations must cancel pending work and leave no callbacks outstanding
    // on destruction. Destroy on the owning thread, outside an event callback.
    virtual ~ICameraSource() = default;

    // Retain only a weak reference; lock it for each dispatch. Empty detaches.
    // Replacing a sink affects subsequent dispatches, not a callback in progress.
    virtual void setEventSink(std::weak_ptr<ICameraEvents> sink) = 0;

    [[nodiscard]] virtual CameraInfo info() const = 0;
    [[nodiscard]] virtual CameraState state() const = 0;
    // Repeated start/stop requests are idempotent. Stop cancels a pending start.
    virtual void start() = 0;
    virtual void stop() = 0;

    // Emit only valid, non-null frames while Streaming. Frame handoff must use a
    // bounded latest-frame queue, dropping old frames instead of accumulating lag.
};

} // namespace micro_robot
