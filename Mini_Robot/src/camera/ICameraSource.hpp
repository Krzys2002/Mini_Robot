#pragma once

#include "camera/CameraTypes.hpp"

#include <memory>

namespace micro_robot {

class ICameraEvents {
public:
    virtual ~ICameraEvents() = default;
    // Owned immutable value; safe to retain after this callback and source teardown.
    virtual void onFrame(FramePtr frame) = 0;
    // state() already reflects state when this callback runs.
    virtual void onCameraStateChanged(CameraState state, Timestamp changedAt) = 0;
    // Borrowed only for this callback; copy to retain. An error is diagnostic,
    // not an implicit state change. Terminal failures also emit a state event.
    virtual void onCameraError(const CameraError& error) = 0;
};

// Thread-affine asynchronous contract; no QObject, capture library, or device API.
// Calls and event dispatch are serialized on the source's owning thread. Worker
// results must be marshalled there. Request methods return promptly and never
// invoke events inline. Completion/failure is reported through state/error events.
// Event handlers must return promptly and must not throw.
// The interface is not thread-safe. Callbacks may request start/stop or replace
// the sink; those requests must not cause nested event dispatch. GUI consumers
// need an adapter on their thread, never a direct callback from a capture worker.
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
    // Cancelling a pending start emits a quiescent state event even if the state
    // is unchanged, so callers can await cancellation before source teardown.
    // Initial state is Stopped. Starting/Stopping are optional pending states;
    // successful start completes in Streaming, stop completes in Stopped.
    // Disconnected means the source is unavailable; Fault means capture failed.
    // Both are quiescent terminal states (no more frames/pending work). start()
    // may attempt recovery from either; failure is reported asynchronously.
    // stop() on a quiescent terminal source may leave that terminal state intact.
    virtual void start() = 0;
    virtual void stop() = 0;

    // Emit only non-null frames passing isValidFrame() while Streaming, after the
    // Streaming state event. Invalid frames report InvalidFrame and are dropped;
    // recoverable errors may leave the source Streaming. Frame handoff must use a
    // bounded latest-frame queue, dropping old frames instead of accumulating lag.
};

} // namespace micro_robot
