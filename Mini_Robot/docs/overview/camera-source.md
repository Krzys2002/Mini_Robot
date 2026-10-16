# Camera source contract

`ICameraSource` and `ICameraEvents` in `micro_robot_camera_api` depend only on
C++20 and common monotonic time types. They expose no Qt objects, UI widgets,
physical camera handles, or perception algorithms. Live, recorded, and simulated
sources can implement the same boundary; only composition code selects a concrete
implementation. `DesktopController` owns the active source, which an
[ICameraSourceFactory](camera-selection.md) creates for the selected device, and
`MainWindow` interacts with controller signals/slots.

Source: [ICameraSource.hpp](../../src/camera/ICameraSource.hpp),
[CameraTypes.hpp](../../src/camera/CameraTypes.hpp), and
[camera tests](../../tests/camera/CameraTypesTest.cpp).

## Operations and lifecycle

`info()` returns an owned `CameraInfo` containing an opaque source ID stable for
the object's lifetime and a display name. Frame identity is scoped to that source;
consumers handling several sources must associate each event sink with its source
ID. `state()` returns the current state without device I/O.

`start()` and `stop()` are prompt, asynchronous, idempotent requests. They never
dispatch events inline. Stop cancels pending startup and suppresses subsequent
frames once shutdown completes. The source starts in `Stopped`:

| State | Meaning |
| --- | --- |
| `Stopped` | Quiescent; ready for a start request. |
| `Starting` | Optional pending startup state. |
| `Streaming` | Startup completed; valid frames may be delivered. |
| `Stopping` | Optional pending shutdown state. |
| `Disconnected` | Source unavailable; no frames or pending capture work. |
| `Fault` | Capture failed; no frames or pending capture work. |

State queries reflect the new state before its event is delivered. Immediate
backends may transition directly between `Stopped` and `Streaming`. A Streaming
event precedes the first frame. Start from `Disconnected`/`Fault` may attempt
recovery; unsuccessful recovery reports an error asynchronously. Stop on an
already quiescent terminal source may preserve that state.

`CameraError` carries a portable code (`Unavailable`, `PermissionDenied`,
`InvalidFrame`, or `BackendFailure`), diagnostic message, and local monotonic
timestamp. Error callbacks do not themselves change state. Recoverable invalid
frames are dropped with `InvalidFrame`; fatal failures also emit `Disconnected`
or `Fault`. Consumers copy errors if they need them after the callback.

## Frames and ownership

`onFrame(FramePtr)` delivers a shared immutable, owned `Frame`:

- Sequence increases across the source lifetime, including restarts; gaps reflect
  skipped frames. It is not a device protocol counter.
- `capturedAt` uses the application's local steady clock, or supplied virtual
  time in deterministic simulation. Replay/media/device clocks must be mapped to
  that domain; the timestamp is not wall time or a persisted media offset.
- Width and height are positive pixel counts. The origin is top-left, +u right,
  +v down. Frames do not contain metric world coordinates or detections.
- `Gray8`, `Rgb8`, and `Rgba8` are packed unsigned 8-bit channels in that order;
  RGBA uses straight alpha. `strideBytes` includes optional row padding.
- `pixels` owns at least `strideBytes * height` bytes, including final-row padding.
  `isValidFrame()` checks format, dimensions, stride, and storage without overflow,
  allocation, or exceptions. Sequence/time ordering remains the source's duty.

Consumers may retain a frame or its pixel storage after the callback, after newer
frames, and after source destruction. Producers must never mutate the published
frame or buffer through another alias. Copy borrowed capture buffers or transfer
exclusive ownership before publishing; `shared_ptr<const ...>` alone does not
freeze a separate mutable alias. Future Qt/OpenCV conversions belong in adapters.

## Threading and subscriptions

Each source is thread-affine, with all public calls and callbacks serialized on
its owning thread. It is not thread-safe. Capture workers marshal results to that
thread; a Qt application adapter bridges to GUI consumers on the GUI thread.
No camera read, blocking wait, or sleep belongs in the GUI event path.

Sources retain only a weak event sink, locking it per dispatch. Empty or expired
sinks skip delivery. Replacing a sink affects the next event, while the current
callback temporarily retains its recipient. Callbacks must be short and must not
throw or destroy the source. They may request start/stop or replace the sink,
without causing nested dispatch.

Frame handoffs use bounded buffering and prefer the latest useful frame. Dropped
frames may produce sequence gaps; worker or queued GUI delivery must not accumulate
an unbounded backlog. This is an implementation obligation, not a queue provided
by the abstract interface.

The owner requests stop, asynchronously awaits `Stopped`, `Disconnected`, or
`Fault`, then destroys the source on its owning thread outside a callback.
Terminal states mean capture work has quiesced. Destruction cancels remaining work
and leaves no outstanding callbacks. Event sinks can be detached independently.

## Verification and scope

Camera-only tests validate packed metadata and malformed/overflow cases without
Qt or concrete backends. Existing interface and simulation tests exercise frame
retention, weak subscription lifetimes, deferred requests, cancellation, and
polymorphism. GUI integration tests inject both `FakeCameraSource` and a test-only
manual source into the same controller/window, including pending and failure
states. Tests do not prove future physical backend scheduling behavior.

Issue #26 defines this interface. [Read-only discovery](camera-discovery.md) is
implemented separately by #29. [Recorded-video replay](recorded-video.md) is
implemented by #47 through this same interface. [Device selection](camera-selection.md)
(#30) creates sources through a factory. Physical capture, pixel preview, and GUI
worker adapters remain separate issue work.
