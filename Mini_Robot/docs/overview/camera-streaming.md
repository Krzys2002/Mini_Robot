# Camera streaming

Issue #31 implements physical camera start/stop through
[QtCameraSource](../../src/camera/QtCameraSource.hpp) and the existing
[ICameraSource](camera-source.md) / [factory](camera-selection.md) boundaries.
The desktop composition uses `QtCameraSourceFactory`; deterministic fake and
recorded-video sources remain available to tests and alternate compositions.

## Operator behavior

Select a discovered camera, then press **Start camera**. The state label changes
through Starting to Streaming, and frame metadata shows the sequence, actual
capture dimensions, and local elapsed capture time. Pixels are not yet displayed
as a video preview. Selection alone never opens a camera.

**Stop camera** cancels a pending start or stops a running stream. The label shows
Stopping until completion, then Stopped. The last frame description remains.
The window respects the layout minimum size as selection details wrap, and
stream buttons retain at least their native style’s preferred height.
Start is disabled while starting, streaming, stopping, replacing a source, or
shutting down. Stop is enabled during startup and streaming; both buttons are
disabled while stopping. Repeated requests are guarded in `DesktopController`,
including requests issued directly through its slots before backend state events.

A failure reports a persistent status-bar message and an Error record through
the controller's application log channel, then a quiescent Fault state permits
retry. Start/stop requests and state changes are logged with the source name in
request records. Device removal still follows the selection clearing rules;
automatic reconnection remains separate work in Issue #46.

On Qt 6.5 or later, camera authorization is checked/requested asynchronously on
the owner thread before capture starts. Permission denial explains that camera
access must be enabled in system settings. On older Qt versions, backend errors
provide diagnostics. macOS bundles include `NSCameraUsageDescription`. The desktop target uses
`qt_add_executable()` so Qt finalization imports the native permission plugin
selected by that plist key. Plain CMake `add_executable()` skips this step even
when the usage description is present. Missing
devices fail without prompting for permission.

## Native adapter and lifecycle

Native QCamera, QMediaCaptureSession, and QVideoSink objects are created, operated,
and released on a dedicated Qt worker thread. Creation of the source starts only
the idle worker. Capture starts after an explicit request. Frame conversion to
owned packed RGBA storage also runs on that worker, outside the GUI thread.
Both native-frame input and converted-frame delivery retain one latest frame;
slow consumers drop old frames rather than queue every image.

Events are marshalled back to the creating thread. State queries already reflect
each event, subscriptions are weak, and callbacks may request stop without nested
dispatch. Request generations suppress results from cancelled operations. Sequence
numbers increase across restarts. Timestamps use the local monotonic clock at
native-frame receipt; they are receipt-time estimates, not sensor exposure times.
A new start reconstructs the native session, including after a failure.

Stop/failure releases native resources before delivering a quiescent state. Owners
await that state before destruction or replacement. A cancelled pending start
acknowledges quiescence even if the source was still Stopped (or a terminal state);
fake and replay sources follow the same rule. The desktop keeps its event loop and
simulation timer running after the last window closes until camera stop and robot
disconnection complete, then quits. Source destruction joins the idle worker and
cancels context-bound queued events.

## Verification

[Native adapter tests](../../tests/ui/QtCameraSourceTest.cpp) cover stopped factory
creation, malformed IDs, absent-device asynchronous failure and retry, repeated
pending-start cancellation, callback thread/state guarantees, and cancellation
from a callback. They require no camera and never request capture permission. A macOS packaging
test also checks the production bundle’s usage description and linked camera
permission plugin without requesting authorization.
[Controller/window tests](../../tests/ui/MainWindowTest.cpp) cover repeated stream
cycles with deterministic frames, frozen metadata after stop, immediate request
guards, failure visibility/logging/retry, switching during pending startup, and
shutdown. [Fake](../../tests/simulation/SimulationTest.cpp) and
[replay](../../tests/camera/RecordedVideoSourceTest.cpp) tests verify cancellation
acknowledgements. These checks do not prove successful physical capture.

Manual acceptance procedure (macOS, then Linux X11/Wayland as available):

1. Launch the built application, select a real device, and press Start. Grant
   camera access if prompted. Confirm Streaming and increasing frame sequences.
2. Press Stop. Confirm Stopped and unchanged frame metadata after waiting.
3. Repeat at least five times; confirm capture resumes and the window responds.
4. Stop during Starting; confirm completion and that another start works.
5. Deny camera permission or make the device unavailable. Confirm a visible error
   and error log, then restore access and retry.
6. Switch devices during capture; confirm the replacement stays stopped. Close
   the window while streaming; confirm the process exits and releases the device.

On 2026-10-16, the maintainer confirmed successful physical capture and correct
reported resolution on macOS after the permission-plugin fix. Physical stop and
repeated cycles, further permission checks, Linux X11/Wayland, and Windows runtime
validation remain manual checks. No hardware success is claimed by automated tests.

Qt references: [QCamera](https://doc.qt.io/qt-6/qcamera.html),
[QVideoSink](https://doc.qt.io/qt-6/qvideosink.html), and
[QVideoFrame](https://doc.qt.io/qt-6/qvideoframe.html).
