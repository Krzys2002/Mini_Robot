# Desktop demo and application lifecycle

Source: [main.cpp](../../src/app/main.cpp),
[DesktopController.hpp](../../src/app/DesktopController.hpp),
[DesktopController.cpp](../../src/app/DesktopController.cpp),
[MainWindow.hpp](../../src/ui/MainWindow.hpp), and
[MainWindow.cpp](../../src/ui/MainWindow.cpp).

Build and launch using the [development guide](../development/build.md).
The current application demonstrates the implemented M1 foundation. It requires no camera,
Bluetooth adapter, calibration, ESP32, or configuration file.

## System camera inventory

The Available system cameras panel displays each device name and opaque ID. Its
initial “Looking for cameras…” message is replaced by an asynchronous snapshot;
zero devices show “No cameras available”. Device changes refresh the full list.
This panel is read-only: it cannot select a device or start physical capture.
Start camera still controls FakeCameraSource regardless of the inventory. See
[camera discovery](camera-discovery.md) for identifiers, logging, and manual checks.

## Using the window

1. Startup shows Ready, a stopped fake camera, disconnected robot, and registered
   Robot 1 with an unknown pose.
2. Start camera enables the fake source. The display shows frame sequence,
   8 by 6 dimensions, and elapsed capture time. Pixels are not rendered as video.
3. Stop camera disables frame production. The last frame description remains.
4. Connect robot establishes a simulated session and displays 3.7 V telemetry.
   The composition supplies a static world pose of (0.25 m, 0.15 m, 0 rad).
5. Disconnect robot ends telemetry; the last stored pose remains visible. A real
   pending connection can be cancelled through the same disconnect button.
6. Quit stops the timer, requests backend shutdown, advances the fakes once to
   drain terminal events, and destroys objects in dependency-safe order.

The 100 ms timer advances both fakes independently of camera start/stop. Pose
updates occur while connected, including when the camera is stopped. They are
explicit demo observations, not measurements derived from the image or telemetry.
The UI offers no velocity command, path execution, robot selection, or E-stop.

## Ownership and threading

The composition root chooses concrete capture, transport, and discovery backends
and owns the model. CameraDiscoveryController separately presents inventory
snapshots through its injected discovery interface and detaches on destruction. Controller
references outlive the controller; its destructor detaches the shared event sink
from both weak backend subscriptions. MainWindow emits requests and displays
controller signals. Widgets use Qt parent ownership and receiver-context
connections. All current calls and callbacks are serialized on the GUI thread.

`initialize(now)` is idempotent, records startup time, registers the selected ID,
and publishes initial state. It cannot restart a shutdown controller. Camera and
connection slots act only after initialization and before shutdown.
`applyPose(id, pose, at)` delegates to WorldModel and publishes only accepted
updates; it does not infer tracking or check connection safety. `shutdown()` is
idempotent, requests camera stop and robot disconnect, and marks the application
as shutting down. It does not itself advance simulation or wait for completion.

`receiveFrame` ignores a null pointer and formats metadata; valid-frame delivery
is the backend contract. Connection/telemetry events for other robot IDs are
ignored. Active flags include pending states, allowing cancellation. Errors are
diagnostic status-bar messages, not structured experiment logs.

[UI tests](../../tests/ui/MainWindowTest.cpp) inject fake and manual backends to
verify startup, buttons, pending transitions, terminal camera recovery, telemetry,
pose presentation, shutdown, and subscription teardown with an offscreen
QApplication. [Discovery tests](../../tests/ui/CameraDiscoveryTest.cpp) also
verify inventory presentation and subscription lifetime with an injected manual
provider. Physical preview, USB visibility/hotplug, native visual inspection, Linux X11/Wayland
interaction, control safety, and standalone packaging remain unverified features.
