# Desktop demo and application lifecycle

Source: [main.cpp](../../src/app/main.cpp),
[DesktopController.hpp](../../src/app/DesktopController.hpp),
[DesktopController.cpp](../../src/app/DesktopController.cpp),
[CameraSelectionController.cpp](../../src/app/CameraSelectionController.cpp),
[MainWindow.hpp](../../src/ui/MainWindow.hpp), and
[MainWindow.cpp](../../src/ui/MainWindow.cpp).

Build and launch using the [development guide](../development/build.md).
The current application demonstrates the implemented M1 foundation. It requires no camera,
Bluetooth adapter, calibration, ESP32, or configuration file. An optional
[configuration file](configuration.md) selects the robot ID, simulation timing,
and logging options.

## Camera selection

The Camera selection panel lists every discovered device in a drop-down. Its
initial “Looking for cameras…” placeholder is replaced by an asynchronous
snapshot; zero devices show “No cameras available”. Device changes refresh the
full list. Devices without a usable ID are listed but disabled. Choosing a device
makes it the active camera source; until physical capture exists (#31) that
source is a simulated feed named after the device. See
[camera selection](camera-selection.md) and [camera discovery](camera-discovery.md)
for identifiers, replacement rules, logging, and manual checks.

## Using the window

1. Startup shows Ready, no camera selected (camera buttons disabled), a
   disconnected robot, and registered Robot 1 with an unknown pose.
2. Select a camera. The Active camera source panel names it and shows Stopped.
3. Start camera enables the source. The display shows frame sequence,
   8 by 6 dimensions, and elapsed capture time. Pixels are not rendered as video.
4. Stop camera disables frame production. The last frame description remains.
   Selecting another camera stops a running source first; the new one starts
   stopped. Removing the selected device clears the selection.
5. Connect robot establishes a simulated session and displays 3.7 V telemetry.
   The composition supplies a static world pose of (0.25 m, 0.15 m, 0 rad).
6. Disconnect robot ends telemetry; the last stored pose remains visible. A real
   pending connection can be cancelled through the same disconnect button.
7. Quit stops the timer, requests backend shutdown, advances the fakes once to
   drain terminal events, and destroys objects in dependency-safe order.

Startup, shutdown, controller state changes, and errors are written to stderr
and to a per-user log file whose path is printed at startup; see
[Application logging](logging.md).

The 100 ms timer advances both fakes independently of camera start/stop. Pose
updates occur while connected, including when the camera is stopped. They are
explicit demo observations, not measurements derived from the image or telemetry.
The UI offers no velocity command, path execution, robot selection, or E-stop.

## Ownership and threading

The composition root chooses the concrete camera factory, transport, and
discovery backends and owns the model. CameraSelectionController presents
inventory snapshots through its injected discovery interface, creates sources
through the injected factory, and detaches on destruction. DesktopController owns
the active camera source; the factory and other references outlive it. Its
destructor detaches the shared event sink from both weak backend subscriptions. MainWindow emits requests and displays
controller signals. Widgets use Qt parent ownership and receiver-context
connections. All current calls and callbacks are serialized on the GUI thread.

`initialize(now)` is idempotent, records startup time, registers the configured
robot ID, and publishes initial state. It cannot restart a shutdown controller.
Camera and connection slots act only after initialization and before shutdown;
camera slots also need an active source. `setCameraSource(source)` replaces the
active source as described in [camera selection](camera-selection.md).
`applyPose(id, pose, at)` delegates to WorldModel and publishes only accepted
updates; it does not infer tracking or check connection safety. `shutdown()` is
idempotent, discards a pending camera replacement, requests camera stop and robot
disconnect, and marks the application as shutting down. It does not itself advance simulation or wait for completion.

`receiveFrame` ignores a null pointer and formats metadata; valid-frame delivery
is the backend contract. Connection/telemetry events for other robot IDs are
ignored. Active flags include pending states, allowing cancellation. Errors are
shown as status-bar messages and logged at Error level through the controller's
injected `LogChannel`; these are operational logs, not structured experiment data.

[UI tests](../../tests/ui/MainWindowTest.cpp) inject fake and manual backends to
verify startup, buttons, pending transitions, terminal camera recovery, telemetry,
pose presentation, shutdown, and subscription teardown with an offscreen
QApplication, plus active-source replacement. [Selection tests](../../tests/ui/CameraSelectionControllerTest.cpp)
also verify device listing, selection, removal, and subscription lifetime with
injected manual discovery and factory backends. Physical preview, USB visibility/hotplug, native visual inspection, Linux X11/Wayland
interaction, control safety, and standalone packaging remain unverified features.
