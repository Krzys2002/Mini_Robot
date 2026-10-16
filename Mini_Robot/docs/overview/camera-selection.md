# Camera selection

Source: [factory interface](../../src/camera/ICameraSourceFactory.hpp),
[selection rules header](../../src/camera/CameraSelection.hpp),
[selection rules implementation](../../src/camera/CameraSelection.cpp),
[controller header](../../src/app/CameraSelectionController.hpp),
[controller implementation](../../src/app/CameraSelectionController.cpp),
[stand-in factory header](../../src/simulation/FakeCameraSourceFactory.hpp), and
[stand-in factory implementation](../../src/simulation/FakeCameraSourceFactory.cpp).

Issue #30 lets the operator choose one of the [discovered devices](camera-discovery.md)
and makes it the active [camera source](camera-source.md). Physical capture is
Issue #31: until it lands, the selected device is served by a simulated feed.

## Architecture and API

[ICameraSourceFactory](../../src/camera/ICameraSourceFactory.hpp) is a Qt-free
boundary in `micro_robot_camera_api`. `supports(device)` is a pure query that opens
nothing. `create(device)` returns a new, stopped source without an event sink, or
nullptr when the device cannot be represented. The caller owns the result and
follows the ICameraSource destruction rules. Only composition code chooses the
concrete factory, so controllers and widgets never learn the capture backend.

[classifyCameraChoices](../../src/camera/CameraSelection.hpp) turns one discovery
snapshot into choices, keeping order and every entry. Selection is keyed by the
opaque device ID, never the display name or list index, so a device is
unselectable when its ID is missing (`MissingId`), shared with another entry
(`DuplicateId`, applied to every entry with that ID), or rejected by the factory
(`Unsupported`). `findSelectableCamera` returns only `Available` devices. Both
functions live in the Qt-free `micro_robot_camera_selection` target.

[CameraSelectionController](../../src/app/CameraSelectionController.cpp) replaces the
issue #29 inventory presenter. It subscribes to discovery, publishes a complete
row list after each snapshot, and owns the current selection:

- `selectCamera(id)` ignores the current selection, creates a source for an
  available device, and passes ownership to `DesktopController::setCameraSource`.
- An unknown/unselectable ID or a factory that returns nullptr leaves the previous
  selection and source untouched, logs a warning/error, and reports a status-bar
  message. `selectionChanged` is re-emitted so the view reverts.
- After each snapshot the selection follows its ID, so a rename keeps it. When
  the ID disappears or becomes ambiguous, the selection is cleared, the active
  source is released, and the operator sees “Camera "…" is no longer available”.
  A reconnected device is listed again but is not reselected automatically.

## Active source replacement

DesktopController owns the active source; there is none until a camera is
selected, and Start/Stop are disabled meanwhile. `setCameraSource(source)`:

1. Replaces a stopped, disconnected, or faulted source immediately.
2. Asks a starting/streaming/stopping source to stop and waits for its quiescent
   state. The swap then completes through a queued call, so a source is never
   destroyed inside its own callback. Start requests are refused while waiting.
3. Lets a later call supersede a pending replacement (the pending source is
   discarded unused). `nullptr` clears the active source the same way.
4. Leaves the new source stopped and resets the frame description; it does not
   auto-start. Shutdown discards a pending replacement.

The window shows the selector, the selected device name and ID, and the active
source's display name. Unselectable rows remain visible but disabled, with the
reason in the row text and the ID in the tooltip. Same-name devices show their
ID in brackets. The empty and initial states are explicit (“No cameras
available”, “Looking for cameras…”). Choices come only from user activation;
programmatic list refreshes never request a selection.

## Stand-in capture backend

[FakeCameraSourceFactory](../../src/simulation/FakeCameraSourceFactory.cpp) supports
any device with an ID and creates a FakeCameraSource with source ID
`simulated:<device id>` and display name `<device name> (simulated feed)`. It keeps
a non-owning registry of live sources so composition can advance all of them from
one clock with `advanceTo(now)`; each source unregisters when destroyed and must
not outlive the factory. Frames are the deterministic 8 by 6 pattern described in
[simulation](simulation.md); no camera is opened and no permission is requested.
Issue #31 is expected to supply a physical factory behind the same interface.

## Logging

Selection records use the application logger's `camera` channel: an inventory
summary (`Camera inventory: N device(s), M selectable`), each selection,
rejected choices, factory failures, and removal of the selected device.
DesktopController logs the active source switch on its `controller` channel. The
discovery adapter's Qt logging category is unchanged. Persisting a preferred
device is not implemented; nothing is written to configuration.

## Verification

[Selection rule tests](../../tests/camera/CameraSelectionTest.cpp) run without Qt.
[Selection UI tests](../../tests/ui/CameraSelectionControllerTest.cpp) inject manual
discovery, factory, and camera backends into the offscreen window. They cover
empty/initial states, disabled rows and labels, selecting and reselecting,
replacing a streaming source, rejected and failing choices, removal while stopped
or streaming, reconnection, renames, ambiguous IDs, and subscription teardown.
[Window tests](../../tests/ui/MainWindowTest.cpp) cover the replacement rules
directly and [simulation tests](../../tests/simulation/SimulationTest.cpp) cover
the stand-in factory. The desktop app was launched headless on Ubuntu, where
discovery reported the integrated camera as selectable; the selector was not
operated interactively.

Manual acceptance checks:

1. Launch the app; confirm the selector lists each camera and Start is disabled.
2. Select a camera; confirm its name/ID and “(simulated feed)” source appear.
3. Start, then select another camera; confirm the stream stops and the new
   source becomes active, stopped.
4. Unplug the selected USB camera; confirm the selection clears without a crash.
5. Reconnect it; confirm it is listed again and can be selected.
