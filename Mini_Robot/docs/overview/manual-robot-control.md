# Manual robot control and connection panel

Issue [#69](https://github.com/Krzys2002/Mini_Robot/issues/69) adds the persistent
robot panel in [RobotControlPanel](../../src/ui/RobotControlPanel.hpp). Widgets
forward operator intent to [DesktopController](../../src/app/DesktopController.hpp),
which routes commands through [RobotManager](robot-manager.md). Simulation and
BLE use this same service API. No radio object or motor algorithm enters a widget.

## Operator workflow

1. Launch normally for simulation, or use `--ble` in a build configured with
   `-DMICRO_ROBOT_BUILD_BLE=ON`. Startup scans; **Scan robots** refreshes the list.
2. Choose a robot by ID. The list shows advertised names, connection state and
   conflicting IDs. A configured/registered robot remains listed when absent from
   the latest scan; its label does not imply it was discovered. BLE verifies Robot
   Info before reporting Connected and refuses ambiguous identities.
3. **Connect** or **Disconnect** the selected robot. Pending connections can be
   cancelled. Motion is enabled only for a requested, connected, non-faulted
   session. Battery voltage and reported operating status refer to that selection;
   unavailable values stay explicitly unknown. Receipt time is monotonic elapsed
   application time; telemetry is not wheel feedback or a localization estimate.
4. Set linear/angular speed and hold **Forward**, **Backward**, **Turn CCW** or
   **Turn CW** using the mouse or Space on the focused button. Release to send an
   explicit Stop. Directions use robot-frame metres/second and radians/second;
   positive rotation is counterclockwise.
5. **Stop** sends an ordinary Stop to the selected connected robot. **E-STOP ALL
   ROBOTS** requests the dedicated latch for every registered session, including
   non-selected/disconnected robots, and inhibits further motion. It stays
   accessible during faults/disconnection when the transport supports it. An
   unsupported transport disables E-stop; ordinary Stop is never labelled E-stop.

Stop/E-stop remain outside the scrollable camera/world content. Switching robot,
scanning, speed changes, window deactivation, focus loss, hiding/disabling the
panel, window close and controller shutdown cancel held motion. A reconnect never
resumes a previous hold. A transport error or reported robot fault/E-stop inhibits
manual drive. Ordinary transport-error recovery requires disconnect/reconnect;
repeated Connect on an already connected session cannot clear that inhibition.

## Bounds, timing and limitations

`ManualControlConfig` is injected into the controller. Defaults are software
bring-up ceilings of **0.1 m/s** and **1 rad/s**, with a **100 ms** refresh interval
and **250 ms** command validity. UI ranges and service-level clamping enforce both
signs; non-finite inputs stop/reject motion. Invalid configuration is rejected.
The owner-thread Qt timer refreshes only while input is held, replacing unsent
motion rather than building a command backlog. A blocked desktop event loop cannot
refresh commands: dispatch-time expiry and the robot watchdog remain necessary.

These software ceilings do **not** establish safe physical PWM, current or voltage
limits. Physical motion requires the hardware/supply/actuator acceptance in
[#154](https://github.com/Krzys2002/Mini_Robot/issues/154) and BLE acceptance in
[#189](https://github.com/Krzys2002/Mini_Robot/issues/189). No powered bench testing
was performed for this change.

The optional `IRobotTransport::supportsEmergencyStop()` / `requestEmergencyStop()`
capability exposes BLE's existing dedicated, retried latch primitive through the
manager. Simulation implements the same persistent per-robot intent. The manager
refuses Velocity after the request, including after reconnect. No clear/re-arm UI
is exposed; #70 owns broader safety policy and recovery. The UI says **requested**
and **physical execution unconfirmed**: submission is not delivery or physical
stopping. Restarting the desktop is not a verified robot-side recovery procedure.

Stop is submitted before disconnect/shutdown, but adapters may discard pending
commands during teardown. Robot stop-on-disconnect/watchdog is still required;
this change adds no acknowledgement wait or physical stop guarantee. Only the
selected robot accepts manual input; E-stop covers all registered sessions, not
unknown/unregistered robots visible only in discovery.

## Verification

Hardware-free tests cover connection/pending/fault gating, finite signed bounds,
mouse/keyboard hold/release, refresh cancellation, focus/deactivation/hide/disable,
selection and telemetry isolation, duplicate IDs, simulation E-stop persistence,
small-window access and the real BLE adapter with a fake native backend. Core
manager tests cover unsupported capability and latch priority/reconnect/isolation.

```sh
ctest --test-dir build/macos -R 'RobotControlPanelTest|ManualServiceBounds|RobotManagerTest' --output-on-failure --no-tests=error
```

Remaining manual checks: real-display mouse/keyboard input and scaling on macOS,
Linux X11/Wayland and Windows; physical forward/reverse/turn polarity, release
stop, disconnect/watchdog and E-stop; actual battery/status telemetry. Automated
offscreen/fake-radio success does not establish those results. Keep #69 open
pending review and any required physical acceptance; no issue is closed here.
