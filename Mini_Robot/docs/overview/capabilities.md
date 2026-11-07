# Current capability and integration limits

This is the concise current-capability summary. Architecture documents own shared
contracts; module/behavior guides own API details and dated verification evidence.
Issue closure, milestone counts and successful component tests do not establish
end-to-end or physical acceptance. Historical proposals are not source inventories.

| Area | Implemented | Integration/acceptance still required |
| --- | --- | --- |
| Camera | Native Qt and Linux V4L2 discovery, selection, bounded capture; deterministic replay adapter | Normal replay selection workflow, broader physical-device/recovery acceptance |
| Intrinsics | Capture, solving, exact camera/resolution profile save/load and undistorted preview; physical-identity gate for Linux persistence; remove solved views and recalculate | Optical/measurement accuracy for each camera/lens; missing/ambiguous identity stays session-only |
| Vision | AprilTag detection, RobotId association, pixel observations, calibrated metric measurement boundary and matched-frame debug overlays | Physical metric-pose acceptance (#224) |
| World | One authoritative robot registry, metric workspace, tracked poses, selectable exponential/Kalman filtering, motion and age/reliability diagnostics | Physical tracking acceptance, obstacles and planning state |
| Communication | Native BLE command/telemetry adapter, RobotManager sessions and deterministic simulation with shared pending Stop priority | Production command/safety interoperability, over-air measurements and remaining multi-link hardware checks |
| Robot firmware | Legacy HTTP prototype; independent actuator expiry and generation gate, clear-to-idle semantics and ordered browser intent | Production BLE runtime/protocol, hardware-safe boot/disable, bounded deceleration and measured watchdog/E-stop acceptance (#158/#162) |
| Control/planning | Pure differential-drive math and transport-level safety primitives | Domain operational safety, manual commands UI, closed-loop controller, paths, trajectories and coordination |
| Recording | Human-readable application logs | Structured experiment records and validated end-to-end latency/error measurements |

The running desktop routes frames to calibration/detection/debug presentation.
Calibrated observations feed tracked WorldModel poses with explicit age and loss
state. They do not produce controller commands; physical accuracy is unverified.
The controller → robot portion of the closed-loop chain remains roadmap work.
Camera receipt-time estimates and synchronous log writes/flushes are not a
validated latency-measurement pipeline. BLE-specific state/diagnostics do not yet
constitute transport-independent operational safety.

## Evidence owners

- [Camera streaming](camera-streaming.md#verification) retains dated macOS capture
  and Continuity-camera checks; [camera backends](camera-backends.md#verification)
  retains dated Linux capture/mode evidence and unrun external-device cases.
- [Intrinsic profiles](calibration-profiles.md) owns save/load, physical identity,
  recalculation behavior and optical/manual limitations.
- [Vision debug](../vision/debug-overlay.md) owns synthetic rendered checks and
  separate physical overlay acceptance (#211); detection commissioning remains #191.
- [BLE lifecycle](ble-connection.md) and [messages](ble-messages.md) own adapter
  acceptance. PR [#213](https://github.com/Krzys2002/Mini_Robot/pull/213) retains
  reported physical connection evidence and the full-discovery correction; this
  connection-only fixture does not establish motors, watchdog or E-stop behavior.
- [Legacy firmware](../../../T1-ESP-C6/README.md) owns prototype safety corrections;
  physical stopping for this audit patch is NOT RUN.
- [Test/CI recipes](../development/testing.md) distinguish the affordable PR job,
  daily platform matrix, local tests and hardware checks. Billing/spending limits
  are GitHub account settings and cannot be repaired by repository code.
