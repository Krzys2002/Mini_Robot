# Current capability and integration limits

This is the concise current-capability summary. Architecture documents own shared
contracts; module/behavior guides own API details and dated verification evidence.
Issue closure, milestone counts and successful component tests do not establish
end-to-end or physical acceptance. Historical proposals are not source inventories.

| Area | Implemented | Integration/acceptance still required |
| --- | --- | --- |
| Camera | Native Qt and Linux V4L2 discovery, selection, bounded capture; deterministic replay adapter | Normal replay selection workflow, broader physical-device/recovery acceptance |
| Timing/provenance | [Localization instrumentation](localization-timing.md): exact measurement bindings, bounded software latency/cadence and quality diagnostics | Commissioned physical cadence/latency/error evidence (#63/#236); current live exposure timestamps and upstream buffering remain unknown |
| Intrinsics | Capture with bounded full-resolution grayscale review storage, solving, exact camera/resolution profile save/load and undistorted preview; physical-identity gate for Linux persistence; remove solved views and recalculate | Optical/measurement accuracy for each camera/lens; missing/ambiguous identity stays session-only |
| Vision | AprilTag detection, RobotId association, pixel observations, calibrated metric measurement boundary and matched-frame debug overlays | Physical metric-pose acceptance (#224) |
| World | One authoritative robot registry, signed metric workspace with guided corners/editable origin, flat-board acquisition/provenance and v1/v2/v3 profiles, directional scale/survey-error maps, conditional prediction and two-point planar distance diagnostics, tracked pose measurements and explicit observation age | Physical scale/error/prediction coverage (#265) and distance/selection acceptance (#268), tracking acceptance, filtering, obstacles and planning state |
| Communication | Native BLE command/telemetry adapter, RobotManager sessions and deterministic simulation with shared pending Stop priority | Production command/safety interoperability, over-air measurements and remaining multi-link hardware checks |
| Robot firmware | C6 production [runtime](firmware-runtime.md) (boots disabled, one safety/actuator gate, one command authority, budget-scheduled supervisor and control tasks, no Wi-Fi/HTTP in the image), persistent RobotId and validated configuration with A/B atomic storage and a USB maintenance path ([provisioning](firmware-provisioning.md); unprovisioned or invalid records cannot arm), open-loop [bounded drive](firmware-drive.md) (body-to-wheel conversion, curvature-keeping saturation, rate/reversal limits, exact output bound, B6 stop ramp separate from immediate E-stop/fault disable; powered checks not run), independent [safety enforcement](firmware-safety.md) (session-gated admission, wrap-safe command watchdog in the supervisor, E-stop latch kept across reconnect, fault power-down of the bridges, supervisor on the task watchdog, nothing restored after reset; host-tested only), the v1 [BLE peripheral](firmware-ble.md) (GATT layout, advertising, one central, CCCD readiness, one CommandResult per frame, overload and disconnect stop; restricted-lab profile, unauthenticated), [telemetry and local diagnostics](firmware-telemetry.md) (periodic and state-change v1 Telemetry on the shared counter; battery voltage from the CodeCell GPIO4 reading under provisional bench evidence (#309, not meter-verified), measured velocity unknown; bounded rate-limited serial log with task jitter, stack and heap; USB-only bench run), with fake-driven host tests, pinned toolchain; proposed [timing budgets](../protocol/timing-budgets.md) with fake-clock timeline tests and a USB-only [timing pilot](../testing/timing-pilot-156.md); separate BLE connection fixture; [restricted-lab security profile](../protocol/security-profile.md) decided (unauthenticated v1) and enforced in the C6 firmware by a compile-time profile guard, a credential/pairing scan and host attack tests (#165; hardware attack checks not run) | A meter-validated battery conversion and low threshold to replace the provisional #309 values (#238), corrected-image B10 burst/stress evidence and B3 lateness resolution (#156/#166), second-central and radio-level attack tests (#165/#166), physically validated actuation (polarity, output bound, stopping) and measured calibration values, HIL watchdog/E-stop/reset measurements and measured safety acceptance (#164–#168, #238) |
| Height-aware localization | Measured-reference camera extrinsics, parallel tag-plane ray intersection, per-RobotId mounting transforms and explicit v4 profile upgrade | Physical per-axis accuracy, throughput and mounting survey acceptance (#224/#63); physical reference monitoring and timing quality (#261/#272) |
| Surveyed references | Separate reference registry and v5 evidence, mixed-height acquisition, strict placement consistency/drift/expiry gating and explicit revalidation | Physical reference commissioning, detection/support/accuracy and camera-loaded performance (#261/#224/#63); timing quality (#272) |
| Control/planning | Pure drive math, transport-level safety, [supervisory loop](supervisory-control.md), [goal-point policy](goal-point-controller.md), explicitly started [live metric feedback](live-visual-control.md) in simulation, and a metric-only user-drawn path editor over the calibrated raw preview | BLE live visual motion remains disabled pending physical acceptance; physical actuation/safety/loop acceptance (#167/#228/#80), measured gains/limits, path execution, trajectories and coordination |
| Recording | Human-readable application logs and per-run [JSON Lines experiment exports](control-experiment-export.md) with pose/target/error/command samples and effective controller configuration | Validated end-to-end latency/error measurements and physical experiment evidence |

The running desktop routes frames to calibration/detection/debug presentation.
Calibrated observations feed tracked WorldModel poses with explicit age and loss
state. Explicitly started visual control can submit bounded commands through
RobotManager to simulation. BLE remains available for connection and manual-drive
work; BLE live visual command submission is disabled until physical acceptance
gates pass. Physical accuracy/response/stopping remain unverified.
Localization instrumentation measures software receipt-to-output intervals; current
live timestamps remain receipt estimates with unknown upstream buffering. This is
not verified exposure-to-output or physical cadence/accuracy acceptance. BLE-specific state/diagnostics do not yet
constitute transport-independent operational safety.

Height-aware axle/body measurements require the [explicit v4 survey and mounting upgrade](elevated-localization.md). Legacy profiles remain tabletop-tag-only; physical ±5 mm and camera-loaded throughput remain unverified (#224/#63).

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
- [Firmware](../../firmware/codecell-c6/README.md) owns the production runtime
  ([structure](firmware-runtime.md), [provisioning](firmware-provisioning.md), [bounded drive](firmware-drive.md), [v1 codec](firmware-protocol.md), [safety](firmware-safety.md)); the drive bounds only the command, and no powered motor test has run;
  the #162 watchdog, E-stop and fault recovery are host-tested on a fake clock, with no HIL measurement yet (#238/#166);
  the v1 codec and command validation are served by the [BLE peripheral](firmware-ble.md) (#163), which is host-tested and was checked on a USB-only board with a BlueZ central, and since #164 it [publishes telemetry](firmware-telemetry.md) with battery and measured velocity honestly unknown;
  retired T1 prototype results do not validate this new runtime. Physical checks remain #238.
- [Test/CI recipes](../development/testing.md) distinguish the affordable PR job,
  daily platform matrix, local tests and hardware checks. Billing/spending limits
  are GitHub account settings and cannot be repaired by repository code.
