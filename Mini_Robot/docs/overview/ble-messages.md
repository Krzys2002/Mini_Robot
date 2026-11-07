# BLE commands and telemetry

Issue #66 extends the [connection adapter](ble-connection.md) with the
[version 1 wire protocol](../protocol/robot-protocol.md). `BleProtocol` is a
Qt-free codec; `BleTransport` owns per-session policy; `QtBleBackend` owns native
GATT writes and notification receipt. Domain consumers still use
`IRobotTransport`, SI units and steady-clock timestamps. No firmware is added.

At the maintainer's request, #66 tracks software implementation and deterministic
acceptance. Physical command delivery, telemetry and adapter E-stop acceptance
are tracked separately in [#189](https://github.com/Krzys2002/Mini_Robot/issues/189).

## Commands and bounded dispatch

`sendCommand()` accepts existing `StopCommand` and `VelocityCommand` values.
Errors are asynchronous. Unsupported domain versions, disconnected sessions,
non-finite/out-of-range values, future timestamps, expired commands and invalid
durations are rejected. Encoding rounds SI velocities to signed mm/s and mrad/s
with half-away-from-zero rounding; it never silently clamps velocity.

Each robot has one pending ordinary intent and one ordinary write in progress.
New velocity replaces pending velocity; a pending Stop cannot be overwritten by
velocity. Stop bypasses velocity rate limiting. After dispatch, pending intent is
rechecked, so waiting never extends its original validity. Velocity TTL is the
remaining whole milliseconds, capped by Robot Info's advertised watchdog limit;
less than one remaining millisecond is rejected. No velocity is retransmitted.

The backend encodes again immediately before calling Qt's native write API.
This is an API handoff, not proof of over-air transmission: Qt/OS requests can
remain queued. E-stop bypasses the application's ordinary slot but cannot preempt
an ATT operation already handed to Qt. The robot watchdog remains essential;
see the protocol's [freshness rules](../protocol/robot-protocol.md#freshness-and-watchdog).

Velocity uses Command / WriteWithoutResponse. Stop uses Command /
WriteWithResponse. Native handoff and ATT confirmation are logged separately
from application `CommandResult`; neither means the robot executed a command.
Command results map robot rejection reasons to `SendCommand` transport errors.

## Adapter E-stop primitive

`BleTransport::requestEmergencyStop(id)` latches intent immediately, removes
pending ordinary intent and uses the dedicated Emergency Stop characteristic
with WriteWithResponse. It retries until a correlated Accepted result or valid
EmergencyStop telemetry confirms it, or the session ends. Native backpressure
keeps at most one unconfirmed E-stop write per endpoint. Write failure/timeout
faults and releases the session.

The intent survives disconnect/fault. On reconnect, the adapter reasserts it
before publishing Connected. Velocity remains blocked while intent is latched,
or validated robot state is EmergencyStop/Fault. There is no implicit clear.
Explicit clear is available after confirmed E-stop on a connected session. A
matching accepted Idle CommandResult is required; timeout, rejection, disconnect
or a newer E-stop retains inhibition. This does not prove physical output state.

## Notifications and domain delivery

Connection setup enables Command Result and Telemetry CCCDs before readiness.
The backend stamps incoming notifications with the desktop steady clock. The
codec validates exact length, wire version, message/channel pairing, state and
result enums before applying data. Reserved flags are ignored and unknown
measurement sentinels become empty optionals. Malformed input reports
`ReceiveTelemetry`, clears pending velocity and blocks new velocity until valid
state arrives; repeated invalid frames fault the link.

Telemetry and Command Result share one robot sequence counter. Duplicates, old
frames and the ambiguous half-range delta are dropped. Accepted wraparound is
unwrapped to 64 bits and gaps are logged. Invalid frames do not advance the
counter. Session generations discard late notifications and reset counters,
pending writes and correlation history.

Battery and measured velocities reach the event sink as `RobotTelemetry`, with
desktop receipt time and unwrapped sequence metadata. `DesktopController`
receives these through its existing interface subscription. Robot state, flags,
uptime and last accepted command are adapter diagnostics, as specified in the
[domain mapping](../protocol/robot-protocol.md#domain-mapping). Telemetry does
not fabricate or update a visual world pose. Silence monitoring belongs to #70.

## Configuration and diagnostic logs

Additional `BleTransportConfig` defaults (injected at construction):

| Field | Default | Meaning |
| --- | --- | --- |
| `minimumVelocityInterval` | 100 ms | Minimum interval between velocity handoffs |
| `emergencyStopRetry` | 100 ms | Retry interval while E-stop is unconfirmed |
| `maximumTrackedCommands` | 128 | Bounded sequence/type/time correlation history; must be below 32768 |
| `maximumInvalidNotifications` | 3 | Consecutive malformed notifications before Fault |
| `phaseTimeout` | 10 s | Also bounds native write confirmation waits |

Enable `[logging] level=debug` in the existing configuration file for robot-tagged
records with session token, wire/domain sequence pairing, handoff/receipt
`steady_us`, unwrapped receive sequence, gaps and command-result
`round_trip_us`. Unmatched results report unknown RTT. Robot uptime is diagnostic,
never a desktop timestamp. RTT includes robot processing and notification delay;
it is not a one-way radio latency measurement. Experiment aggregation remains #72.

## Verification and remaining hardware acceptance

On macOS (2026-10-20), AppleClang 21, Qt 6 and OpenCV 5.0.0:

- Configured `build/issue-66` with `MICRO_ROBOT_BUILD_BLE=ON`, Ninja and Debug;
  built the production desktop, native backend and all tests successfully.
- `ctest --test-dir build/issue-66 --output-on-failure`: **232/232 passed**, including
  **41 BLE checks** and desktop-controller telemetry/error integration.
- Qt/OpenCV-free `build/issue-66-core` (desktop/replay/calibration/BLE OFF):
  built successfully; unfiltered CTest **139/139 passed**, including eight codec tests.
- Tests cover documented bytes, scaling, sentinels, malformed frames, sequence
  wrap/gaps, coalescing, freshness at dispatch, write failures/timeouts, bounded
  result history, E-stop retries/reassertion and connection teardown.

No physical command/telemetry test was run. The original **Commands reach the
ESP32-C6** criterion and the checks below remain open in
[#189](https://github.com/Krzys2002/Mini_Robot/issues/189). Record actual physical
results there; #66's automated acceptance does not claim physical success.
Using a provisioned peripheral and a suitable test harness, record firmware/OS/Qt
versions, Robot Info limits and debug logs:

1. Send fresh velocity and Stop; capture the exact characteristic bytes, write
   modes, correlated results, measured velocities and actual motor response.
2. Request E-stop during ordinary traffic; verify outputs disable, retries end
   on confirmation, motion remains inhibited, and reconnect reasserts the latch.
3. Send malformed/version-incompatible, duplicate and wrapped notifications;
   verify rejection, errors, sequence gaps and valid telemetry recovery.
4. Queue commands through a delayed link and disconnect during writes; confirm
   expired intent is not handed off, old-session events are ignored, watchdog
   stops the robot and teardown completes.

Real radio timing/backpressure and Linux BlueZ/Windows runtime are unverified.
Connection-specific physical checks from #65 remain in
[#187](https://github.com/Krzys2002/Mini_Robot/issues/187).

Safety states in both telemetry and CommandResult notifications inhibit motion.
An E-stop report (including an EmergencyStopLatched rejection) persists through
later Idle telemetry until explicit acknowledged clear. RobotFault rejection and
Fault state reports cancel pending clear and inhibit the manager even if a late
accepted Idle clear result arrives. These use structured transport error codes;
ordinary command-validation errors do not establish a robot safety fault.
