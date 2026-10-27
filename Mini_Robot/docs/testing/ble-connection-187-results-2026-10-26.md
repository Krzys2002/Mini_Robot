# CodeCell physical BLE results — 2026-10-26

Status: **partial physical acceptance; #187 remains open**. The original desktop
failed real CCCD validation. Switching to full GATT discovery resolved that
failure on the tested system. Repeated connections, cancellation, several
physical failures and recovery then passed. No motor or command-safety acceptance
is claimed.

## Environment and evidence

- Desktop baseline: `6979976cb1e3e16499c9c76503620e5db4128a88`, branch
  `test/187-esp32-c6-ble-fixture`; tested with the full-discovery fix in
  [QtBleBackend.cpp](../../src/transport/ble/QtBleBackend.cpp).
- macOS 26.7.1, build 25G241; Qt 6.12.0; Apple Bluetooth BCM_4388, PCIe,
  firmware 23.5.574.4422.
- One user-identified CodeCell ESP32-C6, USB `/dev/cu.usbmodem2101`, USB
  VID:PID `303A:1001`. ESP-ROM identifies ESP32-C6. Physical board revision and
  flash capacity were not independently inspected; the project targets 8 MB.
- The board was already flashed when this session started. Serial `status` and
  real Robot Info reported fixture/firmware 0.1.0, robot ID 1. No upload or flash
  readback was performed in this session; a matching local firmware binary is
  not proof of identical flashed bytes.
- Normal advertised name `MR-1`; macOS native peripheral ID
  `{3ee863c7-7209-f3e9-7192-53f63871b85c}`. A native MAC address was not exposed
  by the desktop scan API.
- Robot Info read over BLE:
  `01 01 01 01 00 00 00 00 01 00 2c 01 70 17 f4 01 64 00`.
  Protocol 1–1, ID 1, firmware 0.1.0, 300 mm/s, 6000 mrad/s, maximum validity
  500 ms, telemetry period 100 ms. These are fixture values, not motor ratings.
- Motors/GPIO were not initialized. Camera acquisition was not started.

Retained, filtered evidence:

- [Desktop phases, states, errors and shutdown](evidence/187-2026-10-26/desktop-events.txt).
- [Serial profiles, identity reads, subscriptions and physical disconnect](evidence/187-2026-10-26/serial-events.txt).
- [Actual GATT properties, CCCD comparison, native identity and Robot Info](evidence/187-2026-10-26/gatt-inspection.txt).

Raw local logs, diagnostic patch and helper script remain in
`build/acceptance-187/run-20261026/`. Temporary `qInfo` diagnostics captured native
metadata and GATT fields; those diagnostics were removed from the final source.
The behavior fix is the switch from `SkipValueDiscovery` to `FullDiscovery`.
The final build, without temporary diagnostics, also passed normal connection,
service filtering/recovery and shutdown during setup/scanning.

Desktop event times below are UTC on 2026-10-26. Serial capture times include
the local `+01:00` offset and device log entries contain milliseconds since boot.
Serial USB buffering can delay host receipt: correlate event order and device
timestamps rather than subtracting host receipt timestamps across logs.

## Defect found and correction

With `profile normal`, the original build discovered MR-1 and connected to the
real peripheral, then faulted at GATT inspection three times (09:47:16.756,
09:49:20.689, 09:51:13.669). Error:

```text
Missing characteristic, required properties or CCCD: Telemetry
```

Diagnostic inspection showed all five characteristics and the expected
Read/Write/Write Without Response/Notify properties. Qt reported both CCCDs
invalid with `SkipValueDiscovery`. On the same system/peripheral,
`FullDiscovery` reported both CCCDs valid and completed identity validation,
Command Result subscription, Telemetry subscription and Ready.

The fix retains every property/identity/descriptor check; it does not waive CCCD
validation. It requests full discovery before those checks. Full discovery can
read readable characteristics during discovery, so the firmware can log an
additional Robot Info read before the explicit identity-validation phase.
Only the explicit read is used to establish authoritative identity.

This is an observed interoperability result on the listed Qt/macOS versions;
the experiment does not determine whether the underlying cause is entirely in
Qt, CoreBluetooth, caching or their interaction. The fix is included in
[PR #213](https://github.com/Krzys2002/Mini_Robot/pull/213).

## Executed physical cases

PASS entries below use the full-discovery build. Initial failures remain visible
in the evidence; they are not counted as successful acceptance.

| Check | Result | UTC evidence / observation |
| --- | --- | --- |
| Service-filtered discovery/native metadata | PASS | MR-1/native UUID captured; wrong-service advertiser excluded below |
| GATT properties and real CCCDs | PASS after fix | Original mode reported CCCDs false; full mode reports both true |
| Authoritative identity/readiness ordering | PASS | Robot Info validated; serial result CCCD=1 precedes telemetry CCCD=1; Ready/Connected follows |
| Cycle 1 | PASS | Connected 09:52:35.329; Disconnected 09:53:08.787 |
| Cycle 2 | PASS | Connected 09:53:25.310; Disconnected 09:54:08.277 |
| Cycle 3 | PASS | Connected 09:54:27.232; Disconnected 09:54:45.263 |
| Cancel scanning | PASS | Scan 09:54:54.841; cancel 09:54:58.788; Disconnected 09:54:58.789; no late Ready before next attempt 09:55:40 |
| Wrong Robot Info ID | PASS | Fault 09:55:52.645, explicit identity mismatch; restore normal, Connected 09:57:02.363 |
| Unsupported protocol range | PASS | Raw range 2–2; Fault 09:57:48.146; restore normal, Connected 09:58:29.304 |
| Actual power loss | PASS | User removed USB and battery power; serial device disappeared; Fault 09:59:07.636 with service invalidation |
| Unreachable board | PASS | Power remained off; scan 09:59:21.977; clear not-found Fault 09:59:31.640 (~9.66 s) |
| Explicit recovery after power restoration | PASS | User restored USB; explicit Connect, fresh subscriptions and Connected 10:00:37.259 |
| Missing Command characteristic | PASS | Actual GATT Command absent; Fault 10:01:30.193; restore normal, Connected 10:02:31.274 |
| Invalid Command properties | PASS | Actual Command Read only; Fault 10:03:17.177; restore normal, Connected 10:04:01.847 |
| Telemetry without Notify | PASS | Notify/auto-CCCD absent; Fault 10:04:41.989; restore normal, Connected 10:05:31.399; this is not an isolated missing-CCCD test |
| Slow identity reads | PASS | Both deliberately delayed reads completed; Connected 10:06:58.971 (~18.70 s total setup, within individual phase bounds) |
| Cancel GATT/identity setup | PASS | ReadingIdentity 10:08:00.144; cancel 10:08:00.342; Disconnected 10:08:00.348 (~6 ms); no late Connected through next attempt 10:08:27 |
| Quit Connected | PASS | Shutdown 10:08:53.112; disconnect and clean exit 10:08:53.136 (~24 ms); firmware logs CCCDs disabled/disconnect |
| Quit during GATT setup | PASS, final build | DiscoveringCharacteristics 10:09:47.515; shutdown 10:09:50.780; clean exit 10:09:50.802 (~22 ms), no Connected in this attempt |
| Wrong service UUID | PASS, final build | Scan 10:11:17.108; not-found Fault 10:11:26.641 (~9.53 s); restore normal, Connected 10:12:06.389 |
| Quit scanning | PASS, final build | Scan 10:12:42.878; shutdown 10:12:46.096; clean exit 10:12:46.138 (~42 ms) |

The power-loss Fault occurred about 0.83 s after the last received telemetry;
this is an observation relative to the last packet, not a precisely instrumented
power-cut latency benchmark. Shutdown durations measure logged shutdown request
to logged completion; the launched processes also returned exit code 0.

The first slow-profile cancellation attempt and first attempt to close during
GATT setup missed their timing windows and reached Connected. They were repeated;
only the accurately timed attempts in the table count for those checks.

## Not run and remaining acceptance

| Check | Status / requirement |
| --- | --- |
| Actual Bluetooth permission prompt | NOT RUN — the app already had access; no prompt appeared |
| Denied permission and explicit recovery | NOT RUN — OS permission was not changed during this session |
| Bluetooth off and explicit recovery | NOT RUN — adapter power was not changed during this session |
| Duplicate advertised IDs | NOT RUN — a second independent peripheral is required |
| Notify present with only Telemetry CCCD missing | NOT RUN — separate malformed GATT fixture required |
| Notify present with only Command Result CCCD missing | NOT RUN — separate malformed GATT fixture required |
| Linux/BlueZ, X11 and Wayland | NOT RUN — macOS session only; retain Linux platform follow-ups |
| Windows | NOT RUN — retain Windows platform follow-up |
| Physical firmware hash/board revision/flash capacity | NOT RUN — runtime firmware version recorded; no readback or physical inspection |

No protocol command, motion, E-stop, watchdog, automatic reconnect or latency
acceptance is claimed. Keep #187 open for the remaining physical cases; this
report and the fix PR provide the reproduction and retest evidence for the
discovery defect found here.

## Automated verification and final state

- `cmake --build build/issue-187 --parallel 3` — passed after the final source change.
- `ctest --test-dir build/issue-187 -L ble --output-on-failure --no-tests=error`
  — 44/44 passed after the final source change.
- The existing preflashed fixture was used; firmware compilation/upload was not
  repeated during this physical session.
- Final serial status: fixture 0.1.0, robot ID 1, `profile normal`, subscriptions
  result=0 and telemetry=0. Desktop process closed cleanly. No user serial logs
  were deleted or overwritten.
