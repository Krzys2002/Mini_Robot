# BLE reconnect recovery bench record — issue #71

Status: **physical reconnect check run (2026-11-22): 6 of 6 idle cycles reconnected
and resumed telemetry.** The robot was idle throughout: no drive input was held or
queued, so this run does not exercise stale motion. Desktop process memory was not
measured. The #312 checklist (20 cycles, out-of-range loss) is not complete. Desktop
leak, out-of-range loss and motion-stop acceptance stay open; motion, watchdog and stop
behaviour remain #189/#228. A local adapter loss check (2026-11-25, below) shows that
removing or powering off the desktop adapter faults instead of passing as link loss.

## Configuration

| Item | Value |
| --- | --- |
| Desktop | branch `feat/71-ble-reconnect-recovery`, commit `246690f`, `build/linux-gcc` with `-DMICRO_ROBOT_BUILD_BLE=ON`, launched with `--ble`. This predates the restack and the Qt controller-error classification (`QtBleErrorPolicy.hpp`); not rerun since |
| Host | Ubuntu (Linux), Wayland session, BlueZ adapter `UbuntuMiniRobot` |
| Security profile | restricted-lab pass-through (unauthenticated, unencrypted) |
| Robot | CodeCell C6, advertised `MR-7`, `robot_id` 7, provisioning generation 3 |
| Firmware | `codecell-c6` 0.1.0, build ID `cc90652114273294`, profile `restricted-lab-v1`: the #309 battery-telemetry image (`61eb9d9`, now #314), not the #310 baseline |
| Power | USB and on-board battery. Bench state confirmed safe by the operator; no drive command was sent at any point |
| Evidence | Desktop stderr log (INFO level) and a timestamped firmware serial stream at 115200 baud. Both were kept locally for this session and are not committed |

## Method

1. The operator clicked Scan, selected MR-7 and Connect, and confirmed that
   telemetry was updating in the panel.
2. Loss was induced as follows:
   - Cycles 1–5: a USB-Serial-JTAG hard reset (RTS pulse with DTR low,
     the same as esptool `hard_reset`).
   - Cycle 6: the operator cut all power (battery and USB).
3. The operator recorded the panel status, then clicked Connect without a
   separate Scan.
4. After each reconnect, the firmware's periodic `RUNTIME`, `BLE` and
   `TELEMETRY` status lines were read while the session was active.

## Results

| Cycle | Loss induced | App on loss | Panel (operator) | Reconnect requested → Connected | Robot after reconnect |
| --- | --- | --- | --- | --- | --- |
| 1 | USB reset | 18:14:57.464 `Disconnected`, "BLE link disconnected unexpectedly" | Connection lost | 18:15:31.866 → 18:15:43.934 | Telemetry updating (operator). Firmware session state not captured |
| 2 | USB reset (on serial port open) | 18:16:09.903 `Disconnected`, unexpected | Connection lost | 18:16:58.866 → 18:17:10.673 | `ready`, CCCDs 1/1, session 1, 188 frames sent, `accepted=0`, Idle, output disabled |
| 3 | USB reset | 18:18:12.574 `Disconnected`, unexpected | Connection lost | 18:18:28.852 → 18:18:40.313 | CCCDs 1/1, 186 sent, 0 notify failures, `accepted=0`, Idle, disabled |
| 4 | USB reset | 18:19:03.072 `Disconnected`, unexpected | Connection lost | 18:19:28.966 → 18:19:40.178 | CCCDs 1/1, 492 sent, 0 failures, `accepted=0`, Idle, disabled |
| 5 | USB reset | 18:20:29.849 `Disconnected`, unexpected | Connection lost | 18:21:10.085 → 18:21:21.848 | CCCDs 1/1, 136 sent, 0 failures, `accepted=0`, Idle, disabled |
| 6 | Power cut (battery + USB) | 18:22:28.487 `Disconnected`, unexpected | Connection lost | 18:22:35.421 → 18:22:46.352 | `reset=power_on`, CCCDs 1/1, 325 sent, 0 failures, `accepted=0 rejected=0`, Idle, disabled |

Times are UTC from the desktop log.

**Run totals**
- **Desktop:** 7 transitions to Connected (the first connection plus 6
  reconnects) and 0 `Fault` states.
- **Robot:** all 66 periodic firmware `RUNTIME` lines report `accepted=0` and
  `estops=0`, and none shows the drive output enabled.
- **Telemetry:** the maximum inter-frame gap after reconnect was 100.17 ms at
  the 100 ms period.
- **Memory:** robot free heap stayed at about 329.8 KB (minimum free about
  327.0 KB) across cycles.
- **Reconnect time:** from Connect to Connected was about 11–12 s, mostly
  scan and GATT setup. This is not a latency measurement; #72 owns timing.

## Acceptance against #71

| Criterion | Result |
| --- | --- |
| Unexpected disconnect is visible | PASS (idle): `Disconnected` (never `Fault`) and "Connection lost unexpectedly" shown in all 6 cycles. Loss during motion and its stop are NOT RUN (#228) |
| Reconnect restores telemetry | PASS (idle): both CCCDs re-enabled, telemetry resumed in every cycle |
| No stale motion command is sent automatically | NOT SHOWN by this run: no drive input was held or queued, so `accepted=0` only shows that nothing was sent while idle. Evidence is automated: `BleTransportTest.RepeatedLinkLossThroughRobotManagerRestoresTelemetryWithoutStaleMotion` (50 cycles, a 10 s-valid velocity submitted before each loss: no automatic write after reconnect, the old intent refused as `StaleSession`) and `DesktopSafetyTest.RepeatedSimulatedLinkLossRestoresTelemetryAndNeverReplaysMotion` |
| Repeated cycles do not leak/crash | OPEN: the app stayed up for 6 cycles and the robot heap was stable, but desktop process memory was not measured |
| Robot identity remains correct | PASS: `robot_id` 7 / advertised MR-7 in every session |
| Out-of-range loss | NOT RUN |

## Observations and limits

- **Signal path not identified.** The desktop build used here did not log which Qt signal
  reported the power-cut loss (`disconnected`, `InvalidService` or a controller error), and
  at the time treated every post-setup controller error as link loss. Since then the
  backend classifies controller errors and keeps the Qt error name in the error message,
  so a rerun can show which signal fired. This run shows the power-cut outcome only.
- **USB unplug alone.** At 18:13:58 USB was unplugged while connected. The board kept running
  on battery and the link stayed up, as expected.
- **Serial-port behaviour.**
  - Opening the serial port with DTR/RTS both low reset the board; cycle 2's
    loss came from this.
  - An RTS pulse with DTR still asserted did not reset it. The first cycle-3
    attempt therefore did nothing, and the link stayed up.
- **Battery reading, outside #71's scope.** The firmware reported `battery_mv=unknown`, quality `implausible`
  (raw about 4421 mV) while on USB; see #309/#238.
- **Not run:**
  - loss with drive input held or queued;
  - desktop process memory across cycles;
  - out-of-range loss;
  - 20 cycles;
  - a second central;
  - E-stop reassertion through reconnect (#189);
  - motion or disconnect stop (#228).

## Local adapter loss check, 2026-11-25 (review of PR #313)

Purpose: confirm that losing the desktop's own Bluetooth adapter faults instead of
passing as recoverable link loss. Robot idle throughout; no drive input.

| Item | Value |
| --- | --- |
| Desktop | `feat/71-ble-reconnect-recovery` at `4a5ec36` plus the uncommitted review fixes (link teardown arbitration, `LinkLost` code, local adapter check), `build/linux-gcc` with BLE, launched with `--ble` and `QT_LOGGING_RULES="qt.bluetooth.bluez=true"` |
| Host | Ubuntu, Qt 6.10.2 (system packages), BlueZ 5.85, adapter `hci0` on `btusb` |
| Robot | MR-7, `robot_id` 7. Firmware image not re-recorded for this run |

**First attempt, before the adapter check (11:37).** `btusb` unbind while connected
was reported as `Disconnected` with "BLE link disconnected unexpectedly (Robot Control
service became invalid)" and "Connection lost unexpectedly": the adapter loss passed
as link loss. Qt 6.10.2's BlueZ backend handles the device's `Connected=false` first
and closes with `NoError`; it then stops watching the adapter, so
`InvalidBluetoothAdapterError` never arrives (same code in 6.12.0). This led to the
local adapter check (`confirmQtBleLinkLoss`).

| Check | Induced by | Result |
| --- | --- | --- |
| A. Adapter removed | `btusb` unbind of `hci0` while connected | PASS: 11:57:56 `Fault`, "Local Bluetooth adapter unavailable (removed); native: Robot Control service became invalid" |
| C. Adapter powered off | `bluetoothctl power off` while connected | PASS: Qt logged `ServicesResolved=false`, `Connected=false`; 11:59:33 `Fault`, "Local Bluetooth adapter unavailable (powered off); native: Robot Control service became invalid" |
| Recovery after A and C | Rebind/power on, explicit Connect | PASS: `Ready`/`Connected` at 11:59:16 and 11:59:59 |
| B. Robot reset with working adapter | Robot reset while connected, 5 cycles | PASS (operator, 5/5): each loss ended in "Connection lost unexpectedly", never `Fault`; explicit Connect restored telemetry. Log lines not reproduced here |

Observation: the first Connect after rebinding the adapter (11:58:22) timed out in
`Connecting` after the 10 s phase timeout. Qt saw `Connected=true` but no
`ServicesResolved`. It faulted as "BLE timeout during Connecting", and the next
Connect succeeded. This was not investigated further.

Fault during an explicit disconnect (finding 2) was not induced on the bench; it is
covered by automated tests only.
