# BLE reconnect recovery bench record — issue #71

Status: **physical reconnect check run (2026-11-22). 6 of 6 cycles passed.** The
operator judged six cycles sufficient for this record. The #312 checklist (20 cycles,
out-of-range loss) is not complete. Motion, watchdog and stop behaviour are not part
of this record; they remain #189/#228.

## Configuration

| Item | Value |
| --- | --- |
| Desktop | branch `feat/71-ble-reconnect-recovery`, commit `246690f`, `build/linux-gcc` with `-DMICRO_ROBOT_BUILD_BLE=ON`, launched with `--ble` |
| Host | Ubuntu (Linux), Wayland session, BlueZ adapter `UbuntuMiniRobot` |
| Security profile | restricted-lab pass-through (unauthenticated, unencrypted) |
| Robot | CodeCell C6, advertised `MR-7`, `robot_id` 7, provisioning generation 3 |
| Firmware | `codecell-c6` 0.1.0, build ID `cc90652114273294`, profile `restricted-lab-v1` |
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
| Unexpected disconnect is visible and safe | PASS: `Disconnected` (never `Fault`), "Connection lost unexpectedly" shown, no robot output |
| Reconnect restores telemetry | PASS: both CCCDs re-enabled, telemetry resumed in every cycle |
| No stale motion command is sent automatically | PASS: firmware `accepted=0` throughout; no operator drive input was given |
| Repeated cycles do not leak/crash | PASS for 6 cycles: app stayed up, robot heap stable. Desktop process memory was not measured |
| Robot identity remains correct | PASS: `robot_id` 7 / advertised MR-7 in every session |

## Observations and limits

- **Signal path not identified.** The desktop does not log which Qt signal reported the power-cut loss
  (`disconnected`, `InvalidService` or a controller error). After setup, all
  three now take the same link-loss path. This run shows the
  power-cut outcome, not which of the #71 mappings fired.
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
  - out-of-range loss;
  - 20 cycles;
  - a second central;
  - E-stop reassertion through reconnect (#189);
  - motion or disconnect stop (#228).
