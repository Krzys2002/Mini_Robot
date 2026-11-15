# Timing pilot on the CodeCell C6, USB only (2026-11-15)

Status: **bench pilot for [#156](https://github.com/Krzys2002/Mini_Robot/issues/156),
non-motor.** It measures the scheduler, clock, handoff, logging, BLE and
watchdog behaviour that the proposed
[timing budgets](../protocol/timing-budgets.md) are derived from. The image is a
measurement fixture that never calls CodeCell, DriveCell, a motor, a sensor or a
GPIO pin. It is **not** evidence of motor output, physical stopping, production
firmware or any #158–#166 acceptance. Powered and physical checks stay with
[#238](https://github.com/Krzys2002/Mini_Robot/issues/238) and
[#166](https://github.com/Krzys2002/Mini_Robot/issues/166).

## Setup

| Item | Value |
| --- | --- |
| Source | Commit `d8c0784275b3b3b37901b09c1a2d4df920c979e6` (pilot added on top of the #155 branch head `8255208`), tracked firmware files unmodified |
| Image | `timing-pilot` environment, `firmware.bin` SHA-256 `a4ec71e1983f637b38788c43c1d3426b33f78721265db80fbb18ba6e1b32687d`; [provenance](evidence/156-2026-11-15/firmware-provenance.json) and [package list](evidence/156-2026-11-15/pkg-list.txt) |
| Toolchain | PlatformIO Core 6.1.19, pioarduino 55.03.37, Arduino-ESP32 3.3.7, ESP-IDF v5.5.2-729-g87912cd291, NimBLE-Arduino 2.3.6, `-std=gnu++20` |
| Board | ESP32-C6FH8 (QFN32) revision v0.2, single core, 160 MHz, 8 MB embedded flash, USB `303A:1001`; CodeCell board revision not inspected |
| Power and actuators | USB data cable only. The battery and the motors were disconnected (operator report); the image initializes no motor, sensor or GPIO |
| FreeRTOS | 1000 Hz tick, unicore, run-time statistics on the esp_timer, task watchdog enabled (5 s default) |
| Host | Ubuntu 26.04.1 LTS, kernel 7.0.0-43-generic, BlueZ 5.85, Intel AX200 adapter, system Python with pyserial and python3-dbus |
| Link | Robot requested 7.5–30 ms, latency 0, 400 ms; every archived session connected and stayed at 30 ms, latency 0, 400 ms |
| Board image before and after | `ble-lifecycle` fixture 0.1.0, ID 1, profile `normal`, left by the #157 spike. Restored afterwards (`048d2bc9…ebe5`, rebuilt from this branch) and confirmed with `status` |

Board and adapter addresses appear in local logs only; the archived logs redact them.

## Method

From `Mini_Robot/firmware/codecell-c6`, with the pinned PlatformIO and the
system Python (the PlatformIO virtual environment has no D-Bus bindings):

```sh
pio run -e timing-pilot                       # writes provenance; upload does not refresh it
pio run -e timing-pilot -t upload --upload-port /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_<MAC>-if00
/usr/bin/python3 scripts/timing_pilot_host.py --port <port> --log serial.log \
  "info" "clock 100000" "handoff 2000 20" "jitter 5 20 20" "jitter 5 20 20 cpu 19 7 3" ...
/usr/bin/python3 scripts/timing_pilot_host.py --port <port> --log ble-nominal.log \
  "ble on 20 5" "telem 100" ble-connect "ble:periodic:100:60:jitter 5 55 20" "ble" ...
```

The exact step lists are the first `host>` lines of each [evidence log](evidence/156-2026-11-15/).

- **Every open of the serial port reset the C6** on this host, including pyserial
  with DTR/RTS deasserted. Each host run is therefore one serial session, and BLE
  scenarios run inside it. Each archived log starts from a fresh boot.
- **Wake-up lateness** is measured from the scheduler tick edge. A probe task
  locates one tick edge before any load task starts, then uses `xTaskDelayUntil`
  and records `now − (edge + ticks × 1 ms)`.
- **Loads.** `cpu <prio> 7 3` is a 7 ms busy / 3 ms sleep task (not
  phase-locked to 5 ms). `log <prio> 80` writes 80-byte lines in a tight loop.
  BLE loads come from the host: write-without-response through BlueZ
  `AcquireWrite` at a fixed period or as fast as the socket accepts, notifications
  through `AcquireNotify`, and E-stop write-with-response through D-Bus.
- **BLE timing.** Receipt is stamped at write-callback entry. A depth-1 FreeRTOS
  queue hands the receipt stamp to a priority-20 consumer, modelling the setpoint
  mailbox. Every write is answered with one 9-byte notification (the result
  path). An E-stop write calls a gate stub that only stamps the time. Telemetry
  is an 18-byte notification task at priority 5.
- Durations are robot-clock (esp_timer) or host-clock (monotonic) intervals,
  never cross-clock. Percentiles are conservative bucket edges
  (`LatencyHistogram`, ≤ 1/16 relative width above 64 µs).

## Results

### Clock and handoffs (idle)

| Quantity | Result |
| --- | --- |
| Back-to-back `esp_timer_get_time()` (100,000) | 1–12 µs, p99 2 µs |
| Depth-1 queue overwrite → receive, priority 1 → 20 (2,000) | 9–11 µs |
| Task notify → wake (2,000) | 8–9 µs |
| Critical section around a 16-byte copy | 1–9 µs |
| Uncontended mutex take + give | 3–11 µs |

### Scheduler wake-up lateness

| Condition | Probe priority | Lateness max (p99) | Non-idle CPU |
| --- | --- | --- | --- |
| Idle, 1 / 5 / 10 ms period | 20 | 3 µs (3 µs) | 0.1–1 % |
| 70 % CPU hog at priority 19 (below) | 20 | 3 µs | 70 % |
| Same hog at 19, probe **below** it | 18 | **4.012 ms** (4.012 ms) | 70 % |
| Logging loop at priority 1 | 20 | 18 µs (3 µs) | 4.9 % |
| Logging loop at priority 21 (above) | 20 | 199 µs (115 µs) | 4.8 % |
| Hog at 19 + logging at 1 | 20 | 3 µs | 73 % |
| BLE S1: 10 Hz writes, 100 ms telemetry | 20 | 356 µs (287 µs) | 1.8 % |
| BLE S3: 10 Hz writes, 5 ms telemetry | 20 / 22 | 298 µs / 12 µs | 6.4 % |
| BLE S4: write flood | 20 / 22 | **801 µs** / 108 µs | 19.7 % / 15.1 % |

No probe overran its period in any archived run. Two development runs (earlier
pilot builds, same load code) saw the priority-21 logger delay the probe by about
30 ms, and S3 with a flood backlog reach 1.403 ms at priority 20. Those logs are
not archived, but the budgets allow for both figures.

### USB serial writes (`Serial.write`, 80-byte lines)

| Case | Calls | Duration max (p99) | Lines received by the host |
| --- | --- | --- | --- |
| Host reading, TX timeout 100 ms (default) | 500 | 38 ms (0.96 ms) | 500 |
| Host reading, TX timeout 0 | 500 | 0.94 ms | 110 (and the next report line was lost) |
| Host paused 15 s, TX timeout 100 ms | 3,000 | 99.9 ms (30 µs) | 76 |
| Host paused 15 s, TX timeout 0 | 3,000 | 0.96 ms | 75 |

### BLE (BlueZ client, 30 ms connection interval)

| Scenario | Robot | Host |
| --- | --- | --- |
| S1: 10 Hz writes, 60 s | 600 writes, 600 handoffs. Inter-arrival 89.9–120.0 ms. Callback 0.35–0.41 ms, result notify 0.33–0.38 ms, receipt→consumer 0.40–0.46 ms. Telemetry 611 sent, period deviation p99 5 µs, max 1.48 ms | 600/600 results. Write→result 34.3–62.2 ms (p50 45.5). Telemetry gaps 83.5–122.6 ms |
| S2: 50 Hz writes, 30 s | 1,500 writes, 1,001 handoffs (depth-1 coalescing). Inter-arrival 0.29–31.0 ms | 1,500/1,500 results, 11.8–61.9 ms |
| E-stop, 42 write-with-response at 200 ms spacing | Callback entry→gate stub 1–2 µs. Callback 0.33–0.38 ms | Write-with-response 37.4–41.3 ms. 42/42 results |
| S3: 5 ms telemetry + 10 Hz writes, 2 × 30 s | 6,191 / 6,206 telemetry, 0 failures, period deviation ≤ 0.16 ms | 6,185 / 6,198 received (99.9 %) |
| S4: flood, 10 s (2 sessions) | 4,459 / 3,321 writes processed, 0 notify failures. Callback ≤ 0.375 ms | BlueZ accepted about 649 k / 636 k frames. 4,175 / 3,022 results received |
| 1 kHz offered, 10 s | 4,455 writes processed | Round trip p50 4.1 s, max 7.9 s: the host queue grew |
| 0.2 s flood (7,548 frames), then E-stop | All 7,548 processed (≈ 0.36 k/s). The E-stop reached the callback after them; gate stub 1 µs | E-stop write-with-response **21.0 s** |
| Disconnects (host-initiated) | Reason 531 every time | — |

Resources after the BLE suite: free heap 312,728 B (minimum 304,796 B). BLE start
cost 38,084 B. Stack free at high-water: `nimble_host` 2,920 B, `ble_ll_task`
4,188 B, pilot tasks ≥ 3,404 B of 4,096 B.

### Task watchdog stall (pilot task subscribed, outputs not involved)

| Timeout | Stall → trigger | Trigger → reset line (panic dump) | Reset → `setup()` | Total |
| --- | --- | --- | --- | --- |
| 100 ms | 101.0 ms | 406.9 ms | 344.9 ms | 852.8 ms |
| 100 ms | 101.3 ms | 406.5 ms | 344.8 ms | 852.5 ms |
| 300 ms | 301.2 ms | 408.9 ms | 343.6 ms | 1,053.7 ms |

These are host-stamped intervals between serial lines. USB latency adds a few
milliseconds of uncertainty. The reset reason after the stall was 6 (task watchdog).

## Interpretation for the budget

- The supervisor needs a priority above `nimble_host`. The scheduler itself is
  precise (≤ 3 µs), but the BLE host task and a mis-prioritized logger cost the
  priority-20 probe up to 0.8 ms (1.4 ms in development). Even so, the 2 ms
  allowance (B3) holds at either priority.
- Logging must never be on a timing path. The cost of a write is set by the USB
  host, not by the firmware.
- The robot-side BLE path is sub-millisecond: callback, result queueing and
  handoff all stay ≤ 0.7 ms. Radio and central-side queueing dominate, and a
  central backlog can delay commands and E-stop by seconds. That is a desktop
  requirement (D1–D2), not a robot bound.
- A task-watchdog reset leaves about 0.75 s between the trigger and `setup()`, and
  the outputs are not controlled by the application during that time (B17).

## Not run

| Check | Reason |
| --- | --- |
| Motor/driver write time, electrical disable, physical stop and deceleration | No actuator access allowed; powered gates in #238 |
| CodeCell/BNO085 sensor access time and blocked-read behaviour | CodeCell initialization was excluded for this pilot |
| Electrical output state during panic, reset and boot | #238 |
| Abrupt radio loss (supervision timeout), CCCD disable, reconnect timing | Only host-initiated disconnects were possible here (#72/#166) |
| macOS central, the Qt desktop as client, a second central | Not available in this session |
| HCI-level timestamps (controller receipt) | `btmon` needs root |
| Serial output with no USB host (battery operation) | No battery allowed; known from source only |
| Long soak (resource trends over hours) | Pilot runs lasted minutes |

## Deviations during the session

The pilot was revised several times before the archived run: static measurement
buffers, the tick-edge calibration order, in-place statistics resets, E-stop
detection by pointer instead of UUID parsing, and absolute deviations. One
revision overflowed `loopTask`'s stack (a 13 KB temporary in the command
handler) and panicked. That was a pilot defect, fixed before the archived image.
Only the image and logs listed above are evidence.
