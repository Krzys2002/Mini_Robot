# CodeCell C6 firmware functions validation record — issue #309

Status: **host validation complete; physical validation partially run (2026-11-20);
formal powered acceptance BLOCKED.** Host results establish build, codec and logic
behaviour only. The bench session below records BLE conformance and an observed manual
drive response with lifted wheels. It is not powered acceptance: the reviewer-approved
supply limit, wiring and disconnect point that #309 and the
[operator steps](#operator-steps-for-the-physical-rows) require were not recorded. It does
not establish measured output bounds, stopping times, or motor safety.

Base at the time of these runs: PR #307 head `ee4e04b` (`feat/165-restricted-lab-enforcement`).
The #71 BLE link-loss fix (`907ee7d`) is not in this base. The branch has since been
restacked onto master; the host and bench results below were not rerun on the restacked
image.

## Host validation (no hardware)

Run on the issue branch, Linux, date 2026-11-20. Build tree: `build/firmware-host-c6`.

| Check | Command | Result |
| --- | --- | --- |
| Firmware host tests | `cmake -S Mini_Robot/firmware/codecell-c6 -B build/firmware-host-c6 -DCMAKE_BUILD_TYPE=Debug` then build, then `ctest --test-dir build/firmware-host-c6 --no-tests=error` | PASS: 343/343 |
| Production image build | `.venv-firmware/bin/pio run -d Mini_Robot/firmware/codecell-c6 -e esp32-c6-devkitc-1` (PlatformIO 6.1.19) | PASS; the post-build image policy and toolchain provenance hooks ran inside the build |
| Image size | from build output | Flash 820464 / 1310720 B (62.6%); RAM 32232 / 327680 B (9.8%) |
| Image SHA-256 | `sha256sum .pio/build/esp32-c6-devkitc-1/firmware.bin` | `f04a0983bc24168110d7b35bc3e1ffa2189bf804c14210d72868cf5e13c18f3f` |
| Provenance | `firmware-provenance.json` | commit `ee4e04b…`, `modified: false`, `untracked_inputs: false` |
| Credential scan | `python3 .github/scripts/check_firmware_credentials.py` | PASS: no credential material in 51 production files |
| Profile guard, wrong profile | `g++ -std=c++20 -fsyntax-only -Iinclude -DCODECELL_SECURITY_PROFILE=\"other\"` on a TU including `ble/SessionPolicy.hpp` | PASS: static assertion fires (rejected) |
| Profile guard, `restricted-lab-v1` | same, with `restricted-lab-v1` | PASS: compiles |

Not run here: ASan/UBSan and fuzz configuration, the timing pilot image, and the
GATT conformance host script (needs a board).

## Physical validation

Each row needs a bench result from a reviewer-approved configuration. Enter the
image SHA-256 above, the board revision, and log locations. Mark a row BLOCKED
if its precondition is not approved.

| Case | Owner issue | Result |
| --- | --- | --- |
| B. Board identification, USB enumeration, upload, banner at 115200, reset reason | #240 | PASS, operator-reported; board revision and photos not recorded |
| C. Unprovisioned `Fault`; provisioning accepted; invalid record refused | #159 | PARTIAL: provisioning accepted and restarts `Provisioned`/`Idle` (observed); unprovisioned `Fault` operator-reported; invalid record NOT RUN |
| D. Boot electrical levels, zero output, no wheel motion at reset | #238 / #154 | BLOCKED: no approved bench configuration recorded; no electrical measurement |
| E. BLE: one central, second refused, Robot Info 0.1.0, Telemetry, CCCD gating | #163/#164 (#166) | PARTIAL: conformance 72/73; C13 FAIL, open in [#315](https://github.com/Krzys2002/Mini_Robot/issues/315); second central NOT RUN |
| F. Drive polarity and measured output bound | #160 (#217) | BLOCKED for acceptance (no approved supply limit, wiring or disconnect point). Observed only: forward, backward and both turns move the lifted wheels (operator-observed); per-wheel sign table and measured output NOT RUN |
| G. Expiry, E-stop latch and clear, disconnect stop, no replay after reconnect | #162 (#228) | BLOCKED with motion (no approved bench configuration); expiry and E-stop checked at zero output in E |
| H. Task-watchdog expiry or fault powers the bridges down | #162 | BLOCKED (no approved bench configuration) |
| I. Restricted-lab attack cases AT-03, AT-09, AT-10 | #165 | NOT RUN |

## Bench session 2026-11-20

The operator set up the bench with the wheels lifted, not touching the ground. This is
not the reviewer-approved configuration of operator step 2: supply voltage and current
limit, wiring and disconnect point were not recorded, so rows D, F, G and H stay BLOCKED.

**Image on the board.** Build ID `9f7c84106174ebd6`, equal to the ELF SHA-256 prefix of
the host-validated image above; no reflash was needed.

**Provisioning** over the USB maintenance console. The board was already provisioned
as `robot_id` 7 by an earlier session. Committed record: generation 3, slot A.

| Field | Value | Source |
| --- | --- | --- |
| `robot_id`, `board_profile`, `capabilities` | 7, 1, 1 | Protocol / existing ID |
| `drive_polarity` | 0 (neither reversed) | Operator |
| `max_output_percent` | 60 | Operator |
| `wheel_diameter_um`, `track_width_um` | 7000, 40000 | Operator |
| `max_wheel_speed_mm_s`, `max_linear_mm_s` | 100, 100 | Placeholder (not measured) |
| `max_angular_mrad_s` | 5000 (`5 rad/s × 20 mm = 100 mm/s`) | Placeholder (not measured) |
| `max_validity_ms`, `telemetry_period_ms` | 200, 100 | Protocol; 250 is refused by the conformance script |

**BLE conformance** (`scripts/gatt_conformance_host.py --robot-id 7 --max-validity-ms 200
--cycles 20`, Linux/BlueZ, USB attached). The first run, with `max_validity_ms` 250, failed
C3 and C6 (×2) by configuration and C14 B3 (`mr_safety` worst wake lateness 14.1 ms
since boot, bound 2 ms). After reprovisioning to 200 ms: **72 passed, 1 failed**. C14 passed.

- **C13 FAIL, open:** BLE telemetry gap of 47.75 s while the serial host stopped reading for
  30 s. Cause not determined; the robot's side was not observable during the pause.
  Remediation is tracked in [#315](https://github.com/Krzys2002/Mini_Robot/issues/315).

**Desktop app over BLE** (`micro_robot_desktop --ble`, Qt 6.10.2, BlueZ 5.85, VirtualBox
guest with a passed-through USB adapter). Discovery found `MR-7`. The first connect timed out
in Connecting (10 s phase timeout). BlueZ `Device1.Connect` failed 3 of 4 times
(`le-connection-abort-by-local`); after an adapter power cycle it succeeded 4 of 4 times in
5.4–6.0 s. The app then connected. The power cycle is a **workaround**: it did not isolate
the cause of the aborts, and the host adapter, VirtualBox USB passthrough, BlueZ and the
robot's advertising were not distinguished.

**Manual drive** (lifted wheels, operator-observed; an observed response, not powered
acceptance):

- Forward and backward at 0.1 m/s move both wheels (wheel speed 100 mm/s, 60% output).
- Turning at the app's built-in 1.0 rad/s did not move the wheels: with a 40 mm track each
  wheel gets 20 mm/s, about 12% output, and the drive has no startup boost.
- Turning at 5.0 rad/s (`config/codecell-c6-bench.ini`) moves the wheels in both directions.
  The motor deadband between these values was not measured (#167).

**Battery telemetry** (image `61eb9d9`, build ID `cc90652114273294`, routine upload; the
provisioning record was kept). The firmware reads GPIO4 with the CodeCell conversion
(`adc_mV × 2 + 65`) and publishes it under provisional evidence: plausible 3000–4400 mV,
`batteryLow` at 3350/3450 mV, 500 ms sample age.

| Power | Observation | Source | Result |
| --- | --- | --- | --- |
| USB connected | Serial `battery_raw_mv=4421`, six reports in a row; `battery_quality=implausible`, telemetry `unknown` | Serial log | As designed: the USB supply on the battery node is above the 4400 mV ceiling |
| LiPo only, USB unplugged | App status line `4.15 V` over BLE | Operator-observed | PASS: reading published end to end |

Not established: accuracy (no multimeter at the battery leads; #238), the `batteryLow`
transition (cell not near 3350 mV), and averaging (one unaveraged conversion per sample).

## Operator steps for the physical rows

1. Keep motor power disconnected for all USB steps (rows B and C). The README
   states that the drive outputs' electrical state during reset and boot is not
   verified.
2. Confirm the approved bench configuration with the reviewer. Record supply
   voltage and limit, wiring, the restraint setup, and the disconnect point.
   Rows D, F, G and H stay BLOCKED until this is recorded.
3. Upload the image with the commands in
   `Mini_Robot/firmware/codecell-c6/README.md` (section "Upload and monitor").
   Do not use the erase target unless it has been authorized.
