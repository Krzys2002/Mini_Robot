# CodeCell C6 firmware functions validation record — issue #309

Status: **host validation complete; physical validation NOT RUN.** Host results
establish build, codec and logic behaviour only. They do not establish motor
safety, BLE behaviour on hardware, or E-stop/watchdog timing on a board.

Base: PR #307 head `ee4e04b` (`feat/165-restricted-lab-enforcement`). The
#71 BLE link-loss fix (`907ee7d`) is not in this base.

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

## Physical validation (NOT RUN)

Each row needs a bench result from a reviewer-approved configuration. Enter the
image SHA-256 above, the board revision, and log locations. Mark a row BLOCKED
if its precondition is not approved.

| Case | Owner issue | Result |
| --- | --- | --- |
| B. Board identification, USB enumeration, upload, banner at 115200, reset reason | #240 | NOT RUN |
| C. Unprovisioned `Fault`; provisioning accepted; invalid record refused | #159 | NOT RUN |
| D. Boot electrical levels, zero output, no wheel motion at reset | #238 / #154 | BLOCKED (bench configuration not approved) |
| E. BLE: one central, second refused, Robot Info 0.1.0, Telemetry, CCCD gating | #163/#164 (#166) | NOT RUN |
| F. Drive polarity and measured output bound | #160 (#217) | BLOCKED (bench configuration not approved) |
| G. Expiry, E-stop latch and clear, disconnect stop, no replay after reconnect | #162 (#228) | BLOCKED (bench configuration not approved) |
| H. Task-watchdog expiry or fault powers the bridges down | #162 | BLOCKED (bench configuration not approved) |
| I. Restricted-lab attack cases AT-03, AT-09, AT-10 | #165 | NOT RUN |

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
