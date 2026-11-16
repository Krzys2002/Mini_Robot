# CodeCell C6 firmware

This is the Arduino/PlatformIO production firmware project (#155 onward). It
replaces the retired T1 C6/S3 projects. The old HTTP server, browser UI, motor
controller and IMU-distance/target experiments are not ported here.

The image runs the production runtime (#158): the drive is disabled at boot, one
local safety/actuator authority (`ActuatorGate`) holds the only drive reference,
one command authority (`CommandProcessor`) is the only way to request motion, and
a 5 ms safety supervisor and a 10 ms control task follow the #156 scheduling model.
See [runtime and threading](../../docs/overview/firmware-runtime.md). The image has
no BLE peripheral, codec or telemetry yet (#161, #163, #164), so nothing commands
motion. The drive is the [bounded actuator contract](../../docs/overview/firmware-drive.md)
(#160). It wakes the H-bridges only with a valid provisioned profile, and it is not
proof of safe electrical boot behaviour or of any physical motor limit (#238). The runtime becomes
ready only with a validated, provisioned identity/configuration record (#159); an
unprovisioned board stays in `Fault` with outputs disabled. Provision it over USB
serial as described in [identity provisioning](../../docs/overview/firmware-provisioning.md). The separate
[BLE lifecycle fixture](../ble-lifecycle/README.md) remains connection-only.

Arduino/PlatformIO is retained (#155): the Arduino core already runs on
FreeRTOS, so no ESP-IDF migration is needed to obtain an RTOS.

| Path | Purpose |
| --- | --- |
| `platformio.ini` | Pinned cross-build for the C6 (`esp32-c6-devkitc-1`) and the measurement-only `timing-pilot` environment |
| `requirements.txt` | Exact PlatformIO Core and Python dependency pins |
| `scripts/toolchain_provenance.py` | Rejects an unpinned Core/platform; writes build provenance |
| `scripts/production_image_policy.py`, `scripts/image_policy.py` | Fails the link if Wi-Fi/AP/HTTP/OTA code is in the image (#158) |
| `include/FirmwareClock.hpp` | Portable C++20 clock contract and conversions, no SDK includes |
| `include/runtime/` | Portable production runtime (#158): ports, safety state, actuator gate, command processor, mailbox, periodic steps and start-up orchestration |
| `include/provisioning/` | Portable identity/configuration record, A/B atomic store, runtime configuration source and USB maintenance console (#159) |
| `include/drive/` | Portable [bounded drive](../../docs/overview/firmware-drive.md) (#160): drive profile, kinematics and actuator map, rate limiter, motor HAL port and `BoundedDriveOutput` |
| `include/TimingBudget.hpp` | Named [timing, resource and stopping budgets](../../docs/protocol/timing-budgets.md) (#156, proposed) with compile-time composition checks |
| `include/LatencyHistogram.hpp` | Fixed-memory latency histogram used by instrumentation |
| `pilot/`, `scripts/timing_pilot_host.py` | [#156 timing pilot](../../docs/testing/timing-pilot-156.md): bench measurement image and its Linux/BlueZ host driver; never a robot image |
| `src/main.cpp`, `src/EspRuntimeAdapters.hpp`, `src/EspProvisioningAdapters.hpp`, `src/CodeCellMotorDriver.hpp` | Arduino entry point and the only ESP-IDF/FreeRTOS/board adapters: clock, critical section, periodic tasks, DRV8837 motor HAL (LEDC), NVS record storage and bond erase |
| `CMakeLists.txt`, `tests/` | Independent GoogleTest host entry point (no Arduino, Qt or OpenCV) |

All commands below run from the repository root.

## Clean installation

Both hosts need Git, Python 3.10 or newer with `venv`, CMake 3.24 or newer and a
C++20 compiler. Nothing here needs Qt, OpenCV, a board, credentials or IDE paths.

Ubuntu 24.04 LTS:

```sh
sudo apt-get install --yes git python3-venv cmake ninja-build g++ libgtest-dev
```

macOS (Apple Silicon or Intel):

```sh
xcode-select --install
brew install cmake ninja python googletest
```

`libgtest-dev`/`googletest` is optional: without it the host build downloads the
hash-checked GoogleTest 1.17.0 archive. Then create the firmware tool environment
(ignored by Git) and install the exact pins:

```sh
python3 -m venv .venv-firmware
.venv-firmware/bin/python -m pip install -r Mini_Robot/firmware/codecell-c6/requirements.txt
.venv-firmware/bin/pio --version   # PlatformIO Core, version 6.1.19
```

Activate it (`source .venv-firmware/bin/activate`) or prefix commands with
`.venv-firmware/bin/`. A different PlatformIO installation, for example the
CLion/VS Code plugin copy, must report the same Core version; otherwise the build
stops with the message in [troubleshooting](#troubleshooting). The first build
downloads the platform, framework, toolchain and libraries into `~/.platformio`.

## Build the C6 image (no upload)

```sh
pio run -d Mini_Robot/firmware/codecell-c6 -e esp32-c6-devkitc-1
pio pkg list -d Mini_Robot/firmware/codecell-c6 -e esp32-c6-devkitc-1
```

The image is `Mini_Robot/firmware/codecell-c6/.pio/build/esp32-c6-devkitc-1/firmware.bin`.
From CMake, the optional `firmware_codecell_c6` target runs the same build when
PlatformIO is found; desktop and default builds never invoke PlatformIO and
configure normally without it.

## Pinned dependencies and build provenance

| Component | Version/source |
| --- | --- |
| PlatformIO Core | 6.1.19 and its full Python dependency set (`requirements.txt`) |
| pioarduino platform | [55.03.37 release asset](https://github.com/pioarduino/platform-espressif32/releases/tag/55.03.37) |
| Arduino-ESP32 framework | 3.3.7 (`esp32-core-3.3.7` release archives), fixed by the platform |
| ESP-IDF libraries | 5.5.0+sha.87912cd291 (`esp32-core-3.3.7-libs`), fixed by the platform |
| RISC-V GCC toolchain | 14.2.0+20251107, fixed by the platform |
| esptool | 5.1.2, fixed by the platform |
| CodeCell | Git commit 795616b3fc6fa14c0e23fbdac8aed697d897d303 (2.0.4) |
| DriveCell | Registry release 1.1.3 (pinned explicitly; CodeCell declares it unversioned) |
| Host GoogleTest fallback | 1.17.0 archive, SHA-256 checked |
| C++ standard | `-std=gnu++20` for the cross-build (all framework defaults removed); C++20 for host tests |

C++20 is also enforced by a compile-time diagnostic in `FirmwareClock.hpp`.
Two guards keep these pins effective:

- `python3 .github/scripts/check_firmware_pins.py` fails on any moving or
  unpinned reference in `firmware/*/platformio.ini`: bare names, version ranges,
  `stable`/branch/tag URLs, Git URLs without a 40-character commit, local paths,
  or a non-exact `requirements.txt` line. Repository automation tests run it.
- `scripts/toolchain_provenance.py` runs before compilation and stops the build
  if PlatformIO Core differs from `requirements.txt` or the resolved platform
  differs from the `platform` release URL.

Every successful build writes
`.pio/build/esp32-c6-devkitc-1/firmware-provenance.json`: Core, platform and
package versions with sources, resolved libraries with their Git commit or
registry source, compiler version, C++ standard flag, SHA-256 of `firmware.bin`,
source commit, whether the image may differ from it (`source.modified`: a tracked
project file changed, or an untracked, non-ignored file exists under a build input such
as `src/`, `include/`, `scripts/` or `platformio.ini`; `source.untracked_inputs` says
which), and host OS/architecture. Untracked files outside the build inputs, such as
bench logs in the project directory, are not counted. It contains no user name or local path. Keep it, together with
`pio pkg list` output and the build log, with every image used for a test.
Release pins are source provenance, not a claim of bit-identical binaries
across hosts.

### Dependency update and security review policy

Change pins only in a dedicated PR that:

1. takes new versions from official upstream releases (pioarduino, Espressif,
   microbots, PyPI), records release notes and checks security advisories for the
   platform, framework/ESP-IDF, toolchain, libraries and Python packages;
2. updates related pins together: the platform URL and the version rows above;
   `requirements.txt` from a clean virtual environment (`pip freeze` after
   `pip install platformio==<version>`); Git dependencies by full commit SHA;
3. passes the pin check, clean builds and host tests on Ubuntu and macOS, and
   attaches the new `firmware-provenance.json` and `pio pkg list` output.

Never replace pins with `stable`, branch heads or version ranges, and never
substitute an unreviewed mirror or package to get past a download failure.

## Host tests without Arduino, Qt, OpenCV or a board

```sh
cmake -S Mini_Robot/firmware/codecell-c6 -B build/firmware-host-c6 -DCMAKE_BUILD_TYPE=Debug
cmake --build build/firmware-host-c6 --parallel 3
ctest --test-dir build/firmware-host-c6 --output-on-failure --no-tests=error
```

`--no-tests=error` makes an empty test selection fail rather than pass. Offline
builds without an installed GoogleTest may set
`-DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=/absolute/path/to/googletest`.
The same pure tests are included in the desktop/core CTest suites (`core`
label) without linking embedded production sources. Fixtures cover injected
time, future/negative readings, microsecond-to-millisecond conversion of the
ESP timer value and the 32-bit millisecond wrap boundary. `TimingBudgetTest`
replays representative stop timelines (command loss, deadline boundaries,
stalled control, E-stop behind a parser flood, notification pressure, clock
wrap points) through a test-only reference model of the #156 budget; it is a
specification oracle, not the #162 supervisor. `LatencyHistogramTest` covers
the instrumentation histogram. `RuntimeComponentsTest` and `FirmwareRuntimeTest`
run the #158 runtime with fake clock, platform, drive, configuration and sensor:
injected start-up failures, stale snapshots, stop/new-command races, mailbox
floods and the control-stall bound ([details](../../docs/overview/firmware-runtime.md#verification)).
`RobotConfigRecordTest`, `ConfigStoreTest`, `MaintenanceConsoleTest` and
`ProvisioningRuntimeTest` cover the #159 record schema, corruption and version
handling, power interruption at every step and byte, the maintenance console and
the provisioning gate ([details](../../docs/overview/firmware-provisioning.md#verification)).
None of these tests exercises a motor controller.

## Timing pilot (bench measurement image, #156)

`pio run -d Mini_Robot/firmware/codecell-c6 -e timing-pilot` builds a separate
measurement image from `pilot/` only. It links NimBLE but neither CodeCell nor
DriveCell, configures no motor, sensor or GPIO pin and uses its own BLE UUIDs and
name (`MR-PILOT-156`), so a v1 desktop cannot mistake it for a robot. Flash it
only USB-powered with the battery and motors disconnected, then restore the image
the board had. Use, results and limits: [timing pilot record](../../docs/testing/timing-pilot-156.md).
Default builds, the CMake `firmware_codecell_c6` target and robots never use it.

## Troubleshooting

| Symptom | Cause and action |
| --- | --- |
| `pio: command not found`; CMake prints `PlatformIO not found; optional firmware targets disabled` | Tool environment missing or not active. Follow [clean installation](#clean-installation) or pass `-DMICRO_ROBOT_PLATFORMIO_EXECUTABLE=<path to pio>`. |
| `Error: PlatformIO Core X is active, but this project is pinned to 6.1.19` | Another PlatformIO installation is first on `PATH`. Use `.venv-firmware/bin/pio` or reinstall from `requirements.txt`. |
| Warning `Obsolete PIO Core v6.1.19 is used (previous was X)` | Another Core (often an IDE plugin) shares `~/.platformio`. The build is still guarded; to isolate it, set `PLATFORMIO_CORE_DIR` to a separate directory. |
| `Error: Resolved platform ... does not match the pinned release` | A stale platform is installed. Run `pio pkg uninstall -g -p espressif32`, then rebuild. |
| `VCSBaseException ... not our ref <sha>` or `Could not find the package` | A pinned commit/release cannot be fetched. Check network/proxy access to GitHub and the PlatformIO registry; if upstream removed it, follow the dependency review, never a moving reference. |
| `check_firmware_pins.py` reports `error: ... lib_deps = ...` | A moving or unpinned reference was introduced. Pin as the message states. |
| `Error: the production image links retired Wi-Fi/AP/HTTP/OTA code (#158)` | A source or library change linked a network, web, DNS or OTA stack. Remove it; experiments need their own environment behind the same `ActuatorGate` ([policy](../../docs/overview/firmware-runtime.md#production-image-policy)). |
| `CodeCell firmware requires C++20` | A build bypassed the pinned configuration or host CMake entry point. |
| `firmware-provenance.json` names a different `sha256` than `firmware.bin` | Provenance is written by the default build target only; `-t upload` does not refresh it. Run `pio run -e <environment>` before archiving, then upload without changing sources. |
| CTest `No tests were found!!!` with `--no-tests=error` | The test executable was not built or discovered. Rebuild the host tree and check configure output. |

## Continuous integration

`.github/workflows/firmware-build-test.yml` runs on `ubuntu-24.04` and `macos-15`
with a 25-minute bound: pin check, installation from `requirements.txt`,
C6 cross-build without upload (including the production image policy), the timing-pilot build, provenance and package listing, and host
configure/build/CTest with `--no-tests=error`. It uses no USB, BLE, credentials,
IDE paths or desktop dependencies. It is called by the standard daily `master`
batch at 02:00 UTC when the daily merge gate requests a build, so it has no PR,
push or manual trigger; the pin checker's unit tests also run in the
`pr-verification.yml` repository automation step. Only executed jobs establish
validation on those hosts.

## Device discovery, upload, monitor and recovery

**Powered tests remain blocked by [#238](https://github.com/Krzys2002/Mini_Robot/issues/238)**
and the [hardware profile](../../docs/testing/codecell-c6-hardware-154.md).
Physical upload, monitoring, recovery and host USB behavior are validated in
[#240](https://github.com/Krzys2002/Mini_Robot/issues/240); until that evidence
exists, the steps below are the procedure to follow, not verified results.

Safe output state: the image never initializes CodeCell or DriveCell. At boot it sets
the four bridge inputs low and wakes the bridges (GPIO18) only after a valid provisioned
drive profile; an unprovisioned board never wakes them. No code in the image commands
motion yet (no BLE peripheral, #163), and the USB console has no motion command. A
**provisioned** board would drive within its record once commands exist, so keep motor
power disconnected until #238 approves the supply and output bound.
The electrical state of the drive outputs during reset, boot and download mode
is not verified. Keep motor power disconnected while uploading or monitoring,
and do not connect an unreviewed motor-power setup merely to upload.

### Discover the serial device

The ESP32-C6 USB port enumerates as USB ID `303a:1001`, "Espressif USB
JTAG/serial debug unit" (observed on Ubuntu during #155, without upload).

```sh
pio device list
```

- **Ubuntu:** the port is `/dev/ttyACM<N>`. Prefer the stable link
  `/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_<MAC>-if00`.
- **macOS:** the port is `/dev/cu.usbmodem<suffix>`; use the `cu.` device, not
  `tty.`. Native USB needs no extra driver.

Select the device explicitly with `--upload-port`/`--port`; do not rely on
automatic selection when other serial devices are attached.

### Serial permissions

- **Ubuntu:** `/dev/ttyACM*` belongs to group `dialout` (mode 0660). Desktop
  sessions may add an access entry for the logged-in user (`getfacl /dev/ttyACM0`
  shows `user:<you>:rw-`); otherwise, or over SSH, run
  `sudo usermod -aG dialout "$USER"` and log out and in again. Do not run
  builds, uploads or monitors with `sudo`. ModemManager may probe new ACM ports
  and briefly hold them (`udevadm info -q property /dev/ttyACM0` shows
  `ID_MM_CANDIDATE=1`); if uploads report the port busy, stop it for the session
  with `sudo systemctl stop ModemManager` or install the
  [PlatformIO udev rules](https://docs.platformio.org/en/latest/core/installation/udev-rules.html).
- **macOS:** `/dev/cu.*` devices are accessible to the logged-in user; no group
  change is needed. Close other serial monitors first, because a port can be
  opened by only one program.

### Upload and monitor (routine)

```sh
pio run -d Mini_Robot/firmware/codecell-c6 -e esp32-c6-devkitc-1 -t upload --upload-port DEVICE
pio device monitor --port DEVICE --baud 115200
```

Observed on Ubuntu during #156: every open of the port (including pyserial
with DTR/RTS deasserted) reset the C6, so a tool that needs uninterrupted
firmware state must keep one serial session open. Upload rebuilds if needed, then writes the bootloader, partition table and
application images to their regions; it does not erase the whole flash, so a stored
identity survives an upload. On a provisioned board the monitor should show
`RUNTIME uptime_ms=... start=running ready=1 state=Idle output=disabled fault=None maintenance=0 provisioning=Provisioned robot_id=<id> ...`
every 5 s (the boot banner is printed before USB CDC reconnects and is often not seen).
An unprovisioned board shows `start=faulted state=Fault output=disabled
fault=ConfigurationUnavailable provisioning=Unprovisioned robot_id=none`; provision it
with the [maintenance console](../../docs/overview/firmware-provisioning.md#usb-maintenance-path)
in the same monitor session (type `help`). Any other `start`, `state` or `fault` value
means the runtime did not reach safe idle;
see [start-up and failure behaviour](../../docs/overview/firmware-runtime.md#start-up-and-failure-behaviour). Record the
device, revision, `firmware-provenance.json` and logs.

### Recovery and destructive erase

- esptool normally resets the C6 into download mode over the built-in USB
  serial/JTAG interface, so a faulty application image can be replaced by a
  routine upload. If the port disappears or the image prevents this, manual
  boot-mode entry is board-specific; the reset/boot mapping of this board
  revision is UNRESOLVED in the hardware profile. Use the manufacturer procedure
  only after confirming it applies and the power gates allow it. This guide does
  not authorize changing boot wiring on a powered assembly.
- **Destructive:** `pio run -d Mini_Robot/firmware/codecell-c6 -e esp32-c6-devkitc-1 -t erase`
  erases the entire flash, including any stored identity, calibration and
  credentials; the `erase_upload` target does the same before uploading. Both are
  separate recovery actions, never part of a routine build or upload; record why
  one was needed and re-upload afterwards. The robot is then `Unprovisioned` and
  must be [provisioned](../../docs/overview/firmware-provisioning.md) again.

### Debugging

Supported: USB serial logging through `pio device monitor` at 115200 baud.
Not supported: on-chip debugging. Although the chip exposes a USB JTAG interface,
no `debug_tool`, OpenOCD or GDB configuration is provided or verified for this
board. Do not claim JTAG support from the generic PlatformIO board definition.
