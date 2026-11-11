# CodeCell C6 firmware

This is a fresh Arduino/PlatformIO build skeleton for the production firmware
work (#155 onward). It replaces the retired T1 C6/S3 projects. The old HTTP server,
browser UI, motor controller and IMU-distance experiments are not ported here.

The current image prints a build-skeleton message and supplies a monotonic clock
adapter. It has no motor initialization, motion authority, command input, BLE
server, telemetry or watchdog implementation. It is not a functioning desktop
endpoint or proof of safe electrical boot behavior. Runtime, actuation, protocol
and safety arrive through #158–#165. The separate
[BLE lifecycle fixture](../ble-lifecycle/README.md) remains connection-only.

## Reproducible build

Use Python 3.12 and PlatformIO Core 6.1.19 in an isolated virtual environment:

```sh
python3 -m venv .venv
.venv/bin/python -m pip install platformio==6.1.19
.venv/bin/pio run -d Mini_Robot/firmware/codecell-c6 -e esp32-c6-devkitc-1
.venv/bin/pio pkg list -d Mini_Robot/firmware/codecell-c6 -e esp32-c6-devkitc-1
```

Commands above run from repository root; put the venv outside the checkout or
ignore it locally. The firmware CMake target `firmware_codecell_c6` delegates
build-only work when PlatformIO is available. Desktop/default builds never invoke
PlatformIO. The manifest pins:

| Component | Version/source |
| --- | --- |
| PlatformIO Core | 6.1.19 (installation/CI) |
| pioarduino platform | 55.03.37 release archive |
| Arduino ESP32 framework / SDK libraries | 3.3.7, as fixed by the platform release |
| RISC-V GCC toolchain | 14.2.0+20251107, as fixed by the platform release |
| CodeCell | Git commit 795616b3fc6fa14c0e23fbdac8aed697d897d303 (2.0.4) |
| DriveCell | Registry release 1.1.3 |
| Host GoogleTest fallback | 1.17.0 |

The framework supplies WebServer; no separate WebServer package is installed,
and the skeleton does not include or start it. CodeCell/DriveCell are declared
for the firmware adapter work; the skeleton itself does not initialize them.
C++20 is required by both host and cross-build, with a compile-time diagnostic
in `FirmwareClock.hpp`. Record `pio --version`, `pio pkg list`, compiler version,
firmware commit, environment and build logs with every test image. Release pins
are source provenance, not a claim of bit-identical binaries across hosts.

Review dependency updates in a separate PR: verify official provenance and
security advisories, change exact pins together, clean-build on macOS and Ubuntu,
run host tests, inspect resolved packages and retain logs. Never replace these
pins with moving `stable`, branch heads or version ranges. On package download
failure, check network/proxy and the pinned upstream asset; do not substitute an
unreviewed dependency. Missing PlatformIO: activate the documented environment.

Sources: [platform release](https://github.com/pioarduino/platform-espressif32/releases/tag/55.03.37),
[PlatformIO dependency specification](https://docs.platformio.org/en/latest/projectconf/sections/env/options/library/lib_deps.html).

## Host tests without Arduino, Qt, OpenCV or a board

```sh
cmake -S Mini_Robot/firmware/codecell-c6 -B build/firmware-host-c6 -DCMAKE_BUILD_TYPE=Debug
cmake --build build/firmware-host-c6 --parallel 3
ctest --test-dir build/firmware-host-c6 --output-on-failure --no-tests=error
```

Install CMake 3.24+, a C++20 compiler and optionally GoogleTest. On Ubuntu use GCC
and CMake; on macOS use Xcode Command Line Tools and CMake. If GTest is unavailable,
CMake downloads 1.17.0; offline users may set
`-DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=/absolute/path/to/googletest`.
The same pure tests are included by desktop/core CTest without linking embedded
production sources. Fixtures cover injected time, future/negative readings and
the 32-bit millisecond wrap boundary. They do not test a motor controller.
The reusable firmware CI matrix performs compile/host tests on Ubuntu and macOS
as part of the standard daily `master` batch at 02:00 UTC, only when the daily
merge gate requests a build. It has no PR, push or manual trigger, so automatic
firmware builds run at most once per day after merge to `master`;
only actual executed jobs establish validation on those hosts.

## Device discovery, upload, monitor and recovery

**Powered tests remain blocked by [#238](https://github.com/Krzys2002/Mini_Robot/issues/238)**
and the [hardware profile](../../docs/testing/codecell-c6-hardware-154.md).
A fresh skeleton is not a verified electrical-disable image. Do not connect an
unreviewed motor-power setup merely to upload or monitor.

After the applicable power/wiring/output gates have evidence and operator approval:

- Enumerate serial interfaces with `pio device list`; select the actual USB device
  explicitly rather than assuming a macOS `/dev/cu.*` or Linux `/dev/ttyACM*` name.
- On Linux, check device permissions and the distribution's serial group/udev
  setup; re-login after group changes. Avoid running routine builds/uploads as root.
- Upload the reviewed revision with `pio run -d Mini_Robot/firmware/codecell-c6 -e esp32-c6-devkitc-1 -t upload --upload-port DEVICE`.
- Monitor with `pio device monitor --port DEVICE --baud 115200`; close competing
  serial sessions first. Retain exact device, revision and logs.
- Upload overwrites application firmware. Full flash erase can destroy identity,
  calibration and credentials; it is a separate destructive recovery action and
  is never part of a routine build/upload.
- For boot-loop recovery use the manufacturer procedure referenced in the hardware
  profile only after confirming board applicability and the power gates. This
  guide does not authorize changing boot wiring on a powered assembly.

No on-device debugger is configured or verified for this physical board. USB
serial logging is the documented diagnostic interface; do not claim JTAG support
from a generic PlatformIO board definition. Flash/monitor/recovery and actual
host/USB behavior require physical follow-up evidence in
[#240](https://github.com/Krzys2002/Mini_Robot/issues/240).
