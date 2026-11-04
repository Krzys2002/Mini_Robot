# Micro-Robot Swarm Desktop

[![Daily master build and test](https://github.com/Krzys2002/Mini_Robot/actions/workflows/daily-build-test.yml/badge.svg?branch=master)](https://github.com/Krzys2002/Mini_Robot/actions/workflows/daily-build-test.yml?query=branch%3Amaster)

C++20 / Qt 6 desktop application for the micro-robot swarm project.
This is the main application in the repository. Open the repository root
folder directly in CLion using the [root CMake entry point](../CMakeLists.txt);
see the [repository README](../README.md) for root build commands and layout.
Standalone builds from this directory remain supported.

Implemented components include physical camera streaming (Qt and Linux V4L2),
recorded-video replay, intrinsic calibration and profiles, AprilTag pixel
observations and debug overlays, a metric WorldModel, RobotManager sessions,
simulated transport, optional Qt BLE transport, configuration, and logging.
The complete calibrated perception-to-motion loop is still unfinished: measured
observations do not yet feed tracked WorldModel poses and closed-loop control.
Displayed poses remain demo data. See [current capabilities and integration
limits](docs/overview/capabilities.md) for the authoritative status and hardware
verification limits.
The system camera list refreshes on device changes and lets the operator select,
start, and stop a camera. See [camera selection](docs/overview/camera-selection.md)
and [streaming](docs/overview/camera-streaming.md).
For controls and shutdown behavior, see the [desktop demo guide](docs/overview/desktop-demo.md).
For log output and the log file location, see [application logging](docs/overview/logging.md).
For platform setup and troubleshooting, see [build setup](docs/development/build.md).

## Prerequisites

- C++20 compiler (on macOS, install Xcode Command Line Tools with `xcode-select --install`).
- CMake 3.24 or newer.
- Ninja for the commands below.
- Qt 6 with the Widgets and Multimedia components for the desktop build.
- OpenCV core/imgproc, calibration, and AprilTag components for the desktop;
  videoio for recorded-video replay and the Linux V4L2 backend. See the
  [version-specific dependency list](docs/development/build.md#required-dependencies).
- Qt 6.2+ Core/Bluetooth when enabling `MICRO_ROBOT_BUILD_BLE` (default OFF).
- Qt Test for desktop tests when `BUILD_TESTING=ON`.

GoogleTest is used for testing. CMake uses an installed GoogleTest package when
available; otherwise it downloads GoogleTest 1.17.0 during configuration, which
requires internet access.

## Build and test

Run these commands from the `Mini_Robot` directory:

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure --no-tests=error
```

On macOS, Linux, and Windows, `cmake --preset macos`, `linux-gcc`, `linux-clang`,
or `windows-msvc` runs the same configuration with a per-preset build directory; see
[CMake presets](docs/development/build.md#cmake-presets) and the
[Ubuntu setup guide](docs/development/linux-setup.md).

If CMake cannot find Qt, add `-DCMAKE_PREFIX_PATH=/path/to/your/Qt/installation`
to the configure command. Use the installation prefix containing Qt's `lib/cmake`
directory.

## Launch on macOS

```sh
open build/src/micro_robot_desktop.app
```

The development app uses the installed Qt libraries; it is not a standalone
redistributable bundle.

## Core-only build

Build and test the domain types, interfaces, simulation, WorldModel, and logging
without Qt or OpenCV:

```sh
cmake --preset core
cmake --build --preset core --parallel
ctest --preset core
```

For selecting suites, running an individual test executable, and adding tests,
see [the testing guide](docs/development/testing.md).
The macOS, Linux, and Windows build/test workflows run once daily on `master`
when new PRs have merged; the badge above shows the latest daily result. Their local
equivalent commands are in [macOS continuous integration](docs/development/testing.md#macos-continuous-integration),
[Linux continuous integration](docs/development/testing.md#linux-continuous-integration),
and [Windows continuous integration](docs/development/testing.md#windows-continuous-integration).
For deployment packaging, see [Windows deployment](docs/development/windows-deployment.md).
For architectural details, see [the architecture overview](docs/overview/architecture.md).

## Documentation

See the [documentation index](docs/README.md) for project overviews, planning,
development guides, and archived historical records.

## Source layout

| Path | Responsibility |
| --- | --- |
| `src/app/`, `src/ui/` | Application composition/controllers and Qt presentation |
| `src/common/`, `src/world/`, `src/control/` | Domain types, world/workspace state, differential-drive math |
| `src/camera/`, `src/calibration/`, `src/vision/` | Camera interfaces/adapters, calibration, marker detection and observations |
| `src/transport/`, `src/simulation/` | Transport contracts, BLE adapter, deterministic fake backends |
| `src/infrastructure/` | Configuration and logging |
| `cmake/`, `tests/` | Platform/build rules, boundary checks, hardware-free and separately labelled hardware tests |
| `config/`, `tools/markers/` | Example runtime settings and optional marker tools |
| `firmware/ble-lifecycle/` | Independent PlatformIO connection fixture; no motor control or command acceptance |
| `docs/` | Current module guides, planning, and explicitly archived history |

Follow [AGENTS.md](AGENTS.md) for architecture and contribution rules.
