# Micro-Robot Swarm — Term Design

This repository contains the **Mini_Robot** desktop application, ESP32 firmware
prototypes, mechanical designs, and thesis material. The root CMake project opens
the desktop application and its host tests directly from this folder in CLion.

## Repository structure

| Path | Purpose |
| --- | --- |
| [`Mini_Robot/`](Mini_Robot/README.md) | Main C++20 / Qt 6 desktop project, CMake modules, configuration, tests, and documentation |
| [`Mini_Robot/firmware/ble-lifecycle/`](Mini_Robot/firmware/ble-lifecycle/README.md) | Separate ESP32-C6 BLE connection fixture; does not drive motors or implement command safety |
| [`Mini_Robot/firmware/codecell-c6/`](Mini_Robot/firmware/codecell-c6/README.md) | Fresh C6 firmware build skeleton, pinned PlatformIO and independent host tests |
| `3D Printing/` | Mechanical/printing assets |
| `Pres/`, `report/` | LaTeX presentation and report sources and generated documents |
| `.github/workflows/` | PR verification and platform build/test workflows |
| `core arhiteture.md` | Historical architecture proposal; consult current Mini_Robot documentation for implemented behavior |

The ESP32 projects are independent builds, not desktop CMake subprojects.
Existing CI also configures `Mini_Robot/` directly; that entry point remains supported.

## Current desktop capabilities

Implemented components include Qt camera discovery/capture, Linux V4L2 adapters,
recorded-video replay, intrinsic camera calibration and profiles, AprilTag pixel
observations and debug overlays, a metric WorldModel, simulated transport,
RobotManager sessions, optional Qt BLE transport, configuration, and logging.

The full vision-to-motion loop is unfinished: observations do not yet update
calibrated tracked WorldModel poses or drive a closed-loop controller. The desktop
still injects demo pose data. BLE connection-fixture acceptance does not establish
motor, watchdog, or emergency-stop acceptance. See
[current capabilities and limits](Mini_Robot/docs/overview/capabilities.md).

## Open the repository in CLion

1. Open this repository folder (`Term_Design`), or its root `CMakeLists.txt` as a project.
2. Install a C++20 toolchain, CMake **3.24+**, and Ninja. Desktop builds also need
   Qt 6 Widgets/Multimedia (plus Qt Test for tests) and OpenCV with the calibration and AprilTag components
   described in the [build guide](Mini_Robot/docs/development/build.md).
3. In **Settings → Build, Execution, Deployment → CMake**, enable the imported
   `macos`, `linux-gcc`, `linux-clang`, or `windows-msvc` preset for your host.
   Use `core` for libraries and tests without Qt or OpenCV.
4. If dependencies are outside CMake's search paths, set `CMAKE_PREFIX_PATH` or
   package-specific paths in a local profile or ignored `CMakeUserPresets.json`.
   Reload CMake after changing configuration.
5. Build and select **`micro_robot_desktop`** to run/debug the application.
   The `core` profile has test executables but no desktop executable.

CLion imports configure/build presets as profiles; newly imported profiles may
need enabling. See [JetBrains' preset guide](https://www.jetbrains.com/help/clion/cmake-presets.html).
No shared `.idea` configuration or machine-specific dependency paths are required.

If an existing CLion workspace still configures `Mini_Robot/src`, select the
repository-root `CMakeLists.txt` in the Project tree and invoke **Load CMake
Project** (Find Action). The configure command must use `-S .../Term_Design`,
with a separate root build directory such as `.../Term_Design/cmake-build-debug`.
`Mini_Robot/src/CMakeLists.txt` defines child targets and cannot be configured
as a standalone project.

## Build and test from the repository root

For the desktop window only (without the test dependency), use:

```sh
cmake --preset desktop
cmake --build --preset desktop --parallel
```

In CLion, enable the **desktop** preset and select **micro_robot_desktop** to
run/debug. CTest dashboard targets such as `Continuous` are not the application.
This preset deliberately disables tests; use a host preset below for validation.

When PlatformIO Core (`pio` or `platformio`) is on the configure environment's
PATH, the same root project also exposes the **firmware_ble_lifecycle** build
target. It delegates to the independent firmware project and never uploads:

```sh
cmake --build build/desktop --target firmware_ble_lifecycle
```

See [CLion desktop and firmware workflow](Mini_Robot/docs/development/clion-workspace.md)
for nested PlatformIO plugin loading, PATH setup, and the same-window workflow.

Choose the preset for your host (`macos` below):

```sh
cmake --preset macos
cmake --build --preset macos --parallel
ctest --preset macos
```

For a hardware-free core build without Qt/OpenCV:

```sh
cmake --preset core
cmake --build --preset core --parallel
ctest --preset core
```

Tests use an installed GoogleTest package or download pinned GoogleTest 1.17.0.
For offline configuration, install GTest or provide its existing source checkout
with `-DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=/path/to/googletest`.
Test presets exclude tests labelled `hardware`.

Presets are shared with `Mini_Robot/CMakePresets.json`. Their build directory is
relative to the folder you configure, so root builds use `build/<preset>/` and
standalone builds use `Mini_Robot/build/<preset>/`. Do not reuse a build directory
between these entry points.

For the root `macos` preset, the development application is at:

```sh
open build/macos/Mini_Robot/src/micro_robot_desktop.app
```

This bundle uses locally installed Qt libraries. Camera/Bluetooth checks require
appropriate devices and operating-system permissions.

## Build options

Append `-D<option>=ON` or `OFF` to a configure command, using a fresh build
directory when changing toolchains.

| Option | Default | Purpose |
| --- | --- | --- |
| `BUILD_TESTING` | ON | GoogleTest/CTest suites |
| `MICRO_ROBOT_BUILD_DESKTOP` | ON | Qt desktop and UI tests |
| `MICRO_ROBOT_BUILD_REPLAY` | ON | OpenCV recorded-video adapter |
| `MICRO_ROBOT_BUILD_BLE` | OFF | Qt Bluetooth transport; requires Qt 6.2+ Core/Bluetooth |
| `MICRO_ROBOT_BUILD_CALIBRATION` | OFF | OpenCV calibration without the desktop |
| `MICRO_ROBOT_BUILD_VISION` | OFF | OpenCV AprilTag detector without the desktop |
| `MICRO_ROBOT_BUILD_MARKER_TOOLS` | OFF | Offline marker printing/verification tools |

The desktop includes calibration and vision automatically. `core` explicitly
disables all optional adapters/tools. Disabling only the desktop leaves replay ON.

## Documentation and contribution

- [Optional local Graphify navigation](Mini_Robot/docs/development/graphify.md)
- [Mini_Robot documentation index](Mini_Robot/docs/README.md)
- [Architecture](Mini_Robot/docs/overview/architecture.md) and [source map](Mini_Robot/docs/overview/source-map.md)
- [Build setup](Mini_Robot/docs/development/build.md), [Ubuntu setup](Mini_Robot/docs/development/linux-setup.md), and [testing](Mini_Robot/docs/development/testing.md)
- [Desktop demo](Mini_Robot/docs/overview/desktop-demo.md) and [runtime configuration](Mini_Robot/docs/overview/configuration.md)
- [Repository agent instructions](AGENTS.md) and [Mini_Robot architecture/workflow rules](Mini_Robot/AGENTS.md)

Linux and macOS are active development targets. Windows remains a portability
target; consult the platform guides for recorded validation rather than assuming
hardware or GUI acceptance from the presence of a preset.
