# Micro-Robot Swarm Desktop

[![Daily master build and test](https://github.com/Krzys2002/Mini_Robot/actions/workflows/daily-build-test.yml/badge.svg?branch=master)](https://github.com/Krzys2002/Mini_Robot/actions/workflows/daily-build-test.yml?query=branch%3Amaster)

C++20 / Qt 6 desktop application for the micro-robot swarm project.
The current M1 foundation supports physical camera streaming and simulated robot
transport. It can launch and run tests without hardware; streaming needs an
available camera and camera access. M1 is not yet complete.
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
- OpenCV core, imgproc, and videoio for recorded-video replay (omit with
  `-DMICRO_ROBOT_BUILD_REPLAY=OFF`).

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

On macOS and Linux, `cmake --preset macos`, `linux-gcc`, or `linux-clang` runs the
same configuration with a per-preset build directory; see
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

Build and test the domain types, interfaces, simulation, WorldModel, and logging without Qt:

```sh
cmake -S . -B build/core -G Ninja -DCMAKE_BUILD_TYPE=Debug \
  -DMICRO_ROBOT_BUILD_DESKTOP=OFF -DBUILD_TESTING=ON
cmake --build build/core --parallel
ctest --test-dir build/core --output-on-failure --no-tests=error
```

For selecting suites, running an individual test executable, and adding tests,
see [the testing guide](docs/development/testing.md).
The macOS and Linux build/test workflows run once daily on `master` when new PRs
have merged; the badge above shows the latest daily result. Their local
equivalent commands are in [macOS continuous integration](docs/development/testing.md#macos-continuous-integration)
and [Linux continuous integration](docs/development/testing.md#linux-continuous-integration).
For architectural details, see [the architecture overview](docs/overview/architecture.md).

## Documentation

See the [documentation index](docs/README.md) for project overviews, planning,
development guides, and archived historical records.
