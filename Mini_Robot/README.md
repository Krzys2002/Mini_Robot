# Micro-Robot Swarm Desktop

C++20 / Qt 6 desktop application for the micro-robot swarm project.
Phase 0 uses a fake camera and simulated robot transport; no physical hardware
is required.

## Prerequisites

- C++20 compiler (on macOS, install Xcode Command Line Tools with `xcode-select --install`).
- CMake 3.24 or newer.
- Ninja for the commands below.
- Qt 6 with the Widgets component for the desktop build.

GoogleTest is used for testing. CMake uses an installed GoogleTest package when
available; otherwise it downloads GoogleTest 1.17.0 during configuration, which
requires internet access.

## Build and test

Run these commands from the `Mini_Robot` directory:

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

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

Build and test the domain types, interfaces, simulation, and WorldModel without Qt:

```sh
cmake -S . -B build/core -G Ninja -DCMAKE_BUILD_TYPE=Debug \
  -DMICRO_ROBOT_BUILD_DESKTOP=OFF -DBUILD_TESTING=ON
cmake --build build/core --parallel
ctest --test-dir build/core --output-on-failure
```

For selecting suites, running an individual test executable, and adding tests,
see [docs/testing.md](docs/testing.md).
The macOS build/test workflow runs on pushes and pull requests; its local
equivalent commands are in [macOS continuous integration](docs/testing.md#macos-continuous-integration).
For architectural details, see [docs/architecture.md](docs/architecture.md).
