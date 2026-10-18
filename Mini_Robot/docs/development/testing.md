# GoogleTest and CTest

Source: [test targets](../../tests/CMakeLists.txt) and
[offscreen UI entry point](../../tests/main.cpp).

Configure with `-DBUILD_TESTING=ON` (the default) to build the GoogleTest
executables in `tests/`. They are separate from `micro_robot_desktop`; running
domain tests does not start the application or require physical hardware.

The root CMake file first looks for an installed GoogleTest CMake package. If
unavailable, FetchContent downloads the pinned GoogleTest 1.17.0 release. For
offline builds, pass `-DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=/path/to/googletest`
pointing to an already available GoogleTest source checkout. Set
`-DBUILD_TESTING=OFF` to omit test targets and GoogleTest discovery/download.

## Build and run

From the repository's `Mini_Robot` directory:

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure --no-tests=error
```

Add `-DCMAKE_PREFIX_PATH=/path/to/Qt` when Qt is not discoverable. CTest discovers
individual GoogleTest cases at test time (`DISCOVERY_MODE PRE_TEST`), so build the
test executables before running CTest. `--no-tests=error` catches an empty test
selection rather than reporting successful verification with no tests run.

| Executable | CTest label | Coverage |
| --- | --- | --- |
| `micro_robot_common_tests` | `core` | Robot identity, units, poses, monotonic time |
| `micro_robot_camera_tests` | none | Packed frame validation, malformed metadata, storage and overflow checks; camera selection rules |
| `micro_robot_interface_tests` | `core` | Camera/transport contracts and ownership |
| `micro_robot_transport_tests` | `core` | Transport domain versions, sequence reservation, metadata defaults and copying |
| `micro_robot_simulation_tests` | `core` | Deterministic backends, fake camera factory, and substitution |
| `micro_robot_world_tests` | `core` | World registry and pose update rules |
| `micro_robot_logging_tests` | `core` | [Logging](../overview/logging.md): format, levels, channels, sink failures, file sink |
| `micro_robot_replay_tests` | `replay` | Generated-video decoding, repeatability, lifecycle, looping, weak sinks, and failure recovery |
| `micro_robot_configuration_tests` | `core` | [Configuration](../overview/configuration.md): defaults, parsing, invalid values, missing/unreadable files, example file |
| `micro_robot_tests` | `ui` | Offscreen Qt window, controller integration, and camera selection |
| `PlatformBoundary.*` (CMake scripts) | `core`, `boundary` | [Platform boundaries](../overview/platform-boundaries.md#enforcement): no OS/backend APIs, Qt platform macros, or concrete-backend includes in platform-independent targets; scanner and link-check self-tests |

The camera suite currently has no CTest label; `-L core` does not include it.
Use unfiltered CTest for all suites, or add a separate
`-R '^(CameraFrameTest|CameraSelectionTest)\.'` selection when testing by label.

Core executables link domain libraries and `GTest::gtest_main`, without Qt or a
`QApplication`. The UI executable has its own test entry point using GoogleTest
and an offscreen `QApplication`; it is built only when
`MICRO_ROBOT_BUILD_DESKTOP=ON`.

[CameraSelectionControllerTest](../../tests/ui/CameraSelectionControllerTest.cpp) uses
injected manual inventory, factory, and camera backends without Qt Multimedia or
hardware. It checks empty inventories, disabled rows, duplicate names/distinct
IDs, selection and source replacement, rejected choices, removal/reconnection,
renames, plain-text presentation, and subscription teardown. Camera contract
tests also compile the discovery and factory interfaces and the selection rules
without Qt. Physical visibility/hotplug checks are listed
in [camera discovery](../overview/camera-discovery.md).

World tests cover insert/query/update/removal, unknown poses, duplicate IDs,
finite-pose validation, per-robot timestamp ordering (including rejected updates),
detached query/snapshot values, and the non-copyable authoritative model contract.
Run only this suite with:

```sh
ctest --test-dir build -R '^WorldModelTest\.' --output-on-failure --no-tests=error
```

Issue #45 requires no manual camera/BLE checks: its domain model has no hardware
dependencies. Desktop integration tests verify its existing controller consumers;
a Qt-free build verifies that the world target remains independent of Qt.

Select suites or individual cases:

```sh
ctest --test-dir build -L core --output-on-failure --no-tests=error
ctest --test-dir build -L ui --output-on-failure --no-tests=error
ctest --test-dir build -R '^RobotIdTest\.' --output-on-failure --no-tests=error
```

You can build and run a dedicated executable directly:

```sh
cmake --build build --target micro_robot_common_tests
./build/tests/micro_robot_common_tests --gtest_filter=RobotIdTest.ComparesByValue
```

With multi-configuration generators, such as Visual Studio, add `--config Debug`
to the build command and `-C Debug` to CTest. Direct execution uses the selected
configuration's subdirectory, for example
`build/tests/Debug/micro_robot_common_tests.exe` on Windows.

## macOS continuous integration

The repository-level `.github/workflows/macos-build-test.yml` runs when a pull
request is opened, reopened, or updated with new commits, and on pushes to `master`.
Its `macOS Apple Silicon / Debug`
job uses the `macos-15` runner and the default AppleClang toolchain. It checks out
the repository with read-only permissions and without persisted credentials,
installs CMake/Ninja/Qt/OpenCV using Homebrew, configures the desktop and tests explicitly,
builds all targets, and runs every CTest case (including offscreen Qt tests).

Each run uses a fresh build directory with no workflow dependency/build cache.
GoogleTest is resolved through the existing CMake package lookup or pinned
FetchContent fallback. No secrets or developer-specific paths are required.
The Qt prefix comes from `brew --prefix qt`, and commands run in `Mini_Robot`
relative to the checkout. Configure/build/test failures fail the job; CTest also
fails if no tests are discovered. Logs are available in the repository's Actions
tab and the pull request checks. New pull request runs cancel superseded runs for
the same ref; a running `master` run is allowed to finish so the README badge reports
a completed result. Pushes to other branches do not trigger it, which avoids
duplicate runs for PR branches. Manual
`workflow_dispatch` runs can validate a selected branch even when PR merge
conflicts prevent automatic CI:

```sh
gh workflow run macos-build-test.yml --repo Krzys2002/Mini_Robot --ref your-branch
```

To run the same commands locally on macOS, start in `Mini_Robot` with a fresh
`build/ci` directory and installed Xcode Command Line Tools:

```sh
brew install cmake ninja qt opencv
cmake -S . -B build/ci -G Ninja -DCMAKE_BUILD_TYPE=Debug \
  -DMICRO_ROBOT_BUILD_DESKTOP=ON -DBUILD_TESTING=ON \
  -DCMAKE_PREFIX_PATH="$(brew --prefix qt)"
cmake --build build/ci --parallel 3
ctest --test-dir build/ci --output-on-failure --no-tests=error
```

This workflow verifies compilation and automated tests, without camera/BLE
hardware or deployment packaging. Windows CI is separate Issue #106.

## Linux continuous integration

The repository-level `.github/workflows/linux-build-test.yml` uses the same
triggers, permissions, checkout, and concurrency rules as the macOS workflow. It
runs two jobs on the `ubuntu-24.04` runner, one per compiler preset:

| Job (status check name) | Preset | Compiler |
| --- | --- | --- |
| `Ubuntu 24.04 / GCC / Debug` | `linux-gcc` | Ubuntu's default `g++` |
| `Ubuntu 24.04 / Clang / Debug` | `linux-clang` | Ubuntu's default `clang++` |

Each job installs the apt packages listed in the
[Ubuntu setup guide](linux-setup.md#build-requirements), prints the CMake, Ninja,
and compiler versions, configures with the desktop and replay adapters explicitly
ON, builds all targets, and runs every CTest case through the test preset (output
on failure, error when no tests are found). Qt UI tests use the offscreen platform,
so no display, camera, Bluetooth adapter, or robot is needed. GoogleTest comes from
the system `libgtest-dev` CONFIG package. The jobs do not use a dependency or build
cache. One compiler's failure does not cancel the other job, and each job reports
its own pass/fail status on the pull request and in the Actions tab.

To run a job's commands locally on Ubuntu 24.04, start in `Mini_Robot`:

```sh
sudo apt-get update
sudo apt-get install --yes build-essential clang cmake ninja-build \
  qt6-base-dev qt6-multimedia-dev libgtest-dev libopencv-dev
cmake --preset linux-gcc -DMICRO_ROBOT_BUILD_DESKTOP=ON -DMICRO_ROBOT_BUILD_REPLAY=ON
cmake --build --preset linux-gcc --parallel 4
ctest --preset linux-gcc
```

Use `linux-clang` in place of `linux-gcc` for the Clang job. Start a manual run with:

```sh
gh workflow run linux-build-test.yml --repo Krzys2002/Mini_Robot --ref your-branch
```

### Required status checks

Branch protection can require these job names as status checks:
`macOS Apple Silicon / Debug`, `Ubuntu 24.04 / GCC / Debug`, and
`Ubuntu 24.04 / Clang / Debug`. GitHub lists a check as selectable only after it
has run at least once in the repository. Renaming a job or matrix entry changes
its check name, so update the branch protection rule at the same time. Branch
protection is configured by a repository administrator; it is not part of the
workflow files.

## Qt-free verification

Use a separate build directory to verify the core without discovering Qt:

```sh
cmake -S . -B build/core -G Ninja -DCMAKE_BUILD_TYPE=Debug \
  -DMICRO_ROBOT_BUILD_DESKTOP=OFF -DBUILD_TESTING=ON
cmake --build build/core --parallel
ctest --test-dir build/core --output-on-failure --no-tests=error
```

Replay tests run without Qt or hardware and require an MJPEG-capable OpenCV
video backend. See the [replay fixture strategy](../overview/recorded-video.md#fixtures-and-verification).
Set `MICRO_ROBOT_BUILD_REPLAY=OFF` to omit that suite and OpenCV discovery.

## Adding tests

`tests/common/TypesTest.cpp` contains the existing example
`TEST(RobotIdTest, ComparesByValue)`, which checks value comparison through the
public domain API. Follow its GoogleTest style for new non-GUI logic tests.

Add a test source to the corresponding executable in `tests/CMakeLists.txt` and
link the domain target it exercises. For a new module, create a separate executable
linked with `GTest::gtest_main`, apply `micro_robot_target_defaults()`, and register
it with `gtest_discover_tests(... DISCOVERY_MODE PRE_TEST PROPERTIES LABELS "core")`.
Register a new platform-independent source target with
`micro_robot_platform_boundary()` so its boundary checks run; platform adapters
stay unregistered. Keep pure logic tests independent of widgets and physical backends. Future path
or calibration tests should use deterministic inputs through their module APIs.
