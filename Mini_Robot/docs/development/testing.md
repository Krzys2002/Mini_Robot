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
| `micro_robot_vision_tests` | `vision` | AprilTag formats/stride, canonical rotations and resulting robot headings, multiple IDs, invalid inputs, negative scenes and generated-video replay; desktop or standalone vision ON |
| `micro_robot_ble_contract_tests` | `core`, `ble` | Qt-free Robot Info, identity/GATT validation, command codec, malformed notifications and sequence wrap |
| `micro_robot_ble_tests` (opt-in) | `ble` | Qt Core fake-radio lifecycle, bounded dispatch/freshness, telemetry/results, E-stop retries, controller delivery, timeouts and recovery; no physical radio |
| `micro_robot_calibration_tests` | `core`, `calibration` | Capture policy plus intrinsic result validation, weighted RMS, warning thresholds and owned snapshots |
| `micro_robot_calibration_detector_tests` | `calibration` | Generated chessboards and analytic intrinsic/distortion recovery, per-frame error and invalid input tests; built with desktop or standalone calibration enabled |
| `micro_robot_common_tests` | `core` | Robot identity, units, poses, monotonic time, angle normalization |
| `micro_robot_vision_observation_tests` | `core`, `vision` | [Robot marker observations](../vision/robot-observation.md): tag-to-RobotId mapping, unknown/duplicate/invalid rejection, projective center, heading over the full rotation range |
| `micro_robot_control_tests` | `core` | [Differential-drive kinematics](../overview/differential-drive.md): analytical wheel targets, inverse conversion, world derivatives, signs, bounds, invalid inputs and overflow |
| `micro_robot_camera_tests` | `core` | Packed frame validation, malformed metadata, storage and overflow checks; camera selection rules |
| `micro_robot_interface_tests` | `core` | Camera/transport contracts and ownership |
| `micro_robot_transport_tests` | `core` | Transport domain versions, sequence reservation, metadata defaults and copying |
| `micro_robot_transport_mock_tests` | `core` | [Robot discovery and the shared transport contract](../overview/robot-communication.md#mock-transport-tests): scans, snapshots, ambiguity, private peer mapping, link loss, and the same `IRobotTransport` checks against simulation and a test-only radio backend |
| `micro_robot_simulation_tests` | `core` | Deterministic backends, fake camera factory, and substitution |
| `micro_robot_world_tests` | `core` | World registry and pose update rules |
| `micro_robot_logging_tests` | `core` | [Logging](../overview/logging.md): format, levels, channels, sink failures, file sink including non-ASCII directories |
| `micro_robot_replay_tests` | `replay` | Generated-video decoding, repeatability, lifecycle, looping, weak sinks, and failure recovery |
| `micro_robot_configuration_tests` | `core` | [Configuration](../overview/configuration.md): defaults, parsing, invalid values, missing/unreadable files, non-ASCII/spaced/relative/symlinked paths, verbatim log paths, example file |
| `micro_robot_marker_tests` (opt-in) | `markers` | [AprilTag print tooling](../vision/apriltag-selection.md): all 16 IDs and rotations, quiet margins, invalid inputs, and reproducibility of committed SVG sheets |
| `micro_robot_tests` | `ui` | Offscreen Qt window, controller integration, camera selection, calibration wizard and bounded vision-worker lifecycle, [GUI compatibility](gui-compatibility.md) (resize, mouse/keyboard input, scaling); with the V4L2 backend also [Linux V4L2 discovery and capture](../overview/camera-backends.md#verification) |
| `GuiCompatibility.ScaleFactor*` | `ui`, `gui` | The GUI compatibility cases repeated with `QT_SCALE_FACTOR` 1.5 and 2 |
| `micro_robot_tests`: `V4l2CameraHardwareTest.*` | `hardware` | Physical V4L2 capture; skipped without `MICRO_ROBOT_TEST_V4L2_DEVICE` and excluded from the regression suite (see [hardware tests](hardware-tests.md)) |
| `DesktopStartup.StartsAndShutsDownCleanly` (CMake script) | `ui` | Production desktop started offscreen with `--exit-after-startup` and a private configuration in a non-ASCII working directory; exit code, startup/shutdown log order, resolved configuration path, no warnings or errors |
| `PlatformBoundary.*` (CMake scripts) | `core`, `boundary` | [Platform boundaries](../overview/platform-boundaries.md#enforcement): no OS/backend APIs, Qt platform macros, or concrete-backend includes in platform-independent targets; scanner and link-check self-tests |

Enable `-DMICRO_ROBOT_BUILD_BLE=ON` for BLE adapter/lifecycle tests, then run
`ctest --test-dir build -L ble --output-on-failure --no-tests=error`. On macOS
with Qt 6.5+, this label also checks the production app's Bluetooth usage
description and statically imported permission plugin without requesting access.
Core-only builds keep thirteen Robot Info/GATT and message-codec tests without Qt.
Physical checks are listed in the [connection guide](../overview/ble-connection.md)
and [message guide](../overview/ble-messages.md).

Enable offline marker generation and its tests with
`-DMICRO_ROBOT_BUILD_MARKER_TOOLS=ON` (default OFF). This adds OpenCV aruco/objdetect
and imgcodecs dependencies only to the offline tool/test targets. Run just those
tests with `ctest --test-dir build -L markers --output-on-failure --no-tests=error`.
The [print guide](../vision/apriltag-selection.md#printing-and-reproducibility)
also describes independent SVG rasterization and full-sheet decoding checks.

Core executables link domain libraries and `GTest::gtest_main`, without Qt or a
`QApplication`. The UI executable has its own test entry point using GoogleTest
and an offscreen `QApplication`; it is built only when
`MICRO_ROBOT_BUILD_DESKTOP=ON`. Set `MICRO_ROBOT_TEST_QPA_PLATFORM` (for example
`wayland`, `xcb`, or `cocoa`) to run it on that platform plugin inside a desktop
session; see [GUI compatibility](gui-compatibility.md#automated-checks).

[CameraSelectionControllerTest](../../tests/ui/CameraSelectionControllerTest.cpp) uses
injected manual inventory, factory, and camera backends without Qt Multimedia or
hardware. It checks empty inventories, disabled rows, duplicate names/distinct
IDs, selection and source replacement, rejected choices, removal/reconnection,
renames, plain-text presentation, and subscription teardown. Camera contract
tests also compile the discovery and factory interfaces and the selection rules
without Qt. Physical visibility/hotplug checks are listed
in [camera discovery](../overview/camera-discovery.md).

With the Linux V4L2 backend, `micro_robot_tests` also contains `V4l2DiscoveryScanTest`,
`V4l2CameraDeviceDiscoveryTest`, and `V4l2CameraSourceTest`. They use temporary
sysfs/dev trees and need no camera. `V4l2CameraHardwareTest` cases carry the
`hardware` label instead of `ui` and are reported as skipped unless
`MICRO_ROBOT_TEST_V4L2_DEVICE` names a camera node; see
[hardware tests](hardware-tests.md) for the command and the optional resolution check.

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

## Regression suite

The regression suite is every CTest test except those labelled `hardware`. It needs
no camera, Bluetooth adapter, robot, or display session, and it is the selection
that CI runs on macOS and Linux. Every test preset applies it (exclude label
`hardware`, output on failure, error when no tests are found):

```sh
ctest --preset macos        # macOS, AppleClang
ctest --preset linux-gcc    # Ubuntu, GCC
ctest --preset linux-clang  # Ubuntu, Clang
ctest --preset core         # any host, core libraries only
```

Equivalent selection for a build directory without presets:

```sh
ctest --test-dir build -LE hardware --output-on-failure --no-tests=error
```

| Area | Hardware-free tests |
| --- | --- |
| Core library | `core` label: common types, control, camera contracts and selection, interfaces, transport types, world model, logging, configuration, calibration capture, platform boundaries. The `core` preset builds and runs them without Qt or OpenCV. |
| Configuration and filesystem paths | `ConfigFileTest`, `ConfigParseTest`, `FileLogSinkTest`: non-ASCII names, spaces, relative paths, symbolic links, unreadable and non-regular files, platform-style log paths kept verbatim |
| Mock camera and robot transport | `FakeCameraTest`, `FakeCameraSourceFactoryTest`, `SimulationTransportTest`, `BackendSubstitutionTest`, interface probes; replay (`replay` label) with generated video |
| Initialization | `DesktopIntegrationTest` (controller startup and shutdown with fake backends) and `DesktopStartup.StartsAndShutsDownCleanly` (the production executable) |
| Platform adapters without devices | Offscreen UI tests; V4L2 tests on temporary device trees; macOS bundle permission check |

The `core` configure preset turns off the desktop, replay, calibration, and marker
targets, so CMake never looks for Qt or OpenCV:

```sh
cmake --preset core
cmake --build --preset core --parallel
ctest --preset core
```

Tests that need devices or a person are listed separately in
[hardware-dependent tests](hardware-tests.md).

## macOS continuous integration

The repository-level `.github/workflows/daily-build-test.yml` checks `master`
once daily at **02:00 UTC**. It calls the reusable macOS and Linux build workflows
only when a newly merged pull request is present in the checked-out `master`
snapshot. There are no PR, push or manual build triggers. Days without new merges
run only the lightweight gate, without installing build dependencies.

The gate compares against the last successful daily run, including successful
no-op checks. Missed schedules and failed runs therefore retain pending merges
for the next daily check. Merge detection uses GitHub's merge commit SHA, covering
merge commits, squash merges and rebase merges. Direct pushes alone do not trigger
builds. On the first run, the gate considers commits from the preceding 24 hours.
GitHub may delay or drop scheduled runs; this is a daily batch rather than an
immediate post-merge guarantee. The schedule takes effect after human merge to
`master`.

The reusable `.github/workflows/macos-build-test.yml` workflow's
`macOS Apple Silicon / Debug`
job uses the `macos-15` runner and the default AppleClang toolchain. It checks out
the repository with read-only permissions and without persisted credentials,
installs CMake/Ninja/Qt/OpenCV using Homebrew, configures the `macos` preset with the
desktop and replay adapters explicitly ON, builds all targets, and runs the
[regression suite](#regression-suite) through the `macos` test preset (including
offscreen Qt tests).

Each run uses a fresh build directory with no workflow dependency/build cache.
GoogleTest is resolved through the existing CMake package lookup or pinned
FetchContent fallback. No secrets or developer-specific paths are required.
The Qt prefix comes from `brew --prefix qt`, and commands run in `Mini_Robot`
relative to the checkout. Configure/build/test failures fail the job; CTest also
fails if no tests are discovered. Logs are available in the repository's Actions
tab under the daily workflow. Running builds are allowed to finish; scheduled
runs share a concurrency group. Both reusable workflows also reject refs other
than `master`. Run the documented presets locally for branch validation.

To run the same commands locally on macOS, start in `Mini_Robot` with installed
Xcode Command Line Tools; remove `build/macos` first for a fresh build:

```sh
brew install cmake ninja qt opencv
cmake --preset macos -DMICRO_ROBOT_BUILD_DESKTOP=ON -DMICRO_ROBOT_BUILD_REPLAY=ON \
  -DCMAKE_PREFIX_PATH="$(brew --prefix qt)"
cmake --build --preset macos --parallel 3
ctest --preset macos
```

This workflow verifies compilation and automated tests, without camera/BLE
hardware or deployment packaging. Windows CI is separate Issue #106.

## Linux continuous integration

The repository-level `.github/workflows/linux-build-test.yml` uses the same
daily caller, read-only checkout, and master-only guard as the macOS workflow. It
runs three jobs on the `ubuntu-24.04` runner:

| Job (status check name) | Preset | Compiler | Builds |
| --- | --- | --- | --- |
| `Ubuntu 24.04 / GCC / Debug` | `linux-gcc` | Ubuntu's default `g++` | Desktop, replay, and all test targets |
| `Ubuntu 24.04 / Clang / Debug` | `linux-clang` | Ubuntu's default `clang++` | Desktop, replay, and all test targets |
| `Ubuntu 24.04 / GCC / Core without Qt and OpenCV` | `core` | Ubuntu's default `g++` | Core libraries and their tests |

The GCC and Clang jobs install the apt packages listed in the
[Ubuntu setup guide](linux-setup.md#build-requirements) and configure with the
desktop and replay adapters explicitly ON. The core job installs only
`build-essential cmake ninja-build libgtest-dev`, so it fails if a core target or
test starts to need Qt or OpenCV. Each job prints the CMake, Ninja, and compiler
versions, builds all of its targets, and runs the [regression suite](#regression-suite)
through its test preset. Qt UI tests use the offscreen platform,
so no display, camera, Bluetooth adapter, or robot is needed. GoogleTest comes from
the system `libgtest-dev` CONFIG package. The jobs do not use a dependency or build
cache. One compiler's failure does not cancel the other job, and each job reports
its own pass/fail status in the daily run in the Actions tab.

To run a job's commands locally on Ubuntu 24.04, start in `Mini_Robot`:

```sh
sudo apt-get update
sudo apt-get install --yes build-essential clang cmake ninja-build \
  qt6-base-dev qt6-multimedia-dev libgtest-dev libopencv-dev
cmake --preset linux-gcc -DMICRO_ROBOT_BUILD_DESKTOP=ON -DMICRO_ROBOT_BUILD_REPLAY=ON
cmake --build --preset linux-gcc --parallel 4
ctest --preset linux-gcc
```

Use `linux-clang` in place of `linux-gcc` for the Clang job. For the core job,
install only `build-essential cmake ninja-build libgtest-dev` and use the commands in
[regression suite](#regression-suite).

### Required status checks

Builds now validate merged `master` snapshots and do not produce per-PR checks.
Repository administrators must remove these build checks from any required
pre-merge checks to avoid blocking PRs while waiting for checks that no longer run.
Branch protection is configured separately from workflow files.

To verify the daily gate locally without hardware or build dependencies:

```sh
# From the repository root, above Mini_Robot:
python3 -m unittest discover -s .github/scripts/tests -p test_daily_build_gate.py -v
```

## Qt-free verification

The `core` preset in [regression suite](#regression-suite) builds without Qt or
OpenCV. To keep the replay adapter, which needs OpenCV but not Qt, use a separate
build directory:

```sh
cmake -S . -B build/qt-free -G Ninja -DCMAKE_BUILD_TYPE=Debug \
  -DMICRO_ROBOT_BUILD_DESKTOP=OFF -DBUILD_TESTING=ON
cmake --build build/qt-free --parallel
ctest --test-dir build/qt-free --output-on-failure --no-tests=error
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
stay unregistered. Keep pure logic tests independent of widgets and physical backends.
A test that needs a physical device gets the `hardware` label and skips without its
device variable; follow the [hardware test rules](hardware-tests.md#rules). Future path
or calibration tests should use deterministic inputs through their module APIs.

## AprilTag pipeline verification

The desktop enables the detector and adds the required OpenCV ArUco/objdetect
component. For a Qt-free detector/replay build use desktop OFF and
`MICRO_ROBOT_BUILD_VISION=ON`; see the [vision build guide](../vision/apriltag-detection.md#build-and-verification).
Run detector, replay and GUI-worker tests together with:

```sh
ctest --test-dir build -R '^(AprilTag|VisionControllerTest)' --output-on-failure --no-tests=error
```

Physical printed-tag detection is not established by generated images or videos;
record the [commissioning checks](../vision/apriltag-detection.md#physical-acceptance-not-run)
separately. Linux/Windows runtime and legacy OpenCV execution were not run locally.
