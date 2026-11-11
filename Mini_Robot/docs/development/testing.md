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
| `micro_robot_calibration_detector_tests` | `calibration` | Generated chessboards, analytic intrinsic/distortion recovery and metric corner-undistortion tests; built with desktop or standalone calibration enabled |
| `micro_robot_common_tests` | `core` | Robot identity, units, poses, monotonic time, angle normalization |
| `micro_robot_vision_observation_tests` | `core`, `vision` | [Robot marker observations](../vision/robot-observation.md): tag-to-RobotId mapping, unknown/duplicate/invalid rejection, projective center, image and metric heading, calibration and workspace rejection |
| `micro_robot_control_tests` | `core` | [Differential-drive kinematics](../overview/differential-drive.md): analytical wheel targets, inverse conversion, world derivatives, signs, bounds, invalid inputs and overflow |
| `micro_robot_camera_tests` | `core` | Packed frame validation, malformed metadata, storage and overflow checks; camera selection rules |
| `micro_robot_interface_tests` | `core` | Camera/transport contracts and ownership |
| `micro_robot_transport_tests` | `core` | Transport domain versions, sequence reservation, metadata defaults and copying |
| `micro_robot_transport_mock_tests` | `core` | [Robot discovery and the shared transport contract](../overview/robot-communication.md#mock-transport-tests): scans, snapshots, ambiguity, private peer mapping, link loss, and the same `IRobotTransport` checks against simulation and a test-only radio backend |
| `micro_robot_simulation_tests` | `core` | Deterministic backends, fake camera factory, and substitution |
| `micro_robot_world_tests` | `core` | World registry, pose update rules, controlled-time tracker lifecycle and measurement invalidation |
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
that CI runs on macOS, Linux, and Windows. Every test preset applies it (exclude label
`hardware`, output on failure, error when no tests are found):

```sh
ctest --preset macos        # macOS, AppleClang
ctest --preset linux-gcc    # Ubuntu, GCC
ctest --preset linux-clang  # Ubuntu, Clang
ctest --preset windows-msvc # Windows, MSVC
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
snapshot. A separate `pr-verification.yml` workflow runs daily at 02:00 UTC on
`master`: one Ubuntu GCC desktop build with native BLE and marker tools enabled,
all hardware-free CTest cases, repository automation and prototype browser tests.
It runs independently of the merge gate, including on days without new merges.

The gate compares against the last successful daily run, including successful
no-op checks. Missed schedules and failed runs therefore retain pending merges
for the next daily check. Merge detection uses GitHub's merge commit SHA, covering
merge commits, squash merges and rebase merges. Direct pushes alone do not trigger
builds. Until a daily suite succeeds, every daily check builds the actual current
baseline without an age cutoff; an extended Actions outage cannot skip older
unverified merges.
GitHub may delay or drop scheduled runs; this is a daily batch rather than an
immediate post-merge guarantee. The schedule takes effect after human merge to
`master`.

The reusable `.github/workflows/macos-build-test.yml` workflow's
`macOS Apple Silicon / Debug`
job uses the `macos-15` runner and the default AppleClang toolchain. It checks out
the repository with read-only permissions and without persisted credentials,
installs CMake/Ninja/Qt/OpenCV using Homebrew, configures the `macos` preset with the
desktop, replay and BLE adapters explicitly ON, builds all targets, and runs the
[regression suite](#regression-suite) through the `macos` test preset (including
offscreen Qt tests).

Each run uses a fresh build directory with no workflow dependency/build cache.
GoogleTest is resolved through the existing CMake package lookup or pinned
FetchContent fallback. No secrets or developer-specific paths are required.
The Qt prefix comes from `brew --prefix qt`, and commands run in `Mini_Robot`
relative to the checkout. Configure/build/test failures fail the job; CTest also
fails if no tests are discovered. Logs are available in the repository's Actions
tab under the daily workflow. Running builds are allowed to finish; scheduled
runs share a concurrency group. Reusable workflows accept the caller ref; the
daily caller is master-only, while the PR job validates the PR merge ref. Run the
documented presets locally for branch validation.

To run the same commands locally on macOS, start in `Mini_Robot` with installed
Xcode Command Line Tools; remove `build/macos` first for a fresh build:

```sh
brew install cmake ninja qt opencv
cmake --preset macos -DMICRO_ROBOT_BUILD_DESKTOP=ON -DMICRO_ROBOT_BUILD_REPLAY=ON -DMICRO_ROBOT_BUILD_BLE=ON \
  -DCMAKE_PREFIX_PATH="$(brew --prefix qt)"
cmake --build --preset macos --parallel 3
ctest --preset macos
```

This workflow verifies compilation and automated tests, without camera/BLE
hardware or deployment packaging.

## Linux continuous integration

The repository-level `.github/workflows/linux-build-test.yml` uses the same
daily caller and read-only checkout as the macOS workflow. It
runs three jobs on the `ubuntu-24.04` runner:

| Job (status check name) | Preset | Compiler | Builds |
| --- | --- | --- | --- |
| `Ubuntu 24.04 / GCC / Debug` | `linux-gcc` | Ubuntu's default `g++` | Desktop, replay, native BLE and all test targets |
| `Ubuntu 24.04 / Clang / Debug` | `linux-clang` | Ubuntu's default `clang++` | Desktop, replay, native BLE and all test targets |
| `Ubuntu 24.04 / GCC / Core without Qt and OpenCV` | `core` | Ubuntu's default `g++` | Core libraries and their tests |

The GCC and Clang jobs install the apt packages listed in the
[Ubuntu setup guide](linux-setup.md#build-requirements) and configure with the
desktop, replay and BLE adapters explicitly ON. The core job installs only
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
  qt6-base-dev qt6-multimedia-dev qt6-connectivity-dev libgtest-dev libopencv-dev
cmake --preset linux-gcc -DMICRO_ROBOT_BUILD_DESKTOP=ON -DMICRO_ROBOT_BUILD_REPLAY=ON -DMICRO_ROBOT_BUILD_BLE=ON
cmake --build --preset linux-gcc --parallel 4
ctest --preset linux-gcc
```

Use `linux-clang` in place of `linux-gcc` for the Clang job. For the core job,
install only `build-essential cmake ninja-build libgtest-dev` and use the commands in
[regression suite](#regression-suite).

## Windows continuous integration

The reusable `.github/workflows/windows-build-test.yml` uses the daily caller
and read-only checkout. Its Windows jobs remain master-only and run on
`windows-2022`; the affordable Linux PR/manual job remains separate:

| Job (status check name) | Preset | Compiler | Builds |
| --- | --- | --- | --- |
| `Windows Server 2022 / MSVC / Desktop + tests` | `windows-msvc` | MSVC `cl` | Desktop, replay, and all test targets |
| `Windows Server 2022 / MSVC / Core without Qt and OpenCV` | `core` | MSVC `cl` | Core libraries and their tests |

The desktop job installs Ninja and OpenCV through Chocolatey, installs Qt 6.8.3
(`win64_msvc2022_64`) through `aqtinstall`, configures the `windows-msvc` preset
with explicit desktop/replay ON, builds all targets, and runs the
[regression suite](#regression-suite). The core job installs only Ninja and runs
the `core` preset against MSVC, so it fails if core targets or tests gain a Qt/OpenCV
dependency. UI tests still run in their offscreen mode and hardware tests remain
excluded by label.

The prefix step selects an x64 MSVC OpenCV config and verifies its sibling
`bin` contains OpenCV DLLs. It adds that directory and Qt's verified `bin` to
`GITHUB_PATH` before configuration/build, so build-time GoogleTest discovery and
CTest can load the runtime libraries. `CMAKE_PREFIX_PATH` alone does not configure
DLL loading. Compiler version reporting reads `cl.exe` file metadata rather than
invoking `cl /Bv` without a source file (which exits with D8003).

Windows runtime verification is **pending**: the workflow is restricted to daily
merged `master` snapshots, so authoring/linting it does not prove a Windows build
or test pass. Require a successful Windows run before claiming #106 validation.

To run the same desktop commands locally from a Visual Studio developer terminal
in `Mini_Robot`:

```powershell
python -m pip install aqtinstall==3.2.1
python -m aqt install-qt windows desktop 6.8.3 win64_msvc2022_64 -m qtmultimedia -O C:\Qt
# Use the bin paired with your selected x64 MSVC OpenCV config.
$env:PATH = "C:\Qt\6.8.3\msvc2022_64\bin;C:\tools\opencv\build\x64\vc16\bin;$env:PATH"
cmake --preset windows-msvc -DMICRO_ROBOT_BUILD_DESKTOP=ON -DMICRO_ROBOT_BUILD_REPLAY=ON `
  -DCMAKE_PREFIX_PATH="C:\Qt\6.8.3\msvc2022_64;C:\path\to\OpenCV\cmake"
cmake --build --preset windows-msvc --parallel 4
ctest --preset windows-msvc
```

Runtime packaging is handled separately; see [Windows deployment](windows-deployment.md).

### Required status checks

The desktop workflow no longer provides a pull request status check. If branch
protection requires `Pull request verification / desktop`, an administrator must
update that rule separately. The daily GCC/Clang/macOS/Windows matrix provides
broader post-merge coverage. Billing/spending-limit failures
that prevent a runner starting are infrastructure failures, not compiler/test
results; only executed jobs count as validation.

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
For multiple labels, apply `set_tests_properties(... PROPERTIES LABELS "core;ble")`
in a CTest script registered with the directory's `TEST_INCLUDE_FILES` after the
corresponding discovery calls. Use the populated `<target>_TESTS` list and guard
against an empty list when that executable has not been built. Do not pass escaped
semicolon lists through `gtest_discover_tests`: parsing differs between CMake
versions. See `tests/DiscoveredCoreLabels.cmake` for BLE/calibration and
`tests/vision/RobotObservationLabels.cmake` for vision. Check the resulting LABELS
arrays with `ctest --show-only=json-v1` and run exact-label filters such as
`ctest --preset core -L '^ble$' --no-tests=error`.
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

## Guided workspace frame regression (#246)

`WorkspaceBoundaryTest`, `WorkspaceFrameTest`, `WorkspaceProfileTest`,
`OpenCvWorkspaceHomographyTest` and `WorkspaceCornerWorkflowTest` cover ordered
perimeters in both image windings, signed origin geometry, native projection/view
round trips, v1-to-v2 migration, exact frame persistence, stale-worker rejection
and downstream measurement invalidation. `QtWorkspaceProfileStoreTest` checks
signed metadata through atomic filesystem storage.

Run the core preset plus the host desktop preset. A focused desktop selection is:

```sh
ctest --test-dir build/macos -R 'Workspace|TabletopHomography|MetricObservation' --output-on-failure --no-tests=error
QT_SCALE_FACTOR=1.5 build/macos/Mini_Robot/tests/micro_robot_tests --gtest_filter='WorkspaceCornerWorkflowTest.*:WorkspaceCalibrationUiTest.Dialog*'
QT_SCALE_FACTOR=2 build/macos/Mini_Robot/tests/micro_robot_tests --gtest_filter='WorkspaceCornerWorkflowTest.*:WorkspaceCalibrationUiTest.Dialog*'
```

These paths are for root builds; standalone executable paths omit `Mini_Robot/`.
The offscreen entry point does not establish native-display or physical-camera
acceptance. Those new checks belong to #259 and remain NOT RUN until executed.

## Workspace chessboard acquisition (#248)

Run root `core` and host presets. Also configure/build/test `Mini_Robot/` with its
own core preset/build directory when validating the shared target integration.
Offline builds can supply the existing GoogleTest source using
`-DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=/path/to/existing/googletest`.

Focused commands from the repository root:

```sh
ctest --test-dir build/macos -R 'Workspace|CalibrationDetectorWorkspace' --output-on-failure --no-tests=error
QT_SCALE_FACTOR=1.5 build/macos/Mini_Robot/tests/micro_robot_tests --gtest_filter='WorkspaceBoardAcquisitionUiTest.*'
QT_SCALE_FACTOR=2 build/macos/Mini_Robot/tests/micro_robot_tests --gtest_filter='WorkspaceBoardAcquisitionUiTest.*'
python3 tools/calibration/audit_workspace_solver.py --binary build/macos/Mini_Robot/tests/micro_robot_calibration_detector_tests
```

The memory audit is opt-in and skipped by default CTest. Its isolated Unix child
processes distinguish legacy full-U, thin-U and complete production-solver RSS/time;
there are no hardware-independent timing assertions. The normal deterministic
capacity tests exercise exact four-point nullspace, 1,525/4,096-point fitting,
separate Check points, 40 reviewed placements yielding 1,525 unique locations,
8,192-row table models, metadata/byte bounds, original review masks, manual
corrections, v1/v2 migration and immutable linked records. UI tests cover exact
frozen-frame ownership, stale work, explicit orientation/identity, labelled zoom,
relative-anchor snapshots, active/draft evidence and camera/origin invalidation.

[The operator/capacity/provenance guide](../overview/workspace-chessboard.md)
records the contract and measured host evidence. All new physical/native-display
acceptance remains NOT RUN in #263. Synthetic process/geometry results do not
establish physical board accuracy or a motion loop.

## Workspace spatial diagnostics (#249)

Software evidence on 2026-11-11, macOS arm64 / AppleClang 21 / Qt 6.12.0 /
OpenCV 5.1.0, child of #248 PR #264 at `cdeb2d10e33c0d1f6194cc7df9f4a1e9c66cfb6d`.
The combined tree includes #246 and #248; independent #247 PR #262 is not in this
lineage. Tested implementation commits: core `6ffaad7`, UI/application `a77cc25`.
These results do not establish reviews/CI or physical acceptance.

From the issue worktree root, configured and built both `core` and `macos` using
`cmake --preset <preset>`, `cmake --build --preset <preset> --parallel 3`, then
`ctest --preset <preset> --parallel 3`. Both configure commands used the existing
offline GoogleTest source via `-DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=/path/to/googletest`.
Root core: **305 passed**. Root desktop: **502 passed, one opt-in SVD audit skipped
(503 selected), zero failures**; the production application built. The independent
standalone entry point (`cd Mini_Robot`, same core configure/build/CTest commands
and its own build directory) also passed **305** tests.

The core `WorkspaceDiagnosticsTest.*` suite covers analytic affine/perspective
scales, homogeneous-scale invariance, signed origins, relative denominators,
fit/native/horizon/survey support, survey gates, Student-t reference quantiles,
synthetic bias/noise and capacity. A seeded 300-trial fixed-predictor Gaussian
simulation with correlated X/Y errors had 298 true errors inside the conservative
point-radius bounds. This is an implementation check under the synthetic model,
not evidence of physical 95% coverage. A complete 8192-reference (4096 per role),
4225-sample run retained all data and took approximately 0.89–0.90 s in the local
root/standalone Debug suites; no portable timing guarantee is asserted.

Focused desktop tests:

```sh
QT_QPA_PLATFORM=offscreen build/macos/Mini_Robot/tests/micro_robot_tests \
  '--gtest_filter=WorkspaceDiagnostics*'
QT_QPA_PLATFORM=offscreen QT_SCALE_FACTOR=1.5 build/macos/Mini_Robot/tests/micro_robot_tests \
  '--gtest_filter=WorkspaceDiagnosticsUiTest.*'
QT_QPA_PLATFORM=offscreen QT_SCALE_FACTOR=2 build/macos/Mini_Robot/tests/micro_robot_tests \
  '--gtest_filter=WorkspaceDiagnosticsUiTest.*'
```

The focused lifecycle/export/UI/integration run passed **7/7**. Each scale-factor
run passed **6/6**. Screenshots at both scales were retained locally and reviewed;
all five map modes render, with labelled units/ranges, reference positions and
grey unavailable cells. Set `MICRO_ROBOT_MAP_SCREENSHOT=/path/map.png` to retain
the evidence-controls test image. The source integration test verifies binding
to active evidence, draft-edit isolation and optical invalidation. Queue tests
pause a worker and replace 100 pending requests; only the running and latest
request execute. Export tests parse complete profile/acquisition text, model,
IDs, observations and null masks, and exercise a failed destination.

The [operator/model guide](../overview/workspace-diagnostics.md) defines evidence,
coverage, correlations, support and export semantics. Physical scale/error/model
coverage and native-display acceptance belong to
[#265](https://github.com/Krzys2002/Mini_Robot/issues/265), related to #63/#224.
**All new physical-camera/native-display checks: NOT RUN.** Linux, Windows,
OpenCV 4 and native input/display behavior were not exercised by these offscreen
macOS tests. Source inspection used `rg` because this worktree had no local graph;
no generated graph or another worktree's cache was used.

## Two-point workspace distance (#250)

Software evidence on 2026-11-11, macOS arm64 / AppleClang 21 / Qt 6.12.0 /
OpenCV 5.1.0. Approved parent is #249 PR #267 at
`1d147fd41fd6f9b791dd07c063fe69e34b14676e`; #246/#248 are in that unmerged
lineage and independent #247 PR #262 is not. Tested implementation commits:
core `f9009ef`, application/UI `6553ea3`. These results do not establish physical
accuracy, actual coverage, human review or remote CI.

From the issue worktree root, ran `cmake --preset core` and `cmake --preset macos`,
using the existing offline GoogleTest source through
`-DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=/path/to/googletest`, followed by
`cmake --build --preset <preset> --parallel 3` and
`ctest --preset <preset> --parallel 3` for each preset. Root core:
**314 passed**. Root desktop: **519 passed, one existing opt-in SVD audit skipped
(520 selected), zero failures**; the production application built. The independent
standalone `Mini_Robot/` entry point was configured/built/tested with its core
preset and its own build directory: **314 passed**.

Focused core `WorkspaceDistanceTest.*` cases are included in both complete core
runs. Nine cases cover known lengths/directions, signed origins and perspective,
zero/near-zero length, logical viewport/pixel-centre round trips, invalid centres,
missing/negative/unjustified selection bounds, projective-horizon footprints,
image-fit hull constraints and conservative calibration/selection propagation.
Tests enclose full-circle perturbations and compare the zero-selection result to
the exact sum of #249's per-endpoint prediction envelopes; independent-error RSS
is not used.

Executed the eight `WorkspaceDistanceUiTest.*` cases offscreen, separately at
`QT_SCALE_FACTOR=1.5` and `2`: **8/8 each**. Both supported views were rendered and
screenshots reviewed. The tests exercise real source-controller loading/activation,
separate measurement freezing without replacing calibration references, stale/bad
undistortion and late optical results, GUI event delivery while a worker is held,
camera/world endpoint preservation, coordinate editing and bound resets, method
and units handling, re-freeze/clear, pending evidence with old ± withheld,
resolution/camera invalidation, draft-vs-active origin changes and larger loaded
coordinate ranges. Synthetic undistortion fakes validate lifecycle/metadata, not
physical optical correction; existing production OpenCV undistorter tests remain
in the host suite.

```sh
QT_QPA_PLATFORM=offscreen QT_SCALE_FACTOR=1.5 build/macos/Mini_Robot/tests/micro_robot_tests \
  '--gtest_filter=WorkspaceDistanceUiTest.*'
QT_QPA_PLATFORM=offscreen QT_SCALE_FACTOR=2 build/macos/Mini_Robot/tests/micro_robot_tests \
  '--gtest_filter=WorkspaceDistanceUiTest.*'
```

Set `MICRO_ROBOT_DISTANCE_SCREENSHOT=/path/distance.png` to retain the evidence
controls/view-switching test's world image and `/path/distance.png.camera.png`
camera image. The black camera raster in these fixtures is synthetic, not physical
equipment evidence. Measurement capture reuses the serialized calibration worker;
no new frame queue or distance worker is introduced. This worktree's local graph
was missing, so navigation used source/`rg` fallback; no foreign graph/cache was
used or generated output committed.

The [operator and propagation guide](../overview/workspace-distance.md) defines
pixel conventions, support, the projective disk bound and conditional coverage.
Physical length residuals, selection-budget justification, actual coverage and
native-display acceptance are owned by
[#268](https://github.com/Krzys2002/Mini_Robot/issues/268), with calibration-model
validation in #265 and related #63/#224 evidence. **All new physical camera and native display checks: NOT RUN.** Linux, Windows, OpenCV 4 and native pointer/display
behavior were not tested here. Fixed-pair conditional model coverage does not
establish a general accuracy guarantee, 3D measurement, motion control or robot
safety.
