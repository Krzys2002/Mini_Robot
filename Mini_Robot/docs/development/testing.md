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

## Finding suites and coverage

Feature coverage, adapter checks and remaining acceptance belong to the owning
[module page](../overview/modules/README.md) and its behavior guide. Enabled test
targets and their sources appear in the [generated inventory](../overview/source-map.md).
List the actual CTest selection after building rather than updating a shared
per-feature table:

```sh
ctest --test-dir build -N
ctest --test-dir build --show-only=json-v1
```

Core tests use GoogleTest without a QApplication; UI tests use an offscreen Qt
application. The `core` preset requires neither Qt nor OpenCV. Tests needing real
devices use the `hardware` label and are excluded from the regression presets;
see [hardware tests](hardware-tests.md).

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

The suite includes pure domain contracts, deterministic simulation/replay,
configuration/filesystem behavior, offscreen application integration and
platform-boundary checks. Concrete coverage belongs to module guides; the
configured CTest inventory is authoritative for names and labels.

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

The repository-level `.github/workflows/macos-build-test.yml` runs when a pull
request is opened, reopened, or updated with new commits, and on pushes to `master`.
Its `macOS Apple Silicon / Debug`
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
tab and the pull request checks. New pull request runs cancel superseded runs for
the same ref; a running `master` run is allowed to finish so the README badge reports
a completed result. Pushes to other branches do not trigger it, which avoids
duplicate runs for PR branches. Manual
`workflow_dispatch` runs can validate a selected branch even when PR merge
conflicts prevent automatic CI:

```sh
gh workflow run macos-build-test.yml --repo Krzys2002/Mini_Robot --ref your-branch
```

To run the same commands locally on macOS, start in `Mini_Robot` with installed
Xcode Command Line Tools; remove `build/macos` first for a fresh build:

```sh
brew install cmake ninja qt opencv
cmake --preset macos -DMICRO_ROBOT_BUILD_DESKTOP=ON -DMICRO_ROBOT_BUILD_REPLAY=ON \
  -DMICRO_ROBOT_BUILD_DOCUMENTATION=ON \
  -DCMAKE_PREFIX_PATH="$(brew --prefix qt)"
cmake --build --preset macos --parallel 3
ctest --preset macos
```

This workflow verifies compilation and automated tests, without camera/BLE
hardware or deployment packaging. Windows CI is separate Issue #106.

## Linux continuous integration

The repository-level `.github/workflows/linux-build-test.yml` uses the same
triggers, permissions, checkout, and concurrency rules as the macOS workflow. It
runs three jobs on the `ubuntu-24.04` runner:

| Job (status check name) | Preset | Compiler | Builds |
| --- | --- | --- | --- |
| `Ubuntu 24.04 / GCC / Debug` | `linux-gcc` | Ubuntu's default `g++` | Desktop, replay, and all test targets |
| `Ubuntu 24.04 / Clang / Debug` | `linux-clang` | Ubuntu's default `clang++` | Desktop, replay, and all test targets |
| `Ubuntu 24.04 / GCC / Core without Qt and OpenCV` | `core` | Ubuntu's default `g++` | Core libraries and their tests |

The GCC and Clang jobs install the apt packages listed in the
[Ubuntu setup guide](linux-setup.md#build-requirements) and configure with the
desktop and replay adapters explicitly ON. The core job installs only
`build-essential cmake ninja-build libgtest-dev python3`, so it fails if a core
target or test starts to need Qt or OpenCV. Python runs documentation tooling
only; ordinary core builds do not require it. Each job prints the CMake, Ninja, and compiler
versions, builds all of its targets, and runs the [regression suite](#regression-suite)
through its test preset. Qt UI tests use the offscreen platform,
so no display, camera, Bluetooth adapter, or robot is needed. GoogleTest comes from
the system `libgtest-dev` CONFIG package. The jobs do not use a dependency or build
cache. One compiler's failure does not cancel the other job, and each job reports
its own pass/fail status on the pull request and in the Actions tab.

To run a job's commands locally on Ubuntu 24.04, start in `Mini_Robot`:

```sh
sudo apt-get update
sudo apt-get install --yes build-essential clang cmake ninja-build \
  qt6-base-dev qt6-multimedia-dev libgtest-dev libopencv-dev python3
cmake --preset linux-gcc -DMICRO_ROBOT_BUILD_DESKTOP=ON -DMICRO_ROBOT_BUILD_REPLAY=ON \
  -DMICRO_ROBOT_BUILD_DOCUMENTATION=ON
cmake --build --preset linux-gcc --parallel 4
ctest --preset linux-gcc
```

Use `linux-clang` in place of `linux-gcc` for the Clang job. For the core job,
install only `build-essential cmake ninja-build libgtest-dev` and use the commands in
[regression suite](#regression-suite). Start a manual run with:

```sh
gh workflow run linux-build-test.yml --repo Krzys2002/Mini_Robot --ref your-branch
```

### Required status checks

Branch protection can require these job names as status checks:
`macOS Apple Silicon / Debug`, `Ubuntu 24.04 / GCC / Debug`,
`Ubuntu 24.04 / Clang / Debug`, and `Ubuntu 24.04 / GCC / Core without Qt and OpenCV`. GitHub lists a check as selectable only after it
has run at least once in the repository. Renaming a job or matrix entry changes
its check name, so update the branch protection rule at the same time. Branch
protection is configured by a repository administrator; it is not part of the
workflow files.

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

## Documentation checks

Enable `MICRO_ROBOT_BUILD_DOCUMENTATION=ON` to include the offline link/anchor
checker and tooling tests in CTest (label `documentation`). The option also
provides a `documentation` build target and build-local CMake source/target map.
CI enables it for every platform configuration, including the core job. Python
3.9+ is required only when this option is enabled. See
[documentation maintenance](documentation.md) for commands and ownership rules.
