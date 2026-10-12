# GoogleTest and CTest

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
| `micro_robot_interface_tests` | `core` | Camera/transport contracts and ownership |
| `micro_robot_simulation_tests` | `core` | Deterministic backends and substitution |
| `micro_robot_world_tests` | `core` | World registry and pose update rules |
| `micro_robot_logging_tests` | `core` | Log formatting, levels, channels, sink failures, file sink |
| `micro_robot_tests` | `ui` | Offscreen Qt window and controller integration |

Core executables link domain libraries and `GTest::gtest_main`, without Qt or a
`QApplication`. The UI executable has its own test entry point using GoogleTest
and an offscreen `QApplication`; it is built only when
`MICRO_ROBOT_BUILD_DESKTOP=ON`.

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

## Qt-free verification

Use a separate build directory to verify the core without discovering Qt:

```sh
cmake -S . -B build/core -G Ninja -DCMAKE_BUILD_TYPE=Debug \
  -DMICRO_ROBOT_BUILD_DESKTOP=OFF -DBUILD_TESTING=ON
cmake --build build/core --parallel
ctest --test-dir build/core -L core --output-on-failure --no-tests=error
```

## Adding tests

`tests/common/TypesTest.cpp` contains the existing example
`TEST(RobotIdTest, ComparesByValue)`, which checks value comparison through the
public domain API. Follow its GoogleTest style for new non-GUI logic tests.

Add a test source to the corresponding executable in `tests/CMakeLists.txt` and
link the domain target it exercises. For a new module, create a separate executable
linked with `GTest::gtest_main`, apply `micro_robot_target_defaults()`, and register
it with `gtest_discover_tests(... DISCOVERY_MODE PRE_TEST PROPERTIES LABELS "core")`.
Keep pure logic tests independent of widgets and physical backends. Future path
or calibration tests should use deterministic inputs through their module APIs.
