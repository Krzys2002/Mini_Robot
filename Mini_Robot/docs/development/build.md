# Build and development setup

Source: [root CMakeLists.txt](../../CMakeLists.txt),
[platform/toolchain module](../../cmake/MicroRobotPlatform.cmake),
[CMake presets](../../CMakePresets.json),
[source targets](../../src/CMakeLists.txt), and
[test targets](../../tests/CMakeLists.txt).
Commands run from the checkout's `Mini_Robot` directory; GitHub workflows live
one level above it in the repository's `.github` directory.

## Required dependencies

- A C++20 compiler and CMake 3.24 or newer.
- Ninja for the examples below; other CMake generators may also be used.
- Qt 6 Widgets and Multimedia for the desktop build. Qt Core is used by the
  application controllers; Multimedia is isolated in the device-discovery adapter.
- OpenCV core, imgproc, and videoio for recorded-video replay (default ON).
  Set `-DMICRO_ROBOT_BUILD_REPLAY=OFF` to omit that adapter and its tests.
- GoogleTest when BUILD_TESTING is ON (the default). CMake first uses an installed
  GTest CONFIG package, then downloads pinned GoogleTest 1.17.0 if unavailable.

Configuration fails if a platform-independent target links a platform library
or a concrete backend; see [platform boundaries](../overview/platform-boundaries.md).

The current source does not discover or link Eigen, spdlog, JSON,
Qt Bluetooth, or network libraries. They are planned dependencies,
not prerequisites for today's simulation demo. The optional runtime settings file is
described in [Application configuration](../overview/configuration.md).

Qt Multimedia supplies read-only [system camera discovery](../overview/camera-discovery.md).
It does not enable physical capture or replace the fake source. Install this
component from the same Qt distribution/version as Widgets.

## Platform and compiler configuration

One CMake configuration serves macOS, Linux, and Windows; switching platform or
compiler needs a fresh build directory but no edits to CMake files.

- The root `CMakeLists.txt` sets C++20 (required, no compiler extensions) for every
  target, including a downloaded GoogleTest.
- [cmake/MicroRobotPlatform.cmake](../../cmake/MicroRobotPlatform.cmake) holds all
  operating-system and compiler conditionals: OS detection, compiler checks, warning
  flags (`micro_robot_target_defaults()`), and desktop executable packaging
  (`micro_robot_configure_desktop_executable()`: app bundle on macOS, GUI subsystem
  on Windows). `src/` and `tests/` CMake files stay platform-neutral.
- Qt 6, OpenCV, and GoogleTest are located with `find_package()`. No Homebrew,
  SDK, or distribution-specific paths are hardcoded; non-standard installations
  are supplied at configure time through `CMAKE_PREFIX_PATH` or `<Package>_DIR`.
- Configure prints the detected platform and compiler, for example
  `Micro Robot: Linux with GNU 15.2.0, C++20`.

| Platform | Supported compilers | Minimum | Validation |
| --- | --- | --- | --- |
| macOS | Apple Clang | 14 | macOS CI (Debug) |
| Linux | GCC, Clang | GCC 11, Clang 14 | Local builds; see [Ubuntu setup](linux-setup.md#verification-status) |
| Windows | MSVC | 19.29 (VS 2019 16.10) | Portability target only; not validated (#106) |

Other compilers, older versions, or other platforms produce a configure warning
but are not blocked.

### CMake presets

[CMakePresets.json](../../CMakePresets.json) provides Debug presets with tests
enabled, each building into `build/<preset>`. Presets are filtered by host, so
`cmake --list-presets` shows only those for the current OS.

| Preset | Host | Compiler |
| --- | --- | --- |
| `macos` | macOS | `clang++` (Apple Clang) |
| `linux-gcc` | Linux | `g++` |
| `linux-clang` | Linux | `clang++` |

Each name works for configure, build, and test:

```sh
cmake --preset linux-gcc
cmake --build --preset linux-gcc
ctest --preset linux-gcc
```

Extra cache variables can be appended to the configure command, for example
`cmake --preset macos -DCMAKE_PREFIX_PATH="$(brew --prefix qt)"`. Machine-specific
presets belong in an untracked `CMakeUserPresets.json`. CLion and other
preset-aware IDEs list the presets as build profiles. Build directories
(`build/`, CLion's `cmake-build-*/`, Visual Studio's `out/`) and
`CMakeUserPresets.json` are ignored by Git.

## macOS

Install Xcode Command Line Tools, then install CMake, Ninja, and Qt through
Homebrew or provide equivalent installations. With Homebrew:

```sh
brew install cmake ninja qt opencv
cmake -S . -B build/debug -G Ninja -DCMAKE_BUILD_TYPE=Debug   -DCMAKE_PREFIX_PATH="$(brew --prefix qt)" -DBUILD_TESTING=ON
cmake --build build/debug --parallel 3
ctest --test-dir build/debug --output-on-failure --no-tests=error
open build/debug/src/micro_robot_desktop.app
```

For Release, use a separate build directory and `-DCMAKE_BUILD_TYPE=Release`.
The app links installed Qt libraries; it is not a deployed standalone bundle.
The current [macOS CI](testing.md#macos-continuous-integration) uses Debug only.

## Ubuntu and Windows

The [Ubuntu setup guide](linux-setup.md) provides GCC/Clang and headless recipes
for the current source, with explicit verification limits.

For Windows, install a C++20 toolchain, CMake, and Qt 6 Widgets/Multimedia
built for that toolchain, plus compatible OpenCV core/imgproc/videoio packages.
Run from its developer terminal and pass the matching Qt prefix:

```sh
cmake -S . -B build/windows -DCMAKE_PREFIX_PATH="C:/Qt/your-compatible-Qt-prefix" -DBUILD_TESTING=ON
cmake --build build/windows --config Debug --parallel
ctest --test-dir build/windows -C Debug --output-on-failure --no-tests=error
```

The prefix is a placeholder; use your actual compatible installation. Visual
Studio multi-configuration generators select Debug at build/test time. Windows
commands have not been executed in this audit; Windows CI/deployment remain #106.

## Domain-only and offline builds

```sh
cmake -S . -B build/core -G Ninja -DCMAKE_BUILD_TYPE=Debug   -DMICRO_ROBOT_BUILD_DESKTOP=OFF -DBUILD_TESTING=ON
cmake --build build/core --parallel
ctest --test-dir build/core --output-on-failure --no-tests=error
```

This omits Qt discovery and all GUI targets, but includes the OpenCV replay adapter.
Disable `MICRO_ROBOT_BUILD_REPLAY` as well for a build with neither Qt nor OpenCV. All domain tests, including the
unlabelled camera tests, run when no label filter is given. For offline builds,
install GTest or add `-DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=/path/to/googletest`.
Use `-DBUILD_TESTING=OFF` to omit tests and the GTest dependency entirely.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| Qt6 package not found | Pass the matching installation prefix via CMAKE_PREFIX_PATH, or disable desktop for a domain-only build. |
| OpenCV package not found | Install OpenCV or set OpenCV_DIR to its CMake package directory; disable replay to omit it. |
| GoogleTest download fails | Network access or installed GTest; alternatively provide an existing source checkout. |
| No tests found | Configure BUILD_TESTING=ON, build test targets, then run CTest from that build directory. |
| Old compiler/Qt or generator persists | Use a fresh build directory when switching toolchains/platforms. |
| Configure warns that a compiler is unsupported or too old | Use a compiler from the platform table above, e.g. via a preset or `-DCMAKE_CXX_COMPILER`. |
| A preset is missing from `cmake --list-presets` | Presets are host-specific; use one for the current OS, and CMake 3.24 or newer. |
| UI tests need a display | The UI test entry point sets QT_QPA_PLATFORM=offscreen; domain tests need no QApplication. |

See [testing](testing.md) for suite selection and test additions, and
[the source map](../overview/source-map.md) for CMake target ownership.
