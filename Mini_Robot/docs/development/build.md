# Build and development setup

Source: [root CMakeLists.txt](../../CMakeLists.txt),
[source targets](../../src/CMakeLists.txt), and
[test targets](../../tests/CMakeLists.txt).
Commands run from the checkout's `Mini_Robot` directory; GitHub workflows live
one level above it in the repository's `.github` directory.

## Required dependencies

- A C++20 compiler and CMake 3.24 or newer.
- Ninja for the examples below; other CMake generators may also be used.
- Qt 6 Widgets and Multimedia for the desktop build. Qt Core is used by the
  application controllers; Multimedia is isolated in the device-discovery adapter.
- GoogleTest when BUILD_TESTING is ON (the default). CMake first uses an installed
  GTest CONFIG package, then downloads pinned GoogleTest 1.17.0 if unavailable.

The current source does not discover or link OpenCV, Eigen, spdlog, JSON,
Qt Bluetooth, or network libraries. They are planned dependencies,
not prerequisites for today's simulation demo. The optional runtime settings file is
described in [Application configuration](../overview/configuration.md).

Qt Multimedia supplies read-only [system camera discovery](../overview/camera-discovery.md).
It does not enable physical capture or replace the fake source. Install this
component from the same Qt distribution/version as Widgets.

## macOS

Install Xcode Command Line Tools, then install CMake, Ninja, and Qt through
Homebrew or provide equivalent installations. With Homebrew:

```sh
brew install cmake ninja qt
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
built for that toolchain.
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

This omits Qt discovery and all GUI targets. All domain tests, including the
unlabelled camera tests, run when no label filter is given. For offline builds,
install GTest or add `-DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=/path/to/googletest`.
Use `-DBUILD_TESTING=OFF` to omit tests and the GTest dependency entirely.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| Qt6 package not found | Pass the matching installation prefix via CMAKE_PREFIX_PATH, or disable desktop for a domain-only build. |
| GoogleTest download fails | Network access or installed GTest; alternatively provide an existing source checkout. |
| No tests found | Configure BUILD_TESTING=ON, build test targets, then run CTest from that build directory. |
| Old compiler/Qt or generator persists | Use a fresh build directory when switching toolchains/platforms. |
| UI tests need a display | The UI test entry point sets QT_QPA_PLATFORM=offscreen; domain tests need no QApplication. |

See [testing](testing.md) for suite selection and test additions, and
[the source map](../overview/source-map.md) for CMake target ownership.
