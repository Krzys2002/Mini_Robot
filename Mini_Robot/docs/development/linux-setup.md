# Ubuntu development setup for the current source

Ubuntu 24.04 LTS is the documented reference environment for these recipes.
The GCC and Clang builds below, including the OpenCV replay adapter, were
executed for #125 on Ubuntu 26.04 LTS (see [verification status](#verification-status));
[Linux CI](testing.md#linux-continuous-integration) (#126) runs the apt installation
and GCC/Clang builds on a clean 24.04 runner; X11/Wayland GUI checks have not been
executed. The other Linux acceptance criteria (#127–#132) are tracked by their own
issues.

## Build requirements

| Requirement | Minimum | Ubuntu 24.04 package |
| --- | --- | --- |
| CMake | 3.24 | `cmake` (3.28) |
| GCC or Clang with C++20 | GCC 11 / Clang 14 | `build-essential` (GCC 13), `clang` (18) |
| Ninja (used by presets and examples) | any recent | `ninja-build` |
| Qt 6 Widgets and Multimedia (desktop build) | Qt 6 | `qt6-base-dev`, `qt6-multimedia-dev` |
| OpenCV core/imgproc/videoio (replay, default ON) | OpenCV 4 | `libopencv-dev` |
| GoogleTest (tests, default ON) | — | `libgtest-dev` (otherwise 1.17.0 is downloaded) |

```sh
sudo apt update
sudo apt install build-essential clang cmake ninja-build qt6-base-dev qt6-multimedia-dev libgtest-dev libopencv-dev
```

Package names are documented by Ubuntu's official catalog:
[CMake](https://packages.ubuntu.com/noble/cmake),
[Qt 6 development files](https://packages.ubuntu.com/noble/qt6-base-dev),
[Qt Multimedia development files](https://packages.ubuntu.com/noble/qt6-multimedia-dev), and
[GoogleTest development files](https://packages.ubuntu.com/noble/libgtest-dev).
Some packages are in Ubuntu's universe component; enable that component if apt
cannot locate them. Ubuntu 22.04 ships CMake 3.22, which is older than the
required 3.24; use a newer CMake there (for example CLion's bundled CMake).
Installing GTest lets CMake use its CONFIG package instead of downloading
1.17.0; the system package may have a different version.

All packages are discovered through `find_package()` from the standard system
prefixes, so no `CMAKE_PREFIX_PATH` or source edits are needed for apt-installed
dependencies. The configure step prints the detected platform and compiler, and
warns if the compiler is not GCC/Clang or is older than the minimum above.

## GCC and Clang builds

Run from `Mini_Robot`. The [CMake presets](build.md#cmake-presets) use a separate
build directory for each compiler (`build/linux-gcc`, `build/linux-clang`):

```sh
cmake --preset linux-gcc
cmake --build --preset linux-gcc
ctest --preset linux-gcc

cmake --preset linux-clang
cmake --build --preset linux-clang
ctest --preset linux-clang
```

Without presets, the equivalent commands are:

```sh
cmake -S . -B build/linux-gcc -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_COMPILER=g++ -DBUILD_TESTING=ON
cmake --build build/linux-gcc --parallel
ctest --test-dir build/linux-gcc --output-on-failure --no-tests=error

cmake -S . -B build/linux-clang -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_COMPILER=clang++ -DBUILD_TESTING=ON
cmake --build build/linux-clang --parallel
ctest --test-dir build/linux-clang --output-on-failure --no-tests=error
```

Append `-DMICRO_ROBOT_BUILD_REPLAY=OFF` to a configure command when OpenCV is not
installed. CTest's Qt test entry point selects offscreen mode. Launch the demo in
a real GUI session with `./build/linux-gcc/src/micro_robot_desktop`. Startup,
resizing, input, display scaling, and X11/Wayland behavior require separate manual
checks. For Wayland plugin issues, inspect the Qt platform plugins installed on the
host; a successful offscreen test does not establish Wayland support.

For Qt-free verification:

```sh
cmake -S . -B build/linux-core -G Ninja -DCMAKE_BUILD_TYPE=Debug   -DMICRO_ROBOT_BUILD_DESKTOP=OFF -DBUILD_TESTING=ON
cmake --build build/linux-core --parallel
ctest --test-dir build/linux-core --output-on-failure --no-tests=error
```

## Verification status

Executed for #125 on Ubuntu 26.04.1 LTS with CMake 4.3.1 and Ninja (both bundled
with CLion), Qt 6.10.2, OpenCV 4.10.0 (`libopencv-dev`), and system GoogleTest
1.17.0. All dependencies were found from `/usr` without `CMAKE_PREFIX_PATH`.

| Configuration | Result |
| --- | --- |
| `linux-gcc` preset, GCC 15.2.0, desktop and replay ON | Configured and built without warnings; 113/113 CTest cases passed (core, ui, replay) |
| `linux-clang` preset, Clang 21.1.8, desktop and replay ON | Configured and built without warnings; 113/113 CTest cases passed (core, ui, replay) |
| GCC 15.2.0, desktop OFF (Qt-free), replay ON | Built; 97/97 CTest cases passed |

For #126, the Ubuntu 24.04 GitHub Actions jobs completed successfully with both
GCC and Clang, including apt dependency installation, build, and all CTest cases:
[GCC run](https://github.com/Krzys2002/Mini_Robot/actions/runs/39126604407/job/117672944975)
and [Clang run](https://github.com/Krzys2002/Mini_Robot/actions/runs/39126604407/job/117672944787).
These results cover the earlier PR revision; updated revisions must pass their own
checks. The minimum compiler versions listed above remain unverified.

## Replay, camera, and BLE dependencies

OpenCV core/imgproc/videoio now decode [recorded local video](../overview/recorded-video.md).
Install `libopencv-dev` for this adapter, or configure `MICRO_ROBOT_BUILD_REPLAY=OFF`
to omit it. Replay tests require an MJPEG writer/decoder. BlueZ is not used by
current [source targets](../../src/CMakeLists.txt); installing it does not enable BLE. Qt Multimedia now queries
the system camera inventory through [the discovery adapter](../overview/camera-discovery.md);
Linux device visibility/hotplug has not been tested. V4L2 capture, configurable
resolution, and permission handling belong to #128; Qt Bluetooth/BlueZ integration
belongs to #129. Those issues must select and document their exact packages.

Camera adapters will need to report missing devices and permissions for
`/dev/video*`; inspect device ownership/ACLs and use the distribution's appropriate
device-access policy. Avoid running the whole GUI as root to bypass access errors.
BLE validation requires a compatible adapter, BlueZ service, and robot hardware;
none are required for the current fake transport or automated domain tests.

[Linux CI](testing.md#linux-continuous-integration) installs the packages above on an
`ubuntu-24.04` GitHub runner and builds and tests the `linux-gcc` and `linux-clang`
presets on every pull request and push to `master`; see that workflow's runs for
results. Configuration/filesystem regression expansion is #132. This guide must be
updated with actual Linux results when those checks are performed.
