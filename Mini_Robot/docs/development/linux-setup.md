# Ubuntu development setup for the current source

Ubuntu 24.04 LTS is the documented reference environment for these recipes.
This audit ran on macOS; it did not execute apt, Linux builds, or X11/Wayland
GUI checks. The instructions prepare the existing simulation demo and are not
proof that all Linux acceptance criteria (#125–#132) have been satisfied.

## Packages

```sh
sudo apt update
sudo apt install build-essential clang cmake ninja-build qt6-base-dev qt6-multimedia-dev libgtest-dev
```

Package names are documented by Ubuntu's official catalog:
[CMake](https://packages.ubuntu.com/noble/cmake),
[Qt 6 development files](https://packages.ubuntu.com/noble/qt6-base-dev),
[Qt Multimedia development files](https://packages.ubuntu.com/noble/qt6-multimedia-dev), and
[GoogleTest development files](https://packages.ubuntu.com/noble/libgtest-dev).
Some packages are in Ubuntu's universe component; enable that component if apt
cannot locate them. Check `cmake --version` (minimum 3.24) and compiler support
for C++20. Installing GTest lets CMake use its CONFIG package instead of downloading
1.17.0; the system package may have a different version.

## GCC and Clang builds

Run from `Mini_Robot`. Use separate directories for each compiler:

```sh
cmake -S . -B build/linux-gcc -G Ninja -DCMAKE_BUILD_TYPE=Debug   -DCMAKE_CXX_COMPILER=g++ -DMICRO_ROBOT_BUILD_DESKTOP=ON -DBUILD_TESTING=ON
cmake --build build/linux-gcc --parallel
ctest --test-dir build/linux-gcc --output-on-failure --no-tests=error

cmake -S . -B build/linux-clang -G Ninja -DCMAKE_BUILD_TYPE=Debug   -DCMAKE_CXX_COMPILER=clang++ -DMICRO_ROBOT_BUILD_DESKTOP=ON -DBUILD_TESTING=ON
cmake --build build/linux-clang --parallel
ctest --test-dir build/linux-clang --output-on-failure --no-tests=error
```

CTest's Qt test entry point selects offscreen mode. Launch the demo in a real
GUI session with `./build/linux-gcc/src/micro_robot_desktop`. Startup, resizing,
input, display scaling, and X11/Wayland behavior require separate manual checks.
For Wayland plugin issues, inspect the Qt platform plugins installed on the host;
a successful offscreen test does not establish Wayland support.

For Qt-free verification:

```sh
cmake -S . -B build/linux-core -G Ninja -DCMAKE_BUILD_TYPE=Debug   -DMICRO_ROBOT_BUILD_DESKTOP=OFF -DBUILD_TESTING=ON
cmake --build build/linux-core --parallel
ctest --test-dir build/linux-core --output-on-failure --no-tests=error
```

## Planned camera and BLE dependencies

OpenCV and BlueZ are not used by current [source targets](../../src/CMakeLists.txt),
so installing them will not enable capture or BLE. Qt Multimedia now queries
the system camera inventory through [the discovery adapter](../overview/camera-discovery.md);
Linux device visibility/hotplug has not been tested. V4L2 capture, configurable
resolution, and permission handling belong to #128; Qt Bluetooth/BlueZ integration
belongs to #129. Those issues must select and document their exact packages.

Camera adapters will need to report missing devices and permissions for
`/dev/video*`; inspect device ownership/ACLs and use the distribution's appropriate
device-access policy. Avoid running the whole GUI as root to bypass access errors.
BLE validation requires a compatible adapter, BlueZ service, and robot hardware;
none are required for the current fake transport or automated domain tests.

The committed build/test workflow currently targets macOS only. Ubuntu CI is
planned by #126; configuration/filesystem regression expansion is #132. This guide
must be updated with actual Linux results when those checks are performed.
