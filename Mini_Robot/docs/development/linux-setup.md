# Ubuntu development setup

This guide takes a clean Ubuntu LTS installation to a built and tested checkout of
the desktop application. It covers the toolchain, libraries, camera access, BlueZ,
and troubleshooting. General CMake options shared with macOS and Windows are in
[build setup](build.md); suite selection and test additions are in
[testing](testing.md). The [verification status](#verification-status) lists the
Linux builds executed so far; X11/Wayland GUI checks belong to #130 and have not
been executed.

## Supported Ubuntu versions

| Ubuntu release | Status |
| --- | --- |
| **24.04 LTS (noble), x86-64** | Reference environment. Every package below comes from the Ubuntu archive. [Linux CI](testing.md#linux-continuous-integration) builds and tests it with GCC and Clang. |
| 26.04 LTS, x86-64 | Same packages and commands. Used for the local builds in [verification status](#verification-status). |
| 22.04 LTS | Not supported as documented: its CMake 3.22 is older than the required 3.24, so the configure step and presets fail. A newer CMake (for example CLion's bundled CMake) is needed; the rest of this setup has not been tried there. |
| Other distributions, arm64 | Not documented or verified. Install the equivalents of the packages below; all of them are found through `find_package()`. |

## Quick start on a clean Ubuntu 24.04

1. Install the tools and libraries (about 230 MB download and 950 MB installed on
   a GitHub `ubuntu-24.04` runner):

   ```sh
   sudo apt update
   sudo apt install git ca-certificates build-essential clang cmake ninja-build \
     qt6-base-dev qt6-multimedia-dev libgtest-dev libopencv-dev
   ```

   Several packages are in Ubuntu's universe component, which a desktop
   installation enables by default. If apt cannot locate a package, run
   `sudo add-apt-repository universe` and repeat the install.

2. Clone the repository. The CMake project is in its `Mini_Robot` subdirectory,
   and all later commands run from there:

   ```sh
   git clone https://github.com/Krzys2002/Mini_Robot.git
   cd Mini_Robot/Mini_Robot
   ```

3. Configure, build, and run every test with GCC:

   ```sh
   cmake --preset linux-gcc
   cmake --build --preset linux-gcc
   ctest --preset linux-gcc
   ```

   The configure output includes `Micro Robot: Linux with GNU <version>, C++20` and
   `Micro Robot: live camera backend V4L2`. CTest ends with
   `100% tests passed`. Without a camera, the two `V4l2CameraHardwareTest` cases
   are reported as skipped; this is expected.

4. Start the desktop application from a graphical session:

   ```sh
   ./build/linux-gcc/src/micro_robot_desktop
   ```

   It runs with a simulated robot transport. A camera is needed only for
   streaming; see [camera device access](#camera-device-access-devvideo).

## Build requirements

| Requirement | Minimum | Ubuntu package | Version on 24.04 |
| --- | --- | --- | --- |
| CMake | 3.24 | `cmake` | 3.28.3 |
| Ninja (used by the presets and examples) | any recent | `ninja-build` | 1.11.1 |
| GCC with C++20 | GCC 11 | `build-essential` (`g++`) | 13.3.0 |
| Clang with C++20 (optional second compiler) | Clang 14 | `clang` | 18.1.3 |
| Qt 6 Widgets and Multimedia (desktop build, default ON) | Qt 6 | `qt6-base-dev`, `qt6-multimedia-dev` | 6.4.2 |
| OpenCV core/imgproc/videoio and calib3d (replay, calibration, and the V4L2 camera backend, default ON) | OpenCV 4 | `libopencv-dev` | 4.6.0 |
| GoogleTest (tests, default ON) | — | `libgtest-dev` | 1.14.0 |
| Git and HTTPS certificates (to clone) | — | `git`, `ca-certificates` | — |

The 24.04 versions are those installed by the Linux CI job. Package descriptions are
in Ubuntu's catalog, for example [CMake](https://packages.ubuntu.com/noble/cmake),
[Qt 6 development files](https://packages.ubuntu.com/noble/qt6-base-dev),
[Qt Multimedia development files](https://packages.ubuntu.com/noble/qt6-multimedia-dev),
[OpenCV development files](https://packages.ubuntu.com/noble/libopencv-dev), and
[GoogleTest development files](https://packages.ubuntu.com/noble/libgtest-dev).

All libraries are found through `find_package()` in the standard system prefixes,
so apt-installed dependencies need no `CMAKE_PREFIX_PATH` or source edits. The
configure step prints the detected platform and compiler. It warns if the compiler
is not GCC/Clang or is older than the minimum above.

### Compilers

The `linux-gcc` and `linux-clang` [presets](build.md#cmake-presets) select `g++` and
`clang++`, Ubuntu's default versions. For another installed version, configure a
separate build directory with an explicit compiler:

```sh
cmake --preset linux-clang -B build/linux-clang-19 -DCMAKE_CXX_COMPILER=clang++-19
```

Clang on Linux compiles against GCC's C++ standard library (libstdc++). It uses the
newest GCC installation directory it finds. `clang++ -v` prints it as
`Selected GCC installation`. That version's `libstdc++-<N>-dev` must be installed;
see [troubleshooting](#troubleshooting).

### Qt 6

`qt6-base-dev` provides Core, Gui, and Widgets. `qt6-multimedia-dev` provides
Multimedia, which the desktop uses for camera discovery and the optional Qt camera
backend. Apt also installs the X11 (`xcb`) and Wayland platform plugins as
dependencies. Install Widgets and Multimedia from the same Qt distribution and
version.

To use a Qt from the Qt online installer instead, pass its prefix at configure time,
for example `cmake --preset linux-gcc -DCMAKE_PREFIX_PATH="$HOME/Qt/6.8.3/gcc_64"`.
The prefix is the directory that contains `lib/cmake/Qt6`. Since Qt 6.5, the X11
platform plugin also needs the system package `libxcb-cursor0`.

### OpenCV

`libopencv-dev` covers all OpenCV components the default build uses: core and
imgproc, videoio for [recorded-video replay](../overview/recorded-video.md) and
V4L2 capture, and calib3d for chessboard calibration capture (objdetect on
OpenCV 5). Its codec dependencies supply the MJPEG writer/decoder needed by the
replay tests. The V4L2 kernel headers come with the C library development files
that `build-essential` installs, so no other camera package is needed.

The offline AprilTag marker tools (`-DMICRO_ROBOT_BUILD_MARKER_TOOLS=ON`, default
OFF) also need imgcodecs and, on OpenCV older than 4.7, the contrib aruco module.
`libopencv-dev` installs both, aruco through its `libopencv-contrib-dev`
dependency.

### GoogleTest

With `libgtest-dev` installed, CMake uses the system GTest CONFIG package and no
download takes place; the system version may differ from 1.17.0. Without it, the
configure step downloads GoogleTest 1.17.0 from GitHub, which needs internet
access. For offline machines, install `libgtest-dev`, or pass an existing source
checkout with `-DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=/path/to/googletest`.
`-DBUILD_TESTING=OFF` omits the tests and the GTest dependency.

### Optional tools

| Tool | Package | Use |
| --- | --- | --- |
| `v4l2-ctl` | `v4l-utils` | List camera nodes and supported formats |
| `bluetoothctl`, `btmon` | `bluez` | Check the Bluetooth adapter; see [Bluetooth and BLE](#bluetooth-and-ble) |
| `gdb` | `gdb` | Debug the Debug builds |

## Configure, build, and test

Run from `Mini_Robot`. The [CMake presets](build.md#cmake-presets) are Debug builds
with tests enabled and use a separate build directory per compiler
(`build/linux-gcc`, `build/linux-clang`):

```sh
cmake --preset linux-gcc
cmake --build --preset linux-gcc
ctest --preset linux-gcc

cmake --preset linux-clang
cmake --build --preset linux-clang
ctest --preset linux-clang
```

`cmake --list-presets` shows the presets available on the host. The test presets
print output for failed tests and fail when no tests are found. Without presets, the
equivalent commands are:

```sh
cmake -S . -B build/linux-gcc -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_COMPILER=g++ -DBUILD_TESTING=ON
cmake --build build/linux-gcc --parallel
ctest --test-dir build/linux-gcc --output-on-failure --no-tests=error

cmake -S . -B build/linux-clang -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_COMPILER=clang++ -DBUILD_TESTING=ON
cmake --build build/linux-clang --parallel
ctest --test-dir build/linux-clang --output-on-failure --no-tests=error
```

### Selecting tests

Build before running CTest; the GoogleTest cases are discovered from the built
executables. Labels and executables are listed in [testing](testing.md#build-and-run).

```sh
ctest --preset linux-gcc -L core
ctest --preset linux-gcc -L ui
ctest --preset linux-gcc -R '^WorldModelTest\.'
./build/linux-gcc/tests/micro_robot_common_tests --gtest_filter='RobotIdTest.*'
```

The Qt UI tests select the offscreen platform themselves, so the whole suite runs
over SSH or without a display. No camera, Bluetooth adapter, or robot is needed.

The two V4L2 hardware tests run only when a camera node is named. The optional
resolution variable adds a size check:

```sh
MICRO_ROBOT_TEST_V4L2_DEVICE=/dev/video0 MICRO_ROBOT_TEST_V4L2_RESOLUTION=1280x720 \
  ctest --preset linux-gcc -R '^V4l2CameraHardwareTest\.'
```

### Other build variants

Use a fresh build directory for each variant.

Qt-free build (domain libraries, simulation, OpenCV replay, and their tests):

```sh
cmake -S . -B build/linux-core -G Ninja -DCMAKE_BUILD_TYPE=Debug -DMICRO_ROBOT_BUILD_DESKTOP=OFF -DBUILD_TESTING=ON
cmake --build build/linux-core --parallel
ctest --test-dir build/linux-core --output-on-failure --no-tests=error
```

Append `-DMICRO_ROBOT_BUILD_REPLAY=OFF` to also build without OpenCV. For a desktop
build without OpenCV, append `-DMICRO_ROBOT_BUILD_REPLAY=OFF -DMICRO_ROBOT_CAMERA_BACKEND=Qt`
to a configure command. The second option replaces the default
[V4L2 camera backend](../overview/camera-backends.md) with Qt Multimedia.

For a Release build, configure a separate directory with
`-DCMAKE_BUILD_TYPE=Release`, for example
`cmake --preset linux-gcc -B build/linux-gcc-release -DCMAKE_BUILD_TYPE=Release`.

### Running the desktop application

```sh
./build/linux-gcc/src/micro_robot_desktop
```

The [desktop demo guide](../overview/desktop-demo.md) describes the controls. The
optional settings file is `~/.config/Micro Robot/micro_robot.ini`
([configuration](../overview/configuration.md)), and the log file is
`~/.local/share/Micro Robot/logs/micro_robot.log` ([logging](../overview/logging.md)).
Qt chooses the X11 or Wayland platform plugin from the session. Startup, resizing,
input, display scaling, and X11/Wayland behavior need manual checks (#130); a
passing offscreen test does not establish them.

## Camera device access (`/dev/video*`)

Each camera appears as a V4L2 node such as `/dev/video0`. UVC cameras usually add
a second metadata node (`/dev/video1`), which the application hides. Nodes belong
to `root:video` with mode `0660`, so the application needs one of:

- **A local desktop session (usual case).** systemd-logind and udev give the user
  at the active seat an ACL on each camera node. No group membership is needed.
  Check with `getfacl /dev/video0`; expect a line such as `user:<you>:rw-`.
- **SSH, other non-seat sessions, or no logind.** Add the user to the `video` group,
  then log out and back in (or reboot) so that `id` lists `video`:

  ```sh
  sudo usermod -aG video "$USER"
  ```

- **Containers.** Pass the node into the container (for example
  `--device /dev/video0`) and run as a user with access to it.

Do not run the GUI as root or make nodes world-writable (`chmod 666`) to bypass
access errors.

Diagnosis:

| Symptom | Check |
| --- | --- |
| No camera listed | `ls /sys/class/video4linux` and `ls -l /dev/video*`; check that the kernel driver loaded (`sudo dmesg \| grep -i uvc`). |
| Camera listed, Start reports "Camera permission denied for /dev/videoN" | `ls -l /dev/videoN`, `getfacl /dev/videoN`, `id`; fix access as above, then press Start again. A camera whose capability query is denied is still listed, so the failure is explicit. |
| Start reports "... cannot open /dev/videoN; it may be in use by another application" | Close other capture applications (browser, video call, `cheese`); `fuser /dev/videoN` names them. |
| A requested resolution is not delivered | Look for `camera_resolution_fallback` in stderr; `v4l2-ctl -d /dev/videoN --list-formats-ext` lists supported modes. |

Inventory and capture logs use the Qt categories `micro_robot.camera.discovery` and
`micro_robot.camera.capture`:

```sh
QT_LOGGING_RULES='micro_robot.camera.*=true' ./build/linux-gcc/src/micro_robot_desktop
```

## Bluetooth and BLE

The current [source targets](../../src/CMakeLists.txt) do not link Qt Bluetooth or
BlueZ. Building and running the tests need no Bluetooth package, adapter, or robot;
the robot connection in the desktop demo is simulated. The packages below are for
the planned Qt Bluetooth transport (#129, #65, #66) and for checking a host's BLE
readiness. Installing them does not enable BLE in the current application.

| Purpose | Ubuntu package | Notes |
| --- | --- | --- |
| Qt Bluetooth headers and CMake package (`Qt6::Bluetooth`) for the BLE transport | `qt6-connectivity-dev` | Universe component. Use the same Qt version as `qt6-base-dev`. |
| BlueZ daemon (`bluetoothd`), which Qt Bluetooth uses through D-Bus | `bluez` | Installed by default on Ubuntu Desktop. Includes `bluetoothctl` and `btmon`. |
| Radio kill-switch control | `rfkill` | Usually preinstalled. |

```sh
sudo apt install bluez rfkill qt6-connectivity-dev
sudo systemctl enable --now bluetooth
```

Runtime requirements for the desktop as a BLE central:

- An adapter with Bluetooth LE support that is present, powered, and not
  rfkill-blocked. `bluetoothctl show` must report `Powered: yes`; BlueZ versions
  that report adapter roles must list `Roles: central`.
- `bluetoothd` running (`systemctl is-active bluetooth`).
- Access to BlueZ on the system D-Bus. Ubuntu's policy
  (`/usr/share/dbus-1/system.d/bluetooth.conf`) lets any local user call
  `org.bluez`, so neither root, `CAP_NET_ADMIN`, nor group membership is needed to
  scan or connect. Do not run the GUI as root to work around a Bluetooth error.
- Containers need the host's system D-Bus socket and a running host `bluetoothd`.
  Passing `/dev` nodes is not enough.

The robot protocol uses no pairing or bonding
([link parameters](../protocol/ble-gatt.md#link-parameters)), so no BlueZ agent
setup is needed.

Diagnosis:

| Symptom | Check |
| --- | --- |
| No adapter | `bluetoothctl list`, `ls /sys/class/bluetooth`, `sudo dmesg \| grep -i bluetooth` (firmware or driver) |
| Adapter present but unusable | `rfkill list bluetooth`, then `rfkill unblock bluetooth`. `bluetoothctl power on`. |
| D-Bus errors such as `org.freedesktop.DBus.Error.ServiceUnknown` | `systemctl status bluetooth`, `journalctl -u bluetooth` |
| Robot not seen | `bluetoothctl`, then `scan le`. Look for the `MR-<robotId>` name ([advertising](../protocol/ble-gatt.md#advertising-and-discovery)). Check that no other central is connected, because the robot stops advertising while it is connected. |
| Link drops or GATT errors | `sudo btmon` records the HCI traffic |

Physical BLE validation requires a compatible adapter, BlueZ, and robot hardware.
CI runners have none of these.

## Troubleshooting

| Symptom | Cause and fix |
| --- | --- |
| `E: Unable to locate package qt6-multimedia-dev` (or `libgtest-dev`, `qt6-connectivity-dev`) | The universe component is disabled. Run `sudo add-apt-repository universe`, then `sudo apt update`. |
| `CMake 3.24 or higher is required` or `Unrecognized "version" field` for the presets | The CMake on `PATH` is too old, for example Ubuntu 22.04's 3.22. Install a newer CMake and check `cmake --version`. |
| `Could not find a package configuration file provided by "Qt6"` | Install `qt6-base-dev`. For a non-apt Qt, pass `-DCMAKE_PREFIX_PATH=<Qt prefix>`. For a build without Qt, set `-DMICRO_ROBOT_BUILD_DESKTOP=OFF`. |
| `Failed to find required Qt component "Multimedia"` | Install `qt6-multimedia-dev` from the same Qt version as `qt6-base-dev`. |
| `Could not find a package configuration file provided by "OpenCV"`, or a missing OpenCV component | Install `libopencv-dev`, or set `OpenCV_DIR` to a custom OpenCV's CMake package directory. For a build without OpenCV, see [other build variants](#other-build-variants). |
| Configure prints `Could NOT find XKB` | Informational message from Qt's optional keyboard support; the build continues. Installing `libxkbcommon-dev` removes it. |
| GoogleTest download fails during configure | No internet access. Install `libgtest-dev` (see [GoogleTest](#googletest)). |
| Clang: `fatal error: 'iostream' file not found` or `cannot find -lstdc++` | Clang selected a GCC installation without the matching libstdc++ headers. Run `clang++ -v` and install `libstdc++-<N>-dev` for the `Selected GCC installation` version `<N>`. |
| Configure warns that the compiler is unsupported or too old | Use GCC 11+ or Clang 14+ through a preset or `-DCMAKE_CXX_COMPILER`. |
| A changed compiler, Qt, or generator is ignored | CMake caches these per build directory. Use a new build directory, or reconfigure with `cmake --preset linux-gcc --fresh`. |
| `c++: fatal error: Killed signal terminated program cc1plus` | The parallel build ran out of memory. Limit jobs, for example `cmake --build --preset linux-gcc --parallel 2`. |
| `No tests were found!!!` | Build the test executables before running CTest, and keep `BUILD_TESTING` ON. |
| A preset is missing from `cmake --list-presets` | Presets are filtered by host; on Linux only `linux-gcc` and `linux-clang` are listed. |
| `qt.qpa.xcb: could not connect to display` or `Could not load the Qt platform plugin` when launching the desktop | No graphical session (for example over SSH). Run the desktop in a desktop session; the tests need no display. For a non-apt Qt 6.5+, install `libxcb-cursor0`. `QT_DEBUG_PLUGINS=1` shows why a plugin fails to load. |
| Camera or Bluetooth problems | See [camera device access](#camera-device-access-devvideo) and [Bluetooth and BLE](#bluetooth-and-ble). |

The [build setup troubleshooting](build.md#troubleshooting) covers the same CMake
symptoms for every platform.

## Verification status

### #131: clean Ubuntu 24.04

The [Linux CI run](https://github.com/Krzys2002/Mini_Robot/actions/runs/39410655644)
for `master` at `caae85d` started from a fresh GitHub `ubuntu-24.04` runner, ran the
apt installation from the [quick start](#quick-start-on-a-clean-ubuntu-2404)
without `git` and `ca-certificates`, then the preset configure, build, and test steps:

| Job | Result |
| --- | --- |
| [GCC 13.3.0](https://github.com/Krzys2002/Mini_Robot/actions/runs/39410655644/job/118587317393) | Configured with Qt 6.4.2, OpenCV 4.6.0, and system GoogleTest 1.14.0; 200/200 CTest cases passed, 2 hardware cases skipped |
| [Clang 18.1.3](https://github.com/Krzys2002/Mini_Robot/actions/runs/39410655644/job/118587317717) | Same packages; 200/200 CTest cases passed, 2 skipped as above |

The runner image is not a minimal installation: it has other tools preinstalled,
and its preinstalled CMake 3.31.6 and Ninja 1.13.2 took precedence over the apt
versions. A minimal `ubuntu:24.04` container and the apt CMake 3.28.3 have not been
tested. The `git clone` step was replaced by the Actions checkout.

On the Ubuntu 26.04.1 developer host (GCC 15.2.0, Clang 21.1.8, Qt 6.10.2,
OpenCV 4.10.0, system GoogleTest 1.17.0, CLion's CMake 4.3.1), at the same source
revision, the documented commands were run from fresh build directories: the `linux-gcc` and `linux-clang` presets
each passed 200/200 CTest cases (2 hardware cases skipped), and the Qt-free build
passed 136/136. The [selecting tests](#selecting-tests) commands and the Release
configure command were also run.

### Earlier issues

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

For #128, on the same host (kernel 7.0.0) with the source tree at that change:

| Configuration | Result |
| --- | --- |
| `linux-gcc` preset (V4L2 camera backend) | Built without warnings; 179/179 CTest cases passed, 2 hardware cases skipped without a camera variable |
| `linux-clang` preset (V4L2 camera backend) | Built without warnings; 179/179 CTest cases passed, 2 skipped as above |
| GCC, `-DMICRO_ROBOT_CAMERA_BACKEND=Qt` | Built without warnings; 167/167 CTest cases passed |
| GCC, desktop and replay OFF (no Qt, no OpenCV) | Built; 117/117 CTest cases passed |
| `V4l2CameraHardwareTest` with `/dev/video0` (integrated USB UVC camera) | Passed at the default size, `1280x720`, and `640x480`; see [camera backends](../overview/camera-backends.md#verification) |

[Linux CI](testing.md#linux-continuous-integration) installs the build packages on
an `ubuntu-24.04` GitHub runner and builds and tests the `linux-gcc` and
`linux-clang` presets on every pull request and push to `master`; see that
workflow's runs for current results. Not yet verified: Ubuntu 22.04 with a newer
CMake, arm64, the minimum compiler versions, X11/Wayland GUI behavior (#130), and
physical BLE (#129, #65, #66). Configuration/filesystem regression expansion is
#132. Update this guide with actual results when those checks are performed.
