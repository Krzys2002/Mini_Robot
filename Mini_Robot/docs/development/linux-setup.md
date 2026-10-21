# Ubuntu development setup for the current source

Ubuntu 24.04 LTS is the documented reference environment for these recipes.
The GCC and Clang builds below, including the OpenCV replay adapter, were
executed for #125 on Ubuntu 26.04 LTS (see [verification status](#verification-status));
[Linux CI](testing.md#linux-continuous-integration) (#126) runs the apt installation
and GCC/Clang builds on a clean 24.04 runner; X11/Wayland GUI checks have not been
executed. The other Linux acceptance criteria (#127–#132) are tracked by their own
issues; Bluetooth preparation (#129) is described under [Bluetooth and BLE](#bluetooth-and-ble).

## Build requirements

| Requirement | Minimum | Ubuntu 24.04 package |
| --- | --- | --- |
| CMake | 3.24 | `cmake` (3.28) |
| GCC or Clang with C++20 | GCC 11 / Clang 14 | `build-essential` (GCC 13), `clang` (18) |
| Ninja (used by presets and examples) | any recent | `ninja-build` |
| Qt 6 Widgets and Multimedia (desktop build) | Qt 6 | `qt6-base-dev`, `qt6-multimedia-dev` |
| OpenCV core/imgproc/videoio (replay/V4L2), calib3d (calibration), aruco (4.2–4.6) or objdetect (4.7+, AprilTag desktop detector) | OpenCV 4.2 for vision | `libopencv-dev` |
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

Append `-DMICRO_ROBOT_BUILD_REPLAY=OFF -DMICRO_ROBOT_CAMERA_BACKEND=Qt` to a configure
command when OpenCV is not installed; the second option replaces the default
[V4L2 camera backend](../overview/camera-backends.md) with Qt Multimedia. CTest's Qt test entry point selects offscreen mode. Launch the demo in
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

For #128, on the same host (kernel 7.0.0) with the source tree at that change:

| Configuration | Result |
| --- | --- |
| `linux-gcc` preset (V4L2 camera backend) | Built without warnings; 179/179 CTest cases passed, 2 hardware cases skipped without a camera variable |
| `linux-clang` preset (V4L2 camera backend) | Built without warnings; 179/179 CTest cases passed, 2 skipped as above |
| GCC, `-DMICRO_ROBOT_CAMERA_BACKEND=Qt` | Built without warnings; 167/167 CTest cases passed |
| GCC, desktop and replay OFF (no Qt, no OpenCV) | Built; 117/117 CTest cases passed |
| `V4l2CameraHardwareTest` with `/dev/video0` (integrated USB UVC camera) | Passed at the default size, `1280x720`, and `640x480`; see [camera backends](../overview/camera-backends.md#verification) |

For #129, on the same host with the source tree at that change:

| Configuration | Result |
| --- | --- |
| `linux-gcc` preset (V4L2 camera backend) | Built; the only warnings are existing `-Wdangling-else` warnings in two calibration test files; 223/223 CTest cases passed, 2 hardware cases skipped as above |
| `linux-clang` preset (V4L2 camera backend) | Incremental build without warnings; 223/223 CTest cases passed, 2 skipped as above |
| GCC, desktop and replay OFF (no Qt, no OpenCV) | Incremental build without warnings; 148/148 CTest cases passed |
| BlueZ readiness | BlueZ 5.85 `bluetoothd` active; `hci0` powered, unblocked, central role available; default D-Bus policy allows `org.bluez`. No scan or connection was run, and Qt Bluetooth was not installed. |

## Replay and camera dependencies

OpenCV core/imgproc/videoio decode [recorded local video](../overview/recorded-video.md)
and, through OpenCV's V4L2 backend, capture from cameras (#128). `libopencv-dev`
covers both. No other camera package is needed: the V4L2 kernel headers come with
the C library development files that `build-essential` installs. `v4l-utils`
(`v4l2-ctl`) is optional for diagnosis. Replay tests require an MJPEG
writer/decoder. Bluetooth packages are described in [Bluetooth and BLE](#bluetooth-and-ble).

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
BlueZ. Building and testing need no Bluetooth package, adapter, or robot. The
[discovery and transport tests](../overview/robot-communication.md#mock-transport-tests)
use simulation and a test-only radio backend. The packages below are for the
planned Qt Bluetooth adapter (#65, #66) and for checking a host's BLE readiness.
The [Qt Bluetooth review](../overview/robot-communication.md#qt-6-bluetooth-platform-review)
explains the choice.

| Purpose | Ubuntu package | Notes |
| --- | --- | --- |
| Qt Bluetooth headers and CMake package (`Qt6::Bluetooth`) for the future BLE adapter | `qt6-connectivity-dev` (pulls in `libqt6bluetooth6`) | Universe component. Use the same Qt version as `qt6-base-dev`. |
| BlueZ daemon (`bluetoothd`), which Qt Bluetooth uses through D-Bus | `bluez` | Installed by default on Ubuntu Desktop. Includes `bluetoothctl` and `btmon`. |
| Radio kill-switch control | `rfkill` (util-linux) | Usually preinstalled. |

```sh
sudo apt install bluez qt6-connectivity-dev
sudo systemctl enable --now bluetooth
```

Runtime requirements for the desktop as a BLE central:

- An adapter with Bluetooth LE support that is present, powered, and not
  rfkill-blocked. `bluetoothctl show` must report `Powered: yes` and
  `Roles: central`.
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

[Linux CI](testing.md#linux-continuous-integration) installs the packages above on an
`ubuntu-24.04` GitHub runner and builds and tests the `linux-gcc` and `linux-clang`
presets on every pull request and push to `master`; see that workflow's runs for
results. Configuration/filesystem regression expansion is #132. This guide must be
updated with actual Linux results when those checks are performed.
