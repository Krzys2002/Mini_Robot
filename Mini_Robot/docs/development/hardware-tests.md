# Hardware-dependent tests

Source: [test targets](../../tests/CMakeLists.txt).

The [regression suite](testing.md#regression-suite) runs in CI on macOS and Linux
and needs no camera, Bluetooth adapter, robot, or display session. This page lists
the checks that are kept out of it because they need physical devices or a
person at the machine. These checks add to the regression suite; they do not
replace it.

## Rules

- A test that needs hardware is registered with the CTest label `hardware`. The
  shared test presets exclude that label, so CI and `ctest --preset <name>` never
  select it.
- A hardware test names its device in an environment variable. Without the variable,
  it is reported as skipped, not failed, so running the unfiltered executable on a
  machine without the device stays green.
- Fakes, replay sources, and temporary device trees cover the logic behind every
  hardware test. A hardware test only checks what they cannot reproduce, for
  example a real driver or device timing.
- Record executed hardware results with date, OS, versions, and device in the
  module's verification section, as in [camera backends](../overview/camera-backends.md#verification).

## Automated tests with hardware

| CTest name | Platform | Needs | Enable with |
| --- | --- | --- | --- |
| `V4l2CameraHardwareTest.DiscoveryListsConfiguredDevice` | Linux, V4L2 backend (desktop build) | A V4L2 capture node the user may open | `MICRO_ROBOT_TEST_V4L2_DEVICE=/dev/videoN` |
| `V4l2CameraHardwareTest.StreamsValidFramesAcrossRestarts` | Same | Same; optionally a mode the device offers | Same, plus optional `MICRO_ROBOT_TEST_V4L2_RESOLUTION=<width>x<height>` |

Run them from `Mini_Robot` after building a preset with the desktop enabled. Select
the label in the build directory directly, because the preset excludes it:

```sh
MICRO_ROBOT_TEST_V4L2_DEVICE=/dev/video0 \
  ctest --test-dir build/linux-gcc -L hardware --output-on-failure --no-tests=error
```

Physical capture through Qt Multimedia (macOS, or Linux with
`-DMICRO_ROBOT_CAMERA_BACKEND=Qt`) has no automated hardware test yet.

## Hardware-free platform checks in CI

These run in the regression suite, but only on the platform they check:

| CTest name | Platform | Checks |
| --- | --- | --- |
| `DesktopPackaging.CameraPermission` | macOS, Qt 6.5 or later | Bundle camera usage description and the linked camera permission plugin, without requesting authorization |
| `V4l2DiscoveryScanTest.*`, `V4l2CameraDeviceDiscoveryTest.*`, `V4l2CameraSourceTest.*` | Linux, V4L2 backend | V4L2 discovery and capture error paths on temporary sysfs/dev trees |

Some V4L2 cases are skipped when run as root, because permission bits do not
restrict root. The configuration case `ConfigFileTest.UnreadableFileUsesDefaultsAndIsAnError`
is skipped for the same reason, and on hosts where permission bits do not prevent
reading. `ConfigFileTest.FollowsSymbolicLinks` is skipped where symbolic links
cannot be created.

## Manual checks

These need a person, a real display, or hardware. Each guide lists its procedure
and the results executed so far:

| Area | Needs | Procedure |
| --- | --- | --- |
| Camera discovery and hotplug | Built-in, USB, or Continuity cameras | [Camera discovery](../overview/camera-discovery.md#macos-and-verification) |
| Camera streaming and permissions | A camera; OS permission prompts | [Camera streaming](../overview/camera-streaming.md#verification) |
| Linux V4L2 capture and modes | A V4L2 camera | [Camera backends](../overview/camera-backends.md#verification) |
| Calibration capture | A camera and a printed chessboard | [Calibration capture](../overview/calibration-capture.md#manual-acceptance-procedure) |
| GUI on a real display (X11, Wayland, macOS) | A desktop session; mouse, keyboard, scaled screens | [GUI compatibility](gui-compatibility.md#manual-checklist); the offscreen UI tests in CI do not prove it |
| Robot communication over BLE | A Bluetooth adapter, BlueZ on Linux, and an ESP32-C6 robot | No physical transport exists yet; automated tests use the [simulation transport](../overview/simulation.md) |
| CodeCell C6 actuator/IMU safety profile (#154) | Physical CodeCell C6 robot, restrained wheels-raised setup, operator supervision | [CodeCell C6 hardware validation profile](../testing/codecell-c6-hardware-154.md) |
