# Qt GUI compatibility on Linux and macOS

Source: [GUI compatibility tests](../../tests/ui/GuiCompatibilityTest.cpp),
[UI test entry point](../../tests/main.cpp), [test targets](../../tests/CMakeLists.txt),
and [MainWindow.cpp](../../src/ui/MainWindow.cpp).

The desktop GUI is one Qt 6 Widgets implementation for every platform. `src/ui` and
`src/app` contain no operating-system conditionals; the build selects only the
camera backend (see [camera backends](../overview/camera-backends.md)). Qt chooses
the platform plugin at startup: `wayland` or `xcb` on Linux and `cocoa` on macOS.

## Automated checks

`GuiCompatibilityTest` builds the same composition as [main.cpp](../../src/app/main.cpp),
including camera selection and the calibration dialog, with fake camera and
transport backends. Mouse and keyboard input is injected at the window's
`QWindow`, so Qt's platform event dispatch and device-pixel scaling apply as they do
to native input.

| Case | Checks |
| --- | --- |
| `MainWindowStartsOnTheRequestedPlatform` | The requested platform plugin is active, the window is exposed, startup state is shown, and every control is visible inside the window at no less than its minimum size. Prints platform, device-pixel ratio, screen, and window sizes. |
| `ResizedWindowKeepsControlsUsableAndRepaints` | Enlarging, shrinking below the layout minimum (clamped), and restoring the window; each size is repainted and keeps controls usable. Rendered pixels follow the device-pixel ratio. |
| `MouseClicksDriveCameraAndRobotControls` | Clicks one logical pixel inside button corners, clicks on disabled buttons, press-and-release outside a button, and camera start/stop by mouse. |
| `KeyboardSelectsCamerasAndActivatesFocusedButtons` | Arrow keys on the closed camera drop-down select cameras; Space activates the focused button. |
| `TabFollowsTheVisualOrderOfEnabledControls` | Tab and Shift+Tab move through enabled controls in visual order and skip disabled ones. Skipped where the platform gives buttons no Tab focus (macOS without keyboard navigation). |
| `CalibrationDialogOpensAndClosesWithTheMainWindow` | The calibration dialog opens as its own window and closes with the main window when the platform window is closed. |
| `ScaleFactorSetsTheDevicePixelRatio` | The device-pixel ratio matches `QT_SCALE_FACTOR` (exactly on offscreen) and the layout remains in logical pixels. |

CTest runs the suite three times: through GoogleTest discovery at scale 1 (label
`ui`), and as `GuiCompatibility.ScaleFactor1_5` and `GuiCompatibility.ScaleFactor2`
(labels `ui;gui`) with `QT_SCALE_FACTOR` set. CI uses the offscreen platform on
Ubuntu and macOS:

```sh
ctest --preset linux-gcc -R 'GuiCompatibility' --output-on-failure
```

Inside a desktop session, `MICRO_ROBOT_TEST_QPA_PLATFORM` selects a real platform
plugin for the whole UI test executable instead of offscreen:

```sh
MICRO_ROBOT_TEST_QPA_PLATFORM=wayland ./build/linux-gcc/tests/micro_robot_tests --gtest_filter='GuiCompatibilityTest.*'
MICRO_ROBOT_TEST_QPA_PLATFORM=xcb QT_SCALE_FACTOR=2 ./build/linux-gcc/tests/micro_robot_tests --gtest_filter='GuiCompatibilityTest.*'
```

On macOS use `MICRO_ROBOT_TEST_QPA_PLATFORM=cocoa`. The windows appear on screen
while the tests run. A case that finds its window shrunk below the layout minimum
by the platform is reported as skipped, with the sizes, instead of failing (see
[small screens](#small-logical-screens)).

The automated checks do not pass input through the display server or window
manager, do not drag window edges, and avoid drop-down popups. Those remain part
of the [manual checklist](#manual-checklist).

## Linux results

Executed for #130 on Ubuntu 26.04.1 LTS (kernel 7.0.0), GNOME Shell 50.1 Wayland
session, Xwayland 24.1.10, Qt 6.10.2 with `qt6-wayland`, GCC 15.2.0 and Clang
21.1.8. The display is a 1920x1080 panel at 125% GNOME fractional scaling
(`scale-monitor-framebuffer` and `xwayland-native-scaling` enabled). GNOME 50 has
no Xorg session, so X11 was exercised through Xwayland.

| Platform plugin | `QT_SCALE_FACTOR` | Device-pixel ratio | `GuiCompatibilityTest` |
| --- | --- | --- | --- |
| offscreen | unset / 1.5 / 2 | 1 / 1.5 / 2 | 7/7 passed each |
| wayland (native) | unset | 1.25 | 7/7 passed, 3 repeated runs |
| wayland (native) | 1.5 | 1.875 | 3 passed, 4 skipped: window shrunk to 620x528 below its 554 px minimum height |
| wayland (native) | 2 | 2.5 | 3 passed, 4 skipped: window shrunk to 620x396 |
| xcb (Xwayland) | unset / 1.5 / 2 | 2 / 3 / 4 | 7/7 passed each, 3 repeated runs at 1 and 2 |

The complete UI executable (all 64 cases, including window, camera selection,
calibration wizard, and V4L2 tests) also passed three times on both `wayland` and
`xcb`; the two V4L2 hardware cases were skipped without
`MICRO_ROBOT_TEST_V4L2_DEVICE`.

`./build/linux-gcc/src/micro_robot_desktop` was started with
`QT_QPA_PLATFORM=wayland` and with `QT_QPA_PLATFORM=xcb`. Both logged startup,
discovered the integrated camera, and kept running until terminated, with no Qt
platform warnings.

Two defects found during validation were fixed:

- The Camera calibration button is shown below the camera selector but was reached
  last by Tab, because it is added after the other controls. It now follows the
  camera selector in the focus chain.
- `SilentSourceCannotLeaveAnOldFramePresentedAsCurrent` failed intermittently on
  the real platforms: its 3.1 s wait used Qt's default coarse timer, which may fire
  up to 5% early, before the 3 s frame watchdog. The wait is now a precise timer.

## Limitations

### Small logical screens

The historical #130 layout needed about 554 logical pixels of height (290x554 minimum here;
fonts and style change it). If the work area is smaller, for example 1920x1080 at
200% (540 logical pixels) or 1280x800 at 150%, the window cannot show all controls:

- **GNOME Wayland** fits the window into the work area below its minimum size, so the
  lowest panels (robot transport and WorldModel) are clipped and cannot be reached.
- **X11/Xwayland** keeps the minimum size, so the window extends past the screen edge.

Issue #69 now places camera/world content in a scrolling container and keeps the
manual robot panel, Stop and E-stop outside it. Automated offscreen checks verify
scroll access, persistent stop controls and 1.5×/2× scaling. The historical native
platform measurements above do not validate the revised layout; repeat the manual
checklist on each real display. Extremely small work areas can still be narrower
or shorter than the persistent panel's font/style minimum.

### Xwayland under fractional scaling

With GNOME's `xwayland-native-scaling`, X11 clients render at the next integer
scale (device-pixel ratio 2 at 125%) and the compositor scales the result down.
Text and lines can look slightly soft compared with the native Wayland plugin. Qt
uses the native plugin in a Wayland session when `qt6-wayland` is installed (see
[Ubuntu setup](linux-setup.md#gui-session-packages)); without it, Qt falls back to
`xcb` through Xwayland.

### Wayland window management

Wayland does not let clients position their windows, and GNOME may refuse to
activate a window that was not started from user input. Neither affects the
current GUI, which neither sets positions nor requires activation. The tests do not
rely on window activation.

### Not yet verified

- Physical mouse, keyboard, and touchpad input through the compositor, window edge
  dragging, maximize/restore, and drop-down popups on Wayland and X11.
- A native Xorg session and other compositors (KDE Plasma, wlroots).
- Multiple monitors with different scale factors, and moving the window between them.
- macOS: the automated checks run offscreen in [macOS CI](testing.md#macos-continuous-integration);
  the `cocoa` run and the manual checklist have not been executed for #130.

## Manual checklist

Run in each session to be verified (GNOME Wayland, X11, macOS), at 100% and at a
fractional or 200% display scale. Start the application from a terminal so that
the log is visible.

1. Launch. The window appears with all panels readable, the status reads Ready,
   and the camera list replaces “Looking for cameras…”.
2. Drag each window edge and corner, maximize, and restore. Controls stay inside
   the window, text is not truncated, and the window cannot be made smaller than
   the persistent robot panel. Camera/world content scrolls when needed; Stop and
   E-stop stay visible (see [small screens](#small-logical-screens)).
3. Open the camera drop-down with the mouse and choose a camera. The popup opens
   next to the drop-down and closes after the choice; Active camera source names it.
4. Click Start camera and Stop camera, Connect robot and Disconnect robot. Status
   labels follow each click.
5. Press Tab and Shift+Tab. Focus moves in visual order: camera drop-down, Camera
   calibration…, Vision debug…, Start camera, robot selector, Scan, Connect,
   then the available manual/stop controls; disabled buttons are skipped.
   Space activates the focused button, and arrow keys change the camera.
6. Open Camera calibration…. The dialog appears as a separate window and can be
   moved and resized. Close it, then reopen it.
7. Change the display scale while the application runs (and on a multi-monitor
   setup, move the window to the other monitor). Text and controls stay sharp and
   correctly sized.
8. Close the main window with the window-manager close button. The log ends with
   “Application shutdown complete (exit code 0)”.

| Session | Result |
| --- | --- |
| Ubuntu 26.04 GNOME Wayland, native plugin | Not yet executed |
| Ubuntu 26.04 GNOME Wayland, `QT_QPA_PLATFORM=xcb` (Xwayland) | Not yet executed |
| Native Xorg session | Not yet executed |
| macOS (Apple Silicon) | Not yet executed |
