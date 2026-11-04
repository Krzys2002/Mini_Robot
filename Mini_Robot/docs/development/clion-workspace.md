# Desktop and firmware in CLion

Open the Git repository root as the CMake project. The desktop CMake entry point
is the root `CMakeLists.txt`; the independent BLE fixture's PlatformIO entry point
is `Mini_Robot/firmware/ble-lifecycle/platformio.ini`. Keep those build models
separate even when using one IDE window.

## Desktop window

Enable the imported `desktop` preset under Settings → Build, Execution,
Deployment → CMake. It uses the native host toolchain and explicitly enables the
GUI and replay while disabling tests. Select `micro_robot_desktop` as the run
configuration/target. It creates MainWindow through the production composition
root; `Continuous`, `Nightly` and `Experimental` are CTest dashboard targets.

```sh
# From the Git root:
cmake --preset desktop
cmake --build --preset desktop --parallel
```

If Qt/OpenCV are outside the toolchain's package search paths, set
`CMAKE_PREFIX_PATH` locally in CLion or an ignored `CMakeUserPresets.json`.
Do not commit Homebrew prefixes or another developer's SDK/toolchain paths.
The macOS bundle is `build/desktop/Mini_Robot/src/micro_robot_desktop.app`;
standalone configuration from `Mini_Robot/` instead uses
`Mini_Robot/build/desktop/src/`. Never reuse a build directory between the two.
Use the `macos`, `linux-gcc`, `linux-clang`, `windows-msvc` or `core` presets for
their existing build/test workflows; `desktop` does not build test executables.

## Firmware build from the same root window

The optional CMake module discovers `firmware/*/platformio.ini` under Mini_Robot
and exposes each as a build-only custom target. The current fixture yields
`firmware_ble_lifecycle`. These targets are not dependencies of the desktop or
the default build. They do not compile firmware using the host compiler.

Install PlatformIO Core and make `pio`/`platformio` visible to CLion's configure
environment, then reload CMake. A GUI-launched IDE may have a different PATH from
your terminal. Alternatively set `MICRO_ROBOT_PLATFORMIO_EXECUTABLE` to your
installed executable in a local CMake profile/preset. No PlatformIO installation
is required for normal desktop/core configuration: when not found, firmware
targets are skipped with a status message. Set
`MICRO_ROBOT_ENABLE_FIRMWARE_TOOLS=OFF` to disable them explicitly.

Select `firmware_ble_lifecycle` in CLion's CMake build targets, or run:

```sh
cmake --build build/desktop --target firmware_ble_lifecycle
```

The target runs `pio run --project-dir <fixture-folder>` using the manifest's
default environment `codecell-c6`. It performs a build only. Equivalent direct
CLI command from the Git root:

```sh
pio run -d Mini_Robot/firmware/ble-lifecycle -e codecell-c6
```

## Native PlatformIO plugin

For plugin code assistance, environments and device commands, use the actual
nested `platformio.ini` as the PlatformIO project entry point. With the root
CMake project still open, select that manifest and look for **Load PlatformIO
Project** (context menu/Find Action). This is the native plugin's project-link
action; it is distinct from merely opening the INI file in the text editor.
Keep the root CMake project loaded. If your plugin version only offers **Open as
Project** in another window, use the CMake firmware build target above for the
same-window build workflow, and a separate native PlatformIO project for indexing.
Plugin linking is local IDE state, not a repository configuration guarantee.

Check that the plugin's PlatformIO Core executable setting resolves correctly.
After manifest changes, reload the PlatformIO project. Do not generate a root
`platformio.ini`, run legacy CLion generation in the repository root, replace the
desktop CMakeLists, or add the firmware with `add_subdirectory`. Old generators'
fixture-local `CMakeLists*.txt`, `.idea` and compilation databases are ignored.

Build is safe to use without attached hardware. Upload/debug/serial-monitor
actions are separate explicit device operations; this setup does not run them.
This firmware remains a connection-only fixture, not a production motor runtime.

References: [JetBrains PlatformIO integration](https://www.jetbrains.com/help/clion/platformio.html),
[PlatformIO's CLion guide](https://docs.platformio.org/en/latest/integration/ide/clion.html).

## Verification (2026-11-04)

This change is stacked on Graphify branch commit `1996c9c` (PR #222).
Executed on macOS / AppleClang 21, Qt 6, OpenCV 5 and PlatformIO Core 6.1.19:

- Root `core` configure/build/CTest: 245 tests passed.
- Standalone `Mini_Robot` `core` configure/build/CTest: 245 tests passed, using a
  different build directory.
- Root `macos` configure/build/CTest: 390 tests passed, including offscreen
  production desktop startup. Desktop test discovery needed execution outside
  the process-restricted sandbox; the initial discovery attempt was blocked.
- Root `desktop` configure and `cmake --build --preset desktop --parallel 4`:
  GUI application built with tests disabled.
- `pio run -d Mini_Robot/firmware/ble-lifecycle -e codecell-c6` and
  `cmake --build build/desktop --target firmware_ble_lifecycle`: both compiled the
  connection fixture successfully; no upload or device operation performed.
- `python3 -m unittest discover -s .github/scripts/tests -v`: 33 passed, including
  five firmware-target integration checks for default-build isolation, exact
  project-directory delegation with spaces, opt-out, missing PlatformIO and name
  collisions. Documentation links, preset contract and diff checks passed.

The installed CLion PlatformIO plugin was confirmed enabled; native project
attachment/indexing was not validated. Interactive desktop GUI/hardware,
Linux/Windows, firmware upload/debug/serial monitor and physical acceptance were
not run. The optional CMake targets provide the verified same-window build path.
