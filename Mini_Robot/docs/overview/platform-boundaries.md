# Platform boundaries

Source: [boundary CMake module](../../cmake/MicroRobotBoundaries.cmake),
[source checker](../../cmake/CheckPlatformBoundary.cmake),
[target registrations](../../src/CMakeLists.txt), and
[checker self-test](../../tests/boundaries/VerifyBoundaryCheck.cmake).

Issue #127 (LNX-03) keeps operating-system-specific code outside the core
application while preserving the `ICameraSource`, `ICameraDeviceDiscovery`, and
`IRobotTransport` abstractions. This guide records the dependency audit, defines
which layer may use which facilities, and describes the checks that keep those
rules true. Build-level platform and compiler configuration (application bundle,
Windows subsystem, compiler support) is LNX-01 (#125), implemented in the
[platform/toolchain module](../../cmake/MicroRobotPlatform.cmake). Linux camera capture is
LNX-04 (#128, [camera backends](camera-backends.md)); robot communication is LNX-05 (#129).

## Layers

| Layer | Targets | May depend on | Must not depend on |
| --- | --- | --- | --- |
| Domain core | `micro_robot_common`, `micro_robot_camera_api`, `micro_robot_camera_selection`, `micro_robot_transport_api`, `micro_robot_world`, `micro_robot_logging`, `micro_robot_configuration` | C++20 standard library and other core targets | Qt, OpenCV, OS headers or platform macros, native frameworks, radio stacks, concrete backends |
| Platform-independent backend | `micro_robot_simulation` | Core targets | Same as the core; only composition and tests may depend on it |
| Application adapters | `micro_robot_desktop_controller` | Core targets, `Qt6::Core` | Device backends (Qt Multimedia/Bluetooth, OpenCV), widgets, OS APIs, concrete backends |
| Presentation | `micro_robot_ui` | Application adapters, `Qt6::Core`, `Qt6::Widgets` | Device backends, OS APIs, concrete backends |
| Platform adapters | `micro_robot_camera_discovery` (Qt Multimedia), `micro_robot_camera_v4l2` (V4L2, OpenCV, Qt Core), `micro_robot_camera_replay` (OpenCV) | The interface they implement and their backend library | Application adapters, presentation |
| Composition root | `micro_robot_desktop` ([main.cpp](../../src/app/main.cpp)) | Everything | — |

The composition root alone constructs concrete backends and injects them as
interface references. Live capture (#31, LNX-04) uses such adapter targets; future
BLE (#65, #66, LNX-05) and Zigbee (#107) implementations are new platform-adapter targets. Platform
conditionals, when a backend differs per operating system, belong in the build
configuration or the composition root, never in core or adapter-independent code.

## Dependency audit

The audit inspected every source and test file on `master` at 7617775 for OS and
native-framework headers, platform macros, Objective-C sources, process/environment
APIs, and backend libraries, and every target's link dependencies.

| Area | Finding | Classification |
| --- | --- | --- |
| Core targets | No OS headers, platform macros, Objective-C, Qt, OpenCV, or native frameworks. | Compliant; now enforced |
| Log timestamps | [Logger](../../src/infrastructure/logging/Logger.cpp) uses calendar arithmetic instead of `gmtime_r`/`gmtime_s`. | Portable standard C++ |
| Core file access | Configuration loading, the file log sink, and replay use `std::filesystem`. | Portable standard C++ |
| Per-user locations | [main.cpp](../../src/app/main.cpp) resolves the configuration and log locations with `QStandardPaths` and converts Qt paths to UTF-8 `std::filesystem::path` values. | Composition root |
| Camera inventory | `QMediaDevices` in [QtCameraDeviceDiscovery](../../src/camera/QtCameraDeviceDiscovery.cpp); Qt selects its platform media backend. On Linux, sysfs, POSIX `open`, and `VIDIOC_QUERYCAP` in [V4l2Device.cpp](../../src/camera/V4l2Device.cpp) (#128). | Platform adapter |
| Live capture | Qt Multimedia in [QtCameraSource](../../src/camera/QtCameraSource.cpp); OpenCV `videoio` with `cv::CAP_V4L2` in [V4l2CameraSource](../../src/camera/V4l2CameraSource.cpp) on Linux. The build option `MICRO_ROBOT_CAMERA_BACKEND` and [main.cpp](../../src/app/main.cpp) select one. | Platform adapter, composition root |
| Recorded video | OpenCV `videoio`, linked privately by [RecordedVideoSource](../../src/camera/RecordedVideoSource.cpp). | Platform adapter |
| Timers | The simulation tick is a `QTimer` in the composition root; discovery coalescing uses `QTimer::singleShot` inside its adapter. Core components read only the steady clock or receive time as a parameter. | Qt at the edges |
| Threads | Live camera adapters each run one `QThread` capture worker. Core targets create no threads; the logger serializes with `std::mutex`. | Qt at the edges |
| Environment | The UI test entry point sets `QT_QPA_PLATFORM` through `qputenv`. | Qt, test-only |
| macOS-only | [Info.plist.in](../../src/app/Info.plist.in) (camera usage description, Continuity Camera opt-in) and the bundle target properties. Since LNX-01, `micro_robot_configure_desktop_executable()` applies them on Apple platforms only. | Build/packaging (LNX-01) |
| Windows-only | The GUI-subsystem executable property, applied only on Windows since LNX-01. | Build/packaging (LNX-01) |
| Radio stacks | None present. `IRobotTransport` exposes no radio addressing or handles. | Compliant |

No core code depends directly on a macOS-only API, so no source move was
required. The issue's change is enforcement: the boundaries were previously
upheld by convention only.

## Facility policy

- **Filesystem:** core code uses `std::filesystem`. Qt (`QStandardPaths`, `QDir`)
  is used only where per-OS conventions matter: standard per-user locations and
  native separators for display, in the composition root. Paths cross into core
  as `std::filesystem::path` built from UTF-8. Configuration uses the portable INI
  parser rather than `QSettings`, whose native storage differs per OS.
- **Time and timers:** core time is `MonotonicClock` from
  [Time.hpp](../../src/common/Time.hpp). Deterministic components, such as
  simulation and replay, take time as a parameter instead of reading clocks or
  sleeping. Scheduling uses `QTimer` in the composition root or a Qt adapter.
- **Threading:** the interfaces are thread-affine. Calls and events stay on the
  owning thread. Capture and future radio workers live in their adapter (`QThread`
  or standard threads) and marshal results to the owning thread. GUI updates use
  queued Qt connections.
- **Logging:** the application `Logger` is Qt-free. Adapters may use Qt logging
  categories for backend diagnostics.

## Platform-neutral device discovery

[ICameraDeviceDiscovery](../../src/camera/ICameraDeviceDiscovery.hpp) is the
project's discovery pattern. Any discovery boundary follows the same rules:

- Inventory is separate from capture or connection; discovery never opens a device.
- Results are complete value snapshots. An empty inventory is a valid result.
- Device IDs are opaque backend strings. They are never synthesized from display
  names or list positions, and they carry no promise of stability across
  operating systems or backends.
- Native handles, device paths, and radio addresses never cross the interface.
- Refresh requests return promptly; snapshots arrive asynchronously, never inline.
- Event sinks are weak subscriptions, dispatched on the backend's owning thread.

Robot discovery is defined by LNX-05 (#129) and the BLE work (#65). It must
follow this pattern behind the transport layer, mapping radio identities to
`RobotId` privately. LNX-04 (#128) lists Linux camera nodes that cannot be
opened for permission reasons, and reports the permission error when one is started
([camera backends](camera-backends.md#linux-discovery)).

## Replacing a backend

| Interface | Current implementations | Planned backends |
| --- | --- | --- |
| `ICameraDeviceDiscovery` | `QtCameraDeviceDiscovery`, `V4l2CameraDeviceDiscovery` (Linux); manual test double | Windows validation |
| `ICameraSourceFactory` | `QtCameraSourceFactory`, `V4l2CameraSourceFactory` (Linux), `FakeCameraSourceFactory`; test doubles | — |
| `ICameraSource` | `QtCameraSource`, `V4l2CameraSource` (Linux), `FakeCameraSource`, `RecordedVideoSource`; test doubles | Windows validation |
| `IRobotTransport` | `SimulationTransport`; test doubles | BLE (#65, #66, LNX-05), Zigbee gateway (#107) |

To add or swap a backend:

1. Implement the interface in a new adapter target that links its backend
   library privately.
2. Do not register the adapter with `micro_robot_platform_boundary()`. If it is
   platform-independent, register it with `BACKEND`.
3. Construct it in the composition root and inject it by interface reference.
   Camera sources reach the controller through an `ICameraSourceFactory`.

`DesktopController`, `CameraSelectionController`, `WorldModel`, and the UI do
not change. The UI integration test
`AlternativeBackendsSupportPendingStatesAndCancellation` runs the same controller
against non-simulation backends. The camera-selection tests drive the presenter
through manual inventory, factory, and camera backends without Qt Multimedia.

## Enforcement

Targets declare their layer in [src/CMakeLists.txt](../../src/CMakeLists.txt):

```cmake
micro_robot_platform_boundary(micro_robot_world)
micro_robot_platform_boundary(micro_robot_simulation BACKEND)
micro_robot_platform_boundary(micro_robot_desktop_controller ALLOW Qt6::Core)
```

**At configure time**, `micro_robot_check_platform_boundaries()` walks each
registered target's link dependencies, transitively through first-party targets.
Configuration fails when a registered target reaches:

- any imported target or system library not listed in its `ALLOW`; or
- a concrete backend. A concrete backend is any unregistered first-party target
  or a `BACKEND` target. This rule does not apply to `BACKEND` targets themselves.

Each first-party dependency is classified by its own registration, wherever it
is defined, including targets added through nested `add_subdirectory()` calls.
Unregistered first-party targets are treated as backends, so the check fails
closed: a new platform-independent target must be registered before other
registered targets can depend on it. Headers of backends defined anywhere under
`src/` become forbidden includes for the source scan. This is checked on every
configure, independent of `BUILD_TESTING`.

**At test time**, one CTest per registered target, `PlatformBoundary.<target>`
(labels `core` and `boundary`), scans the preprocessor lines of the target's
sources. It reports:

- OS and platform conditionals, such as `__APPLE__`, `TARGET_OS_*`, `_WIN32`,
  `__linux__`, or Qt's `Q_OS_*` macros (also in targets that may use Qt);
- POSIX, Linux, Windows, and Apple framework headers;
- OpenCV, GStreamer, FFmpeg, and Qt Multimedia/Bluetooth/SerialPort headers;
- any Qt header in a target that does not allow a Qt module;
- quoted includes of concrete backend headers;
- Objective-C(++) sources.

Comments may mention platforms freely. Two self-tests cover the checks themselves:

- `PlatformBoundary.CheckerSelfTest` checks that the scanner reports each class of
  violation, including `Q_OS_MACOS`, `Q_OS_LINUX`, and `Q_OS_WIN` conditionals. It
  also checks that platform-neutral and ordinary Qt Core fixtures pass.
- `PlatformBoundary.LinkCheckSelfTest` configures a
  [fixture project](../../tests/boundaries/link_project/CMakeLists.txt). A
  registered core target must be rejected when it links an unregistered backend
  or a `BACKEND` target defined in child directories, and accepted when it links
  a registered platform-independent target there.

Run the checks alone with:

```sh
ctest --test-dir build -L boundary --output-on-failure --no-tests=error
```

The scan uses explicit header patterns. A header outside those patterns that
belongs to a library the target does not link fails to compile instead, because
the target does not receive that library's include paths. Platform adapters and
the composition root are not scanned.

## Verification

On Ubuntu 26.04 (GCC 15.2, Clang 21.1, Qt 6.10.2, OpenCV 4.10), the desktop and
Qt-free configurations were built and their full CTest suites passed, including
the boundary checks. Deliberately injected violations were rejected at configure
time (a core target linking `Qt6::Core`, an application adapter reaching
`Qt6::Multimedia` or the simulation backend through another target, a plain
system library) and by the source scan (an OS header, an `__APPLE__` conditional,
a Qt include in the logger, a simulation header in the controller). The
child-directory backend and `Q_OS_*` self-tests fail against the first revision
of these checks and pass against the current one.

A clean macOS build with all tests passing was reported in review of the first
revision. The review follow-up was built and tested on Linux only. The macOS CI
workflow runs the same CTest suite, including the boundary checks. Hardware
behavior is unaffected by this issue.
