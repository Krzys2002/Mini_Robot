# Historical architecture proposal

This document is a design sketch retained for context. Proposed folders, types,
frameworks and dependencies below do not describe the implemented repository.
Use the [current architecture](Mini_Robot/docs/overview/architecture.md),
[current capability](Mini_Robot/docs/overview/capabilities.md), and
[source map](Mini_Robot/docs/overview/source-map.md) for current guidance.

**1\. Directory structure**

```
Mini_Robot/
├── AGENTS.md
├── CMakeLists.txt
├── CMakePresets.json
├── cmake/
│   └── Dependencies.cmake
├── config/
│   └── default.json
├── docs/
│   └── architecture.md
├── src/
│   ├── CMakeLists.txt
│   ├── common/
│   │   ├── Types.hpp
│   │   ├── Units.hpp
│   │   ├── Time.hpp
│   │   └── Result.hpp
│   ├── core/
│   │   ├── AppConfig.hpp
│   │   ├── ILogger.hpp
│   │   ├── ApplicationCore.hpp
│   │   └── ApplicationCore.cpp
│   ├── camera/
│   │   ├── CameraTypes.hpp
│   │   └── ICameraSource.hpp
│   ├── transport/
│   │   ├── TransportTypes.hpp
│   │   └── IRobotTransport.hpp
│   ├── world/
│   │   ├── RobotState.hpp
│   │   ├── WorldSnapshot.hpp
│   │   ├── WorldModel.hpp
│   │   └── WorldModel.cpp
│   ├── simulation/
│   │   ├── FakeCameraSource.hpp/.cpp
│   │   ├── SimulationTransport.hpp/.cpp
│   │   └── FakeWorldData.hpp/.cpp
│   ├── infrastructure/
│   │   ├── logging/
│   │   │   └── SpdlogLogger.hpp/.cpp
│   │   └── configuration/
│   │       └── ConfigLoader.hpp/.cpp
│   ├── ui/
│   │   ├── MainWindow.hpp/.cpp
│   │   └── DesktopController.hpp/.cpp
│   └── app/
│       ├── main.cpp
│       └── Application.hpp/.cpp
└── tests/
    ├── CMakeLists.txt
    ├── common/
    ├── world/
    ├── simulation/
    ├── configuration/
    ├── core/
    └── support/
        └── RecordingLogger.hpp
```

The `hpp/.cpp` notation denotes separate header and source files. Future hardware adapters belong under `camera/backends/` and `transport/backends/`; they do not need directories or targets in Phase 0.

**2\. Module responsibilities**

| Module | Responsibility |
|---|---|
| `common` | Small shared value types, units, monotonic timestamps, and error results. Standard C++ only. |
| `camera` | Camera source contract and frame metadata. No capture implementation or image processing. |
| `transport` | Transport contract, connection events, and typed command/telemetry boundary. No radio or wire protocol. |
| `world` | One authoritative robot registry in metric world coordinates; controlled updates and read-only snapshots. |
| `core` | Application orchestration through injected interfaces. Contains no widgets or concrete backend selection. |
| `simulation` | Deterministic synthetic frames, connection/telemetry events, and scripted world observations. |
| `infrastructure` | Logging implementation and configuration parsing. Third-party dependencies stay here. |
| `ui` | Widgets, display conversion, and Qt signals/slots connecting user actions to the application core. |
| `app` | Composition root: configuration, logging, backend creation, ownership, startup, and shutdown. |

`MainWindow` handles layout and presentation. `DesktopController` handles the Qt bridge. Neither creates a camera or transport implementation.

**3\. Dependencies between modules**

Here, `A → B` means A depends on B:

```
camera interfaces ───→ common
transport interfaces → common
world ───────────────→ common

core → common + camera interfaces + transport interfaces + world
simulation → common + camera interfaces + transport interfaces

infrastructure → core contracts + spdlog / nlohmann_json
ui → core + Qt Core/Gui/Widgets

app → ui + core + simulation + infrastructure
```

Key boundaries:

- `core`, `world`, camera contracts, transport contracts, and simulation remain Qt-free.
- Only `app` knows which concrete implementations were selected.
- Public interface headers expose no `QImage`, `QVideoFrame`, `cv::Mat`, BLE handles, or vendor types.
- Future hardware targets privately link their hardware libraries.
- Domain configuration is typed C++; JSON remains an infrastructure detail.

For Phase 0, a `QTimer` can drive small, non-blocking core update steps. Fakes advance explicitly with supplied timestamps, so tests need neither timers nor sleeps.

Future acquisition and communication workers deliver events through queued Qt connections at the desktop boundary. They do not mutate `WorldModel` directly. Frame delivery uses a bounded latest-frame mailbox to prevent stale backlogs.

**4\. Classes and interfaces**

| Type | Proposed contract |
|---|---|
| `RobotId`, `CameraId` | Strong identifiers; avoid interchangeable bare integers or strings. |
| `Pose2D` | Position in metres and heading in radians. Define world axes and positive rotation explicitly. |
| `Timestamp`, `Duration` | Based on `std::chrono::steady_clock`; used for event ordering and freshness. |
| `Result<T>`, `Error` | Explicit success/error values suitable for C++20; distinguish failures from “no data available.” |
| `CameraInfo` | Source identity, name, and supported/basic frame metadata. |
| `Frame` | Sequence, capture timestamp, dimensions, stride, pixel format, and owned immutable pixel storage. |
| `PoseMeasurement` | Robot ID, world pose, quality, and observation timestamp. |
| `RobotState` | Optional observed pose, tracking status, connection status, and separate observation/transport timestamps. |

An unknown pose must remain unknown rather than appearing as `(0, 0, 0)`.

**`ICameraSource`**

A non-`QObject` interface with a virtual destructor and operations for:

- source information;
- opening and closing;
- current source state;
- non-blocking retrieval of the latest frame;
- retrieval of status/error events.

“No frame yet” is a normal result. Opening initiates source setup; implementations report readiness or failure without blocking the GUI.

**`IRobotTransport`**

A non-`QObject` interface with a virtual destructor and operations for:

- starting discovery;
- requesting connection/disconnection by robot ID;
- submitting a typed command;
- retrieving timestamped discovery, connection, telemetry, and error events;
- stopping the backend.

Command acceptance means accepted for dispatch, not confirmed execution. Phase 0 establishes this contract; the simulation only records submissions. Motor control, command encoding, and real protocol behavior remain deferred.

**`WorldModel`**

A plain C++ class with private storage and operations to:

- apply pose measurements;
- update connection status separately;
- find a robot;
- return a `WorldSnapshot`.

Use one writer, initially `ApplicationCore`. Snapshots prevent presentation code from modifying authoritative state. Older pose measurements should not overwrite newer ones.

No filtering, planning, control, or obstacle processing belongs here in Phase 0.

**`ApplicationCore`**

Receives camera and transport interfaces through dependency injection and owns the authoritative `WorldModel`. Provides start/stop, bounded update processing, backend status, and snapshots.

Use `std::unique_ptr` for backend ownership. Inject the logger by reference with an explicit lifetime. Avoid globals and service locators.

**Fake implementations**

- `FakeCameraSource`: synthetic test frames and scripted failures.
- `SimulationTransport`: scripted discovery, connection, disconnection, and telemetry; records submitted commands.
- `FakeWorldData`: scripted metric observations supplied to the core independently of image frames.

Synthetic frames do not produce robot localization in Phase 0. That requires the later vision pipeline. No robot dynamics simulation is needed yet.

**Logging and configuration**

- `ILogger`: a small Qt-free logging contract.
- `SpdlogLogger`: console/file logging behind that contract.
- `AppConfig`: typed backend selection, fake-data settings, logging settings, and UI update interval.
- `ConfigLoader`: parse and validate JSON into `AppConfig`.

Use defaults plus an explicit configuration-file override. Resolve writable paths in the desktop layer using Qt path facilities; do not assume the working directory or hardcode macOS paths. Log monotonic event timing and optionally wall-clock time for human-readable records.

**5\. CMake targets**

| Target | Kind | Dependencies |
|---|---|---|
| `micro_robot_common` | `INTERFACE` | C++20 |
| `micro_robot_camera_api` | `INTERFACE` | common |
| `micro_robot_transport_api` | `INTERFACE` | common |
| `micro_robot_world` | Static library | common |
| `micro_robot_core` | Static library | world, interface targets |
| `micro_robot_simulation` | Static library | common, interface targets |
| `micro_robot_infrastructure` | Static library | core contracts; private spdlog and JSON dependencies |
| `micro_robot_ui` | Static library | core, Qt Core/Gui/Widgets |
| `micro_robot_desktop` | Executable | UI, core, simulation, infrastructure |
| `micro_robot_tests` | Test executable | Qt-free libraries, `GTest::gtest_main` |

Build policy:

- Apply C++20 and warnings per target; disable compiler extensions.
- Enable Qt AUTOMOC only for Qt targets.
- Provide `MICRO_ROBOT_BUILD_DESKTOP` so core tests can configure without Qt installed.
- Use `BUILD_TESTING`, CTest, and GoogleTest discovery.
- Keep dependency versions reproducible; prefer installed packages with an explicit pinned-fetch fallback.
- Use portable presets without embedded Homebrew or developer-specific paths.
- Build a macOS application bundle; keep compiler and packaging conditionals in CMake for future Windows support.

Phase 0 requires only Qt Core/Gui/Widgets, spdlog, nlohmann/json, and GoogleTest. Multimedia, Bluetooth, OpenCV, AprilTag, and Eigen remain deferred.

Initial tests should cover world updates and timestamp ordering, configuration validation, deterministic fake behavior, and core startup/shutdown with injected backends. They should run without a GUI event loop or physical hardware.
