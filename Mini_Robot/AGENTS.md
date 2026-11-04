# Micro-Robot Swarm Desktop Control System — Agent Instructions

## Repository entry points and current implementation

Read the parent `../AGENTS.md` for repository-wide build ownership and maintenance
rules. The parent `../README.md` describes opening the whole repository in CLion.
The root CMake project adds this directory; standalone configuration here remains
supported and is used by existing CI. Shared presets live in this directory and
are included by the root preset file. Keep both entry points working, including
CTest discovery from the repository root. Do not assume `CMAKE_SOURCE_DIR` points
to `Mini_Robot`; use project/current-directory paths for application resources.

Use `docs/overview/capabilities.md` and `docs/overview/source-map.md` for current
implementation status. Camera adapters, intrinsic calibration/profiles, AprilTag
pixel observations/debugging, WorldModel, RobotManager, simulated transport, and
optional Qt BLE transport exist. Metric observation/tracking integration and the
complete closed-loop motion chain remain unfinished. The BLE fixture is a separate
connection-only PlatformIO build, not production motor/safety firmware.

From either entry point, `cmake --preset core`, `cmake --build --preset core`,
and `ctest --preset core` exercise hardware-free core suites without Qt/OpenCV.
Use the host desktop preset for GUI/adapter changes. A root build's desktop
artifact is under `build/<preset>/Mini_Robot/src/`; a standalone build's is under
`build/<preset>/src/`. Document actual verification, including unrun hardware checks.

## Optional structural navigation

Read the parent [repository instructions](../AGENTS.md), including the optional
Graphify policy. The [Graphify guide](docs/development/graphify.md) documents the
root-level wrapper, freshness, and recovery. From this directory use
`python3 ../tools/graphify/project_graph.py status`. Useful exploration targets
include ICameraSource, IRobotTransport, RobotManager, WorldModel, and their tests;
ambiguous class/constructor names need full node IDs. Check actual CMake target
ownership, interfaces, thread/lifetime contracts, and test bodies before editing.
Qt signal/slot paths and conditional builds are not completely modeled. No graph
result establishes fresh tracked poses, a completed perception-to-motion chain,
or physical motor/watchdog/E-stop acceptance. Preserve all architecture and safety
rules below. The ignored local engineering skill is not the team policy source.

## 1. Project purpose

Build a cross-platform desktop application for supervising a swarm of ESP32-C6 differential-drive micro-robots using external visual feedback.

The desktop application is the supervisory layer. It acquires camera data, estimates robot state, maintains a metric world model, accepts operator tasks, plans paths/trajectories, generates supervisory motion commands, communicates with robots, and records experiment data.

Primary platform:
- macOS on Apple Silicon;
- CLion;
- C++20;
- Qt 6;
- CMake.

Required additional platforms:
- Linux (Ubuntu LTS reference environment), supported from the earliest M1 foundation work;
- Windows with the same core source tree (retain as a project target; do not imply Windows validation has already been completed).

Cross-platform policy: develop on macOS, but do not introduce changes that knowingly break Linux. Keep Linux compilation and headless tests in CI as early as practical. Hardware-dependent capabilities may be verified separately.

Robot platform:
- ESP32-C6;
- low-level motor/safety control remains on the robot;
- desktop-to-robot transport starts with BLE and may later add Zigbee;
- Wi-Fi is reserved for bandwidth-heavy streams such as onboard video.

## 2. Instruction precedence

When implementing work, follow this order:

1. the current user request;
2. the assigned GitHub issue and its acceptance criteria;
3. this `AGENTS.md`;
4. the installed `micro-robot-swarm-desktop` skill and its references;
5. existing repository conventions and code.

If two requirements conflict, do not silently choose one. Preserve safety and architecture rules, explain the conflict, and make the smallest justified change.

## 3. Development model

Development is issue-driven.

A GitHub issue defines the implementation scope. Do not implement unrelated future issues merely because their architecture is visible.

Before coding:
- read the entire issue;
- confirm its dependencies are satisfied or identify the missing dependency;
- inspect the affected code and tests;
- identify architecture boundaries that must remain unchanged;
- plan the smallest coherent implementation that satisfies the acceptance criteria.

Completion is defined by the issue acceptance criteria plus relevant build/tests, not by the presence of code alone.

## 4. High-level architecture

Preserve this dependency direction:

```text
CameraSource(s)
    -> Vision / Calibration
    -> RobotTracker
    -> WorldModel
    -> Planning / Task layer
    -> Trajectory / Supervisory Control
    -> RobotManager
    -> RobotTransport
    -> ESP32-C6
```

Cross-cutting services:
- configuration;
- logging/measurement;
- simulation/replay;
- safety state;
- UI visualization.

The GUI observes and commands application services. It is not the place where perception, planning, transport, or control algorithms live.

## 5. Architectural invariants

Keep hardware/platform-specific code isolated behind interfaces.

Core application logic must not directly depend on:
- physical camera APIs;
- BLE implementation details;
- Zigbee implementation details;
- ESP32 hardware;
- Apple-only APIs;
- Windows-only APIs;
- Linux-only APIs;
- concrete GUI widgets.

Use abstractions such as:
- `ICameraSource`;
- `IRobotTransport`;
- `IPathPlanner` when planning becomes active;
- `ITrajectoryController` when trajectory tracking becomes active;
- a separate future depth-sensor abstraction for LiDAR/depth data.

Simulation, replay, and physical implementations must use compatible domain-facing interfaces.

Prefer dependency injection over hidden global state.

Avoid unnecessary singletons.

Do not bypass an existing abstraction to finish an issue faster.

## 6. World model and coordinate conventions

Maintain one authoritative application-level `WorldModel` for current world/robot state.

Do not allow independent subsystems to maintain conflicting authoritative robot poses.

Use explicit coordinate frames:
- image frame: pixels `(u, v)`;
- world/table frame: metric Cartesian coordinates;
- robot body frame: robot-relative coordinates.

Internal physical units:
- distance: metres unless a strongly typed representation explicitly states otherwise;
- angles: radians;
- time/duration: `std::chrono`;
- timestamps for latency and freshness: monotonic/steady clock.

UI may display millimetres and degrees, but conversions must occur at presentation boundaries.

Document:
- world origin;
- positive X/Y directions;
- zero heading direction;
- positive rotation convention.

Planning and control must operate in calibrated world coordinates, not raw pixels.

## 7. Camera and perception rules

Camera acquisition and vision processing are separate responsibilities.

A camera source provides timestamped frames and source metadata. It does not perform AprilTag detection or path planning.

The vision pipeline may include:
- intrinsic undistortion;
- fiducial detection;
- marker/robot ID association;
- conversion to world coordinates;
- quality metadata.

Use AprilTag-compatible fiducials for robot identity/orientation unless an issue explicitly selects another approach.

Use intrinsic camera calibration plus planar workspace calibration/homography for the tabletop system.

Calibration profiles must be associated with the camera/device identity and resolution where required.

Recorded-video/replay sources should remain first-class tools for deterministic debugging and tests.

## 8. Tracking rules

Treat a detection as a measurement, not permanent robot state.

Tracking must explicitly handle:
- timestamp/age;
- fresh/stale/lost state;
- angular wrapping;
- filtering;
- velocity/angular-velocity estimates when required;
- confidence/quality when available.

Never use indefinitely stale poses for control.

## 9. Communication rules

`IRobotTransport` isolates higher-level logic from radio technology.

Initial real transport:
- BLE Central on desktop;
- ESP32-C6 BLE peripheral/GATT service.

Future transports:
- Zigbee through a coordinator/gateway;
- optional Wi-Fi for suitable command/data channels.

High-rate video must not be sent over BLE or Zigbee.

Protocol design should support:
- robot identity;
- explicit message/command type;
- sequence numbers;
- command freshness where relevant;
- telemetry/state;
- error reporting;
- emergency stop;
- protocol/version evolution.

Do not expose BLE handles/UUID mechanics to planning/control modules.

## 10. Control and safety split

The desktop provides supervisory/outer-loop control. The ESP32-C6 provides fast low-level motor control and local safety.

Robot-side responsibilities include:
- wheel/motor regulation;
- PWM/motor output;
- command watchdog;
- safe motor stop;
- local actuator limits.

Desktop responsibilities include:
- desired pose/path/trajectory;
- external-vision state feedback;
- supervisory `v`/`omega` or equivalent commands;
- planning/coordination;
- high-level safety state.

Define explicit operational/safety states as the system grows, including at least concepts equivalent to:
- disconnected;
- idle/ready;
- running;
- paused;
- lost tracking;
- lost connection;
- emergency stop;
- fault.

Safety rules override normal task execution.

## 11. Planning rules

Separate:
- path geometry;
- trajectory/time parameterization;
- trajectory/path tracking control;
- multi-robot coordination.

Recommended progression:
- user-drawn path;
- path simplification/smoothing;
- Pure Pursuit or equivalent path following;
- manual obstacles;
- occupancy grid;
- obstacle inflation;
- A*;
- automatic obstacle extraction;
- multi-robot conflict prediction;
- prioritized planning;
- optional CBS/ORCA research extensions.

Do not introduce advanced planners when a simpler issue-scoped method satisfies the requirement.

## 12. Multi-camera and future depth sensing

Design camera ownership/calibration so multiple camera sources can be added later.

Each camera may require its own:
- intrinsic calibration;
- resolution;
- distortion model;
- world/extrinsic transform;
- timing metadata.

Transform measurements into the common world frame before fusion.

Treat iPhone LiDAR as a separate future depth-sensor pipeline, preferably through a small ARKit companion application sending depth/pose information over Wi-Fi. Do not couple the cross-platform desktop core directly to Apple ARKit.

## 13. C++ rules

Use C++20.

Prefer:
- RAII;
- `std::unique_ptr` for clear exclusive ownership;
- `std::shared_ptr` only when ownership is genuinely shared;
- const correctness;
- `enum class`;
- explicit domain types where they prevent unit/identity mistakes;
- `std::chrono` for time;
- standard containers/algorithms;
- small cohesive classes/functions.

Avoid:
- raw owning pointers;
- hidden lifetime assumptions;
- large god objects;
- business logic in Qt widgets;
- unnecessary inheritance when composition is clearer.

Public APIs must make ownership, units, thread expectations, and failure behavior understandable.

## 14. Qt and concurrency rules

Qt GUI objects remain on the GUI thread.

Use signals/slots or other explicit asynchronous boundaries for worker-to-GUI communication.

Potential workers include:
- camera acquisition;
- vision;
- planning/control;
- communication.

Do not block the GUI thread with:
- camera reads;
- BLE operations;
- planning loops;
- long OpenCV operations;
- sleeps/waits.

Use bounded buffering. For live visual feedback, prefer processing the newest useful frame over creating an unbounded stale-frame backlog.

Do not put application/business logic directly in `MainWindow`.

## 15. Configuration and logging

Configuration must not be scattered as duplicated literals throughout modules.

Use configuration/profile structures for values such as:
- camera selection/resolution;
- calibration data;
- workspace dimensions/transforms;
- transport parameters;
- controller/planner parameters;
- safety thresholds.

Use structured logging/measurement with timestamps and robot IDs where applicable.

Preserve data useful for thesis evaluation, including when relevant:
- camera FPS;
- perception latency;
- detection success;
- localization error;
- communication latency/jitter;
- control tracking error;
- planner time/path length;
- multi-robot separation/collisions.

## 16. Testing rules

Use GoogleTest for C++ unit tests unless the repository explicitly changes framework.

Core logic must be testable without a GUI or physical hardware.

Add/update tests for non-trivial pure logic, especially:
- units/angle normalization;
- coordinate transforms;
- calibration/profile parsing;
- protocol serialization;
- stale/fresh rules;
- path simplification;
- occupancy/inflation;
- A*;
- controller geometry;
- collision prediction.

Use deterministic integration inputs where practical:
- recorded video;
- simulated robot transport;
- fixed maps;
- deterministic trajectories.

Hardware-in-the-loop tests complement but do not replace unit/integration tests.

## 17. Build rules

The project must build from the command line with CMake.

### Early Linux compatibility requirements (M1)

- Maintain one C++20/CMake/Qt6/OpenCV source tree for macOS and Linux; preserve the existing Windows portability goal.
- Prefer portable CMake targets and `find_package()` over hardcoded Homebrew prefixes, macOS SDK paths, or Linux-specific include/library paths.
- Use platform conditionals only at well-defined adapter/build boundaries; do not leak operating-system APIs into the domain core.
- Validate Ubuntu LTS builds in GitHub Actions with GCC; include Clang validation when the corresponding Linux build issue is implemented. Continue macOS CI.
- Run GoogleTest/CTest tests on Linux without cameras, Bluetooth adapters, or physical robots; use mock `ICameraSource` and `IRobotTransport` implementations.
- Keep Qt6 GUI behavior portable; manually check startup, input, and scaling on Linux X11 and Wayland when GUI work is in scope. Headless CI alone does not prove GUI compatibility.
- Keep camera selection behind `ICameraSource`: use OpenCV/V4L2 on Linux where appropriate and a supported native/OpenCV backend on macOS. Report unavailable devices and permission errors without crashing.
- Keep BLE behind `IRobotTransport`; investigate Qt Bluetooth/BlueZ on Linux, but do not assume CI has Bluetooth hardware. Zigbee remains a separate future transport.
- Use Qt or standard C++ facilities for paths, timers, threads, and configuration wherever practical.
- Document reproducible Ubuntu LTS setup, dependencies, CMake build/test commands, camera permissions, and BlueZ requirements in `docs/development/linux-setup.md` when the documentation issue is implemented.
- Track the Linux foundation work through LNX-01 to LNX-08: build, CI, abstraction boundaries, camera discovery, transport preparation, GUI verification, setup documentation, and regression tests. Follow each GitHub issue's scope and acceptance criteria; do not implement all eight as part of an unrelated issue.
- Linux compatibility is an early requirement, not a reason to postpone core M1 features. Later milestones still own full hardware validation and packaging.

### Build and verification workflow

After meaningful changes:
1. configure if required;
2. build affected targets;
3. run relevant tests;
4. run broader tests when the change crosses module boundaries.

Do not claim a build/test passed unless it was actually executed successfully.

If hardware or platform access prevents a required test, state exactly what was and was not verified.

## 18. GitHub issue workflow

Every implementation issue uses its own branch.

### Start work

1. Ensure the working tree is clean or preserve unrelated user changes without overwriting them.
2. Fetch the remote default branch.
3. Create a new branch from the current remote default branch.
4. Use a branch name based on the issue, for example:
   - `feat/23-camera-calibration-wizard`
   - `fix/35-lost-tracking`
   - `test/41-localization-benchmark`
   - `docs/12-protocol-specification`
5. Never implement an issue directly on `main`, `master`, or another protected/default branch.

If uncommitted unrelated changes make safe branching impossible, stop and report the problem instead of discarding them.

### Commits

Use logical atomic commits.

A commit should implement one coherent part of the issue and leave the repository in a buildable/testable state whenever practical.

Do not mechanically create one commit per checkbox if several checkboxes form one inseparable change. Do not combine unrelated feature points into a single large commit.

Use Conventional Commit style, for example:
- `feat(camera): enumerate available video devices`
- `feat(calibration): persist intrinsic profiles`
- `test(world): cover homography conversion`
- `fix(tracking): handle heading wrap boundary`
- `docs(protocol): document BLE command framing`

Create commits as feature points are completed; do not wait until the end and squash all implementation into one commit.

Never commit:
- build outputs;
- generated temporary files;
- credentials/secrets;
- local IDE state that does not belong in the repository.

Do not commit a known failing state unless the issue explicitly requires a red/green test workflow and the repository convention allows it.

### Before PR

Before declaring the issue complete:
1. build affected targets;
2. run relevant tests;
3. review `git diff` and commit history;
4. compare the result against every acceptance criterion;
5. update documentation affected by public behavior/interfaces;
6. ensure unrelated work is absent from the branch.

### Push and pull request

After the issue is complete and verified:
1. push only the issue branch to the remote;
2. create a Pull Request automatically;
3. target the repository default branch;
4. use a concise descriptive PR title in Conventional Commit style: `type(scope): description` (omit the scope when unnecessary);
5. include `Closes #<issue-number>` when the PR should close the issue after merge;
6. summarize implementation by logical feature point;
7. list tests/build commands actually run and their results;
8. note any hardware/manual verification still required;
9. never merge the PR automatically.

Codex may create/update the PR, but final review and merge belong to the human maintainer.

PR titles must follow the same naming convention as commit messages. Choose the
type and optional scope for the final change, for example:

- `feat(camera): implement native stream start/stop`
- `docs(vision): define AprilTag family, print sizes, and robot ID mapping`
- `ci: add Linux build validation to GitHub Actions`

Never force-push unless explicitly instructed.

## 19. Pull request body expectations

Default PR body structure:

```markdown
## Summary
- ...

## Implementation
- ...

## Validation
- `cmake ...` — passed
- `cmake --build ...` — passed
- `ctest ...` — passed

## Manual / hardware checks
- ...

## Notes / limitations
- ...

Closes #123
```

Only claim commands/checks that were actually executed.

## 20. Scope discipline

Do not implement unrelated future work while completing an issue.

It is acceptable to add a small prerequisite refactor when it is necessary to implement the issue safely, but:
- keep it minimal;
- explain why it is required;
- preserve behavior;
- test it;
- commit it separately when it is logically distinct.

If an issue requires an architectural change that contradicts these instructions, surface the decision rather than silently rewriting project architecture.

## 21. Definition of done for an issue

An issue is implementation-complete only when:
- all required behavior is implemented;
- acceptance criteria are satisfied or explicitly documented as blocked;
- required tests are present and passing;
- affected targets build;
- documentation is updated where needed;
- changes are split into logical atomic commits;
- branch contains no unrelated changes;
- branch is pushed;
- PR is created with validation evidence;
- PR is not merged by Codex.
