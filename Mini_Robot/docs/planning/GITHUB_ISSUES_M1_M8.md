# GitHub Issue Catalog — M1 to M8

Project: **Design and implementation of a control system for a micro-robot swarm using external visual feedback**

This catalog is written as implementation contracts for Codex and maintainers. Create each section as a GitHub Issue and assign the listed milestone. Actual GitHub issue numbers are assigned by GitHub; branch names use that real number.

## Global Codex execution contract

For every implementation issue:

1. Read the whole issue and check dependencies.
2. Inspect the existing repository before designing the change.
3. Fetch the remote default branch and create **one new branch for this issue**. Never work directly on the default/protected branch.
4. Branch format: `<type>/<issue-number>-<short-slug>`, e.g. `feat/23-camera-calibration-wizard`.
5. Implement only the issue scope plus the smallest necessary prerequisite refactor.
6. Preserve `AGENTS.md` and the `micro-robot-swarm-desktop` skill architecture.
7. Create **logical atomic commits as feature points are completed**. Do not force one commit per checkbox; do not combine unrelated points.
8. Use Conventional Commit messages.
9. Build affected targets and run relevant tests after meaningful changes and before completion.
10. Review the diff and acceptance criteria.
11. Push only the issue branch.
12. Automatically create a Pull Request to the default branch with implementation summary and actual validation results. Use `Closes #<issue-number>` when appropriate.
13. **Never merge the PR automatically.** Human review/merge is required.
14. Never force-push unless explicitly instructed.

A blocked hardware/manual check must be stated in the PR; it must never be reported as passed without execution.

## Labels

Suggested label families: `app`, `core`, `camera`, `vision`, `calibration`, `world`, `tracking`, `protocol`, `ble`, `communication`, `control`, `safety`, `path`, `trajectory`, `planner`, `multi-robot`, `coordination`, `test`, `ci`, `windows`, `zigbee`, `lidar`, `video`, `task`, `optional`.

---

# M1 — Application Foundation & Camera Infrastructure

## Catalog Issue 1 — `[APP] Create Qt6/CMake application skeleton`

### Purpose

Establish the clean cross-platform application base and source layout before hardware or vision integration.

### Required behavior / tasks

- [ ] Create root CMake project using C++20
- [ ] Configure Qt 6 Core/Gui/Widgets targets required by the initial UI
- [ ] Create application executable and minimal MainWindow shell
- [ ] Create modular source/include/test directory structure aligned with AGENTS.md
- [ ] Add Debug and Release configuration support
- [ ] Add command-line build instructions to README
- [ ] Verify a clean macOS configure/build/launch

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Clean checkout configures with CMake
- [ ] Application builds without errors on macOS
- [ ] Application launches and displays the main window
- [ ] Core source layout does not place application logic in MainWindow
- [ ] README documents reproducible build commands

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

None.

### Priority

High

### Milestone

M1 — Application Foundation & Camera Infrastructure

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-create-qt6-cmake-application-skeleton`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `build: create C++20 Qt6 CMake application skeleton`
- `docs(build): document macOS configure and build workflow`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 2 — `[APP] Implement application logging and measurement foundation`

### Purpose

Create shared logging usable by all modules and suitable for later thesis measurements.

### Required behavior / tasks

- [ ] Define centralized logging service/interface
- [ ] Support debug/info/warning/error severity
- [ ] Use timestamps suitable for operational logs
- [ ] Include application startup/shutdown events
- [ ] Support module/robot context where applicable
- [ ] Add file logging with configurable destination/rotation policy if practical
- [ ] Document distinction between human logs and structured experiment measurements

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Multiple modules can emit timestamped logs
- [ ] Errors are distinguishable from normal events
- [ ] Startup/shutdown are recorded
- [ ] Logging can be used without coupling core logic to GUI widgets
- [ ] Logging failures do not crash the application

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 1

### Priority

Medium

### Milestone

M1 — Application Foundation & Camera Infrastructure

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-implement-application-logging-and-measurement-foundatio`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(logging): add shared application logging service`
- `docs(logging): document logging and measurement conventions`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 3 — `[APP] Implement configuration and profile loading`

### Purpose

Provide validated external configuration instead of scattering tunable values through source code.

### Required behavior / tasks

- [ ] Select initial configuration representation
- [ ] Define application settings service/model
- [ ] Load settings on startup with sensible defaults
- [ ] Handle missing and invalid files safely
- [ ] Expose validated values to modules through explicit interfaces
- [ ] Define locations for later calibration/workspace profiles
- [ ] Log configuration warnings without duplicating parsing logic

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Application starts with valid configuration
- [ ] Application starts with defaults when optional config is absent
- [ ] Invalid values produce actionable diagnostics
- [ ] Configuration access is centralized
- [ ] No GUI widget becomes the authoritative settings store

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 1
- Catalog Issue 2

### Priority

Medium

### Milestone

M1 — Application Foundation & Camera Infrastructure

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-implement-configuration-and-profile-loading`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(config): add validated application settings loader`
- `test(config): cover defaults and invalid configuration`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 4 — `[CORE] Define common domain types, units, time and result conventions`

### Purpose

Create shared types so vision, simulation, planning, communication and control use consistent identity, pose, units and timestamps.

### Required behavior / tasks

- [ ] Define RobotId
- [ ] Define Pose2D and basic vector/velocity domain representations as needed
- [ ] Use metres and radians internally
- [ ] Define monotonic timestamp/time aliases using std::chrono
- [ ] Define angle normalization helpers
- [ ] Define explicit error/result convention for core services
- [ ] Document coordinate/unit conventions
- [ ] Add focused unit tests

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Core types compile without GUI dependencies
- [ ] Pose representation is reusable by simulation and later vision
- [ ] Units and angle conventions are documented
- [ ] Timestamp usage is consistent
- [ ] Angle helper tests cover wrap boundaries

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 1

### Priority

High

### Milestone

M1 — Application Foundation & Camera Infrastructure

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-define-common-domain-types-units-time-and-result-conven`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(core): define robot pose unit and time domain types`
- `test(core): cover unit and angle conventions`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 5 — `[CORE] Define camera source and frame abstractions`

### Purpose

Create the stable camera boundary used by system cameras, recorded video and future network sources.

### Required behavior / tasks

- [ ] Define ICameraSource lifecycle/state contract
- [ ] Define timestamped Frame/FrameMetadata representation without embedding application UI
- [ ] Define source/device metadata and resolution/fps information
- [ ] Define frame ownership/lifetime expectations
- [ ] Define asynchronous delivery or bounded pull semantics compatible with Qt workers
- [ ] Define error/state reporting
- [ ] Document threading expectations

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] GUI/application services can depend on ICameraSource rather than concrete camera APIs
- [ ] Multiple camera implementations can conform to the same contract
- [ ] Frame timestamps and metadata are available
- [ ] Interface does not perform vision processing
- [ ] Ownership/thread expectations are documented

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 4

### Priority

High

### Milestone

M1 — Application Foundation & Camera Infrastructure

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-define-camera-source-and-frame-abstractions`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(camera): define source and timestamped frame abstractions`
- `test(camera): validate frame metadata invariants`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 6 — `[CORE] Define robot transport and protocol domain boundary`

### Purpose

Create a transport-independent command/telemetry interface suitable for simulation, BLE and later Zigbee.

### Required behavior / tasks

- [ ] Define IRobotTransport connection lifecycle
- [ ] Define domain RobotCommand and RobotTelemetry/Status representations
- [ ] Define connection-state and transport-error reporting
- [ ] Reserve sequence/version/freshness fields needed by later real transports
- [ ] Keep interface free of BLE/Zigbee handles or UUID types
- [ ] Document asynchronous/thread expectations

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Simulation and future BLE/Zigbee can implement the same interface
- [ ] Higher-level code sends domain commands without knowing transport type
- [ ] Transport state/errors can be observed
- [ ] Protocol domain model can evolve/version without GUI coupling

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 4

### Priority

High

### Milestone

M1 — Application Foundation & Camera Infrastructure

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-define-robot-transport-and-protocol-domain-boundary`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(transport): define transport-independent robot command model`
- `test(transport): cover command and status domain invariants`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 7 — `[WORLD] Create authoritative WorldModel skeleton`

### Purpose

Create the application-level state container that later receives tracked robot poses, obstacles and task/planning state.

### Required behavior / tasks

- [ ] Define WorldModel responsibility and ownership
- [ ] Add storage/query/update API for known robot state
- [ ] Keep update API timestamp-aware
- [ ] Reserve clean extension points for workspace/obstacle/task state without implementing future features
- [ ] Keep WorldModel independent of camera and transport implementations
- [ ] Add unit tests for robot insert/update/query

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Robot state can be inserted updated and retrieved by RobotId
- [ ] WorldModel has no direct physical camera/BLE dependencies
- [ ] Tests demonstrate independent robot state
- [ ] Public API documents authoritative-state semantics

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 4

### Priority

High

### Milestone

M1 — Application Foundation & Camera Infrastructure

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-create-authoritative-worldmodel-skeleton`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(world): add authoritative robot state model`
- `test(world): cover robot state update and lookup`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 8 — `[SIM] Implement deterministic SimulationTransport and fake robot source`

### Purpose

Enable UI/application development and tests without physical ESP32-C6 hardware.

### Required behavior / tasks

- [ ] Implement IRobotTransport in simulation
- [ ] Simulate connect/disconnect and connection state
- [ ] Accept domain commands
- [ ] Generate deterministic telemetry/status
- [ ] Allow configurable/fixed robot IDs
- [ ] Add hooks for deterministic fault/loss scenarios used later
- [ ] Log simulated communication without special-casing higher-level code

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Application connects to a simulated robot through IRobotTransport
- [ ] Commands are accepted and telemetry returned
- [ ] Behavior can be deterministic in tests
- [ ] Higher-level code does not need to detect simulation type

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 6
- Catalog Issue 2

### Priority

High

### Milestone

M1 — Application Foundation & Camera Infrastructure

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-implement-deterministic-simulationtransport-and-fake-ro`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(sim): add deterministic robot transport simulator`
- `test(sim): cover connect command and telemetry flow`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 9 — `[TEST] Configure GoogleTest and CTest infrastructure`

### Purpose

Establish repeatable unit/integration testing from the command line.

### Required behavior / tasks

- [ ] Add GoogleTest dependency using repository-approved dependency mechanism
- [ ] Create test targets separate from application executable
- [ ] Integrate tests with CTest
- [ ] Add one representative core test and one simulated-service test
- [ ] Document test commands
- [ ] Ensure test setup works from a clean build directory

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Tests build successfully
- [ ] ctest runs from command line
- [ ] At least representative core and simulation tests pass
- [ ] Tests do not require GUI or physical hardware

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 1

### Priority

High

### Milestone

M1 — Application Foundation & Camera Infrastructure

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `test/<issue-number>-configure-googletest-and-ctest-infrastructure`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `test(build): configure GoogleTest and CTest targets`
- `docs(test): document local test commands`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 10 — `[CAMERA] Enumerate and select available camera devices`

### Purpose

Expose cameras provided by the OS and let the operator choose the active source.

### Required behavior / tasks

- [ ] Implement Qt camera-device discovery behind the camera service boundary
- [ ] Expose human-readable name and stable identifier where available
- [ ] Add refresh/device-change handling
- [ ] Add camera selection UI
- [ ] Handle zero-camera state
- [ ] Persist preferred device only when safe to do so
- [ ] Log device discovery and selection

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Internal/USB cameras exposed by macOS appear when available
- [ ] Compatible iPhone Continuity Camera appears when macOS exposes it
- [ ] User can select among devices
- [ ] Device removal does not leave an invalid selected object that crashes the app

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 1
- Catalog Issue 5

### Priority

High

### Milestone

M1 — Application Foundation & Camera Infrastructure

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-enumerate-and-select-available-camera-devices`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(camera): enumerate system video devices`
- `feat(ui): add active camera selection control`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 11 — `[CAMERA] Implement live camera start/stop and responsive preview`

### Purpose

Deliver current live frames to the UI without blocking or accumulating unbounded stale frames.

### Required behavior / tasks

- [ ] Implement concrete Qt system camera source
- [ ] Add start/stop lifecycle and state transitions
- [ ] Display latest frame in the application while preserving aspect ratio
- [ ] Handle frame-format conversion at a clear boundary
- [ ] Measure/display active resolution and delivered/displayed FPS
- [ ] Use bounded/latest-frame behavior rather than unbounded queue growth
- [ ] Keep GUI responsive during repeated start/stop cycles

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Live video is visible and updates continuously
- [ ] Start/stop can be repeated without restart
- [ ] GUI remains responsive
- [ ] Aspect ratio is preserved
- [ ] Measured resolution/FPS are shown
- [ ] No unbounded frame backlog is created

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 10

### Priority

High

### Milestone

M1 — Application Foundation & Camera Infrastructure

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-implement-live-camera-start-stop-and-responsive-preview`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(camera): implement Qt live camera source`
- `feat(ui): display responsive live preview and stream metrics`
- `test(camera): cover source state transitions where practical`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 12 — `[CAMERA] Handle camera disconnect reconnect and source failure`

### Purpose

Make camera loss a recoverable application state rather than a crash.

### Required behavior / tasks

- [ ] Detect source/device loss
- [ ] Stop invalid frame processing
- [ ] Publish clear source/error state to UI/logging
- [ ] Refresh available devices after topology changes
- [ ] Allow reselection/restart after reconnect
- [ ] Verify repeated unplug/reconnect or simulated failure behavior

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Application does not crash when an active camera disappears
- [ ] User sees clear state/error feedback
- [ ] Reconnected or alternative camera can be selected and started
- [ ] Stale frames are not presented as current indefinitely

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 11

### Priority

Medium

### Milestone

M1 — Application Foundation & Camera Infrastructure

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-handle-camera-disconnect-reconnect-and-source-failure`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `fix(camera): handle source loss and invalid frame state`
- `feat(camera): support camera reselection after reconnect`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 13 — `[CAMERA] Add recorded-video camera source for deterministic replay`

### Purpose

Provide repeatable video input through the same frame abstraction used by live cameras.

### Required behavior / tasks

- [ ] Implement recorded-video source conforming to ICameraSource
- [ ] Support selecting a local video file
- [ ] Deliver frames with deterministic timestamps/playback policy
- [ ] Support start stop EOF and optional loop
- [ ] Ensure future vision consumers do not distinguish live versus replay source
- [ ] Add a small integration test or documented fixture path strategy

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Recorded video feeds the same downstream frame interface
- [ ] Start/stop/EOF behavior is well-defined
- [ ] Replay is repeatable for debugging
- [ ] No vision-code change is required to switch source types

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 5
- Catalog Issue 9

### Priority

Medium

### Milestone

M1 — Application Foundation & Camera Infrastructure

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-add-recorded-video-camera-source-for-deterministic-repl`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(camera): add recorded video replay source`
- `test(camera): cover deterministic replay lifecycle`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 14 — `[CI] Add macOS build and test continuous integration`

### Purpose

Verify the repository foundation from a clean environment on each change.

### Required behavior / tasks

- [ ] Add CI workflow for supported macOS runner/toolchain
- [ ] Configure CMake build
- [ ] Run relevant tests via CTest
- [ ] Cache dependencies only if it does not hide correctness
- [ ] Keep secrets out of workflow
- [ ] Document local equivalent commands

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] CI configures and builds the application on macOS
- [ ] CI runs tests
- [ ] Failure is visible when build/tests fail
- [ ] Workflow does not depend on developer-local paths

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 1
- Catalog Issue 9

### Priority

Medium

### Milestone

M1 — Application Foundation & Camera Infrastructure

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-add-macos-build-and-test-continuous-integration`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `ci: add macOS CMake build and test workflow`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---


# M2 — Vision, Calibration & Robot Localization

## Catalog Issue 15 — `[VISION] Select AprilTag family marker size and robot-ID mapping`

### Purpose

Choose the fiducial configuration based on expected camera height, field of view, tag size and robot count.

### Required behavior / tasks

- [ ] Compare appropriate AprilTag families supported by chosen detector
- [ ] Select physical tag size and printable layout
- [ ] Define tag ID to RobotId mapping rules
- [ ] Generate printable test markers
- [ ] Document expected camera height/resolution assumptions
- [ ] Record rationale and limitations

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Tag family and physical size are documented
- [ ] Enough unique IDs exist for expected swarm size
- [ ] Printable markers are available
- [ ] Selection rationale is reproducible

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 11
- Catalog Issue 13

### Priority

High

### Milestone

M2 — Vision, Calibration & Robot Localization

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-select-apriltag-family-marker-size-and-robot-id-mapping`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `docs(vision): define AprilTag family size and robot ID mapping`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 16 — `[VISION] Integrate AprilTag detector into frame pipeline`

### Purpose

Detect zero or more fiducials from live and recorded frames without coupling detector logic to GUI widgets.

### Required behavior / tasks

- [ ] Add detector dependency and build integration
- [ ] Convert frame representation to detector input efficiently
- [ ] Run detection in appropriate worker/context
- [ ] Return marker ID ordered corners timestamp and detector quality when available
- [ ] Support multiple tags per frame
- [ ] Measure detector processing time
- [ ] Handle detector failures without crashing pipeline

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Printed tags are detected on recorded and live frames
- [ ] Multiple tags can be returned per frame
- [ ] Detection result includes ID ordered corners and timestamp
- [ ] GUI remains responsive
- [ ] Basic processing-time measurement is available

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 15
- Catalog Issue 11
- Catalog Issue 13

### Priority

High

### Milestone

M2 — Vision, Calibration & Robot Localization

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-integrate-apriltag-detector-into-frame-pipeline`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(vision): integrate AprilTag detector`
- `feat(vision): expose timestamped multi-tag observations`
- `test(vision): cover observation mapping with deterministic samples`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 17 — `[VISION] Define marker observation robot identity center and image-space heading`

### Purpose

Convert raw detector outputs into a stable application observation model before metric calibration.

### Required behavior / tasks

- [ ] Define MarkerObservation/RobotObservation measurement structure
- [ ] Map supported marker IDs to RobotId
- [ ] Calculate marker center from ordered corners
- [ ] Define provisional image-space heading convention
- [ ] Normalize angles consistently
- [ ] Handle unknown/unsupported IDs safely
- [ ] Add tests for mapping center and rotation cases

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Known tags map to correct RobotId
- [ ] Unknown tags cannot corrupt tracked robot state
- [ ] Center follows translation
- [ ] Heading follows full rotation range consistently
- [ ] Observation includes timestamp/quality metadata

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 16
- Catalog Issue 4

### Priority

High

### Milestone

M2 — Vision, Calibration & Robot Localization

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-define-marker-observation-robot-identity-center-and-ima`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(vision): add robot marker observation model`
- `feat(vision): compute marker center and provisional heading`
- `test(vision): cover ID mapping and rotation`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 18 — `[VISION] Add vision debug overlay and performance diagnostics`

### Purpose

Visualize what perception sees so calibration/tracking errors can be diagnosed.

### Required behavior / tasks

- [ ] Draw tag boundary and center
- [ ] Display tag and RobotId
- [ ] Draw orientation indicator
- [ ] Display observation age/quality where useful
- [ ] Show detector processing time/FPS in diagnostics
- [ ] Keep overlay mapping correct when display is resized
- [ ] Support several tags simultaneously

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Overlay aligns with physical marker image
- [ ] IDs are readable and not mixed
- [ ] Heading indicator rotates correctly
- [ ] Display resizing does not shift overlay
- [ ] Multiple detections are independently visualized

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 17

### Priority

Medium

### Milestone

M2 — Vision, Calibration & Robot Localization

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-add-vision-debug-overlay-and-performance-diagnostics`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(vision): add fiducial debug overlay`
- `feat(ui): expose detector diagnostics`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 19 — `[CALIB] Implement intrinsic camera calibration wizard and target capture`

### Purpose

Create an in-app workflow to collect valid calibration observations from a known target.

### Required behavior / tasks

- [ ] Select ChArUco or chessboard workflow and document target dimensions
- [ ] Create calibration-wizard UI state machine
- [ ] Capture candidate frames from active camera
- [ ] Detect and visualize calibration corners/markers
- [ ] Reject unusable or redundant views with clear feedback
- [ ] Track capture progress/diversity
- [ ] Allow cancel restart and recapture

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] User can complete capture without external script
- [ ] Calibration target is detected from varied positions/orientations
- [ ] Bad frames are rejected or flagged
- [ ] User can visually inspect accepted detections

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 11

### Priority

High

### Milestone

M2 — Vision, Calibration & Robot Localization

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-implement-intrinsic-camera-calibration-wizard-and-targe`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(calibration): add intrinsic calibration capture workflow`
- `feat(calibration): validate and visualize calibration observations`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 20 — `[CALIB] Calculate camera intrinsics distortion and reprojection error`

### Purpose

Turn collected target observations into a validated intrinsic camera model.

### Required behavior / tasks

- [ ] Calculate camera matrix fx fy cx cy
- [ ] Calculate supported distortion coefficients
- [ ] Calculate aggregate and per-frame reprojection error
- [ ] Validate finite/plausible output
- [ ] Expose result to vision/calibration services
- [ ] Display calibration quality and warning threshold
- [ ] Add deterministic tests for profile/result validation

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Valid camera matrix and distortion vector are produced
- [ ] Reprojection error is shown
- [ ] Invalid calibration is rejected
- [ ] Result can be consumed without GUI dependency

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 19

### Priority

High

### Milestone

M2 — Vision, Calibration & Robot Localization

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-calculate-camera-intrinsics-distortion-and-reprojection`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(calibration): compute intrinsic camera model`
- `feat(calibration): report reprojection error`
- `test(calibration): validate calibration result handling`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 21 — `[CALIB] Persist calibration profiles per camera and resolution`

### Purpose

Save/load intrinsic calibration safely and ensure incompatible profiles are not silently reused.

### Required behavior / tasks

- [ ] Define versioned calibration profile format
- [ ] Store camera/device identifier and resolution
- [ ] Store camera matrix distortion and reprojection error
- [ ] Load/select compatible profile
- [ ] Reject/warn on incompatible resolution/device
- [ ] Add optional undistorted preview for verification
- [ ] Add round-trip serialization tests

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Calibration survives application restart
- [ ] Correct profile is selected for camera/resolution
- [ ] Mismatched profile is rejected or requires explicit user action
- [ ] Undistorted preview can be toggled when profile exists
- [ ] Profile round-trip tests pass

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 20
- Catalog Issue 3

### Priority

High

### Milestone

M2 — Vision, Calibration & Robot Localization

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-persist-calibration-profiles-per-camera-and-resolution`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(calibration): persist versioned intrinsic profiles`
- `feat(calibration): add optional undistorted preview`
- `test(calibration): cover profile compatibility and round trip`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 22 — `[WORLD] Define tabletop world coordinate frame and workspace calibration model`

### Purpose

Create an unambiguous physical coordinate system shared by localization planning and control.

### Required behavior / tasks

- [ ] Define world origin and positive X/Y axes
- [ ] Define zero heading and positive angular direction
- [ ] Use metres/radians internally
- [ ] Define workspace bounds/dimensions
- [ ] Define versioned workspace-calibration profile structure
- [ ] Create visual documentation/overlay convention
- [ ] Add invariant tests for conversions where possible

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Coordinate convention is documented and visible in UI/docs
- [ ] Workspace bounds use metric units
- [ ] Future Pose2D values have unambiguous angle/axis meaning
- [ ] Workspace calibration is separate from intrinsic calibration

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 4
- Catalog Issue 21

### Priority

High

### Milestone

M2 — Vision, Calibration & Robot Localization

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-define-tabletop-world-coordinate-frame-and-workspace-ca`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(world): define metric workspace coordinate model`
- `docs(world): document tabletop frame and heading convention`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 23 — `[WORLD] Implement camera-to-table homography calibration`

### Purpose

Compute and persist planar transform between image coordinates and the physical tabletop.

### Required behavior / tasks

- [ ] Support acquiring at least four non-degenerate image/world correspondences
- [ ] Validate workspace points and ordering
- [ ] Compute homography using calibrated/undistorted image convention consistently
- [ ] Calculate mapping residual/check points where possible
- [ ] Persist transform in workspace profile
- [ ] Provide UI feedback for invalid calibration
- [ ] Add deterministic transform tests

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Known image points map to expected world positions
- [ ] Degenerate/invalid calibration is rejected
- [ ] Transform survives save/load
- [ ] Convention for distorted versus undistorted coordinates is documented and consistent

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 21
- Catalog Issue 22

### Priority

High

### Milestone

M2 — Vision, Calibration & Robot Localization

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-implement-camera-to-table-homography-calibration`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(world): add tabletop homography calibration`
- `feat(world): persist workspace transform`
- `test(world): cover image-to-world homography`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 24 — `[WORLD] Convert marker observations to metric robot Pose2D`

### Purpose

Produce `(RobotId, x, y, theta, timestamp)` measurements in the common world frame.

### Required behavior / tasks

- [ ] Transform marker center and required corners to world coordinates
- [ ] Compute world-space heading from transformed geometry
- [ ] Populate metric Pose2D and timestamp/quality
- [ ] Reject observations outside invalid/un-calibrated conditions
- [ ] Feed measurement toward tracker/WorldModel through explicit boundary
- [ ] Add known-point/known-angle tests

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Visible calibrated tag produces metric x y theta
- [ ] Heading follows world convention
- [ ] No raw pixel pose is passed to planning/control
- [ ] Uncalibrated state is represented explicitly
- [ ] Tests cover representative points/angles

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 17
- Catalog Issue 23

### Priority

High

### Milestone

M2 — Vision, Calibration & Robot Localization

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-convert-marker-observations-to-metric-robot-pose2d`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(world): convert fiducial observations to metric poses`
- `test(world): cover metric pose and heading conversion`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 25 — `[TRACKING] Implement per-robot tracker with fresh stale and lost states`

### Purpose

Convert independent pose measurements into persistent robot state with explicit observation age.

### Required behavior / tasks

- [ ] Create RobotTracker keyed by RobotId
- [ ] Store/update latest measurement and timestamps
- [ ] Define configurable fresh stale lost thresholds
- [ ] Publish tracking state/age
- [ ] Handle new and reappearing robots
- [ ] Ensure old pose is not treated as current indefinitely
- [ ] Update WorldModel through defined interface
- [ ] Add time-controlled tests

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Multiple robot IDs maintain independent state
- [ ] Fresh/stale/lost transitions are deterministic
- [ ] Reappearance restores tracking safely
- [ ] WorldModel can inspect state age
- [ ] Tests do not depend on wall-clock sleeps

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 24
- Catalog Issue 7

### Priority

High

### Milestone

M2 — Vision, Calibration & Robot Localization

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-implement-per-robot-tracker-with-fresh-stale-and-lost-s`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(tracking): add per-robot tracking lifecycle`
- `feat(tracking): implement stale and lost observation rules`
- `test(tracking): cover time-based state transitions`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 26 — `[TRACKING] Implement heading unwrapping and exponential pose smoothing`

### Purpose

Reduce measurement noise without introducing angular discontinuities.

### Required behavior / tasks

- [ ] Implement normalized angle and unwrap helpers using existing conventions
- [ ] Apply configurable exponential smoothing to position
- [ ] Smooth orientation in a wrap-safe representation
- [ ] Define initialization/reset behavior
- [ ] Expose raw and filtered pose for diagnostics
- [ ] Add tests around +/-pi and repeated measurements

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] 179° to -179° equivalent transition is treated as small rotation
- [ ] Stationary filtered pose is less noisy than raw samples
- [ ] Filter parameters are configurable
- [ ] Initialization does not create large transients
- [ ] Tests cover wrap boundaries

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 25

### Priority

High

### Milestone

M2 — Vision, Calibration & Robot Localization

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-implement-heading-unwrapping-and-exponential-pose-smoot`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(tracking): implement wrap-safe heading continuity`
- `feat(tracking): add configurable exponential pose filter`
- `test(tracking): cover smoothing and angle boundaries`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 27 — `[TRACKING] Estimate velocity angular velocity confidence and observation age`

### Purpose

Extend tracked state with motion estimates and reliability metadata needed by control.

### Required behavior / tasks

- [ ] Calculate translational velocity from timestamped poses
- [ ] Calculate angular velocity using unwrapped heading
- [ ] Handle tiny invalid and very large dt
- [ ] Filter estimates if required
- [ ] Define confidence representation using detector quality/age where meaningful
- [ ] Expose age velocity and confidence to UI/control
- [ ] Add deterministic tests

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Stationary robot reports approximately zero motion
- [ ] Known synthetic motion produces plausible estimates
- [ ] Invalid dt does not produce unstable values
- [ ] Control code can inspect observation age/confidence

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 26
- Catalog Issue 18

### Priority

High

### Milestone

M2 — Vision, Calibration & Robot Localization

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-estimate-velocity-angular-velocity-confidence-and-obser`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(tracking): estimate linear and angular velocity`
- `feat(tracking): expose confidence and observation age`
- `test(tracking): cover motion estimation edge cases`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 28 — `[TRACKING] Add optional Kalman pose filter and comparison harness`

### Purpose

Provide a more advanced estimator only after the simple tracker is functional.

> **Optional / research extension:** this issue does not block the minimum thesis system unless explicitly promoted to required scope.

### Required behavior / tasks

- [ ] Define state and measurement vectors appropriate to available data
- [ ] Implement predict/correct with timestamp-aware dt
- [ ] Expose process/measurement noise configuration
- [ ] Support selecting exponential versus Kalman filter
- [ ] Create replay comparison harness
- [ ] Record stability/lag metrics and document whether improvement justifies complexity

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Kalman option works on replay/live tracked poses
- [ ] Output remains stable when stationary and responsive while moving
- [ ] Simple filter remains available
- [ ] Comparison data is recorded rather than assuming Kalman is superior

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 27
- Catalog Issue 13

### Priority

Low

### Milestone

M2 — Vision, Calibration & Robot Localization

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-add-optional-kalman-pose-filter-and-comparison-harness`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(tracking): add optional Kalman pose estimator`
- `test(tracking): compare estimators on deterministic replay`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 29 — `[TEST] Measure localization and orientation accuracy across workspace`

### Purpose

Quantitatively validate camera -> marker -> calibration -> tracking performance for thesis use.

### Required behavior / tasks

- [ ] Define physical test-point grid and ground-truth measurement procedure
- [ ] Collect raw and filtered pose data at multiple workspace locations/orientations
- [ ] Calculate mean median RMSE standard deviation and maximum position error
- [ ] Calculate orientation error where ground truth is available
- [ ] Record detector success/loss rate
- [ ] Save raw measurements and analysis-friendly export
- [ ] Document camera height resolution lighting tag size and calibration profile

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Multiple points cover center and workspace edges
- [ ] Mean RMSE and max position error are reported
- [ ] Orientation accuracy is reported or limitation documented
- [ ] Procedure is repeatable
- [ ] Raw results are preserved for thesis plots

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 24
- Catalog Issue 25
- Catalog Issue 26

### Priority

High

### Milestone

M2 — Vision, Calibration & Robot Localization

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `test/<issue-number>-measure-localization-and-orientation-accuracy-across-wo`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `test(localization): add repeatable workspace accuracy protocol`
- `docs(thesis): record localization benchmark methodology`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---


# M3 — Robot Communication & Manual Safety Control

## Catalog Issue 30 — `[PROTOCOL] Define versioned robot command telemetry and BLE GATT contract`

### Purpose

Specify the wire-level behavior before coupling desktop code to BLE APIs.

### Required behavior / tasks

- [ ] Define protocol version and robot identity semantics
- [ ] Define command types including stop/estop and supervisory velocity
- [ ] Define telemetry/status payload fields
- [ ] Define sequence numbers acknowledgement/error semantics as needed
- [ ] Define command freshness/watchdog expectations
- [ ] Define BLE service and characteristic UUID layout
- [ ] Define JSON bring-up representation and binary migration boundary if needed
- [ ] Document endianness/scaling for binary fields

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Protocol document is sufficient to implement both desktop and ESP32 sides
- [ ] Unknown versions/messages can be rejected safely
- [ ] E-stop and stale-command behavior are explicit
- [ ] Domain model remains transport independent

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 6

### Priority

High

### Milestone

M3 — Robot Communication & Manual Safety Control

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `docs/<issue-number>-define-versioned-robot-command-telemetry-and-ble-gatt-c`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `docs(protocol): define versioned robot message contract`
- `docs(ble): define GATT service and safety semantics`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 31 — `[BLE] Implement BLE device discovery connection and state reporting`

### Purpose

Create the concrete BLE transport lifecycle on macOS through Qt Bluetooth.

### Required behavior / tasks

- [ ] Implement BleTransport behind IRobotTransport
- [ ] Scan/filter candidate robot peripherals
- [ ] Expose discovered identity/name/address abstraction where platform permits
- [ ] Connect/disconnect and discover required service/characteristics
- [ ] Publish connection state and errors
- [ ] Keep BLE callbacks off business/UI logic
- [ ] Support cancellation/timeout

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] ESP32-C6 peripheral can be discovered and connected repeatedly
- [ ] Connection/service-discovery states are observable
- [ ] Failure/timeout produces clear error
- [ ] Higher-level modules only use IRobotTransport

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 30

### Priority

High

### Milestone

M3 — Robot Communication & Manual Safety Control

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-implement-ble-device-discovery-connection-and-state-rep`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(ble): implement robot discovery and connection lifecycle`
- `test(ble): cover state machine with fakes where practical`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 32 — `[BLE] Implement BLE command writes telemetry notifications and protocol validation`

### Purpose

Complete the bidirectional data path using the documented protocol.

### Required behavior / tasks

- [ ] Serialize domain commands to protocol frames
- [ ] Write command/estop characteristics appropriately
- [ ] Subscribe to telemetry/status notifications
- [ ] Parse and validate incoming messages/version/sequence
- [ ] Reject malformed data safely
- [ ] Expose telemetry through IRobotTransport
- [ ] Log latency-relevant timestamps/sequence IDs

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Commands reach the ESP32-C6
- [ ] Telemetry/status updates reach desktop domain model
- [ ] Malformed/version-incompatible messages are handled
- [ ] Sequence information can support measurement/debugging

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 31
- Catalog Issue 30

### Priority

High

### Milestone

M3 — Robot Communication & Manual Safety Control

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-implement-ble-command-writes-telemetry-notifications-an`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(ble): add command serialization and GATT writes`
- `feat(ble): parse telemetry notifications`
- `test(protocol): cover serialization and malformed frames`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 33 — `[COMM] Implement RobotManager connection ownership and per-robot state`

### Purpose

Centralize transport sessions and map telemetry/commands to RobotId without leaking BLE details.

### Required behavior / tasks

- [ ] Create RobotManager application service
- [ ] Own/coordinate transport connections using dependency injection
- [ ] Map connected endpoints to RobotId
- [ ] Route telemetry/status to appropriate robot state
- [ ] Provide send command/stop API for higher-level control
- [ ] Expose connection state to UI/WorldModel as appropriate
- [ ] Prepare for several robots without implementing M7 coordination

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] One or more robot sessions can be represented without state cross-talk
- [ ] Control code addresses RobotId not BLE objects
- [ ] Disconnect cleans up ownership safely
- [ ] Simulation and BLE can be selected behind same service boundary

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 32
- Catalog Issue 8

### Priority

High

### Milestone

M3 — Robot Communication & Manual Safety Control

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-implement-robotmanager-connection-ownership-and-per-rob`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(comm): add transport-independent RobotManager`
- `test(comm): cover robot command and telemetry routing`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 34 — `[SIM] Keep simulation protocol behavior compatible with real command model`

### Purpose

Update simulator so manual/control development can exercise the same semantics as the real protocol.

### Required behavior / tasks

- [ ] Support current RobotCommand variants
- [ ] Support sequence/freshness semantics where meaningful
- [ ] Simulate telemetry/status fields used by UI
- [ ] Simulate disconnect/watchdog/error scenarios
- [ ] Add deterministic tests for safety transitions

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Manual UI/control can switch between simulation and BLE without changing command types
- [ ] Safety/error scenarios can be reproduced without hardware
- [ ] Simulator tests remain deterministic

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 30
- Catalog Issue 33

### Priority

Medium

### Milestone

M3 — Robot Communication & Manual Safety Control

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-keep-simulation-protocol-behavior-compatible-with-real-`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(sim): align simulator with robot protocol semantics`
- `test(sim): add disconnect and watchdog scenarios`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 35 — `[UI] Implement manual robot control and connection panel`

### Purpose

Provide safe operator controls for communication bring-up before autonomous control.

### Required behavior / tasks

- [ ] Display discovered/connected robot state
- [ ] Allow connect/disconnect
- [ ] Provide bounded forward/back/turn or v/omega controls
- [ ] Send explicit stop when manual input is released/disabled as appropriate
- [ ] Show key telemetry such as battery/status
- [ ] Disable invalid controls when disconnected/faulted
- [ ] Make E-stop visually distinct and always accessible when applicable

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Operator can manually move connected robot
- [ ] Commands are bounded
- [ ] UI state follows connection state
- [ ] Stop/E-stop remain accessible
- [ ] Simulation and BLE paths use the same UI service API

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 33

### Priority

High

### Milestone

M3 — Robot Communication & Manual Safety Control

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-implement-manual-robot-control-and-connection-panel`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(ui): add robot connection and telemetry panel`
- `feat(ui): add bounded manual drive controls`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 36 — `[SAFETY] Implement emergency stop command freshness and lost-connection safety rules`

### Purpose

Make communication/control failures fail safe before autonomous movement is introduced.

### Required behavior / tasks

- [ ] Define desktop safety state transitions for connection loss and E-stop
- [ ] Ensure E-stop bypasses normal queued task/control flow
- [ ] Define command freshness timestamps/sequence behavior
- [ ] Verify robot-side watchdog stops motors when commands disappear
- [ ] Reject stale queued commands after reconnect
- [ ] Provide explicit clear/reset behavior for E-stop
- [ ] Add simulation tests and hardware verification checklist

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] E-stop results in motor stop independent of planner state
- [ ] Loss of command stream triggers robot-side safe stop
- [ ] Reconnect does not replay stale motion command
- [ ] Safety state is visible to operator
- [ ] Simulation tests cover safety transitions

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 32
- Catalog Issue 33
- Catalog Issue 34

### Priority

Critical

### Milestone

M3 — Robot Communication & Manual Safety Control

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-implement-emergency-stop-command-freshness-and-lost-con`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(safety): add estop and command freshness model`
- `feat(safety): handle lost connection and stale command recovery`
- `test(safety): cover simulated watchdog transitions`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 37 — `[BLE] Implement BLE reconnect recovery and repeated lifecycle testing`

### Purpose

Make normal radio interruptions recoverable without corrupting robot identity or command state.

### Required behavior / tasks

- [ ] Detect unexpected disconnect
- [ ] Transition RobotManager/UI to safe disconnected state
- [ ] Allow controlled reconnect/rescan
- [ ] Re-establish service/notification subscriptions
- [ ] Reset/reconcile sequence state
- [ ] Require safe command state after reconnect
- [ ] Stress repeated connect/disconnect cycles

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Unexpected disconnect is visible and safe
- [ ] Reconnect restores telemetry
- [ ] No stale motion command is sent automatically
- [ ] Repeated cycles do not leak/crash
- [ ] Robot identity remains correct

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 36

### Priority

Medium

### Milestone

M3 — Robot Communication & Manual Safety Control

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-implement-ble-reconnect-recovery-and-repeated-lifecycle`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `fix(ble): recover safely from unexpected disconnect`
- `test(ble): exercise repeated reconnect lifecycle`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 38 — `[TEST] Benchmark BLE communication latency jitter loss and reconnect time`

### Purpose

Create repeatable communication measurements for control design and thesis reporting.

### Required behavior / tasks

- [ ] Define timestamp/sequence measurement method
- [ ] Measure command-to-robot acknowledgement/telemetry round-trip or best observable latency
- [ ] Measure jitter over repeated samples
- [ ] Detect/report lost or out-of-order messages where measurable
- [ ] Measure reconnect time
- [ ] Record test conditions and payload/update rate
- [ ] Export raw measurements

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Latency distribution is reported
- [ ] Jitter statistic is reported
- [ ] Loss/out-of-order behavior is reported or measurement limitation documented
- [ ] Reconnect time is measured
- [ ] Procedure is repeatable

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 32
- Catalog Issue 37

### Priority

High

### Milestone

M3 — Robot Communication & Manual Safety Control

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `test/<issue-number>-benchmark-ble-communication-latency-jitter-loss-and-rec`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `test(comm): add BLE latency and jitter benchmark procedure`
- `docs(thesis): record communication benchmark methodology`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---


# M4 — External-Vision Closed-Loop Robot Control

## Catalog Issue 39 — `[CONTROL] Implement differential-drive kinematic and command conversion model`

### Purpose

Create pure/testable geometry relating robot pose, supervisory velocity and wheel targets/domain commands.

### Required behavior / tasks

- [ ] Define unicycle/differential-drive model
- [ ] Represent v and omega in SI units
- [ ] Define robot geometry parameters needed for conversion
- [ ] Implement bounded conversion helpers where desktop needs them
- [ ] Keep low-level wheel PID outside desktop core
- [ ] Add analytical unit tests including straight and pure rotation cases

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Model equations and units are documented
- [ ] Known v/omega cases produce expected wheel targets if conversion is used
- [ ] Core model has no GUI/BLE dependency
- [ ] Tests cover sign conventions

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 4
- Catalog Issue 30

### Priority

High

### Milestone

M4 — External-Vision Closed-Loop Robot Control

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-implement-differential-drive-kinematic-and-command-conv`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(control): add differential-drive kinematic model`
- `test(control): cover velocity and wheel conversion geometry`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 40 — `[CONTROL] Create supervisory control loop service and scheduler`

### Purpose

Establish a timestamp-aware outer-loop service separate from GUI and transport callbacks.

### Required behavior / tasks

- [ ] Define controller input/output boundary
- [ ] Consume latest valid tracked robot state and target state
- [ ] Use configured control period without blocking GUI
- [ ] Reject/stop on stale tracking or lost connection
- [ ] Route bounded commands through RobotManager
- [ ] Record control-cycle timestamps and error terms
- [ ] Support simulation for deterministic testing

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Control loop can run in simulation without GUI/hardware
- [ ] GUI remains responsive
- [ ] No command is generated from stale/lost state
- [ ] Loop timing and output are observable
- [ ] Controller implementation can be replaced without rewriting RobotManager

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 27
- Catalog Issue 33
- Catalog Issue 36
- Catalog Issue 39

### Priority

High

### Milestone

M4 — External-Vision Closed-Loop Robot Control

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-create-supervisory-control-loop-service-and-scheduler`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(control): add supervisory control-loop service`
- `test(control): cover stale-state gating and simulated scheduling`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 41 — `[CONTROL] Implement baseline goal-point controller`

### Purpose

Drive a differential robot toward a desired world-space point/pose using external pose feedback.

### Required behavior / tasks

- [ ] Define position/heading error in world/robot frame
- [ ] Implement stable baseline v/omega control law
- [ ] Normalize heading error
- [ ] Add configurable gains speed/angular limits and arrival tolerances
- [ ] Stop or settle cleanly at goal
- [ ] Handle target behind robot according to documented policy
- [ ] Add pure logic tests/simulation scenarios

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Controller converges in representative simulated starts
- [ ] Outputs respect limits
- [ ] Arrival condition is deterministic
- [ ] Heading wrap does not create large command jump
- [ ] Controller is independent of BLE and GUI

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 40

### Priority

High

### Milestone

M4 — External-Vision Closed-Loop Robot Control

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-implement-baseline-goal-point-controller`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(control): implement goal-point feedback controller`
- `test(control): cover convergence and angle edge cases`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 42 — `[CONTROL] Integrate live visual pose feedback with robot commands`

### Purpose

Close the real desktop camera-to-command loop for one robot.

### Required behavior / tasks

- [ ] Select tracked robot and desired target
- [ ] Feed latest filtered metric pose into control loop
- [ ] Send command through RobotManager/BLE
- [ ] Display actual pose target and control state
- [ ] Stop on stale/lost tracking or disconnect
- [ ] Log camera timestamp control timestamp sequence and command

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] One physical robot can move toward target using overhead visual feedback
- [ ] Tracking loss stops/pauses motion safely
- [ ] Communication loss stops/pauses motion safely
- [ ] No control path reads raw image pixels
- [ ] Loop timing is logged

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 41
- Catalog Issue 24
- Catalog Issue 32
- Catalog Issue 36

### Priority

High

### Milestone

M4 — External-Vision Closed-Loop Robot Control

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-integrate-live-visual-pose-feedback-with-robot-commands`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(control): connect visual tracker to live supervisory commands`
- `feat(ui): display target and controller state`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 43 — `[SAFETY] Add control saturation freshness and operational state machine`

### Purpose

Harden autonomous control against stale measurements excessive commands and unsafe state transitions.

### Required behavior / tasks

- [ ] Define READY/RUNNING/PAUSED/LOST_TRACKING/LOST_CONNECTION/ESTOP/FAULT semantics
- [ ] Gate command generation by operational state
- [ ] Apply linear/angular command saturation
- [ ] Define tracking-age threshold for control
- [ ] Define safe resume requirements
- [ ] Prevent automatic restart after E-stop without explicit clear
- [ ] Add state-machine tests

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Unsafe state cannot emit nonzero autonomous motion
- [ ] Stale tracking transitions out of running state
- [ ] All outputs are saturated
- [ ] Resume behavior is explicit
- [ ] Tests cover main transitions

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 42

### Priority

Critical

### Milestone

M4 — External-Vision Closed-Loop Robot Control

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-add-control-saturation-freshness-and-operational-state-`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(safety): add autonomous control operational state machine`
- `test(safety): cover control gating and resume behavior`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 44 — `[UI] Add calibrated target-point selection and control panel`

### Purpose

Let the operator select/enter a world-space target while visualizing control status.

### Required behavior / tasks

- [ ] Support click-to-target on calibrated view
- [ ] Convert display click to image/world coordinates correctly
- [ ] Allow numeric target entry if useful
- [ ] Display selected robot target error and operational state
- [ ] Provide start pause cancel and E-stop controls
- [ ] Prevent target command when workspace/calibration/robot state is invalid

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Clicked target corresponds to expected metric location
- [ ] UI resizing does not alter target coordinates
- [ ] Invalid/un-calibrated states disable autonomous start
- [ ] Start/pause/cancel map cleanly to control service

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 42
- Catalog Issue 43

### Priority

High

### Milestone

M4 — External-Vision Closed-Loop Robot Control

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-add-calibrated-target-point-selection-and-control-panel`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(ui): add metric target selection and controller controls`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 45 — `[MEASURE] Add controller experiment logging and export`

### Purpose

Record synchronized state/command data for tuning and thesis evaluation.

### Required behavior / tasks

- [ ] Define structured sample schema with timestamps robot pose target errors command and tracking age
- [ ] Log at control-cycle or well-defined sample rate
- [ ] Include configuration/gain metadata
- [ ] Export CSV/JSON or other analysis-friendly format
- [ ] Avoid blocking control loop on disk I/O
- [ ] Document file format

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] A complete run can be reconstructed from export
- [ ] Timestamps permit latency/error analysis
- [ ] Gain/config metadata is saved
- [ ] Logging does not stall control loop

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 42

### Priority

Medium

### Milestone

M4 — External-Vision Closed-Loop Robot Control

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-add-controller-experiment-logging-and-export`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(measure): add structured control-run logging`
- `docs(measure): document control experiment export format`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 46 — `[TEST] Benchmark closed-loop point regulation`

### Purpose

Quantitatively evaluate the external-vision controller from varied initial conditions.

### Required behavior / tasks

- [ ] Define test targets and initial poses
- [ ] Run repeated trials
- [ ] Calculate final position error and optional heading error
- [ ] Calculate completion/settling time
- [ ] Record overshoot/path length where meaningful
- [ ] Correlate failures with tracking/communication events
- [ ] Preserve raw run logs and summary tables

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Multiple repeat trials are reported
- [ ] Mean/RMSE/max endpoint error are reported
- [ ] Completion-time statistics are reported
- [ ] Safety failures/aborted runs are counted explicitly
- [ ] Procedure is reproducible

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 43
- Catalog Issue 45

### Priority

High

### Milestone

M4 — External-Vision Closed-Loop Robot Control

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `test/<issue-number>-benchmark-closed-loop-point-regulation`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `test(control): define repeatable point-regulation benchmark`
- `docs(thesis): record closed-loop control methodology`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---


# M5 — Operator Path Drawing, Trajectory Generation & Tracking

## Catalog Issue 47 — `[PATH] Implement world-space mouse path editor`

### Purpose

Allow the operator to draw and edit a path over the camera view while storing geometry only in calibrated world coordinates.

### Required behavior / tasks

- [ ] Add path-drawing interaction for selected robot
- [ ] Map widget/display coordinates to source image then world coordinates
- [ ] Store path points in metres
- [ ] Handle view resizing/letterboxing correctly
- [ ] Support clear undo/finish/cancel
- [ ] Reject points outside calibrated workspace
- [ ] Visualize current raw path

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Drawn path remains unchanged in world coordinates after resize
- [ ] Points correspond to expected tabletop locations
- [ ] Invalid workspace state prevents path creation
- [ ] Path editing is independent of camera display scaling

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 44
- Catalog Issue 23

### Priority

High

### Milestone

M5 — Operator Path Drawing, Trajectory Generation & Tracking

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-implement-world-space-mouse-path-editor`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(path): add calibrated mouse path editor`
- `test(path): cover display-to-world coordinate mapping`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 48 — `[PATH] Define path trajectory and execution domain models`

### Purpose

Separate geometric path data from time-parameterized trajectory and runtime execution state.

### Required behavior / tasks

- [ ] Define Path/PathPoint representation in world units
- [ ] Define Trajectory sample/segment representation
- [ ] Define execution states such as pending/running/paused/completed/cancelled/failed
- [ ] Associate paths/tasks with RobotId
- [ ] Define serialization/debug representation if useful
- [ ] Add invariants/tests

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Path is independent of GUI pixels
- [ ] Trajectory is distinguishable from path geometry
- [ ] Execution state is explicit
- [ ] Models can be used by simulation and real control

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 4
- Catalog Issue 47

### Priority

High

### Milestone

M5 — Operator Path Drawing, Trajectory Generation & Tracking

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-define-path-trajectory-and-execution-domain-models`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(path): add path trajectory and execution domain models`
- `test(path): cover model invariants`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 49 — `[PATH] Implement path simplification and smoothing`

### Purpose

Convert noisy high-density mouse samples into a usable smooth geometric route.

### Required behavior / tasks

- [ ] Implement configurable Ramer-Douglas-Peucker or equivalent simplification
- [ ] Implement spline/curve or corner smoothing method
- [ ] Preserve path endpoints
- [ ] Respect workspace bounds
- [ ] Expose raw simplified and smoothed versions for debugging
- [ ] Add deterministic geometric tests

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Simplification reduces redundant points
- [ ] Endpoints are preserved
- [ ] Smoothing produces continuous usable geometry
- [ ] Algorithm is deterministic for fixed input
- [ ] Invalid/degenerate paths are handled explicitly

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 48

### Priority

High

### Milestone

M5 — Operator Path Drawing, Trajectory Generation & Tracking

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-implement-path-simplification-and-smoothing`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(path): implement geometric path simplification`
- `feat(path): add configurable path smoothing`
- `test(path): cover representative drawn shapes`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 50 — `[TRAJ] Generate bounded trajectory/reference from smoothed path`

### Purpose

Create a motion reference that respects configured speed and sampling constraints.

### Required behavior / tasks

- [ ] Parameterize smoothed path by distance
- [ ] Generate reference points/segments at defined spacing/time policy
- [ ] Apply maximum translational speed and any curvature-based limit used
- [ ] Define desired heading/tangent where needed
- [ ] Handle short/degenerate paths
- [ ] Expose trajectory length/duration estimate
- [ ] Add tests

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Generated reference follows path geometry
- [ ] Configured speed limits are respected
- [ ] Trajectory starts/ends correctly
- [ ] Degenerate paths fail safely
- [ ] Length/duration are deterministic

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 49

### Priority

High

### Milestone

M5 — Operator Path Drawing, Trajectory Generation & Tracking

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-generate-bounded-trajectory-reference-from-smoothed-pat`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(trajectory): generate bounded references from smoothed paths`
- `test(trajectory): cover speed limits and endpoints`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 51 — `[CONTROL] Implement Pure Pursuit baseline path follower`

### Purpose

Track a geometric path/trajectory using external pose feedback with configurable lookahead.

### Required behavior / tasks

- [ ] Implement nearest/progress tracking and lookahead selection
- [ ] Compute curvature/angular command using documented Pure Pursuit geometry
- [ ] Use configurable lookahead and speed policy
- [ ] Handle path completion
- [ ] Prevent progress from jumping incorrectly on self-near paths as practical
- [ ] Integrate saturation/safety state from M4
- [ ] Add simulation tests

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Controller follows representative straight/curved simulated paths
- [ ] Outputs remain bounded
- [ ] Completion occurs near final point
- [ ] Lost tracking/connection still stops motion
- [ ] Lookahead parameter is configurable

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 50
- Catalog Issue 43

### Priority

High

### Milestone

M5 — Operator Path Drawing, Trajectory Generation & Tracking

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-implement-pure-pursuit-baseline-path-follower`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(control): implement Pure Pursuit path follower`
- `test(control): cover lookahead and representative paths`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 52 — `[UI] Visualize desired path trajectory progress and actual robot trace`

### Purpose

Provide operator/debug visibility into what was planned and what the robot actually executed.

### Required behavior / tasks

- [ ] Draw raw/smoothed/active path selectively
- [ ] Draw current lookahead/target point when useful
- [ ] Draw actual robot trace in world coordinates
- [ ] Show execution state and progress
- [ ] Keep overlays aligned with camera/world view resize
- [ ] Allow clearing/exporting trace

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Desired and actual routes are visually distinguishable
- [ ] Overlay remains calibrated after resize
- [ ] Execution state/progress updates live
- [ ] Trace can support later error analysis

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 51
- Catalog Issue 47

### Priority

Medium

### Milestone

M5 — Operator Path Drawing, Trajectory Generation & Tracking

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-visualize-desired-path-trajectory-progress-and-actual-r`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(ui): visualize planned path and actual robot trace`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 53 — `[CONTROL] Implement path execution pause cancel resume and safe completion`

### Purpose

Define runtime behavior around operator interruptions and controller state changes.

### Required behavior / tasks

- [ ] Implement start/pause/cancel actions through control service
- [ ] Ensure pause/cancel sends safe zero command
- [ ] Define whether resume continues from current pose/progress or requires re-evaluation
- [ ] Handle tracking/connection interruptions
- [ ] Mark completion/failure states explicitly
- [ ] Prevent stale queued commands on restart
- [ ] Add state-machine tests

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Pause/cancel stop motion safely
- [ ] Resume behavior is documented and deterministic
- [ ] Loss/recovery cannot silently continue unsafe stale trajectory
- [ ] Completed path ends with safe stop
- [ ] Tests cover lifecycle

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 51
- Catalog Issue 43

### Priority

High

### Milestone

M5 — Operator Path Drawing, Trajectory Generation & Tracking

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-implement-path-execution-pause-cancel-resume-and-safe-c`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(control): add path execution lifecycle`
- `test(control): cover pause cancel resume and completion`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 54 — `[TEST] Benchmark path-following accuracy on representative shapes`

### Purpose

Evaluate the mouse-path to trajectory to visual-control pipeline quantitatively.

### Required behavior / tasks

- [ ] Define straight curve S-turn and mixed path set
- [ ] Run repeated trials
- [ ] Calculate cross-track error over time
- [ ] Report RMSE mean/max cross-track error
- [ ] Report completion time and path-length ratio
- [ ] Record lookahead/speed/gains and camera setup
- [ ] Preserve raw traces

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] At least several representative shapes are tested
- [ ] Cross-track RMSE/max are reported
- [ ] Completion-time statistics are reported
- [ ] Parameter set is documented
- [ ] Raw desired/actual trajectories are preserved

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 52
- Catalog Issue 53
- Catalog Issue 45

### Priority

High

### Milestone

M5 — Operator Path Drawing, Trajectory Generation & Tracking

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `test/<issue-number>-benchmark-path-following-accuracy-on-representative-sha`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `test(path): define repeatable path-following benchmark`
- `docs(thesis): record trajectory tracking methodology`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---


# M6 — Obstacles & Autonomous Single-Robot Path Planning

## Catalog Issue 55 — `[WORLD] Add obstacle model and manual world-space obstacle editor`

### Purpose

Introduce obstacle geometry independently of automatic detection so planning can be developed deterministically.

### Required behavior / tasks

- [ ] Define obstacle/polygon representation in world coordinates
- [ ] Add WorldModel obstacle storage/update API
- [ ] Add UI for rectangle/polygon creation editing removal
- [ ] Validate obstacles within workspace
- [ ] Persist/load obstacle map if useful
- [ ] Render obstacles in calibrated view
- [ ] Add geometry tests

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Obstacles are stored in metric world coordinates
- [ ] Manual editing survives view scaling
- [ ] Planner-facing model does not depend on GUI objects
- [ ] Invalid polygons are rejected or normalized

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 7
- Catalog Issue 47

### Priority

High

### Milestone

M6 — Obstacles & Autonomous Single-Robot Path Planning

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-add-obstacle-model-and-manual-world-space-obstacle-edit`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(world): add metric obstacle model`
- `feat(ui): add manual obstacle editor`
- `test(world): cover obstacle validation`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 56 — `[PLANNER] Implement occupancy-grid representation from workspace obstacles`

### Purpose

Rasterize metric obstacle geometry into a configurable planning grid.

### Required behavior / tasks

- [ ] Define grid origin resolution width/height relative to WorldModel
- [ ] Rasterize workspace bounds and obstacles
- [ ] Provide world-to-grid and grid-to-world conversion
- [ ] Represent free occupied and optionally unknown cells
- [ ] Reject out-of-bounds conversion safely
- [ ] Visualize grid for debugging
- [ ] Add deterministic tests

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Known world points map to correct cells
- [ ] Obstacle polygons occupy expected cells
- [ ] Round-trip conversions are within grid resolution
- [ ] Grid has no dependency on GUI drawing implementation

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 55

### Priority

High

### Milestone

M6 — Obstacles & Autonomous Single-Robot Path Planning

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-implement-occupancy-grid-representation-from-workspace-`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(planner): add metric occupancy grid`
- `test(planner): cover grid conversion and rasterization`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 57 — `[PLANNER] Implement robot-footprint obstacle inflation and clearance model`

### Purpose

Convert finite robot geometry and safety margin into point-planning configuration space.

### Required behavior / tasks

- [ ] Define configurable robot radius/footprint approximation and safety margin
- [ ] Inflate occupied cells/geometry consistently
- [ ] Handle workspace boundary clearance
- [ ] Expose original and inflated grids for diagnostics
- [ ] Add tests around narrow passages/corners

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Planned robot-center path maintains configured clearance
- [ ] Workspace edges respect footprint
- [ ] Inflation amount matches configuration within grid resolution
- [ ] Narrow passages close/open predictably as radius changes

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 56
- Catalog Issue 39

### Priority

High

### Milestone

M6 — Obstacles & Autonomous Single-Robot Path Planning

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-implement-robot-footprint-obstacle-inflation-and-cleara`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(planner): inflate obstacles for robot footprint`
- `test(planner): cover clearance and narrow passages`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 58 — `[PLANNER] Implement A* planner behind IPathPlanner`

### Purpose

Create the baseline deterministic single-robot planner on the inflated occupancy grid.

### Required behavior / tasks

- [ ] Define/implement IPathPlanner if not already active
- [ ] Implement A* with documented 4/8-connected motion model
- [ ] Use admissible/consistent heuristic for chosen cost
- [ ] Reject occupied/out-of-bounds start and goal
- [ ] Return explicit success/failure reason
- [ ] Track planning time and expanded nodes
- [ ] Add known-map unit tests

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Planner finds expected path on deterministic maps
- [ ] Planner reports no-path cleanly
- [ ] Start/goal validation works
- [ ] Path stays in free inflated cells
- [ ] Tests cover reachable unreachable and boundary cases

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 57

### Priority

High

### Milestone

M6 — Obstacles & Autonomous Single-Robot Path Planning

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-implement-a-planner-behind-ipathplanner`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(planner): implement A* occupancy-grid planner`
- `test(planner): cover deterministic maps and failure cases`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 59 — `[PLANNER] Post-process A* path into executable smooth route`

### Purpose

Convert grid-cell path into shorter/smoother world-space route compatible with M5 tracking.

### Required behavior / tasks

- [ ] Convert cell sequence to metric path
- [ ] Remove unnecessary collinear/line-of-sight waypoints without crossing inflated obstacles
- [ ] Apply safe smoothing method
- [ ] Revalidate clearance after smoothing
- [ ] Feed result into existing trajectory generator
- [ ] Add tests with corners/narrow passages

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Post-processed path remains collision-free
- [ ] Waypoint count/path length is reduced where possible
- [ ] Smoothing never cuts through inflated obstacles
- [ ] Output can be executed by existing trajectory follower

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 58
- Catalog Issue 50

### Priority

High

### Milestone

M6 — Obstacles & Autonomous Single-Robot Path Planning

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-post-process-a-path-into-executable-smooth-route`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(planner): simplify and smooth A* paths safely`
- `test(planner): verify post-processing clearance`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 60 — `[VISION] Implement classical tabletop obstacle detection pipeline`

### Purpose

Extract obstacle geometry from overhead imagery without requiring deep learning.

### Required behavior / tasks

- [ ] Define controlled-arena assumptions/background/reference method
- [ ] Implement color/contrast/background segmentation as appropriate
- [ ] Apply morphology/connected-component/contour cleanup
- [ ] Filter table border/robot tags/known robots as needed
- [ ] Convert detected image geometry to world polygons
- [ ] Attach timestamp/confidence/persistence metadata
- [ ] Provide debug masks/overlays

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Known test obstacles are detected in controlled conditions
- [ ] Robot markers are not systematically treated as static obstacles
- [ ] Detected geometry maps to expected world locations
- [ ] Pipeline can be disabled without affecting manual obstacle planning

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 23
- Catalog Issue 55

### Priority

Medium

### Milestone

M6 — Obstacles & Autonomous Single-Robot Path Planning

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-implement-classical-tabletop-obstacle-detection-pipelin`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(vision): detect tabletop obstacle candidates`
- `feat(world): convert obstacle contours to world geometry`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 61 — `[WORLD] Add temporal obstacle persistence and safe occupancy updates`

### Purpose

Prevent single noisy vision frames from causing unstable replanning.

### Required behavior / tasks

- [ ] Track detected obstacle observations over time
- [ ] Define appear/disappear persistence thresholds
- [ ] Merge/update nearby obstacle geometry conservatively
- [ ] Keep manual and detected obstacle provenance distinct
- [ ] Update occupancy grid atomically/safely
- [ ] Trigger replanning request rather than directly controlling robot
- [ ] Add deterministic sequence tests

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Transient noise does not immediately become persistent obstacle
- [ ] Removed obstacle eventually clears according to policy
- [ ] Manual obstacles are not overwritten by detector
- [ ] Planner receives coherent obstacle snapshots

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 60
- Catalog Issue 56

### Priority

Medium

### Milestone

M6 — Obstacles & Autonomous Single-Robot Path Planning

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-add-temporal-obstacle-persistence-and-safe-occupancy-up`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(world): add temporal detected-obstacle tracking`
- `test(world): cover obstacle appearance and disappearance persistence`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 62 — `[TEST] Benchmark autonomous single-robot planning and obstacle navigation`

### Purpose

Evaluate planning correctness performance and execution success with repeatable maps/scenarios.

### Required behavior / tasks

- [ ] Define map/scenario set with varying obstacle density/narrow passages
- [ ] Record planning time expanded nodes path length and post-processed length
- [ ] Execute selected scenarios on simulation and hardware where feasible
- [ ] Measure navigation success/minimum clearance/collisions
- [ ] Evaluate automatic obstacle detection false positives/negatives if implemented
- [ ] Preserve maps configurations and raw results

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Planner metrics are reported across several scenarios
- [ ] Unsafe/failed plans are counted explicitly
- [ ] Execution success and clearance are documented
- [ ] Automatic detection quality is reported separately if used

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 58
- Catalog Issue 59
- Catalog Issue 54

### Priority

High

### Milestone

M6 — Obstacles & Autonomous Single-Robot Path Planning

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `test/<issue-number>-benchmark-autonomous-single-robot-planning-and-obstacle`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `test(planner): define autonomous navigation benchmark suite`
- `docs(thesis): record planning and obstacle methodology`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---


# M7 — Multi-Robot Coordination & Collision Avoidance

## Catalog Issue 63 — `[MULTI] Scale robot registry tracking communication and UI to multiple active robots`

### Purpose

Ensure identities state connections and commands remain isolated before adding coordination logic.

### Required behavior / tasks

- [ ] Audit WorldModel RobotTracker and RobotManager for N robots
- [ ] Support multiple simultaneous robot rows/selection in UI
- [ ] Maintain per-robot connection tracking task and execution state
- [ ] Route commands/telemetry strictly by RobotId
- [ ] Use per-robot structured logging context
- [ ] Add simulation test with several robots

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] At least several simulated robots maintain independent state
- [ ] No command/telemetry cross-talk occurs
- [ ] UI clearly identifies selected/active robot
- [ ] Data structures do not assume a single global robot

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 25
- Catalog Issue 33
- Catalog Issue 53

### Priority

High

### Milestone

M7 — Multi-Robot Coordination & Collision Avoidance

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-scale-robot-registry-tracking-communication-and-ui-to-m`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(multi): support multiple active robot states and sessions`
- `test(multi): verify identity and command isolation`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 64 — `[MULTI] Implement per-robot task path and trajectory ownership`

### Purpose

Represent concurrent assignments without shared mutable path state.

### Required behavior / tasks

- [ ] Define per-robot task/execution record
- [ ] Associate target/path/trajectory/controller state with RobotId
- [ ] Allow independent start pause cancel
- [ ] Prevent a path assigned to one robot from being executed by another
- [ ] Expose task state to UI/logging
- [ ] Add deterministic multi-robot simulation tests

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Concurrent robot tasks remain independent
- [ ] Cancel/pause one robot does not stop unrelated robot unless global safety action
- [ ] Task ownership is explicit
- [ ] Tests cover concurrent execution state

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 63
- Catalog Issue 48

### Priority

High

### Milestone

M7 — Multi-Robot Coordination & Collision Avoidance

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-implement-per-robot-task-path-and-trajectory-ownership`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(multi): add per-robot task and trajectory ownership`
- `test(multi): cover independent execution lifecycle`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 65 — `[COORD] Implement space-time robot collision prediction`

### Purpose

Detect predicted conflicts between active robot trajectories before/while executing coordinated tasks.

### Required behavior / tasks

- [ ] Represent robot footprint and safety separation in conflict checker
- [ ] Sample or analytically compare time-parameterized trajectories
- [ ] Detect vertex/position and crossing/edge-like conflicts
- [ ] Return involved robots conflict time/location/minimum predicted separation
- [ ] Visualize conflicts for debugging
- [ ] Add deterministic crossing/following tests

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Known crossing trajectories produce a conflict
- [ ] Safe parallel trajectories do not produce false conflict under configured separation
- [ ] Conflict identifies robots and time interval/location
- [ ] Checker is independent of UI/transport

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 64
- Catalog Issue 50
- Catalog Issue 57

### Priority

High

### Milestone

M7 — Multi-Robot Coordination & Collision Avoidance

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-implement-space-time-robot-collision-prediction`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(coord): add space-time trajectory conflict detection`
- `test(coord): cover crossing and non-conflicting trajectories`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 66 — `[COORD] Implement reservation and right-of-way baseline`

### Purpose

Resolve simple conflicts by delaying/yielding according to deterministic priority rules.

### Required behavior / tasks

- [ ] Define robot/task priority policy
- [ ] Add space-time reservation representation
- [ ] Delay/re-time lower-priority route when safe
- [ ] Handle dead-end/no-schedule result explicitly
- [ ] Prevent starvation with documented policy if needed
- [ ] Visualize reservations/wait actions
- [ ] Add intersection tests

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Two-robot crossing can be scheduled without collision
- [ ] Priority decision is deterministic
- [ ] No feasible schedule is reported explicitly
- [ ] Wait actions preserve safety separation

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 65

### Priority

High

### Milestone

M7 — Multi-Robot Coordination & Collision Avoidance

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-implement-reservation-and-right-of-way-baseline`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(coord): add priority and space-time reservation model`
- `test(coord): cover two-robot crossing schedules`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 67 — `[COORD] Implement prioritized multi-robot planning baseline`

### Purpose

Plan robots sequentially against existing space-time reservations as the primary scalable coordination baseline.

### Required behavior / tasks

- [ ] Define prioritized planner input/output
- [ ] Plan each robot route with prior reservations as dynamic constraints
- [ ] Support reordering policy/configuration
- [ ] Return per-robot success/failure reason
- [ ] Integrate with existing path/trajectory representation
- [ ] Measure planner time with increasing robot count
- [ ] Add bottleneck/crossing scenarios

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Baseline scenarios produce conflict-free coordinated plans
- [ ] Failure of one robot does not silently invalidate others
- [ ] Planner reports ordering and timing
- [ ] Planning time can be measured

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 66
- Catalog Issue 58

### Priority

High

### Milestone

M7 — Multi-Robot Coordination & Collision Avoidance

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-implement-prioritized-multi-robot-planning-baseline`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(coord): implement prioritized multi-robot planner`
- `test(coord): add crossing and bottleneck scenarios`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 68 — `[COORD] Implement Conflict-Based Search comparison planner`

### Purpose

Add CBS as an optional research comparison after prioritized planning is stable.

> **Optional / research extension:** this issue does not block the minimum thesis system unless explicitly promoted to required scope.

### Required behavior / tasks

- [ ] Define MAPF abstraction compatible with chosen discretization
- [ ] Implement high-level conflict tree and low-level constrained search
- [ ] Reuse tested conflict semantics
- [ ] Add limits/timeouts to protect UI/application
- [ ] Compare solution cost/planning time to prioritized baseline
- [ ] Document assumptions versus continuous robot execution

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] CBS solves small benchmark scenarios correctly
- [ ] Timeout/failure is explicit
- [ ] Results are compared to baseline rather than replacing it blindly
- [ ] Core application can disable CBS

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 67

### Priority

Low

### Milestone

M7 — Multi-Robot Coordination & Collision Avoidance

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-implement-conflict-based-search-comparison-planner`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(coord): add optional CBS planner`
- `test(coord): compare CBS on small MAPF benchmarks`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 69 — `[COORD] Add optional local reciprocal collision-avoidance layer`

### Purpose

Evaluate ORCA-style/local velocity avoidance as an optional complement to global plans.

> **Optional / research extension:** this issue does not block the minimum thesis system unless explicitly promoted to required scope.

### Required behavior / tasks

- [ ] Define desired velocity input and safe velocity output boundary
- [ ] Integrate or implement chosen ORCA-compatible method
- [ ] Respect differential-drive feasibility strategy/document approximation
- [ ] Keep global path planner authoritative for destination
- [ ] Add controlled encounter simulations
- [ ] Measure deviation and minimum separation

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Layer can be enabled/disabled
- [ ] Known encounters maintain configured separation in supported scenarios
- [ ] Global destination intent is preserved
- [ ] Limitations for nonholonomic robots are documented

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 65
- Catalog Issue 67

### Priority

Low

### Milestone

M7 — Multi-Robot Coordination & Collision Avoidance

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-add-optional-local-reciprocal-collision-avoidance-layer`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(coord): add optional local reciprocal avoidance`
- `test(coord): evaluate encounter scenarios`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 70 — `[UI] Add multi-robot coordination visualization and controls`

### Purpose

Expose assignments conflicts priorities reservations and execution state without cluttering core logic.

### Required behavior / tasks

- [ ] Display each robot with unique stable visual identity
- [ ] Show assigned path/goal and execution state
- [ ] Highlight predicted conflicts
- [ ] Display priority/wait/reservation information when enabled
- [ ] Allow global pause/E-stop plus per-robot task controls
- [ ] Avoid encoding coordination logic in UI widgets

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Operator can distinguish robots/tasks/conflicts
- [ ] Global safety action remains obvious
- [ ] UI observes coordination services rather than recalculating conflicts
- [ ] Display remains usable for target swarm size

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 67
- Catalog Issue 63

### Priority

Medium

### Milestone

M7 — Multi-Robot Coordination & Collision Avoidance

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-add-multi-robot-coordination-visualization-and-controls`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(ui): visualize multi-robot tasks and conflicts`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 71 — `[TEST] Benchmark multi-robot coordination and scaling`

### Purpose

Quantify safety efficiency and planner scaling on repeatable scenarios.

### Required behavior / tasks

- [ ] Define crossing bottleneck swap and shared-goal-area scenarios
- [ ] Run increasing robot counts supported by simulation/hardware
- [ ] Measure success rate collision count/minimum separation
- [ ] Measure makespan/task completion time and path length
- [ ] Measure planning/replanning time
- [ ] Compare independent planning versus prioritized baseline
- [ ] Compare optional CBS/ORCA only if implemented
- [ ] Preserve scenario seeds/maps/results

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] At least baseline coordination is quantitatively evaluated
- [ ] Independent-planning conflicts are demonstrated where appropriate
- [ ] Minimum separation and completion metrics are reported
- [ ] Scaling trend is documented

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 67
- Catalog Issue 70

### Priority

High

### Milestone

M7 — Multi-Robot Coordination & Collision Avoidance

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `test/<issue-number>-benchmark-multi-robot-coordination-and-scaling`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `test(coord): add repeatable multi-robot benchmark suite`
- `docs(thesis): record coordination evaluation methodology`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---


# M8 — Cross-Platform & Advanced Sensor/Communication Extensions

## Catalog Issue 72 — `[WINDOWS] Add Windows build CI and deployment path`

### Purpose

Verify the same core application works on Windows without contaminating core modules with platform-specific logic.

### Required behavior / tasks

- [ ] Add Windows CMake/CI build using supported Qt toolchain
- [ ] Run unit tests on Windows CI
- [ ] Resolve path/permission/platform assumptions behind isolated adapters
- [ ] Verify camera discovery/preview on Windows hardware when available
- [ ] Verify BLE path on Windows hardware when available
- [ ] Document Qt deployment/runtime packaging workflow
- [ ] Create packaging preset/script if useful

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Core source tree builds on macOS and Windows
- [ ] Automated tests run on Windows
- [ ] Platform-specific code is isolated
- [ ] Minimum camera workflow is manually verified or limitation documented
- [ ] Deployment instructions are reproducible

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 14

### Priority

High

### Milestone

M8 — Cross-Platform & Advanced Sensor/Communication Extensions

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-add-windows-build-ci-and-deployment-path`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `ci(windows): add CMake build and test workflow`
- `fix(platform): isolate Windows-specific runtime behavior`
- `docs(windows): document deployment and verification`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 73 — `[ZIGBEE] Implement optional Zigbee transport through coordinator gateway`

### Purpose

Add Zigbee as a replaceable transport without changing planner/controller command semantics.

> **Optional / research extension:** this issue does not block the minimum thesis system unless explicitly promoted to required scope.

### Required behavior / tasks

- [ ] Define desktop-to-coordinator gateway protocol
- [ ] Select coordinator hardware/firmware role
- [ ] Implement ZigbeeTransport or gateway adapter behind IRobotTransport
- [ ] Map domain robot identity/commands/telemetry
- [ ] Support connection/network status and errors
- [ ] Measure latency/reliability against BLE
- [ ] Document network formation/addressing

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Switching BLE to Zigbee does not change planner/controller APIs
- [ ] Several addressed robots can exchange commands/telemetry
- [ ] Gateway failures surface through transport state
- [ ] Latency/reliability are measured

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 30
- Catalog Issue 33

### Priority

Low

### Milestone

M8 — Cross-Platform & Advanced Sensor/Communication Extensions

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-implement-optional-zigbee-transport-through-coordinator`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `docs(zigbee): define coordinator gateway protocol`
- `feat(zigbee): add transport adapter`
- `test(zigbee): benchmark gateway communication`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 74 — `[CAMERA] Generalize camera calibration management for second overhead camera`

### Purpose

Prepare multiple calibrated camera sources with independent intrinsics and world transforms.

> **Optional / research extension:** this issue does not block the minimum thesis system unless explicitly promoted to required scope.

### Required behavior / tasks

- [ ] Allow multiple camera instances/profiles concurrently
- [ ] Associate each with intrinsic profile and resolution
- [ ] Calibrate each camera to common world frame
- [ ] Track per-camera timestamps and health
- [ ] Provide camera-specific diagnostics
- [ ] Do not fuse observations yet

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Two cameras can run with distinct calibration profiles
- [ ] Both express observations in the same world frame
- [ ] Camera identity/calibration cannot be mixed accidentally
- [ ] Single-camera workflow still works

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 21
- Catalog Issue 23

### Priority

Medium

### Milestone

M8 — Cross-Platform & Advanced Sensor/Communication Extensions

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-generalize-camera-calibration-management-for-second-ove`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `refactor(camera): support multiple calibrated camera instances`
- `test(camera): cover camera profile isolation`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 75 — `[FUSION] Fuse duplicate robot observations from multiple overhead cameras`

### Purpose

Combine simultaneous/common-frame measurements while handling occlusion and differing quality.

> **Optional / research extension:** this issue does not block the minimum thesis system unless explicitly promoted to required scope.

### Required behavior / tasks

- [ ] Associate observations by RobotId
- [ ] Define timestamp compatibility window
- [ ] Transform all inputs to common world frame before fusion
- [ ] Define confidence/covariance-weighted fusion policy
- [ ] Handle one-camera occlusion and handover
- [ ] Expose source/contribution diagnostics
- [ ] Test deterministic overlapping trajectories

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Robot pose remains continuous through camera overlap/handover
- [ ] Single valid camera still provides usable state
- [ ] Stale second-camera measurement is not fused as current
- [ ] Fusion improves or at least characterizes accuracy/occlusion resilience

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 74
- Catalog Issue 25

### Priority

Low

### Milestone

M8 — Cross-Platform & Advanced Sensor/Communication Extensions

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-fuse-duplicate-robot-observations-from-multiple-overhea`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(fusion): combine multi-camera robot observations`
- `test(fusion): cover overlap occlusion and handover`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 76 — `[VIDEO] Add per-robot onboard camera stream over Wi-Fi`

### Purpose

Display bandwidth-heavy robot video independently of BLE/Zigbee control transport.

> **Optional / research extension:** this issue does not block the minimum thesis system unless explicitly promoted to required scope.

### Required behavior / tasks

- [ ] Define supported network stream endpoint/protocol
- [ ] Implement network video source/player abstraction
- [ ] Associate stream with RobotId
- [ ] Add optional per-robot video panel
- [ ] Display latency/fps/bandwidth diagnostics
- [ ] Ensure disabling video has no effect on control radio
- [ ] Optionally support recording

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Robot video can be enabled/disabled independently
- [ ] Control transport remains operational if stream fails
- [ ] Stream identity is associated with correct robot
- [ ] No video payload is sent through BLE/Zigbee

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 63

### Priority

Low

### Milestone

M8 — Cross-Platform & Advanced Sensor/Communication Extensions

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-add-per-robot-onboard-camera-stream-over-wi-fi`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(video): add robot Wi-Fi stream source`
- `feat(ui): add per-robot video panel`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 77 — `[LIDAR] Define iPhone ARKit LiDAR companion protocol and depth-source boundary`

### Purpose

Design the future Apple-only depth producer without coupling ARKit into the cross-platform core.

> **Optional / research extension:** this issue does not block the minimum thesis system unless explicitly promoted to required scope.

### Required behavior / tasks

- [ ] Define desktop DepthSensorSource-facing domain data
- [ ] Define network message/frame format for RGB/depth/intrinsics/pose/timestamp
- [ ] Define coordinate-frame metadata and synchronization requirements
- [ ] Create minimal iOS ARKit companion prototype or protocol fixture
- [ ] Implement desktop receiver behind isolated adapter
- [ ] Document calibration/registration procedure

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Cross-platform core does not depend on ARKit headers
- [ ] Depth messages include enough metadata for 3D reconstruction/registration
- [ ] Recorded fixture can be received without live iPhone
- [ ] Protocol is versioned

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 22

### Priority

Low

### Milestone

M8 — Cross-Platform & Advanced Sensor/Communication Extensions

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-define-iphone-arkit-lidar-companion-protocol-and-depth-`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `docs(lidar): define ARKit depth transport protocol`
- `feat(depth): add desktop depth-source receiver`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 78 — `[LIDAR] Convert iPhone depth data into tabletop obstacle geometry`

### Purpose

Use depth/point-cloud information to detect shapes above the calibrated table plane and update WorldModel obstacles.

> **Optional / research extension:** this issue does not block the minimum thesis system unless explicitly promoted to required scope.

### Required behavior / tasks

- [ ] Back-project depth to 3D using provided intrinsics
- [ ] Transform points to common world frame
- [ ] Estimate/use calibrated table plane
- [ ] Remove table plane and filter noise
- [ ] Cluster/extract obstacle footprint polygons
- [ ] Update obstacle observations with timestamps/confidence
- [ ] Compare alignment to overhead-camera obstacle geometry

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Depth-derived obstacle footprint aligns with tabletop coordinates
- [ ] Table surface is not classified as obstacle
- [ ] Noisy/invalid depth is rejected safely
- [ ] Planner can consume resulting obstacles through existing WorldModel path

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 77
- Catalog Issue 55

### Priority

Low

### Milestone

M8 — Cross-Platform & Advanced Sensor/Communication Extensions

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-convert-iphone-depth-data-into-tabletop-obstacle-geomet`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(depth): reconstruct table-registered point cloud`
- `feat(depth): extract obstacle footprints`
- `test(depth): validate recorded depth fixtures`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 79 — `[VISION] Add optional semantic object recognition layer`

### Purpose

Label selected scene objects without making geometric planning depend on recognition availability.

> **Optional / research extension:** this issue does not block the minimum thesis system unless explicitly promoted to required scope.

### Required behavior / tasks

- [ ] Define semantic observation separate from obstacle geometry
- [ ] Select/train/integrate detector only for thesis-relevant classes
- [ ] Map detections to world geometry where useful
- [ ] Associate labels/confidence with obstacles/entities
- [ ] Measure inference latency and accuracy
- [ ] Allow recognition to be disabled

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Planning still works with recognition disabled
- [ ] Recognized labels do not replace safety geometry
- [ ] Latency/accuracy are measured for claimed classes
- [ ] Semantic data has explicit confidence

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 60

### Priority

Low

### Milestone

M8 — Cross-Platform & Advanced Sensor/Communication Extensions

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-add-optional-semantic-object-recognition-layer`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(vision): add optional semantic recognition observations`
- `test(vision): evaluate recognition performance`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

## Catalog Issue 80 — `[TASK] Add optional autonomous multi-robot task allocation`

### Purpose

Assign goals/tasks to robots automatically while reusing existing planning and coordination services.

> **Optional / research extension:** this issue does not block the minimum thesis system unless explicitly promoted to required scope.

### Required behavior / tasks

- [ ] Define task/robot capability and cost model
- [ ] Implement baseline assignment algorithm such as Hungarian for one-shot assignment
- [ ] Integrate assignment with per-robot task model
- [ ] Handle unavailable/failed robot reassignment policy
- [ ] Feed assigned goals into coordinated planner
- [ ] Measure assignment/planning time and task completion efficiency
- [ ] Document assumptions and scope

### Architectural constraints

- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
- Do not implement unrelated future milestone features.

### Acceptance criteria

- [ ] Tasks are assigned deterministically for fixed costs
- [ ] Assignments respect robot availability/capability rules
- [ ] Existing manual assignment remains available
- [ ] Coordination handles generated goals through existing APIs
- [ ] Performance is measured

### Validation expected

- [ ] Build all affected targets.
- [ ] Run relevant automated tests.
- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
- [ ] Record any required manual/hardware checks and their actual result.
- [ ] Review the final diff against every acceptance criterion.

### Dependencies

- Catalog Issue 64
- Catalog Issue 67

### Priority

Low

### Milestone

M8 — Cross-Platform & Advanced Sensor/Communication Extensions

### Branch and commit guidance

Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-add-optional-autonomous-multi-robot-task-allocation`

Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
- `feat(task): add baseline autonomous task assignment`
- `test(task): cover assignment and unavailable-robot cases`

### Pull request completion

Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**

---

