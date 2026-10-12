# Early Linux Compatibility — GitHub Issues

Repository: `Krzys2002/Mini_Robot`\
Target milestone: **M1 — Application Foundation (Phases 0–1)** (match actual milestone by name)\
Platforms: macOS (primary), Ubuntu Linux (secondary).\

> Issue identifiers LNX-01–LNX-08 are planning IDs, not GitHub issue numbers. Create each issue only once.

## LNX-01 — Establish Cross-Platform Build Configuration
**Priority:** High\
**Labels:** `enhancement`, `build`, `linux`, `cross-platform`\
**Dependencies:** Existing M1 CMake initialization issue.

### Description
Configure CMake to build on macOS and Linux without source changes or hardcoded platform paths.

### Tasks
- [ ] Detect OS using CMake.
- [ ] Support Apple Clang on macOS.
- [ ] Support GCC and Clang on Linux.
- [ ] Set consistent C++ standard.
- [ ] Use `find_package()` for Qt6 and OpenCV.
- [ ] Avoid hardcoded Homebrew paths.
- [ ] Isolate platform-specific CMake configuration.
- [ ] Ignore generated build directories in Git.
- [ ] Document Linux build requirements.

### Acceptance Criteria
- [ ] Configure and build on macOS.
- [ ] Configure and build on Ubuntu with GCC and Clang.
- [ ] Switching platforms requires no manual CMake edits.

## LNX-02 — Add Linux Build Validation to GitHub Actions
**Priority:** High\
**Labels:** `ci`, `linux`, `testing`, `automation`\
**Dependencies:** LNX-01.

### Description
Extend CI to automatically validate Linux builds alongside macOS.

### Tasks
- [ ] Add Ubuntu runner.
- [ ] Install Qt6 and OpenCV development packages.
- [ ] Configure and build with CMake.
- [ ] Run GoogleTest through CTest.
- [ ] Run on pull requests and pushes to main.
- [ ] Make failures visible in workflow status.
- [ ] Add build badges to README.

### Acceptance Criteria
- [ ] macOS and Ubuntu CI jobs run.
- [ ] Linux build and unit tests pass without manual intervention.
- [ ] Required branch protection checks can be enabled (configure separately if permitted).

## LNX-03 — Introduce Platform-Independent System Interfaces
**Priority:** High\
**Labels:** `architecture`, `linux`, `cross-platform`\
**Dependencies:** Existing M1 application architecture issue.

### Description
Keep operating-system-specific code outside the core application, preserving `ICameraSource` and `IRobotTransport` abstractions.

### Tasks
- [ ] Audit OS-specific dependencies.
- [ ] Separate platform implementations from core logic.
- [ ] Use Qt abstractions for filesystem, timers and threading where appropriate.
- [ ] Define platform-neutral device discovery.
- [ ] Keep camera and communication backends replaceable.
- [ ] Document abstraction boundaries.

### Acceptance Criteria
- [ ] Core code has no direct macOS-only API dependencies.
- [ ] Core interfaces compile on macOS and Linux.
- [ ] Backends can be changed without modifying control logic.

## LNX-04 — Implement Cross-Platform Camera Discovery
**Priority:** Medium\
**Labels:** `camera`, `linux`, `cross-platform`\
**Dependencies:** LNX-01, LNX-03, existing ICameraSource issue.

### Description
Prepare common camera discovery and capture, using OpenCV V4L2 on Linux and an available macOS backend. Calibration and marker detection remain in M2.

### Tasks
- [ ] Review OpenCV backend differences.
- [ ] Select cameras through common interface.
- [ ] Implement Linux camera enumeration and V4L2 capture.
- [ ] Handle missing devices and permissions.
- [ ] Support configurable capture resolution.
- [ ] Validate Linux frame acquisition.
- [ ] Document `/dev/video*` permissions.

### Acceptance Criteria
- [ ] USB camera opens on macOS and Linux.
- [ ] Same interface exposes frames on both.
- [ ] Camera failures are handled without crashing.

## LNX-05 — Prepare Cross-Platform Robot Communication
**Priority:** Medium\
**Labels:** `communication`, `linux`, `architecture`\
**Dependencies:** LNX-03, existing IRobotTransport issue.

### Description
Prepare Linux-compatible ESP32-C6 transport architecture, without requiring complete hardware BLE implementation in M1. Zigbee remains a separate transport.

### Tasks
- [ ] Review Qt6 Bluetooth support on Linux.
- [ ] Document BlueZ and BLE requirements.
- [ ] Define platform-neutral discovery.
- [ ] Preserve `IRobotTransport` abstraction.
- [ ] Isolate transport code from robot control.
- [ ] Implement mock transport tests.
- [ ] Document Linux communication dependencies.

### Acceptance Criteria
- [ ] Communication interfaces compile on both platforms.
- [ ] Mock tests pass on both.
- [ ] Core control code does not depend on macOS Bluetooth APIs.

## LNX-06 — Validate Qt6 GUI Compatibility on Linux
**Priority:** Medium\
**Labels:** `gui`, `qt6`, `linux`, `testing`\
**Dependencies:** LNX-01, existing GUI initialization issue.

### Description
Validate initial Qt6 GUI behavior on Linux and macOS.

### Tasks
- [ ] Build and launch GUI on Linux.
- [ ] Test window resize and rendering.
- [ ] Test mouse and keyboard input.
- [ ] Check high-DPI scaling and event handling.
- [ ] Test X11 and Wayland sessions.
- [ ] Document limitations.

### Acceptance Criteria
- [ ] Main GUI starts and responds on Linux.
- [ ] Basic interaction works on both platforms.
- [ ] No separate Linux GUI implementation is required.

## LNX-07 — Create Linux Development Environment Documentation
**Priority:** Low\
**Labels:** `documentation`, `linux`, `development`\
**Dependencies:** LNX-01.

### Description
Write reproducible Ubuntu LTS development setup instructions in `docs/development/linux-setup.md`.

### Tasks
- [ ] Specify supported Ubuntu LTS version.
- [ ] Document GCC, Clang, CMake, Qt6, OpenCV and GoogleTest.
- [ ] Document BlueZ dependencies.
- [ ] Provide configure, build and test commands.
- [ ] Add troubleshooting tips.

### Acceptance Criteria
- [ ] Developer can build on clean Ubuntu using documented steps.
- [ ] Dependencies and test commands are complete.

## LNX-08 — Establish Cross-Platform Regression Testing
**Priority:** Medium\
**Labels:** `testing`, `linux`, `cross-platform`\
**Dependencies:** LNX-02, LNX-03.

### Description
Create hardware-independent regression tests executed on macOS and Linux.

### Tasks
- [ ] Test core library on Linux.
- [ ] Test configuration and filesystem path handling.
- [ ] Test mock camera and robot transport.
- [ ] Test initialization where feasible.
- [ ] Integrate with CTest and GitHub Actions.
- [ ] Document hardware-dependent tests separately.

### Acceptance Criteria
- [ ] Shared unit tests pass on macOS and Linux.
- [ ] CI runs regression tests automatically.
- [ ] No camera or robot hardware is needed for these tests.

## Recommended order
1. LNX-01 — Cross-Platform CMake
2. LNX-02 — Linux GitHub Actions
3. LNX-03 — Platform-Independent Interfaces
4. LNX-07 — Linux Development Documentation
5. LNX-08 — Cross-Platform Regression Tests
6. LNX-06 — Qt6 GUI Compatibility
7. LNX-04 — Camera Discovery
8. LNX-05 — Robot Communication

## Milestone M1 completion addition
> The application must configure, compile, and pass all platform-independent unit tests on macOS and Ubuntu Linux through automated GitHub Actions workflows. The initial Qt6 GUI must launch successfully on both operating systems.

Keep comprehensive packaging, hardware validation and final compatibility checks in M7.
