# Micro-Robot Swarm Desktop Control System — Milestone Plan

Project: **Design and implementation of a control system for a micro-robot swarm using external visual feedback**

This file records the original M1–M8 development order. Early Linux foundation
work is added by the [Linux catalog](linux_compatibility_issues.md); current
[AGENTS.md](../../AGENTS.md) platform policy and assigned GitHub issues take
precedence over the original platform timing here. `AGENTS.md` and the Codex skill
remain stage-independent; milestones define what capability is being delivered
and GitHub issues define the implementation contract.

## Milestone policy

Each milestone has:
- a capability goal;
- a bounded scope;
- prerequisite milestones;
- concrete deliverables;
- measurable exit criteria;
- thesis evidence that should be preserved.

An issue may be moved earlier only if its dependencies are satisfied and doing so does not violate architecture boundaries. Optional/research issues may remain open without blocking the minimum thesis system when explicitly marked optional.

---

# M1 — Application Foundation & Camera Infrastructure

## Purpose

Create a stable cross-platform desktop foundation before introducing computer-vision algorithms or physical robot communication. Establish the interfaces, build/test infrastructure, application services, camera abstraction, simulation boundary, and live video workflow that all later milestones depend on.

## Scope

M1 includes:
- C++20 / Qt 6 / CMake application skeleton;
- modular source layout;
- logging and configuration;
- common domain types, units, timestamps and result/error conventions;
- `ICameraSource` and initial frame metadata model;
- `IRobotTransport` abstraction and domain-level command/telemetry boundary;
- `WorldModel` skeleton;
- simulation/fake robot transport;
- GoogleTest/CTest infrastructure;
- camera enumeration, selection, start/stop and preview;
- disconnect/reconnect behavior;
- recorded-video source for deterministic development;
- initial CI/build verification for macOS.

## Explicitly out of scope

Do not add AprilTag detection, intrinsic calibration, BLE, robot motion control, A*, MAPF or LiDAR merely to finish M1.

## Deliverables

- application builds and launches on macOS;
- GUI can select an available camera without knowing its concrete implementation;
- live frames are displayed responsively;
- recorded video can be substituted for a live camera;
- fake robot transport can produce deterministic telemetry through `IRobotTransport`;
- core model/services can be tested without GUI/hardware;
- architecture and build instructions are documented.

## Exit criteria

M1 is complete when:
1. a clean checkout can configure/build from CMake;
2. the application starts, logs startup/shutdown and loads configuration/defaults;
3. at least one camera can be enumerated, selected, started, stopped and reselected;
4. camera removal does not crash the application;
5. recorded video can feed the same consumer path as live video;
6. simulation transport can connect/send/receive through the transport abstraction;
7. unit tests run under CTest;
8. no core service depends directly on a physical camera or BLE API.

## Thesis evidence to preserve

- architecture diagram;
- module/dependency diagram;
- camera source list screenshots;
- measured preview FPS and frame timestamps;
- notes on why replay/simulation were introduced.

---

# M2 — Vision, Calibration & Robot Localization

## Purpose

Turn camera frames into reliable robot state estimates in a common metric tabletop coordinate frame. This milestone creates the complete perception chain from fiducial marker detection through calibration and tracking.

## Prerequisite

M1 complete.

## Scope

M2 includes:
- AprilTag family/size selection and robot-ID mapping;
- fiducial detection and image-space observations;
- marker center and orientation;
- detection overlays and quality/timestamp metadata;
- intrinsic camera calibration wizard using ChArUco/chessboard;
- camera matrix/distortion calculation;
- reprojection error and calibration profile persistence;
- tabletop/world frame definition;
- homography/world calibration;
- pixel-to-metric conversion;
- `Pose2D` `(x,y,theta)` output;
- persistent `RobotTracker` state;
- stale/lost tracking rules;
- heading angle unwrapping;
- exponential smoothing and velocity estimation;
- optional Kalman-filter comparison;
- quantitative localization benchmark.

## Deliverables

- printed robot tags are uniquely detected;
- robot pose is reported in calibrated metric coordinates;
- calibration is performed from the desktop application and persists per camera/resolution;
- tracking explicitly distinguishes fresh/stale/lost observations;
- visualization shows tag, robot ID, heading and metric pose;
- localization error is measured over known physical points.

## Exit criteria

1. multiple unique tags can be detected simultaneously;
2. intrinsic calibration can be completed, saved and reloaded;
3. workspace homography maps known image points to metric positions;
4. each visible robot produces `(RobotId, x, y, theta, timestamp)`;
5. temporary marker loss and angle wraparound are handled safely;
6. stationary tracking is measurably stable after filtering;
7. benchmark reports mean, RMSE and maximum localization error plus orientation error where practical.

## Thesis evidence to preserve

- selected tag family/size and reasoning;
- calibration target/specification;
- reprojection error;
- position/orientation accuracy across workspace;
- raw vs filtered tracking plots;
- detection success versus distance/lighting if time permits.

---

# M3 — Robot Communication & Manual Safety Control

## Purpose

Establish reliable desktop-to-ESP32-C6 communication and safe manual robot operation without yet closing the loop through external vision.

## Prerequisites

M1 complete. M2 is recommended for integrated UI but not required for basic transport bring-up.

## Scope

- domain-level robot command/telemetry protocol;
- BLE GATT service contract and protocol versioning;
- BLE discovery, connect/disconnect and state reporting;
- command writes and telemetry/status notifications;
- `RobotManager` connection ownership;
- sequence numbers/freshness semantics;
- manual velocity/turn commands;
- emergency stop;
- lost-connection handling;
- robot-side command watchdog expectations;
- reconnect/recovery behavior;
- protocol-compatible simulation path;
- communication latency/jitter/reconnect measurement.

## Deliverables

- one ESP32-C6 can be discovered and connected from the desktop;
- desktop sends bounded commands and receives telemetry;
- manual control works with clear connection/safety state;
- E-stop is always available when connected;
- stale/lost commands cause safe stop;
- the same high-level command model works with simulation transport.

## Exit criteria

1. BLE discovery and connection work repeatedly;
2. command/telemetry messages have documented version and semantics;
3. sequence/freshness behavior is tested;
4. disconnect during motion produces a safe robot stop;
5. E-stop is verifiably independent of planner state;
6. communication latency/jitter and reconnect time are logged.

## Thesis evidence to preserve

- protocol table;
- BLE service/characteristic diagram;
- measured command latency and jitter;
- disconnect/watchdog safety demonstration.

---

# M4 — External-Vision Closed-Loop Robot Control

## Purpose

Close the supervisory loop using the external camera. The desktop compares desired and measured robot pose and commands robot linear/angular velocity while the ESP32-C6 continues low-level motor regulation and safety.

## Prerequisites

M2 and M3 complete.

## Scope

- differential-drive/unicycle kinematic model;
- supervisory control update scheduler;
- goal-point controller;
- integration of tracked pose with command generation;
- command saturation and freshness;
- lost-tracking/lost-connection control transitions;
- target selection UI;
- controller telemetry/logging;
- quantitative closed-loop point-regulation experiments.

## Deliverables

- user selects a target point/pose in world coordinates;
- robot autonomously drives toward it using external visual feedback;
- control pauses/stops safely on stale tracking or lost communication;
- command and pose histories can be exported/logged.

## Exit criteria

1. a single robot reaches multiple target locations from varied initial poses;
2. target/pose errors use the defined world frame and radians internally;
3. control loop does not run on stale pose data;
4. commands respect configured limits;
5. experiments report final position error, settling/task time and control update latency.

## Thesis evidence to preserve

- control equations and architecture;
- commanded versus measured pose/velocity plots;
- endpoint error statistics;
- safety transition results.

---

# M5 — Operator Path Drawing, Trajectory Generation & Tracking

## Purpose

Implement the minimum thesis path-planning interaction: the operator draws a route directly on the calibrated camera/world view and the robot follows a smooth trajectory derived from that input.

## Prerequisite

M4 complete.

## Scope

- path editor on the camera/world overlay;
- screen/image/world coordinate conversion;
- persistent world-space path model;
- raw path simplification;
- path smoothing/spline generation;
- trajectory/time parameterization;
- Pure Pursuit or equivalent baseline path follower;
- desired/actual path visualization;
- pause/cancel/resume semantics;
- cross-track and completion metrics.

## Deliverables

- operator selects a robot and draws a path with the mouse;
- path is stored in metric world coordinates;
- path is simplified/smoothed without violating workspace boundaries;
- robot follows the path under visual feedback;
- UI overlays desired and actual motion.

## Exit criteria

1. display resize does not corrupt world-space path coordinates;
2. raw input can be simplified and smoothed deterministically;
3. robot follows several representative shapes;
4. pause/E-stop/lost-tracking behavior remains safe;
5. cross-track RMSE/max error and completion time are reported.

## Thesis evidence to preserve

- raw vs simplified vs smoothed path examples;
- Pure Pursuit/lookahead explanation;
- desired vs measured trajectory plots;
- tracking error statistics.

---

# M6 — Obstacles & Autonomous Single-Robot Path Planning

## Purpose

Add a geometric world representation for obstacles and generate collision-free paths automatically around them for a single robot.

## Prerequisite

M5 complete.

## Scope

- obstacle representation in world coordinates;
- manual obstacle editing first;
- occupancy-grid representation;
- robot footprint/safety-margin obstacle inflation;
- A* planner;
- path simplification/smoothing after planning;
- replanning hooks;
- classical automatic obstacle detection from overhead camera where feasible;
- temporal filtering of detected obstacles;
- planner accuracy/performance experiments.

## Deliverables

- user can define obstacles and see them in the world view;
- planner finds a collision-free path from start to goal;
- robot footprint is respected through configuration-space inflation;
- generated path is executable by M5 tracking;
- optional camera-based obstacle extraction can update the world model.

## Exit criteria

1. planner rejects occupied/invalid goals;
2. known maps produce valid A* paths;
3. paths keep required clearance from inflated obstacles;
4. planner failure is represented explicitly rather than as an empty/unsafe path;
5. benchmark reports planning time, path length and navigation success.

## Thesis evidence to preserve

- occupancy grid examples;
- obstacle inflation illustration;
- A* cost/heuristic description;
- planner-time/path-length measurements;
- automatic-detection false-positive/negative notes if implemented.

---

# M7 — Multi-Robot Coordination & Collision Avoidance

## Purpose

Scale the system from several independently tracked robots to coordinated motion where planned trajectories are checked and adjusted to avoid robot-robot conflicts.

## Prerequisites

M5 complete; M6 strongly recommended.

## Scope

- multi-robot registry/state/task management;
- multiple active transport sessions as supported;
- independent paths/trajectories;
- robot-robot collision prediction in space/time;
- simple right-of-way/reservation model;
- prioritized planning baseline;
- optional CBS implementation/comparison;
- optional ORCA/local reciprocal avoidance;
- coordination visualization;
- repeatable crossing/bottleneck experiments.

## Deliverables

- multiple robots retain independent identity, telemetry, tracking and task state;
- simultaneous commands do not cross between robots;
- predicted trajectory conflicts are surfaced;
- baseline coordination resolves common crossing conflicts;
- advanced algorithms can be compared without changing perception/transport architecture.

## Exit criteria

1. at least two robots execute concurrent tasks without state cross-talk;
2. conflict prediction identifies controlled crossing conflicts;
3. prioritized/reservation planning produces collision-free schedules in baseline scenarios;
4. minimum separation is measured;
5. experiments report success rate, completion time, path length and planner time as robot count increases.

## Thesis evidence to preserve

- MAPF conflict definitions used;
- prioritized-planning behavior;
- crossing/bottleneck scenario plots;
- minimum separation and scalability measurements;
- CBS/ORCA comparison only if implemented.

---

# M8 — Cross-Platform & Advanced Sensor/Communication Extensions

## Purpose

Complete portability and add optional extensions without disturbing the validated core architecture. M8 is deliberately modular: not every issue must be required for the minimum thesis result.

## Prerequisites

Core features from M1–M7 as applicable to each extension.

## Required M8 scope

### Windows support
- Windows CI/build;
- Qt runtime deployment/packaging;
- camera and BLE permission/device verification;
- platform-specific code isolation.

## Optional extension scope

### Zigbee
- USB coordinator/gateway architecture;
- `ZigbeeTransport` using the same domain command model;
- no planner/controller changes.

### Second overhead camera
- per-camera intrinsics and world transforms;
- common timestamps;
- duplicate observation association;
- confidence-aware fusion in the world frame.

### Robot onboard camera
- Wi-Fi streaming path;
- per-robot video panel;
- no use of BLE/Zigbee for video.

### iPhone LiDAR
- separate ARKit companion app/protocol;
- RGB/depth/pose transport over Wi-Fi;
- table-plane removal and obstacle extraction;
- transform into the same desktop world frame.

### Object recognition
- semantic labels as an optional layer above obstacle geometry;
- path planning must remain functional when recognition is disabled.

### Autonomous task allocation
- robot-goal assignment;
- assignment optimization/scheduling;
- integration with existing coordinated planning.

## Exit criteria

Required:
1. the desktop core builds on macOS and Windows from the same source tree;
2. platform-specific behavior remains isolated;
3. at least the minimum camera/control workflow can be exercised on Windows.

For each optional extension that is claimed complete:
- it must use existing abstractions rather than bypass them;
- it must include its own measurable acceptance test;
- disabling it must not break the minimum system.

## Thesis evidence to preserve

- macOS/Windows build matrix;
- deployment notes;
- extension-specific diagrams and measurements;
- clear distinction between required thesis functionality and research/bonus work.

---

# Thesis minimum and recommended stopping points

## Minimum complete thesis system

M1 through M5 plus quantitative evaluation provide the essential system:
- selectable/calibrated camera;
- robot ID/orientation/localization;
- BLE communication;
- external visual closed-loop control;
- mouse-drawn path generation/following;
- measurements and safety behavior.

## Strong extended thesis system

Add M6 and the baseline portions of M7:
- obstacles;
- A*;
- several robots;
- conflict prediction/prioritized coordination.

## Research/bonus system

Advanced M7 algorithms and optional M8 extensions:
- CBS/ORCA;
- Zigbee;
- second camera fusion;
- robot video;
- iPhone LiDAR;
- object recognition;
- autonomous task allocation.
