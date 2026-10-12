# GitHub Import Report

Verified at: 2026-10-10T14:29:26.185470+00:00

Repository: [Krzys2002/Mini_Robot](https://github.com/Krzys2002/Mini_Robot)

Project: [GitHub Project #4](https://github.com/users/Krzys2002/projects/4), owner `Krzys2002`.

Sources: [MILESTONES.md](MILESTONES.md) and [GITHUB_ISSUES_M1_M8.md](GITHUB_ISSUES_M1_M8.md).

The user approved the import plan, existing-issue matches, M1 update, and Critical → Urgent mapping before execution.

## Result

- All eight source roadmap milestones exist: seven created, M1 updated in place.
- All 80 catalog entries are represented: 71 new issues plus nine catalog entries mapped to ten existing issues.
- Catalog issue 10 maps to existing issues #29 and #30, which separately cover camera enumeration and selection.
- All 81 mapped GitHub issues are included in Project #4 exactly once and have their correct milestone.
- All 71 new issue titles and complete Markdown bodies were checked against the source. Task checkboxes, acceptance criteria, dependencies, priorities, and implementation guidance were preserved.
- All new issues have Todo status, the mapped source priority, and no assignees.
- Existing issue titles, descriptions, checklists, open/closed states, assignees, and Project statuses were preserved.
- No duplicate issues were created. Existing Project field definitions and options are unchanged; no view mutations were made.
- Both original Markdown source files remain unchanged.

## Milestones

| Milestone | URL | Action | Due date |
|---|---|---|---|
| M1 — Application Foundation & Camera Infrastructure | [Milestone 2](https://github.com/Krzys2002/Mini_Robot/milestone/2) | Updated existing | 2026-10-12T00:00:00Z |
| M2 — Vision, Calibration & Robot Localization | [Milestone 3](https://github.com/Krzys2002/Mini_Robot/milestone/3) | Created | None |
| M3 — Robot Communication & Manual Safety Control | [Milestone 4](https://github.com/Krzys2002/Mini_Robot/milestone/4) | Created | None |
| M4 — External-Vision Closed-Loop Robot Control | [Milestone 5](https://github.com/Krzys2002/Mini_Robot/milestone/5) | Created | None |
| M5 — Operator Path Drawing, Trajectory Generation & Tracking | [Milestone 6](https://github.com/Krzys2002/Mini_Robot/milestone/6) | Created | None |
| M6 — Obstacles & Autonomous Single-Robot Path Planning | [Milestone 7](https://github.com/Krzys2002/Mini_Robot/milestone/7) | Created | None |
| M7 — Multi-Robot Coordination & Collision Avoidance | [Milestone 8](https://github.com/Krzys2002/Mini_Robot/milestone/8) | Created | None |
| M8 — Cross-Platform & Advanced Sensor/Communication Extensions | [Milestone 9](https://github.com/Krzys2002/Mini_Robot/milestone/9) | Created | None |

M1 retains its pre-existing due date, October 12, 2026, as approved. The source specifies no due dates. M2–M8 were created without due dates. The unrelated robot-body milestone was left unchanged.

## Issues

Catalog identifiers are source identifiers, not GitHub issue numbers. For reused issues, the table shows the actual retained GitHub title; source-title differences are documented below.

| Catalog | GitHub issue title | Number / URL | Assigned milestone | Project status | Priority | Labels | Action / body verification |
|---|---|---|---|---|---|---|---|
| 1 | [APP] Create Qt6/CMake application skeleton | [#17](https://github.com/Krzys2002/Mini_Robot/issues/17) | M1 — Application Foundation & Camera Infrastructure | Done | High | app | Reused; original preserved; source differs |
| 2 | [APP] Implement application logging system | [#23](https://github.com/Krzys2002/Mini_Robot/issues/23) | M1 — Application Foundation & Camera Infrastructure | Todo | Medium | app | Reused; original preserved; source differs |
| 3 | [APP] Implement configuration loading | [#24](https://github.com/Krzys2002/Mini_Robot/issues/24) | M1 — Application Foundation & Camera Infrastructure | Todo | Medium | app | Reused; original preserved; source differs |
| 4 | [CORE] Define RobotId, Pose2D, timestamps and unit types | [#25](https://github.com/Krzys2002/Mini_Robot/issues/25) | M1 — Application Foundation & Camera Infrastructure | Done | High | core | Reused; original preserved; source differs |
| 5 | [CORE] Define ICameraSource interface | [#26](https://github.com/Krzys2002/Mini_Robot/issues/26) | M1 — Application Foundation & Camera Infrastructure | Todo | High | core | Reused; original preserved; source differs |
| 6 | [CORE] Define robot transport and protocol domain boundary | [#44](https://github.com/Krzys2002/Mini_Robot/issues/44) | M1 — Application Foundation & Camera Infrastructure | Todo | High | core | Created; exact source match |
| 7 | [WORLD] Create authoritative WorldModel skeleton | [#45](https://github.com/Krzys2002/Mini_Robot/issues/45) | M1 — Application Foundation & Camera Infrastructure | Todo | High | None | Created; exact source match |
| 8 | [SIM] Implement SimulationTransport / fake robot source | [#27](https://github.com/Krzys2002/Mini_Robot/issues/27) | M1 — Application Foundation & Camera Infrastructure | Todo | High | sim | Reused; original preserved; source differs |
| 9 | [TEST] Configure GoogleTest target | [#28](https://github.com/Krzys2002/Mini_Robot/issues/28) | M1 — Application Foundation & Camera Infrastructure | Todo | High | test | Reused; original preserved; source differs |
| 10 | [CAMERA] Enumerate available cameras | [#29](https://github.com/Krzys2002/Mini_Robot/issues/29) | M1 — Application Foundation & Camera Infrastructure | Todo | High | camera | Reused; original preserved; source differs |
| 10 | [CAMERA] Add camera selection UI | [#30](https://github.com/Krzys2002/Mini_Robot/issues/30) | M1 — Application Foundation & Camera Infrastructure | Todo | High | camera | Reused; original preserved; source differs |
| 11 | [CAMERA] Implement start/stop live stream | [#31](https://github.com/Krzys2002/Mini_Robot/issues/31) | M1 — Application Foundation & Camera Infrastructure | Todo | High | camera | Reused; original preserved; source differs |
| 12 | [CAMERA] Handle camera disconnect reconnect and source failure | [#46](https://github.com/Krzys2002/Mini_Robot/issues/46) | M1 — Application Foundation & Camera Infrastructure | Todo | Medium | camera | Created; exact source match |
| 13 | [CAMERA] Add recorded-video camera source for deterministic replay | [#47](https://github.com/Krzys2002/Mini_Robot/issues/47) | M1 — Application Foundation & Camera Infrastructure | Todo | Medium | camera | Created; exact source match |
| 14 | [CI] Add macOS build and test continuous integration | [#48](https://github.com/Krzys2002/Mini_Robot/issues/48) | M1 — Application Foundation & Camera Infrastructure | Todo | Medium | ci | Created; exact source match |
| 15 | [VISION] Select AprilTag family marker size and robot-ID mapping | [#49](https://github.com/Krzys2002/Mini_Robot/issues/49) | M2 — Vision, Calibration & Robot Localization | Todo | High | vision | Created; exact source match |
| 16 | [VISION] Integrate AprilTag detector into frame pipeline | [#50](https://github.com/Krzys2002/Mini_Robot/issues/50) | M2 — Vision, Calibration & Robot Localization | Todo | High | vision | Created; exact source match |
| 17 | [VISION] Define marker observation robot identity center and image-space heading | [#51](https://github.com/Krzys2002/Mini_Robot/issues/51) | M2 — Vision, Calibration & Robot Localization | Todo | High | vision | Created; exact source match |
| 18 | [VISION] Add vision debug overlay and performance diagnostics | [#52](https://github.com/Krzys2002/Mini_Robot/issues/52) | M2 — Vision, Calibration & Robot Localization | Todo | Medium | vision | Created; exact source match |
| 19 | [CALIB] Implement intrinsic camera calibration wizard and target capture | [#53](https://github.com/Krzys2002/Mini_Robot/issues/53) | M2 — Vision, Calibration & Robot Localization | Todo | High | calibration | Created; exact source match |
| 20 | [CALIB] Calculate camera intrinsics distortion and reprojection error | [#54](https://github.com/Krzys2002/Mini_Robot/issues/54) | M2 — Vision, Calibration & Robot Localization | Todo | High | calibration | Created; exact source match |
| 21 | [CALIB] Persist calibration profiles per camera and resolution | [#55](https://github.com/Krzys2002/Mini_Robot/issues/55) | M2 — Vision, Calibration & Robot Localization | Todo | High | calibration | Created; exact source match |
| 22 | [WORLD] Define tabletop world coordinate frame and workspace calibration model | [#56](https://github.com/Krzys2002/Mini_Robot/issues/56) | M2 — Vision, Calibration & Robot Localization | Todo | High | None | Created; exact source match |
| 23 | [WORLD] Implement camera-to-table homography calibration | [#57](https://github.com/Krzys2002/Mini_Robot/issues/57) | M2 — Vision, Calibration & Robot Localization | Todo | High | None | Created; exact source match |
| 24 | [WORLD] Convert marker observations to metric robot Pose2D | [#58](https://github.com/Krzys2002/Mini_Robot/issues/58) | M2 — Vision, Calibration & Robot Localization | Todo | High | None | Created; exact source match |
| 25 | [TRACKING] Implement per-robot tracker with fresh stale and lost states | [#59](https://github.com/Krzys2002/Mini_Robot/issues/59) | M2 — Vision, Calibration & Robot Localization | Todo | High | tracking | Created; exact source match |
| 26 | [TRACKING] Implement heading unwrapping and exponential pose smoothing | [#60](https://github.com/Krzys2002/Mini_Robot/issues/60) | M2 — Vision, Calibration & Robot Localization | Todo | High | tracking | Created; exact source match |
| 27 | [TRACKING] Estimate velocity angular velocity confidence and observation age | [#61](https://github.com/Krzys2002/Mini_Robot/issues/61) | M2 — Vision, Calibration & Robot Localization | Todo | High | tracking | Created; exact source match |
| 28 | [TRACKING] Add optional Kalman pose filter and comparison harness | [#62](https://github.com/Krzys2002/Mini_Robot/issues/62) | M2 — Vision, Calibration & Robot Localization | Todo | Low | tracking, research | Created; exact source match |
| 29 | [TEST] Measure localization and orientation accuracy across workspace | [#63](https://github.com/Krzys2002/Mini_Robot/issues/63) | M2 — Vision, Calibration & Robot Localization | Todo | High | test | Created; exact source match |
| 30 | [PROTOCOL] Define versioned robot command telemetry and BLE GATT contract | [#64](https://github.com/Krzys2002/Mini_Robot/issues/64) | M3 — Robot Communication & Manual Safety Control | Todo | High | protocol | Created; exact source match |
| 31 | [BLE] Implement BLE device discovery connection and state reporting | [#65](https://github.com/Krzys2002/Mini_Robot/issues/65) | M3 — Robot Communication & Manual Safety Control | Todo | High | ble | Created; exact source match |
| 32 | [BLE] Implement BLE command writes telemetry notifications and protocol validation | [#66](https://github.com/Krzys2002/Mini_Robot/issues/66) | M3 — Robot Communication & Manual Safety Control | Todo | High | ble | Created; exact source match |
| 33 | [COMM] Implement RobotManager connection ownership and per-robot state | [#67](https://github.com/Krzys2002/Mini_Robot/issues/67) | M3 — Robot Communication & Manual Safety Control | Todo | High | None | Created; exact source match |
| 34 | [SIM] Keep simulation protocol behavior compatible with real command model | [#68](https://github.com/Krzys2002/Mini_Robot/issues/68) | M3 — Robot Communication & Manual Safety Control | Todo | Medium | sim | Created; exact source match |
| 35 | [UI] Implement manual robot control and connection panel | [#69](https://github.com/Krzys2002/Mini_Robot/issues/69) | M3 — Robot Communication & Manual Safety Control | Todo | High | None | Created; exact source match |
| 36 | [SAFETY] Implement emergency stop command freshness and lost-connection safety rules | [#70](https://github.com/Krzys2002/Mini_Robot/issues/70) | M3 — Robot Communication & Manual Safety Control | Todo | Urgent | None | Created; exact source match |
| 37 | [BLE] Implement BLE reconnect recovery and repeated lifecycle testing | [#71](https://github.com/Krzys2002/Mini_Robot/issues/71) | M3 — Robot Communication & Manual Safety Control | Todo | Medium | ble | Created; exact source match |
| 38 | [TEST] Benchmark BLE communication latency jitter loss and reconnect time | [#72](https://github.com/Krzys2002/Mini_Robot/issues/72) | M3 — Robot Communication & Manual Safety Control | Todo | High | test | Created; exact source match |
| 39 | [CONTROL] Implement differential-drive kinematic and command conversion model | [#73](https://github.com/Krzys2002/Mini_Robot/issues/73) | M4 — External-Vision Closed-Loop Robot Control | Todo | High | control | Created; exact source match |
| 40 | [CONTROL] Create supervisory control loop service and scheduler | [#74](https://github.com/Krzys2002/Mini_Robot/issues/74) | M4 — External-Vision Closed-Loop Robot Control | Todo | High | control | Created; exact source match |
| 41 | [CONTROL] Implement baseline goal-point controller | [#75](https://github.com/Krzys2002/Mini_Robot/issues/75) | M4 — External-Vision Closed-Loop Robot Control | Todo | High | control | Created; exact source match |
| 42 | [CONTROL] Integrate live visual pose feedback with robot commands | [#76](https://github.com/Krzys2002/Mini_Robot/issues/76) | M4 — External-Vision Closed-Loop Robot Control | Todo | High | control | Created; exact source match |
| 43 | [SAFETY] Add control saturation freshness and operational state machine | [#77](https://github.com/Krzys2002/Mini_Robot/issues/77) | M4 — External-Vision Closed-Loop Robot Control | Todo | Urgent | None | Created; exact source match |
| 44 | [UI] Add calibrated target-point selection and control panel | [#78](https://github.com/Krzys2002/Mini_Robot/issues/78) | M4 — External-Vision Closed-Loop Robot Control | Todo | High | None | Created; exact source match |
| 45 | [MEASURE] Add controller experiment logging and export | [#79](https://github.com/Krzys2002/Mini_Robot/issues/79) | M4 — External-Vision Closed-Loop Robot Control | Todo | Medium | None | Created; exact source match |
| 46 | [TEST] Benchmark closed-loop point regulation | [#80](https://github.com/Krzys2002/Mini_Robot/issues/80) | M4 — External-Vision Closed-Loop Robot Control | Todo | High | test | Created; exact source match |
| 47 | [PATH] Implement world-space mouse path editor | [#81](https://github.com/Krzys2002/Mini_Robot/issues/81) | M5 — Operator Path Drawing, Trajectory Generation & Tracking | Todo | High | path | Created; exact source match |
| 48 | [PATH] Define path trajectory and execution domain models | [#82](https://github.com/Krzys2002/Mini_Robot/issues/82) | M5 — Operator Path Drawing, Trajectory Generation & Tracking | Todo | High | path | Created; exact source match |
| 49 | [PATH] Implement path simplification and smoothing | [#83](https://github.com/Krzys2002/Mini_Robot/issues/83) | M5 — Operator Path Drawing, Trajectory Generation & Tracking | Todo | High | path | Created; exact source match |
| 50 | [TRAJ] Generate bounded trajectory/reference from smoothed path | [#84](https://github.com/Krzys2002/Mini_Robot/issues/84) | M5 — Operator Path Drawing, Trajectory Generation & Tracking | Todo | High | None | Created; exact source match |
| 51 | [CONTROL] Implement Pure Pursuit baseline path follower | [#85](https://github.com/Krzys2002/Mini_Robot/issues/85) | M5 — Operator Path Drawing, Trajectory Generation & Tracking | Todo | High | control | Created; exact source match |
| 52 | [UI] Visualize desired path trajectory progress and actual robot trace | [#86](https://github.com/Krzys2002/Mini_Robot/issues/86) | M5 — Operator Path Drawing, Trajectory Generation & Tracking | Todo | Medium | None | Created; exact source match |
| 53 | [CONTROL] Implement path execution pause cancel resume and safe completion | [#87](https://github.com/Krzys2002/Mini_Robot/issues/87) | M5 — Operator Path Drawing, Trajectory Generation & Tracking | Todo | High | control | Created; exact source match |
| 54 | [TEST] Benchmark path-following accuracy on representative shapes | [#88](https://github.com/Krzys2002/Mini_Robot/issues/88) | M5 — Operator Path Drawing, Trajectory Generation & Tracking | Todo | High | test | Created; exact source match |
| 55 | [WORLD] Add obstacle model and manual world-space obstacle editor | [#89](https://github.com/Krzys2002/Mini_Robot/issues/89) | M6 — Obstacles & Autonomous Single-Robot Path Planning | Todo | High | None | Created; exact source match |
| 56 | [PLANNER] Implement occupancy-grid representation from workspace obstacles | [#90](https://github.com/Krzys2002/Mini_Robot/issues/90) | M6 — Obstacles & Autonomous Single-Robot Path Planning | Todo | High | planner | Created; exact source match |
| 57 | [PLANNER] Implement robot-footprint obstacle inflation and clearance model | [#91](https://github.com/Krzys2002/Mini_Robot/issues/91) | M6 — Obstacles & Autonomous Single-Robot Path Planning | Todo | High | planner | Created; exact source match |
| 58 | [PLANNER] Implement A* planner behind IPathPlanner | [#92](https://github.com/Krzys2002/Mini_Robot/issues/92) | M6 — Obstacles & Autonomous Single-Robot Path Planning | Todo | High | planner | Created; exact source match |
| 59 | [PLANNER] Post-process A* path into executable smooth route | [#93](https://github.com/Krzys2002/Mini_Robot/issues/93) | M6 — Obstacles & Autonomous Single-Robot Path Planning | Todo | High | planner | Created; exact source match |
| 60 | [VISION] Implement classical tabletop obstacle detection pipeline | [#94](https://github.com/Krzys2002/Mini_Robot/issues/94) | M6 — Obstacles & Autonomous Single-Robot Path Planning | Todo | Medium | vision | Created; exact source match |
| 61 | [WORLD] Add temporal obstacle persistence and safe occupancy updates | [#95](https://github.com/Krzys2002/Mini_Robot/issues/95) | M6 — Obstacles & Autonomous Single-Robot Path Planning | Todo | Medium | None | Created; exact source match |
| 62 | [TEST] Benchmark autonomous single-robot planning and obstacle navigation | [#96](https://github.com/Krzys2002/Mini_Robot/issues/96) | M6 — Obstacles & Autonomous Single-Robot Path Planning | Todo | High | test | Created; exact source match |
| 63 | [MULTI] Scale robot registry tracking communication and UI to multiple active robots | [#97](https://github.com/Krzys2002/Mini_Robot/issues/97) | M7 — Multi-Robot Coordination & Collision Avoidance | Todo | High | swarm | Created; exact source match |
| 64 | [MULTI] Implement per-robot task path and trajectory ownership | [#98](https://github.com/Krzys2002/Mini_Robot/issues/98) | M7 — Multi-Robot Coordination & Collision Avoidance | Todo | High | swarm | Created; exact source match |
| 65 | [COORD] Implement space-time robot collision prediction | [#99](https://github.com/Krzys2002/Mini_Robot/issues/99) | M7 — Multi-Robot Coordination & Collision Avoidance | Todo | High | collision | Created; exact source match |
| 66 | [COORD] Implement reservation and right-of-way baseline | [#100](https://github.com/Krzys2002/Mini_Robot/issues/100) | M7 — Multi-Robot Coordination & Collision Avoidance | Todo | High | collision | Created; exact source match |
| 67 | [COORD] Implement prioritized multi-robot planning baseline | [#101](https://github.com/Krzys2002/Mini_Robot/issues/101) | M7 — Multi-Robot Coordination & Collision Avoidance | Todo | High | collision | Created; exact source match |
| 68 | [COORD] Implement Conflict-Based Search comparison planner | [#102](https://github.com/Krzys2002/Mini_Robot/issues/102) | M7 — Multi-Robot Coordination & Collision Avoidance | Todo | Low | collision, research | Created; exact source match |
| 69 | [COORD] Add optional local reciprocal collision-avoidance layer | [#103](https://github.com/Krzys2002/Mini_Robot/issues/103) | M7 — Multi-Robot Coordination & Collision Avoidance | Todo | Low | collision, research | Created; exact source match |
| 70 | [UI] Add multi-robot coordination visualization and controls | [#104](https://github.com/Krzys2002/Mini_Robot/issues/104) | M7 — Multi-Robot Coordination & Collision Avoidance | Todo | Medium | None | Created; exact source match |
| 71 | [TEST] Benchmark multi-robot coordination and scaling | [#105](https://github.com/Krzys2002/Mini_Robot/issues/105) | M7 — Multi-Robot Coordination & Collision Avoidance | Todo | High | test | Created; exact source match |
| 72 | [WINDOWS] Add Windows build CI and deployment path | [#106](https://github.com/Krzys2002/Mini_Robot/issues/106) | M8 — Cross-Platform & Advanced Sensor/Communication Extensions | Todo | High | None | Created; exact source match |
| 73 | [ZIGBEE] Implement optional Zigbee transport through coordinator gateway | [#107](https://github.com/Krzys2002/Mini_Robot/issues/107) | M8 — Cross-Platform & Advanced Sensor/Communication Extensions | Todo | Low | zigbee, research | Created; exact source match |
| 74 | [CAMERA] Generalize camera calibration management for second overhead camera | [#108](https://github.com/Krzys2002/Mini_Robot/issues/108) | M8 — Cross-Platform & Advanced Sensor/Communication Extensions | Todo | Medium | camera, research | Created; exact source match |
| 75 | [FUSION] Fuse duplicate robot observations from multiple overhead cameras | [#109](https://github.com/Krzys2002/Mini_Robot/issues/109) | M8 — Cross-Platform & Advanced Sensor/Communication Extensions | Todo | Low | research | Created; exact source match |
| 76 | [VIDEO] Add per-robot onboard camera stream over Wi-Fi | [#110](https://github.com/Krzys2002/Mini_Robot/issues/110) | M8 — Cross-Platform & Advanced Sensor/Communication Extensions | Todo | Low | research | Created; exact source match |
| 77 | [LIDAR] Define iPhone ARKit LiDAR companion protocol and depth-source boundary | [#111](https://github.com/Krzys2002/Mini_Robot/issues/111) | M8 — Cross-Platform & Advanced Sensor/Communication Extensions | Todo | Low | research | Created; exact source match |
| 78 | [LIDAR] Convert iPhone depth data into tabletop obstacle geometry | [#112](https://github.com/Krzys2002/Mini_Robot/issues/112) | M8 — Cross-Platform & Advanced Sensor/Communication Extensions | Todo | Low | research | Created; exact source match |
| 79 | [VISION] Add optional semantic object recognition layer | [#113](https://github.com/Krzys2002/Mini_Robot/issues/113) | M8 — Cross-Platform & Advanced Sensor/Communication Extensions | Todo | Low | vision, research | Created; exact source match |
| 80 | [TASK] Add optional autonomous multi-robot task allocation | [#114](https://github.com/Krzys2002/Mini_Robot/issues/114) | M8 — Cross-Platform & Advanced Sensor/Communication Extensions | Todo | Low | research | Created; exact source match |

## Priority and label handling

- Source High, Medium, and Low map directly to existing Project options.
- Source Critical maps to the existing Urgent option for catalog issues 36 and 43.
- Priorities were applied to all mapped issues. Existing #17 changed from Urgent to source High; its Done status and closed state remain unchanged.
- Existing labels were used. Category aliases: CALIB → calibration, MULTI → swarm, COORD → collision. Explicit optional/research issues also receive the existing research label. Existing issue labels were retained.
- No labels were created. Some source categories have no existing matching label: communication, world, safety, trajectory, windows, video, lidar, task, coordination, multi-robot, optional. UI and measurement headings also lack corresponding labels. Where an existing applicable alias or research label was available, it was used; otherwise no substitute was invented.
- No team members, deadlines, iteration dates, or start dates were assigned. Repository milestones provide Project milestone values.

## Errors, skipped creations, and source differences

Nine catalog entries were skipped for creation because ten existing issues already cover their scope. No catalog entry was omitted. Reusing the older issues avoids duplicate work, but their bodies do not exactly match the source; this is the approved preservation exception.

Dependencies remain in their original `Catalog Issue N` form. They were not rewritten into GitHub numbers or converted into new task descriptions. Use this report’s Catalog column to resolve them.

No GitHub API execution errors were recorded.

All 941 import verification checks passed, with the approved existing-body differences documented below.

### Catalog 1 → GitHub #17

- Source title: `[APP] Create Qt6/CMake application skeleton`
- Existing title: `[APP] Create Qt6/CMake application skeleton`
- Title matches source: yes.
- Body differs; existing Markdown and checkbox completion were retained.

<details>
<summary>Complete description diff: existing GitHub body → source body</summary>

```diff
--- existing-issue-17
+++ source-catalog-1
@@ -1,24 +1,60 @@
-### Description
-Create the initial desktop application structure using Qt6 and CMake.
+### Purpose
 
-The goal is to establish a clean, cross-platform application base before integrating cameras, robots, or computer-vision algorithms.
+Establish the clean cross-platform application base and source layout before hardware or vision integration.
 
-The structure should make it possible to add new modules later without tightly coupling them to the GUI.
+### Required behavior / tasks
 
-### Tasks
-- [x] Create root `CMakeLists.txt`
-- [x] Configure Qt6 dependencies
-- [x] Create application executable target
-- [x] Create basic `MainWindow`
-- [x] Create initial source/include directory structure
-- [x] Configure Debug build
-- [x] Configure Release build
-- [x] Verify build on macOS
-- [x] Add basic README build instructions
+- [ ] Create root CMake project using C++20
+- [ ] Configure Qt 6 Core/Gui/Widgets targets required by the initial UI
+- [ ] Create application executable and minimal MainWindow shell
+- [ ] Create modular source/include/test directory structure aligned with AGENTS.md
+- [ ] Add Debug and Release configuration support
+- [ ] Add command-line build instructions to README
+- [ ] Verify a clean macOS configure/build/launch
 
-### Acceptance Criteria
-- [x] Project configures successfully with CMake
-- [x] Project builds without errors
-- [x] Application launches on macOS
-- [x] Main window is displayed
-- [x] Application can be built from a clean checkout
+### Architectural constraints
+
+- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
+- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
+- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
+- Do not implement unrelated future milestone features.
+
+### Acceptance criteria
+
+- [ ] Clean checkout configures with CMake
+- [ ] Application builds without errors on macOS
+- [ ] Application launches and displays the main window
+- [ ] Core source layout does not place application logic in MainWindow
+- [ ] README documents reproducible build commands
+
+### Validation expected
+
+- [ ] Build all affected targets.
+- [ ] Run relevant automated tests.
+- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
+- [ ] Record any required manual/hardware checks and their actual result.
+- [ ] Review the final diff against every acceptance criterion.
+
+### Dependencies
+
+None.
+
+### Priority
+
+High
+
+### Milestone
+
+M1 — Application Foundation & Camera Infrastructure
+
+### Branch and commit guidance
+
+Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-create-qt6-cmake-application-skeleton`
+
+Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
+- `build: create C++20 Qt6 CMake application skeleton`
+- `docs(build): document macOS configure and build workflow`
+
+### Pull request completion
+
+Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**
```

</details>

### Catalog 2 → GitHub #23

- Source title: `[APP] Implement application logging and measurement foundation`
- Existing title: `[APP] Implement application logging system`
- Title matches source: no; retained.
- Body differs; existing Markdown and checkbox completion were retained.

<details>
<summary>Complete description diff: existing GitHub body → source body</summary>

```diff
--- existing-issue-23
+++ source-catalog-2
@@ -1,22 +1,60 @@
-### Description
-Create a shared logging mechanism that can be used by all modules of the application.
+### Purpose
 
-Logging will later be required for camera state, robot communication, tracking, controller output, errors, and thesis measurements.
+Create shared logging usable by all modules and suitable for later thesis measurements.
 
-### Tasks
-- [ ] Create centralized logging interface/module
-- [ ] Support informational messages
-- [ ] Support warnings
-- [ ] Support errors
-- [ ] Include timestamps
-- [ ] Log application startup
-- [ ] Log application shutdown
-- [ ] Make logging accessible from other modules
-- [ ] Decide whether logs are also written to a file
-- [ ] Add file logging if selected
+### Required behavior / tasks
 
-### Acceptance Criteria
-- [ ] Startup and shutdown are visible in logs
-- [ ] Different modules can produce log messages
-- [ ] Log entries contain timestamps
-- [ ] Errors are distinguishable from normal messages
+- [ ] Define centralized logging service/interface
+- [ ] Support debug/info/warning/error severity
+- [ ] Use timestamps suitable for operational logs
+- [ ] Include application startup/shutdown events
+- [ ] Support module/robot context where applicable
+- [ ] Add file logging with configurable destination/rotation policy if practical
+- [ ] Document distinction between human logs and structured experiment measurements
+
+### Architectural constraints
+
+- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
+- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
+- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
+- Do not implement unrelated future milestone features.
+
+### Acceptance criteria
+
+- [ ] Multiple modules can emit timestamped logs
+- [ ] Errors are distinguishable from normal events
+- [ ] Startup/shutdown are recorded
+- [ ] Logging can be used without coupling core logic to GUI widgets
+- [ ] Logging failures do not crash the application
+
+### Validation expected
+
+- [ ] Build all affected targets.
+- [ ] Run relevant automated tests.
+- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
+- [ ] Record any required manual/hardware checks and their actual result.
+- [ ] Review the final diff against every acceptance criterion.
+
+### Dependencies
+
+- Catalog Issue 1
+
+### Priority
+
+Medium
+
+### Milestone
+
+M1 — Application Foundation & Camera Infrastructure
+
+### Branch and commit guidance
+
+Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-implement-application-logging-and-measurement-foundatio`
+
+Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
+- `feat(logging): add shared application logging service`
+- `docs(logging): document logging and measurement conventions`
+
+### Pull request completion
+
+Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**
```

</details>

### Catalog 3 → GitHub #24

- Source title: `[APP] Implement configuration and profile loading`
- Existing title: `[APP] Implement configuration loading`
- Title matches source: no; retained.
- Body differs; existing Markdown and checkbox completion were retained.

<details>
<summary>Complete description diff: existing GitHub body → source body</summary>

```diff
--- existing-issue-24
+++ source-catalog-3
@@ -1,20 +1,61 @@
-### Description
-Create a configuration system that allows application settings to be stored outside the source code.
+### Purpose
 
-This will later be used for camera settings, calibration profiles, communication parameters, and other configurable system values.
+Provide validated external configuration instead of scattering tunable values through source code.
 
-### Tasks
-- [ ] Define configuration module
-- [ ] Choose the initial configuration format
-- [ ] Load configuration during startup
-- [ ] Provide default values
-- [ ] Handle missing configuration file
-- [ ] Handle invalid configuration values
-- [ ] Expose configuration values to application modules
-- [ ] Log configuration loading errors
+### Required behavior / tasks
 
-### Acceptance Criteria
-- [ ] Application starts correctly with a valid configuration
-- [ ] Application starts with sensible defaults if configuration is absent
-- [ ] Invalid configuration produces a useful warning/error
-- [ ] Configuration access is not duplicated throughout the application
+- [ ] Select initial configuration representation
+- [ ] Define application settings service/model
+- [ ] Load settings on startup with sensible defaults
+- [ ] Handle missing and invalid files safely
+- [ ] Expose validated values to modules through explicit interfaces
+- [ ] Define locations for later calibration/workspace profiles
+- [ ] Log configuration warnings without duplicating parsing logic
+
+### Architectural constraints
+
+- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
+- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
+- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
+- Do not implement unrelated future milestone features.
+
+### Acceptance criteria
+
+- [ ] Application starts with valid configuration
+- [ ] Application starts with defaults when optional config is absent
+- [ ] Invalid values produce actionable diagnostics
+- [ ] Configuration access is centralized
+- [ ] No GUI widget becomes the authoritative settings store
+
+### Validation expected
+
+- [ ] Build all affected targets.
+- [ ] Run relevant automated tests.
+- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
+- [ ] Record any required manual/hardware checks and their actual result.
+- [ ] Review the final diff against every acceptance criterion.
+
+### Dependencies
+
+- Catalog Issue 1
+- Catalog Issue 2
+
+### Priority
+
+Medium
+
+### Milestone
+
+M1 — Application Foundation & Camera Infrastructure
+
+### Branch and commit guidance
+
+Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-implement-configuration-and-profile-loading`
+
+Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
+- `feat(config): add validated application settings loader`
+- `test(config): cover defaults and invalid configuration`
+
+### Pull request completion
+
+Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**
```

</details>

### Catalog 4 → GitHub #25

- Source title: `[CORE] Define common domain types, units, time and result conventions`
- Existing title: `[CORE] Define RobotId, Pose2D, timestamps and unit types`
- Title matches source: no; retained.
- Body differs; existing Markdown and checkbox completion were retained.

<details>
<summary>Complete description diff: existing GitHub body → source body</summary>

```diff
--- existing-issue-25
+++ source-catalog-4
@@ -1,23 +1,61 @@
-### Description
-Create common data types shared by vision, tracking, simulation, communication, and control modules.
+### Purpose
 
-The purpose is to avoid every subsystem defining its own representation of robot identity, position, orientation, and time.
+Create shared types so vision, simulation, planning, communication and control use consistent identity, pose, units and timestamps.
 
-### Tasks
-- [x] Define `RobotId`
-- [x] Define `Pose2D`
-- [x] Define representation for `x`
-- [x] Define representation for `y`
-- [x] Define representation for orientation `theta`
-- [x] Define timestamp type
-- [x] Establish distance units
-- [x] Establish angle units
-- [x] Establish time units
-- [x] Document coordinate/unit conventions
-- [x] Add basic unit tests
+### Required behavior / tasks
 
-### Acceptance Criteria
-- [x] Common types compile independently of GUI code
-- [x] The same pose type can be used by simulation and future vision modules
-- [x] Units are documented
-- [x] Timestamp representation is consistent across modules
+- [ ] Define RobotId
+- [ ] Define Pose2D and basic vector/velocity domain representations as needed
+- [ ] Use metres and radians internally
+- [ ] Define monotonic timestamp/time aliases using std::chrono
+- [ ] Define angle normalization helpers
+- [ ] Define explicit error/result convention for core services
+- [ ] Document coordinate/unit conventions
+- [ ] Add focused unit tests
+
+### Architectural constraints
+
+- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
+- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
+- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
+- Do not implement unrelated future milestone features.
+
+### Acceptance criteria
+
+- [ ] Core types compile without GUI dependencies
+- [ ] Pose representation is reusable by simulation and later vision
+- [ ] Units and angle conventions are documented
+- [ ] Timestamp usage is consistent
+- [ ] Angle helper tests cover wrap boundaries
+
+### Validation expected
+
+- [ ] Build all affected targets.
+- [ ] Run relevant automated tests.
+- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
+- [ ] Record any required manual/hardware checks and their actual result.
+- [ ] Review the final diff against every acceptance criterion.
+
+### Dependencies
+
+- Catalog Issue 1
+
+### Priority
+
+High
+
+### Milestone
+
+M1 — Application Foundation & Camera Infrastructure
+
+### Branch and commit guidance
+
+Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-define-common-domain-types-units-time-and-result-conven`
+
+Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
+- `feat(core): define robot pose unit and time domain types`
+- `test(core): cover unit and angle conventions`
+
+### Pull request completion
+
+Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**
```

</details>

### Catalog 5 → GitHub #26

- Source title: `[CORE] Define camera source and frame abstractions`
- Existing title: `[CORE] Define ICameraSource interface`
- Title matches source: no; retained.
- Body differs; existing Markdown and checkbox completion were retained.

<details>
<summary>Complete description diff: existing GitHub body → source body</summary>

```diff
--- existing-issue-26
+++ source-catalog-5
@@ -1,21 +1,60 @@
-### Description
-Create an abstraction representing a source of camera/video frames.
+### Purpose
 
-The application must not depend directly on a specific physical camera implementation. Later implementations may include a macOS camera, USB camera, recorded video, or simulated camera.
+Create the stable camera boundary used by system cameras, recorded video and future network sources.
 
-### Tasks
-- [ ] Create `ICameraSource`
-- [ ] Define start operation
-- [ ] Define stop operation
-- [ ] Define frame delivery mechanism
-- [ ] Define source state
-- [ ] Define error reporting
-- [ ] Define basic frame metadata
-- [ ] Document frame ownership
+### Required behavior / tasks
+
+- [ ] Define ICameraSource lifecycle/state contract
+- [ ] Define timestamped Frame/FrameMetadata representation without embedding application UI
+- [ ] Define source/device metadata and resolution/fps information
+- [ ] Define frame ownership/lifetime expectations
+- [ ] Define asynchronous delivery or bounded pull semantics compatible with Qt workers
+- [ ] Define error/state reporting
 - [ ] Document threading expectations
 
-### Acceptance Criteria
-- [ ] GUI can work through `ICameraSource`
-- [ ] GUI does not need to know the concrete camera implementation
-- [ ] Multiple implementations can use the same interface
-- [ ] Interface does not unnecessarily depend on application UI
+### Architectural constraints
+
+- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
+- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
+- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
+- Do not implement unrelated future milestone features.
+
+### Acceptance criteria
+
+- [ ] GUI/application services can depend on ICameraSource rather than concrete camera APIs
+- [ ] Multiple camera implementations can conform to the same contract
+- [ ] Frame timestamps and metadata are available
+- [ ] Interface does not perform vision processing
+- [ ] Ownership/thread expectations are documented
+
+### Validation expected
+
+- [ ] Build all affected targets.
+- [ ] Run relevant automated tests.
+- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
+- [ ] Record any required manual/hardware checks and their actual result.
+- [ ] Review the final diff against every acceptance criterion.
+
+### Dependencies
+
+- Catalog Issue 4
+
+### Priority
+
+High
+
+### Milestone
+
+M1 — Application Foundation & Camera Infrastructure
+
+### Branch and commit guidance
+
+Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-define-camera-source-and-frame-abstractions`
+
+Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
+- `feat(camera): define source and timestamped frame abstractions`
+- `test(camera): validate frame metadata invariants`
+
+### Pull request completion
+
+Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**
```

</details>

### Catalog 8 → GitHub #27

- Source title: `[SIM] Implement deterministic SimulationTransport and fake robot source`
- Existing title: `[SIM] Implement SimulationTransport / fake robot source`
- Title matches source: no; retained.
- Body differs; existing Markdown and checkbox completion were retained.

<details>
<summary>Complete description diff: existing GitHub body → source body</summary>

```diff
--- existing-issue-27
+++ source-catalog-8
@@ -1,20 +1,60 @@
-### Description
-Create a fake robot transport so the application can be developed without requiring physical ESP32-C6 robots.
+### Purpose
 
-The simulation should provide predictable robot state/telemetry and accept commands through the same interface that real transports will later use.
+Enable UI/application development and tests without physical ESP32-C6 hardware.
 
-### Tasks
-- [ ] Implement `IRobotTransport`
-- [ ] Simulate connect
-- [ ] Simulate disconnect
-- [ ] Accept outgoing commands
-- [ ] Generate basic robot telemetry/state
-- [ ] Allow deterministic test behavior
-- [ ] Log simulated communication
-- [ ] Add simple configuration if useful
+### Required behavior / tasks
 
-### Acceptance Criteria
-- [ ] Application can connect to a simulated robot
-- [ ] Commands can be sent
-- [ ] Fake telemetry is returned
-- [ ] Higher-level code does not need to distinguish simulation from a future physical transport
+- [ ] Implement IRobotTransport in simulation
+- [ ] Simulate connect/disconnect and connection state
+- [ ] Accept domain commands
+- [ ] Generate deterministic telemetry/status
+- [ ] Allow configurable/fixed robot IDs
+- [ ] Add hooks for deterministic fault/loss scenarios used later
+- [ ] Log simulated communication without special-casing higher-level code
+
+### Architectural constraints
+
+- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
+- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
+- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
+- Do not implement unrelated future milestone features.
+
+### Acceptance criteria
+
+- [ ] Application connects to a simulated robot through IRobotTransport
+- [ ] Commands are accepted and telemetry returned
+- [ ] Behavior can be deterministic in tests
+- [ ] Higher-level code does not need to detect simulation type
+
+### Validation expected
+
+- [ ] Build all affected targets.
+- [ ] Run relevant automated tests.
+- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
+- [ ] Record any required manual/hardware checks and their actual result.
+- [ ] Review the final diff against every acceptance criterion.
+
+### Dependencies
+
+- Catalog Issue 6
+- Catalog Issue 2
+
+### Priority
+
+High
+
+### Milestone
+
+M1 — Application Foundation & Camera Infrastructure
+
+### Branch and commit guidance
+
+Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-implement-deterministic-simulationtransport-and-fake-ro`
+
+Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
+- `feat(sim): add deterministic robot transport simulator`
+- `test(sim): cover connect command and telemetry flow`
+
+### Pull request completion
+
+Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**
```

</details>

### Catalog 9 → GitHub #28

- Source title: `[TEST] Configure GoogleTest and CTest infrastructure`
- Existing title: `[TEST] Configure GoogleTest target`
- Title matches source: no; retained.
- Body differs; existing Markdown and checkbox completion were retained.

<details>
<summary>Complete description diff: existing GitHub body → source body</summary>

```diff
--- existing-issue-28
+++ source-catalog-9
@@ -1,19 +1,58 @@
-### Description
-Add a dedicated unit-test target using GoogleTest.
+### Purpose
 
-The test target will be used progressively for common types, `WorldModel`, path logic, calibration helpers, and other non-GUI components.
+Establish repeatable unit/integration testing from the command line.
 
-### Tasks
-- [ ] Add GoogleTest dependency
-- [ ] Create test executable
-- [ ] Integrate tests with CMake
+### Required behavior / tasks
+
+- [ ] Add GoogleTest dependency using repository-approved dependency mechanism
+- [ ] Create test targets separate from application executable
 - [ ] Integrate tests with CTest
-- [ ] Add one example test
-- [ ] Verify tests run locally
-- [ ] Document test command
+- [ ] Add one representative core test and one simulated-service test
+- [ ] Document test commands
+- [ ] Ensure test setup works from a clean build directory
 
-### Acceptance Criteria
+### Architectural constraints
+
+- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
+- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
+- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
+- Do not implement unrelated future milestone features.
+
+### Acceptance criteria
+
 - [ ] Tests build successfully
-- [ ] Tests can be run from command line
-- [ ] At least one passing test exists
-- [ ] Test executable is separate from application executable
+- [ ] ctest runs from command line
+- [ ] At least representative core and simulation tests pass
+- [ ] Tests do not require GUI or physical hardware
+
+### Validation expected
+
+- [ ] Build all affected targets.
+- [ ] Run relevant automated tests.
+- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
+- [ ] Record any required manual/hardware checks and their actual result.
+- [ ] Review the final diff against every acceptance criterion.
+
+### Dependencies
+
+- Catalog Issue 1
+
+### Priority
+
+High
+
+### Milestone
+
+M1 — Application Foundation & Camera Infrastructure
+
+### Branch and commit guidance
+
+Suggested branch after GitHub assigns the issue number: `test/<issue-number>-configure-googletest-and-ctest-infrastructure`
+
+Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
+- `test(build): configure GoogleTest and CTest targets`
+- `docs(test): document local test commands`
+
+### Pull request completion
+
+Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**
```

</details>

### Catalog 10 → GitHub #29

- Source title: `[CAMERA] Enumerate and select available camera devices`
- Existing title: `[CAMERA] Enumerate available cameras`
- Title matches source: no; retained.
- Body differs; existing Markdown and checkbox completion were retained.

<details>
<summary>Complete description diff: existing GitHub body → source body</summary>

```diff
--- existing-issue-29
+++ source-catalog-10
@@ -1,18 +1,60 @@
-### Description
-Implement camera device discovery so the application can list cameras exposed by the operating system.
+### Purpose
 
-This should support the Mac internal camera, USB cameras, and compatible iPhone Continuity Camera devices when macOS exposes them.
+Expose cameras provided by the OS and let the operator choose the active source.
 
-### Tasks
-- [ ] Query available camera devices
-- [ ] Store/display camera name
-- [ ] Store stable device identifier where available
-- [ ] Refresh list when devices change
-- [ ] Handle zero available cameras
-- [ ] Log discovered cameras
+### Required behavior / tasks
 
-### Acceptance Criteria
-- [ ] Mac internal camera appears
-- [ ] Connected USB camera appears
-- [ ] Available cameras can be distinguished
-- [ ] Missing/disconnected cameras do not crash the application
+- [ ] Implement Qt camera-device discovery behind the camera service boundary
+- [ ] Expose human-readable name and stable identifier where available
+- [ ] Add refresh/device-change handling
+- [ ] Add camera selection UI
+- [ ] Handle zero-camera state
+- [ ] Persist preferred device only when safe to do so
+- [ ] Log device discovery and selection
+
+### Architectural constraints
+
+- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
+- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
+- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
+- Do not implement unrelated future milestone features.
+
+### Acceptance criteria
+
+- [ ] Internal/USB cameras exposed by macOS appear when available
+- [ ] Compatible iPhone Continuity Camera appears when macOS exposes it
+- [ ] User can select among devices
+- [ ] Device removal does not leave an invalid selected object that crashes the app
+
+### Validation expected
+
+- [ ] Build all affected targets.
+- [ ] Run relevant automated tests.
+- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
+- [ ] Record any required manual/hardware checks and their actual result.
+- [ ] Review the final diff against every acceptance criterion.
+
+### Dependencies
+
+- Catalog Issue 1
+- Catalog Issue 5
+
+### Priority
+
+High
+
+### Milestone
+
+M1 — Application Foundation & Camera Infrastructure
+
+### Branch and commit guidance
+
+Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-enumerate-and-select-available-camera-devices`
+
+Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
+- `feat(camera): enumerate system video devices`
+- `feat(ui): add active camera selection control`
+
+### Pull request completion
+
+Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**
```

</details>

### Catalog 10 → GitHub #30

- Source title: `[CAMERA] Enumerate and select available camera devices`
- Existing title: `[CAMERA] Add camera selection UI`
- Title matches source: no; retained.
- Body differs; existing Markdown and checkbox completion were retained.

<details>
<summary>Complete description diff: existing GitHub body → source body</summary>

```diff
--- existing-issue-30
+++ source-catalog-10
@@ -1,15 +1,60 @@
-### Description
-Add UI controls that allow the user to choose one of the discovered camera devices.
+### Purpose
 
-### Tasks
-- [ ] Add camera list/dropdown
-- [ ] Populate it from available camera devices
-- [ ] Allow selecting a camera
-- [ ] Display current selection
-- [ ] Disable invalid/unavailable selections
-- [ ] Connect selection to camera-source creation/configuration
+Expose cameras provided by the OS and let the operator choose the active source.
 
-### Acceptance Criteria
-- [ ] User can select an available camera
-- [ ] Changing selection changes the active source
-- [ ] UI handles device removal gracefully
+### Required behavior / tasks
+
+- [ ] Implement Qt camera-device discovery behind the camera service boundary
+- [ ] Expose human-readable name and stable identifier where available
+- [ ] Add refresh/device-change handling
+- [ ] Add camera selection UI
+- [ ] Handle zero-camera state
+- [ ] Persist preferred device only when safe to do so
+- [ ] Log device discovery and selection
+
+### Architectural constraints
+
+- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
+- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
+- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
+- Do not implement unrelated future milestone features.
+
+### Acceptance criteria
+
+- [ ] Internal/USB cameras exposed by macOS appear when available
+- [ ] Compatible iPhone Continuity Camera appears when macOS exposes it
+- [ ] User can select among devices
+- [ ] Device removal does not leave an invalid selected object that crashes the app
+
+### Validation expected
+
+- [ ] Build all affected targets.
+- [ ] Run relevant automated tests.
+- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
+- [ ] Record any required manual/hardware checks and their actual result.
+- [ ] Review the final diff against every acceptance criterion.
+
+### Dependencies
+
+- Catalog Issue 1
+- Catalog Issue 5
+
+### Priority
+
+High
+
+### Milestone
+
+M1 — Application Foundation & Camera Infrastructure
+
+### Branch and commit guidance
+
+Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-enumerate-and-select-available-camera-devices`
+
+Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
+- `feat(camera): enumerate system video devices`
+- `feat(ui): add active camera selection control`
+
+### Pull request completion
+
+Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**
```

</details>

### Catalog 11 → GitHub #31

- Source title: `[CAMERA] Implement live camera start/stop and responsive preview`
- Existing title: `[CAMERA] Implement start/stop live stream`
- Title matches source: no; retained.
- Body differs; existing Markdown and checkbox completion were retained.

<details>
<summary>Complete description diff: existing GitHub body → source body</summary>

```diff
--- existing-issue-31
+++ source-catalog-11
@@ -1,19 +1,62 @@
-### Description
-Implement explicit control of camera streaming.
+### Purpose
 
-The user should be able to start and stop capture without restarting the application.
+Deliver current live frames to the UI without blocking or accumulating unbounded stale frames.
 
-### Tasks
-- [ ] Add Start action
-- [ ] Add Stop action
-- [ ] Connect actions to camera source
-- [ ] Report streaming state
-- [ ] Prevent invalid state transitions
-- [ ] Handle camera start failure
-- [ ] Log start/stop operations
+### Required behavior / tasks
 
-### Acceptance Criteria
-- [ ] Stream starts successfully
-- [ ] Stream stops successfully
-- [ ] Repeated start/stop cycles work
-- [ ] Failure produces a visible error/log message
+- [ ] Implement concrete Qt system camera source
+- [ ] Add start/stop lifecycle and state transitions
+- [ ] Display latest frame in the application while preserving aspect ratio
+- [ ] Handle frame-format conversion at a clear boundary
+- [ ] Measure/display active resolution and delivered/displayed FPS
+- [ ] Use bounded/latest-frame behavior rather than unbounded queue growth
+- [ ] Keep GUI responsive during repeated start/stop cycles
+
+### Architectural constraints
+
+- Follow repository `AGENTS.md` and the `micro-robot-swarm-desktop` skill.
+- Keep domain logic independent of GUI widgets and concrete hardware APIs unless this issue is specifically implementing an adapter.
+- Use metric world coordinates, radians, and monotonic timestamps internally where physical/time data are involved.
+- Do not implement unrelated future milestone features.
+
+### Acceptance criteria
+
+- [ ] Live video is visible and updates continuously
+- [ ] Start/stop can be repeated without restart
+- [ ] GUI remains responsive
+- [ ] Aspect ratio is preserved
+- [ ] Measured resolution/FPS are shown
+- [ ] No unbounded frame backlog is created
+
+### Validation expected
+
+- [ ] Build all affected targets.
+- [ ] Run relevant automated tests.
+- [ ] Add/update tests for non-trivial pure logic introduced by the issue.
+- [ ] Record any required manual/hardware checks and their actual result.
+- [ ] Review the final diff against every acceptance criterion.
+
+### Dependencies
+
+- Catalog Issue 10
+
+### Priority
+
+High
+
+### Milestone
+
+M1 — Application Foundation & Camera Infrastructure
+
+### Branch and commit guidance
+
+Suggested branch after GitHub assigns the issue number: `feat/<issue-number>-implement-live-camera-start-stop-and-responsive-preview`
+
+Expected logical commit split (adapt to the actual repository; do not create empty/mechanical commits):
+- `feat(camera): implement Qt live camera source`
+- `feat(ui): display responsive live preview and stream metrics`
+- `test(camera): cover source state transitions where practical`
+
+### Pull request completion
+
+Codex should push the issue branch and create the PR automatically after validation. The PR must summarize implemented feature points, list commands/checks actually run, note any unverified hardware/manual checks, and reference the GitHub issue with `Closes #<issue-number>` when appropriate. **Do not merge the PR.**
```

</details>

## Verification method

GitHub CLI (`gh api`) retrieved all open and closed repository issues and milestones via paginated REST endpoints. GraphQL retrieved Project #4 fields, options, and all items with pagination; the Project exceeded 100 items after import, so both item pages were read. Verification used fresh post-import API responses, not only creation responses.

Checks compared milestone descriptions and newly created issue bodies exactly with the extracted source sections; compared existing bodies and statuses with pre-import snapshots; checked milestones, labels, priorities, membership, duplicate titles for newly created issues, field definitions, assignees, and source SHA-256 hashes.

No application build or hardware test was required for this metadata-only import.

| Source | SHA-256 before and after |
|---|---|
| `docs/MILESTONES.md` | `7ba2e09cd2bbfc6d1d41ecbb35a5d51c548eb039d502f49a9055b7f55d2d5d0b` |
| `docs/GITHUB_ISSUES_M1_M8.md` | `75d2216703a4f654a14c27dad3e85742f6fe0f8bc3efb9167c595efd67a7d50d` |
