# GitHub Project Gantt — Applied M1–M5 Schedule

> Historical snapshot from 2026-10-10. Counts, statuses, dependencies, and dates
> reflect that recorded operation and omit later Linux issues. For current state,
> use [GitHub issues](https://github.com/Krzys2002/Mini_Robot/issues),
> [Project #4](https://github.com/users/Krzys2002/projects/4), and the
> [planning index](../planning/README.md).

**APPLIED AND VERIFIED. M6–M8 excluded. Hard deadline: 2026-11-28, Europe/Warsaw.**

25 nominal h/person/week, 75% effective capacity and 10% contingency; software effort divided by six cumulatively, physical tests/setup divided by three. Bars show calendar spans, with the day after each inclusive Deadline as the chart end. Weekends are included visually but have zero planned capacity. Main dates finish November 26; November 11 nonworking sensitivity moves completion to November 27. Completed #17/#25 retain their data and are omitted from future bars. `crit` marks the native dependency-only critical path, not Issue Priority or Status. See the [complete schedule and verified changes](GITHUB_PROJECT_SCHEDULE.md).

## Milestone forecast

```mermaid
gantt
    title Mini Robot - M1 to M5 approved forecast
    dateFormat YYYY-MM-DD
    axisFormat %m-%d
    todayMarker off

    section Hard deadline
    November 28 target :milestone, target, 2026-11-28, 0d

    section Active scope
    M1 remaining work :M2, 2026-10-12, 2026-10-17
    M2 remaining work :M3, 2026-10-16, 2026-11-04
    M3 remaining work :M4, 2026-10-13, 2026-11-05
    M4 remaining work :M5, 2026-10-15, 2026-11-14
    M5 remaining work :M6, 2026-11-12, 2026-11-27
```

## All 53 unfinished issues

```mermaid
gantt
    title Mini Robot - software /6 and physical tests /3
    dateFormat YYYY-MM-DD
    axisFormat %m-%d
    todayMarker off

    section Hard deadline
    November 28 target :milestone, final_target, 2026-11-28, 0d

    section M1 — Application Foundation & Camera Infrastructure
    #23 Implement application logging system :I23, 2026-10-12, 2026-10-13
    #26 Define ICameraSource interface :crit, I26, 2026-10-12, 2026-10-13
    #28 Configure GoogleTest target :I28, 2026-10-12, 2026-10-13
    #44 Define robot transport and protocol domain boundary :I44, 2026-10-12, 2026-10-13
    #45 Create authoritative WorldModel skeleton :I45, 2026-10-12, 2026-10-13
    #24 Implement configuration loading :I24, 2026-10-13, 2026-10-14
    #27 Implement SimulationTransport / fake robot source :I27, 2026-10-13, 2026-10-14
    #29 Enumerate available cameras :crit, I29, 2026-10-13, 2026-10-14
    #47 Add recorded-video camera source for deterministic replay :I47, 2026-10-13, 2026-10-14
    #48 Add macOS build and test continuous integration :I48, 2026-10-13, 2026-10-15
    #30 Add camera selection UI :crit, I30, 2026-10-14, 2026-10-15
    #31 Implement start/stop live stream :crit, I31, 2026-10-15, 2026-10-16
    #46 Handle camera disconnect reconnect and source failure :I46, 2026-10-16, 2026-10-17

    section M2 — Vision, Calibration & Robot Localization
    #49 Select AprilTag family marker size and robot-ID mapping :I49, 2026-10-16, 2026-10-20
    #53 Implement intrinsic camera calibration wizard and target capture :crit, I53, 2026-10-16, 2026-10-20
    #50 Integrate AprilTag detector into frame pipeline :I50, 2026-10-20, 2026-10-22
    #54 Calculate camera intrinsics distortion and reprojection error :crit, I54, 2026-10-20, 2026-10-21
    #55 Persist calibration profiles per camera and resolution :crit, I55, 2026-10-21, 2026-10-22
    #51 Define marker observation robot identity center and image-space heading :I51, 2026-10-22, 2026-10-23
    #56 Define tabletop world coordinate frame and workspace calibration model :crit, I56, 2026-10-22, 2026-10-23
    #52 Add vision debug overlay and performance diagnostics :I52, 2026-10-23, 2026-10-24
    #57 Implement camera-to-table homography calibration :crit, I57, 2026-10-23, 2026-10-27
    #58 Convert marker observations to metric robot Pose2D :crit, I58, 2026-10-27, 2026-10-28
    #59 Implement per-robot tracker with fresh stale and lost states :crit, I59, 2026-10-28, 2026-10-29
    #60 Implement heading unwrapping and exponential pose smoothing :crit, I60, 2026-10-29, 2026-10-30
    #61 Estimate velocity angular velocity confidence and observation age :crit, I61, 2026-10-30, 2026-10-31
    #63 Measure localization and orientation accuracy across workspace :I63, 2026-10-30, 2026-11-03
    #62 Add optional Kalman pose filter and comparison harness :I62, 2026-11-02, 2026-11-04

    section M3 — Robot Communication & Manual Safety Control
    #64 Define versioned robot command telemetry and BLE GATT contract :I64, 2026-10-13, 2026-10-15
    #65 Implement BLE device discovery connection and state reporting :I65, 2026-10-15, 2026-10-17
    #66 Implement BLE command writes telemetry notifications and protocol validation :I66, 2026-10-19, 2026-10-21
    #67 Implement RobotManager connection ownership and per-robot state :I67, 2026-10-21, 2026-10-23
    #68 Keep simulation protocol behavior compatible with real command model :I68, 2026-10-23, 2026-10-27
    #69 Implement manual robot control and connection panel :I69, 2026-10-23, 2026-10-24
    #70 Implement emergency stop command freshness and lost-connection safety rules :I70, 2026-10-27, 2026-10-29
    #71 Implement BLE reconnect recovery and repeated lifecycle testing :I71, 2026-10-29, 2026-10-31
    #72 Benchmark BLE communication latency jitter loss and reconnect time :I72, 2026-11-02, 2026-11-05

    section M4 — External-Vision Closed-Loop Robot Control
    #73 Implement differential-drive kinematic and command conversion model :I73, 2026-10-15, 2026-10-16
    #74 Create supervisory control loop service and scheduler :crit, I74, 2026-11-02, 2026-11-04
    #75 Implement baseline goal-point controller :crit, I75, 2026-11-04, 2026-11-05
    #76 Integrate live visual pose feedback with robot commands :crit, I76, 2026-11-05, 2026-11-07
    #77 Add control saturation freshness and operational state machine :crit, I77, 2026-11-09, 2026-11-11
    #79 Add controller experiment logging and export :I79, 2026-11-09, 2026-11-10
    #78 Add calibrated target-point selection and control panel :crit, I78, 2026-11-11, 2026-11-12
    #80 Benchmark closed-loop point regulation :I80, 2026-11-11, 2026-11-14

    section M5 — Operator Path Drawing, Trajectory Generation & Tracking
    #81 Implement world-space mouse path editor :crit, I81, 2026-11-12, 2026-11-14
    #82 Define path trajectory and execution domain models :crit, I82, 2026-11-16, 2026-11-17
    #83 Implement path simplification and smoothing :crit, I83, 2026-11-17, 2026-11-18
    #84 Generate bounded trajectory/reference from smoothed path :crit, I84, 2026-11-18, 2026-11-19
    #85 Implement Pure Pursuit baseline path follower :crit, I85, 2026-11-19, 2026-11-21
    #86 Visualize desired path trajectory progress and actual robot trace :crit, I86, 2026-11-23, 2026-11-24
    #87 Implement path execution pause cancel resume and safe completion :crit, I87, 2026-11-23, 2026-11-24
    #88 Benchmark path-following accuracy on representative shapes :crit, I88, 2026-11-24, 2026-11-27
```

All 53 issue deadlines and five milestone deadlines fit November 28. No changes to GitHub were made.

Verified against GitHub after application on 2026-10-10T18:14:32.446434+02:00. Bars represent approved plans, not completed work. No changes were pushed.
