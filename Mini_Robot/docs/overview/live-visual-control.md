# Live visual pose feedback

Issue #76 composes calibrated camera → metric observations → tracker → WorldModel
→ supervisory loop → RobotManager → selected transport for one selected robot.
`VisualControlPanel` presents the matched preview, target entry and status.
`LiveVisualController` maps selected pixels into the calibrated workspace and
owns target and control decisions.

## Operation and motion ownership

Select/connect the robot with the existing selector and start calibrated camera
tracking. Apply target X/Y in metres and heading in radians, then explicitly
Start visual control. A click on the matched calibrated preview sets metric X/Y
and uses the heading field. Targets must be finite and inside the active metric
workspace with a frame identity. The panel shows actual filtered pose, target,
position error, heading error, tracking state/age and operational state. Target
edits are disabled while running. Launch, target editing, fresh data, reconnect
and E-stop clear all require an explicit Start before motion can resume.

Live visual command submission is enabled for simulation only. In BLE mode the
target panel remains visible but Start is disabled until physical acceptance
gates are complete; this boundary is enforced in the controller and cannot be
overridden by application tuning. Manual BLE control retains its separate
limits and acceptance requirements.

Start requires fresh accepted measurement provenance matching the authoritative
robot timestamp/frame and current source, plus normal ready session gates.
`ingestMetricBatch` returns only measurements actually Applied by RobotTracker;
the production `connectVisionTracking` bridge passes that immutable metadata to
visual control after ingestion. Rejected duplicate/future data cannot restore
cleared source provenance. Control uses filtered WorldModel poses, not raw
detector poses; original measurement timestamps are preserved.

DesktopController arbitrates normal motion. Visual Start cancels held manual
input. Manual input, Stop/clear, scanning, robot selection, disconnect, camera
replacement/stop, workspace activation/change and shutdown interrupt visual
control before proceeding. E-stop first latches every manager session and cancels
pending motion, then interrupts visual control so the run state is recorded as
E-stop. Manual drive actions are disabled while visual control owns commands;
Stop/E-stop remain in the always-visible robot panel.

Pause submits a Stop attempt, ends the current run and retains the target for
an explicit restart. Cancel clears the target and submits a Stop attempt if a
run is active. You can cancel a paused target. The always-visible robot panel
provides the global E-stop action.

The preview and click handling share the aspect-fit viewport and pixel-centre
convention. The injected `OpenCvCalibrationUndistorter` uses the active
intrinsics to undistort the selected pixel before the workspace homography
projects it into metres. Clicks in the letterbox margins are ignored. A click
requires a current preview with the active camera, resolution and calibration
revision, a matching WorldModel frame and a point-undistortion adapter. The
preview must meet the control freshness limit and be at least as recent as any
accepted measurement for the selected robot. Resizing preserves the metric target.
Identity ambiguity, robot fault/error and session loss also interrupt. A different
robot clears the old target/provenance; a different frame requires a new target.

The core loop rechecks age/session/E-stop/fault/frame gates before calculation
and submission. Timer and measurement-driven polls share its phase/deadline.
Arrival submits zero then Stop and ends the run. Stale/lost/invalidated tracking
pauses command generation and attempts Stop; disconnect may make Stop unavailable.
Fresh data/reconnection never resumes motion without explicit Start. Submission
and software inhibition are not physical stopping evidence.

Controller or monotonic-clock faults remain latched across Start attempts. The
visual panel offers a separate acknowledgement action only after the manager
session, camera provenance, tracking age, target and workspace gates are valid;
acknowledgement returns the control state to Ready but does not start motion.
Manager safety faults and E-stop remain controlled by their existing manager
recovery path. When the selected robot changes, the old loop is stopped and
discarded so its paused/fault state cannot label the new robot.

## Composition, settings and teardown

`main.cpp` constructs the [goal policy](goal-point-controller.md),
[scheduler](supervisory-control.md), accepted-measurement bridge and panel.
Simulation and `--ble` use the same RobotManager path, but only simulation enables
live visual command submission. The post-ingestion
connection uses visual control as its QObject context, preventing late vision
shutdown callbacks from accessing a destroyed consumer.

`LiveVisualSettings::fromApplication` injects configured goal gains/tolerances,
manual v/omega limits and command validity, a period capped at 50 ms and the
tracking stale threshold as maximum age. The example config supplies bring-up
defaults and supports per-experiment tuning. These are software settings, not
measured actuator limits, physical accuracy or approved gains. Supply
commissioned settings before physical trials. The same manual velocity limits
are used by the controller and loop; the loop remains the final command bound.

Model, coordinator, manager and timer share the existing owner thread. Detection
stays on its bounded worker; policy callbacks are prompt/nonblocking. Shutdown
interrupts visual motion before stopping vision/camera and disconnecting. No new
pose authority or unbounded control/vision queue is added.

## Trace contract

Recorded control/Stop attempts produce compact JSON lines on module
`visual-control` through ControlTraceWriter. Its asynchronous handoff permits
at most 256 pending lines, each at most 8,192 bytes; overflow/oversize records
are dropped and counted. The shutdown log reports `dropped_records`. No sink/file
write runs in the control callback. The writer drains/joins after the GUI event
loop exits, before Logger destruction. Other existing log calls remain synchronous;
this is not a global logging redesign or real-time latency guarantee.

Fields include robot/source/frame identity, original `camera_time_ns`, accepted
`frame_sequence`, scheduled/control/submission timestamps, lateness, skipped
period count, cycle-captured operational state/frame, filtered and raw poses,
measurement quality/confidence, domain command/Stop sequence, result/submission
status, command type, v/omega and available error terms. Optional/unavailable fields are null. Integer timestamps
and IDs are decimal strings, avoiding JSON floating-point precision loss. Times
use this process's steady-clock epoch, not a remote clock. The timestamp-source
field explicitly marks exposure time unverified: Frame.capturedAt does not prove
sensor exposure time or known upstream buffering.

ControlLoop's optional prompt/nonthrowing sequence supplier adds correlation IDs
to RobotCommand metadata and cycle records; prior callers retain unassigned
metadata. Live control allocates IDs across runs. These are not BLE wire sequence
or execution acknowledgements. With `logging.level=debug`, existing BLE tx/result
logs map domain IDs to adapter token/wire sequence and robot result. The application
trace's `wire_sequence` remains null because allocation belongs to the adapter.
Keep submission, ATT completion, robot acceptance and physical stopping distinct.

## Per-run experiment export

Each explicit Start creates a JSON Lines run in the per-user application data
directory under `control-runs` unless `logging.control_run_directory` overrides
it. The status panel displays the path after the writer opens the file and shows
worker-side export failure instead of an unusable path. The file
contains run_start metadata (target and effective gains/limits), one sample per
recorded cycle/Stop attempt (immutable measured/target pose, errors, age,
timestamps, state and command outcomes), and a run_end completeness summary.
The versioned [control experiment export format](control-experiment-export.md)
defines fields, units, time bases and drop/error semantics.

Run files share ControlTraceWriter's bounded asynchronous handoff. Sample
pressure is counted and reported in run_end; two queue slots are reserved for
run delimiters. File I/O errors are logged and shown in the panel. The footer
reason differentiates arrival, operator stop, manual takeover, connection loss,
camera/workspace changes, E-stop, safety/controller faults and other endings.
queue_complete reflects queue
pressure only, so verify the export's start/end records and flag before analysis.
The app log's per-cycle traces remain available when no dedicated run export can
be opened. This is software telemetry, not physical motion or stopping evidence.

## Physical acceptance and approved stack

Existing benchmark [#80](https://github.com/Krzys2002/Mini_Robot/issues/80) owns
physical #76 approach and tracking/communication-loss checks, with dedicated
setup/evidence/test/acceptance lists added there. All are **NOT RUN**. #167
actuator calibration remains a hardware gate; #154/#166/#228 safety prerequisites
remain in force. Physical metric/age evidence remains #224/#226/#236. Do not arm
a physical run until its commissioning prerequisites and supervised envelope
are verified; software/fake-radio results do not establish physical readiness.

The maintainer approved stacking on #75 PR #292 at
`5f2f5fa10212a0c68187a3072ec8eb68e2a5c003`, including #74 PR #291 at
`c86ddbdac58b6821ad699ff5396f8c2da765156b`. Both were open/non-draft with no
reviews/check rollup when inspected. Initial child base:
`feat/75-goal-point-controller`; merge order #291 → #292 → this child. After
parent merges, verify actual code and review child bases/diffs; squash merges
may require child-only replay. Retarget/history changes require approval.

## Local verification on 2026-11-17

Code commits `83d6675` (domain correlation) and `a4a6fa8` (application/UI and
acceptance-filtered provenance) were tested on macOS arm64 / AppleClang 21 /
Qt 6.12 / OpenCV 5.1, combined with the approved #74/#75 parent tree.
Root configure commands:

```sh
cmake --preset core -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=/Users/krzysztofsawicki/Documents/Put/sem6/Term_Design/build/macos/_deps/googletest-src
cmake --preset macos -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=/Users/krzysztofsawicki/Documents/Put/sem6/Term_Design/build/macos/_deps/googletest-src -DCMAKE_PREFIX_PATH=/opt/homebrew/opt/qt -DMICRO_ROBOT_BUILD_BLE=ON
```

For each preset, ran `cmake --build --preset <preset> --parallel 3` and
`ctest --preset <preset> --parallel 3`. Root core: **707/707 passed**. Production
desktop/native Qt BLE targets built; host suite: **1,033 passed, two existing
opt-in audits skipped (1,035 selected)**. Standalone `Mini_Robot/` used the same
core configure/GoogleTest argument, build and CTest commands with its independent
build directory: **707/707 passed**. Portable firmware-host suites are included
by the existing build; no firmware sources or contracts were changed.

The 16 new cases cover domain sequence/accepted-metadata core logic, filtered
metric bridge and traces, stale/lost/source/frame invalidation, rejected-duplicate
provenance, connection/recovery, manual/visual arbitration, selection/ambiguity,
fault/E-stop/shutdown, arrival, UI controls, real SimulationTransport, bounded
logging with a blocked sink, and fake-driven production BLE sequence/Stop handoff.
An initial mixed-pointer initializer-list compile error was corrected; an initial
frame-change regression exposed discarded new-frame metadata and was corrected.
Final complete suites pass; no unresolved upstream/child test failure remains.
Existing compiler/linker and offscreen-platform warnings are non-fatal.

The focused panel test also passed at `QT_SCALE_FACTOR=1.5` and `2`; retained
offscreen screenshots were visually checked for units, fields, status and Stop
legibility. This is synthetic rendering, not native display or physical evidence.
Child-only diff/acceptance review, whitespace and changed-document relative-file
links passed. The child worktree had no usable local graph; navigation used
source/rg fallback without borrowing generated output from another worktree.

Camera, physical BLE, powered robot, native display, Linux and Windows execution
were **NOT RUN**. #80's physical acceptance remains entirely unchecked, with
#167/#228 and related hardware/safety gates still required.
