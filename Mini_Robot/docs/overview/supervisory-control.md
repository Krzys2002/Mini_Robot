# Supervisory control loop

Issue #74 provides an opt-in service and scheduler. It does not select a production
control policy, add target-selection UI, or activate autonomous movement in
`main.cpp`. A [baseline goal-point policy](goal-point-controller.md) is supplied
separately by #75; production activation and physical closed-loop commissioning
remain separate work. The existing manual-drive composition is unchanged.

## Boundaries and ownership

`micro_robot_control_loop` is Qt/OpenCV/hardware-free. `ControlLoop` borrows one
authoritative `WorldModel`, `RobotManager`, an `IController`, and a clock. The
composition owns these dependencies and keeps them alive longer than the loop.
All calls, including clock and controller calls, use the same owning thread as
the model, manager and transport. No independent authoritative pose is maintained.
`WorldModel::workspaceFrameId()` permits checking the active frame without copying
the entire registry.

Each loop addresses one RobotId and one metric target pose. Call `setTarget`,
then explicitly `start`, then `poll` from the scheduler. The policy receives the
latest copied robot state, tracking age/confidence, target and cycle timestamp.
It returns optional body-frame `(v [m/s], omega [rad/s])`; positive v is forward,
positive omega is counterclockwise. Pose/target coordinates are world metres and
radians. Heading error is the shortest signed angular difference. Pose timestamps
remain the original observation timestamps, with no implicit prediction.

Policy implementations must be prompt and bounded, with no blocking I/O,
GUI access, transport access or loop reentrancy. A slow/heavy planner belongs
upstream behind a bounded asynchronous handoff; it must not run in `calculate`.
The service catches policy exceptions and inhibits. Policies can be replaced by
constructing another loop with the same manager/model and a different controller;
RobotManager requires no policy changes.

The composition must give a running loop exclusive normal motion authority for
its RobotId, stop it before switching to manual/another loop, and stop it before
destroying dependencies. This issue does not implement an application-wide
manual/autonomous mode arbiter. E-stop remains independent in RobotManager.

## Timing and safety

`ControlLoopConfig` supplies a positive whole-millisecond period, command validity
strictly longer than that period, a positive maximum observation age, and finite
positive symmetric v/omega limits. Select limits/validity within the chosen
adapter's negotiated envelope; adapter validation and dispatch-time freshness
remain authoritative. There are no assumed physical robot defaults.

The injected clock defaults to `steady_clock::now`. Tests replace it with virtual
time. The first poll after start is due immediately. Subsequent polls preserve
the configured phase. Late polls run one cycle and count skipped periods; they
never replay queued control cycles or wait/sleep to catch up.

Before evaluating a policy, and again immediately before submission, the loop
requires Fresh tracking, observation age strictly below the configured limit,
an unchanged target world-frame identity and a connected, requested, Ready
RobotManager session without E-stop/fault/removal. Unknown, future-dated,
stale/lost/invalidated poses are not controller inputs. Rechecking after calculation
discards an output when its observation expires or the session/frame changes.
Safety is also checked on polls before the next deadline.

Invalid/unavailable policy output, exceptions, clock reversal, lost tracking,
session loss and frame changes inhibit the loop and attempt a Stop through
RobotManager. Inhibition is latched: fresh measurements/reconnection alone do not
restart motion; explicit `start` is required. A replaced world frame additionally
requires a newly installed target. Finite requested v/omega are scaled together
to their configured bounds, preserving curvature. Commands carry the actual
submission clock timestamp and configured validity.

Stop/command submission means handed to the adapter, never delivered or physically
executed. A disconnected adapter can reject Stop; firmware disconnect/watchdog,
E-stop and local motor enforcement remain essential. This service cannot recall
an already transmitted command. Physical stopping/production integration is
tracked in #228 and firmware validation issues; tracking accuracy/age diagnostics
remain #236. All physical checks are **NOT RUN** by the scheduler tests.

## Qt adapter and observability

`QtControlLoopScheduler` lives in the Qt Core desktop-controller target and uses
a precise QTimer on the same owner event-loop thread. It performs no wait, sleep,
thread join, camera read or radio operation. `start` arms the core and timer;
`stop` stops the timer and core; destruction stops the loop. Dependencies must
outlive the adapter. Do not move the adapter or its dependencies between threads
while active. Prompt policy execution allows other queued GUI events to run.

After a poll records a cycle it emits `cycleCompleted`. Query the detached
`lastCycle()` to observe scheduled/start/finish timestamps, lateness, skipped
period count, original observation time/age, x/y/distance/heading error, requested
and bounded output, result, command submission and Stop attempt. Only the latest
record is retained, so memory is bounded; external observers may persist records
without adding synchronous file I/O to the policy or cycle.

## Verification

`ControlLoopTest` uses the production RobotManager and SimulationTransport with
virtual time and scripted replaceable policies. It covers measured/target inputs,
errors and bounds, per-robot isolation, phase/skips/no bursts, age thresholds,
stale/lost/invalidation, disconnect/reconnect, E-stop/fault, mid-calculation changes,
invalid/unavailable/throwing policies, clock reversal, frame changes and restart.
The simulation models transport semantics, not physical robot dynamics.

`ControlLoopSchedulerTest` runs the real Qt timer adapter alongside a GUI event-loop
heartbeat and verifies cycle signals and teardown. This is offscreen software
evidence under the prompt-policy contract, not native-platform responsiveness or
a portable real-time timing guarantee. Run root core and host presets; because
target registration is shared, also validate the standalone core entry point.
Physical camera, radio, motor, native GUI, Linux and Windows checks remain
**NOT RUN** unless separately recorded.

### Local validation on 2026-11-17

Implementation `c2cb808` on `feat/74-supervisory-control-loop`, based on master
`b81e520`, was tested on macOS arm64 / AppleClang 21 / Qt 6.12 / OpenCV 5.1.
From the worktree root, `cmake --preset core` and `cmake --preset macos` were
configured with the existing GoogleTest source via
`-DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=/Users/krzysztofsawicki/Documents/Put/sem6/Term_Design/build/macos/_deps/googletest-src`.
The macOS configure also supplied `-DCMAKE_PREFIX_PATH=/opt/homebrew/opt/qt`
and `-DMICRO_ROBOT_BUILD_BLE=ON`.

- `cmake --build --preset core --parallel 3` and
  `ctest --preset core --parallel 3`: **685/685 passed**.
- `cmake --build --preset macos --parallel 3` and
  `ctest --preset macos --parallel 3`: production desktop/Qt BLE targets built;
  **997 passed, two existing opt-in audits skipped (999 selected)**.
- From `Mini_Robot/`, the independent `core` configure (same GoogleTest source),
  build and CTest commands: **685/685 passed** in its separate build tree.
- The 14 new core cases and one new offscreen scheduler/event-loop case passed.
  An initial test compile failed because explicit unit wrappers were initialized
  with nested empty braces; that test source was corrected before passing builds.
- `git diff --check` passed. Existing confidence-initializer, GoogleTest conversion
  and duplicate-library warnings are non-fatal and outside this change.

The worktree had no local graph; navigation used source/`rg` fallback. No generated
graph or borrowed graph environment was used. The builds include existing portable
firmware-host suites, but no firmware contracts or production firmware sources
were changed. No camera, BLE radio, powered robot, native display, Linux or Windows
execution was performed. These results establish software behavior only.
