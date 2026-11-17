# Baseline goal-point controller

Issue #75 supplies `GoalPointController`, a forward-only, stateless `IController`
implementation for the [supervisory loop](supervisory-control.md). It adds no BLE,
GUI, camera, motor, planner or clock dependency. Production composition/target UI
and physical closed-loop commissioning remain separate work; `main.cpp` does not
automatically activate this controller.

## Inputs, errors and arrival

`evaluate(measured, target)` is a pure calculation returning optional
`GoalPointEvaluation`: state, errors and a body-frame velocity command. Both poses
use world metres/radians with zero heading along +X and positive rotation CCW.
For world displacement `(dx, dy)`, measured heading `theta` and range `rho`:

```text
rho = hypot(dx, dy)
forwardError =  cos(theta) dx + sin(theta) dy
leftError    = -sin(theta) dx + cos(theta) dy
bearingError = wrap(atan2(dy, dx) - theta)
headingError = wrap(targetHeading - theta)
```

Heading inputs are normalized before subtraction; errors use the existing
`(-pi, pi]` convention. At exactly zero range, bearing error is defined as zero.
Nonfinite poses or nonrepresentable displacement/body-frame errors return no
evaluation. The `calculate(ControlInput)` adapter additionally rejects a missing
pose or Unknown/Stale/Lost tracking. The loop still owns actual age/session checks,
and submission-time gates, freshness and Stop routing.

Inside the inclusive position tolerance, translation is zero. With
`alignFinalHeading=true`, heading alignment continues until the absolute heading
error is inside its inclusive tolerance; then state is Arrived and both outputs
are exactly zero. With `alignFinalHeading=false`, position alone establishes
arrival. Repeating identical arrived inputs always returns zero. There is no
hidden completion latch or hysteresis: a changed/noisy observation outside the
tolerance is evaluated again. Physical settling/noise tuning is not established
by deterministic tests.

## Feedback law and bounds

All parameters are supplied through `GoalPointControllerConfig`, with no assumed
physical robot gains or limits. Gains, speed limits and tolerances must be finite
and positive. `bearingGainPerSecond > positionGainPerSecond` is required;
heading tolerance is at most pi and the turn-in-place angle is in `(0, pi/2]`.
Constructor validation throws `invalid_argument` otherwise.

Outside position tolerance, using `alpha = bearingError`:

```text
omega = clamp(kBearing alpha, -omegaMax, omegaMax)
if abs(alpha) >= turnInPlaceAngle:
    v = 0
    state = TurningToPoint
else:
    v = min(kPosition rho, vMax) max(0, cos(alpha))
    state = Approaching
```

Thus a point behind the robot is approached by turning in place first, never by
reversing. Except at the exactly opposite bearing, turns follow the shortest
signed angular error. The antipodal tie maps to +pi and chooses CCW. As with any
shortest-turn policy, the preferred direction has a cut at the antipodal error;
this is distinct from a spurious command jump caused by equivalent raw headings
at +pi/-pi. The tests check wrap-equivalent headings and small wrapped errors.

Within position tolerance, final alignment uses
`omega = clamp(kFinal headingError, -omegaMax, omegaMax)` and `v = 0`.
Proportional saturation compares magnitudes before multiplication to avoid
overflow even for extreme finite gains; zero error remains zero when the
saturation threshold underflows. Outputs stay within configured bounds.
Choose controller limits within the loop and adapter/robot limits; the loop's
independent bounds remain in force. These are semantic v/omega setpoints,
never wheel PID or PWM.

In the ideal continuous, unsaturated approach region,
`rho_dot = -kPosition rho cos(alpha)^2` and
`alpha_dot = kPosition cos(alpha) sin(alpha) - kBearing alpha`.
The gain ordering makes the bearing correction dominate the geometric term
there; the in-place stage reduces larger bearing errors before translation.
Final heading regulation is proportional on the shortest error. This motivates
the baseline; it is not a guarantee for arbitrary sampled periods, gains,
actuator saturation, latency, noise, slip or physical robot behavior. The caller
must select a compatible control period and experimentally validated settings.

## Deterministic verification and limits

`micro_robot_goal_controller_tests` covers error frames/signs, proportional and
saturated output, turn/forward policy, exact tolerance boundaries, pose versus
point goals, repeated arrival, heading wrap, invalid/unavailable inputs,
configuration validation and finite extremes.

Nine scenarios execute the production `ControlLoop` → `RobotManager` →
`SimulationTransport` boundary with virtual 20 ms steps. The harness integrates
the existing ideal no-slip unicycle derivative and feeds the resulting pose back
through `RobotTracker` as fresh measurements. Starts include forward, backwards
orientation, a point behind the robot, lateral and signed-world goals, heading
wrap, final-heading-only correction, a point-only goal and an already-arrived
robot. Each checks all command bounds and five consecutive arrived zero outputs.
The 80 s simulated horizon is a test bound, not a physical settling-time claim.
No production robot-physics model or extra authoritative WorldModel is added.

Run root core and host presets, plus the standalone core entry point for shared
target registration. Physical camera/radio/motor and native GUI/platform checks
are **NOT RUN** unless separately recorded. Existing physical stopping/integration
validation remains #228 and firmware issues; physical tracking/age acceptance is
#236. No physical acceptance checklist is introduced or duplicated by #75.

## Approved stack

This work is stacked on #74 PR #291 at
`c86ddbdac58b6821ad699ff5396f8c2da765156b`, explicitly approved by the maintainer.
The child branch is `feat/75-goal-point-controller`; its initial PR base is
`feat/74-supervisory-control-loop`. The parent was open, non-draft, with no review
or check rollup when inspected. Its local results are recorded in the supervisory
guide, not presented as remote CI. Merge #291 first, then verify actual master
source and review/rebase or replay the child-only diff as appropriate. Retargeting
or remote history changes require approval; no parent merge is performed here.

## Local validation on 2026-11-17

Implementation commit `59c12cb` was tested on macOS arm64 / AppleClang 21 /
Qt 6.12 / OpenCV 5.1. Configure commands from the child worktree root:

```sh
cmake --preset core -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=/Users/krzysztofsawicki/Documents/Put/sem6/Term_Design/build/macos/_deps/googletest-src
cmake --preset macos -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=/Users/krzysztofsawicki/Documents/Put/sem6/Term_Design/build/macos/_deps/googletest-src -DCMAKE_PREFIX_PATH=/opt/homebrew/opt/qt -DMICRO_ROBOT_BUILD_BLE=ON
```

For each preset, ran `cmake --build --preset <preset> --parallel 3` followed by
`ctest --preset <preset> --parallel 3`. Core: **705/705 passed**. Host production
desktop/Qt BLE build succeeded; **1,017 tests passed, two existing opt-in audits
skipped (1,019 selected)**. The independent standalone `Mini_Robot/` core configure
(same GoogleTest argument), build and CTest commands also passed **705/705** in
their own build directory. The focused controller binary was run during development;
the final 20 controller tests are included in all three complete suites.

These are combined-parent-and-child tree results; there were no test failures
to attribute to either. Existing WorldModel initializer, GoogleTest conversion
and duplicate-library warnings remain non-fatal. The child-only diff was reviewed
against the approved parent and all software acceptance criteria; whitespace and
relative file-link checks in the seven changed documentation files passed.
No local graph was available in the child worktree, so navigation used source/rg
fallback without reusing another worktree's graph or environment.

No physical camera/radio/motor, native display, Linux or Windows validation was
performed. Ideal simulated convergence does not establish physical stability,
accuracy, safe gains/limits, stopping, or production autonomous readiness.
