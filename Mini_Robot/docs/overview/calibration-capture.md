# Intrinsic calibration target capture

Issue #53 implements observation capture. Issue #54 adds the
[intrinsics solver and quality reporting](intrinsic-calibration.md).
Calibration-profile persistence (#55) and tabletop homography (#57) remain separate.
Calibration capture does not change AprilTag identity or the WorldModel; the
separate vision worker can continue processing frames.

## Target and operator workflow

Use the [printable chessboard](../calibration/chessboard-9x6-25mm.svg):

- **9 columns × 6 rows of inner corners**, formed by 10 × 7 squares.
- Square side: **0.025 m (25 mm)**. Black/white board extent: 250 × 175 mm.
- White border: 15 mm on each side. Total printed extent: 280 × 205 mm.
- Print in landscape at 100% without fit-to-page. Measure square dimensions in
  both directions and mount on a flat, rigid surface. Do not use a stretched,
  curved or glossy target. The chosen workflow is chessboard, not ChArUco.

Select and start a camera in the main window, then click **Camera calibration…**.
Full chessboard detection runs at up to 5 Hz. Select one of three **Live preview**
modes:

- **Hold until next detection** (default): keep the last detected corner positions
  fixed until the next result. They can misalign if the board moves. A result
  without corners clears the grid.
- **Track between detections**: estimate corner positions on intervening frames
  using optical flow. Hide uncertain/expired grids and skip obsolete results
  rather than drawing an old estimate on a newer frame.
- **Matched image and corners (up to 5 Hz)**: advance the preview image and its
  detected corners together. Video follows the detection rate, retaining exact
  frame alignment even when the board moves. Capture can reuse that sample.

The selection lasts for this application session, including dialog close/reopen;
it is not written into calibration profiles. Hold and Track keep raw video
advancing; Matched holds each detected sample. All three preserve the captured
dataset when switched. Mode changes invalidate pending
old-mode work; a paused captured-view review stays paused.
Click **Start / restart capture** and **Capture view** while holding the board
still and in focus. Each usable, distinct view is accepted automatically.

Green dots show usable live samples or accepted captured inner corners. Red dots
show rejected detections when corners are available. Live evaluation uses the same
quality/diversity rules without adding an observation. **Cyan dots are held or
tracked corners**, without a fresh assessment of the current image. The yellow
ring identifies the first corner. Visually check alignment with the printed intersections. Use the accepted-view selector
to inspect any stored view. **Capture view** selects the displayed sample (raw
source pixels even in corrected mode), reusing a full detection only when it
belongs to that exact frame. Held corners are never saved for a newer image:
Capture runs a full-resolution
detection on its selected frame unless that exact frame already has a full result.
All existing quality checks remain. Neither holding nor tracking the overlay
adds observations.
Every completed capture (accepted, rejected or detector failure) shows its result
for five seconds, then resumes live preview automatically. Its duration is defined
by `CalibrationController::CaptureReviewDuration` in `src/app/CalibrationController.hpp`.
The interval begins at presentation, after any asynchronous corrected-preview
processing finishes; no sleep blocks
the GUI. Accepted observations remain stored. The last capture reason stays in a
separate non-modal feedback line through the transition, including redundant-view
rejections. `captureFeedbackAvailable(reason, accepted)` is the reusable event for
future non-modal notifications; `captureFeedback()` retains the latest reason.

A new capture replaces the interval. **Live view** resumes immediately; explicitly
selecting a stored view pauses it until the next action. Restart, cancel, camera or
resolution changes, hiding/closing the wizard and shutdown cancel obsolete timers.
A manual review/live choice made during detection is preserved when the result
arrives; the observation and feedback are retained without replacing that choice.
If no valid live frame is available at expiry (including a frame older than two
seconds or a future timestamp), the preview clears and asks the operator to start
or reconnect the camera. Fresh input restores live presentation. A sample
older than two seconds is rejected for capture; wait for a fresh sample. Integer image
coordinates denote pixel centres; presentation adds half a pixel before scaling
and uses the image's aspect-preserving bounds, including letterboxing.

Move the board around the image, change its distance, rotate it in the image,
and tilt it out of plane. Keep its full outer border visible. The progress text
shows view count, occupied image regions, scale spread and rotation spread.
The capture policy is a heuristic for useful diversity, not proof of a
well-conditioned calibration. Tilt and reprojection quality must also be
assessed visually and through solver reprojection errors.

**Remove / recapture selected** deletes that view and returns to live preview;
then capture its replacement. **Start / restart capture** clears all observations.
**Cancel / close** or Escape discards an unfinished session. **Calculate intrinsics**
completes capture and solves the in-memory dataset on a worker. Closing a completed
dialog preserves it until restart, camera loss/change or application exit.
Use **Save profile** after solving to persist the result; see the
[profile guide](calibration-profiles.md).

## Placement, rotation and next-capture help

Open **Board placement and capture help** in the wizard for the setup checklist,
axis definitions and common rejection remedies. The **Next capture** line uses
only accepted-view measurements. Read the last capture feedback alongside it:
a rejected frame does not fill a coverage cell or improve progress. Help can be
collapsed; the content scrolls on a small window or scaled display so controls
and wrapped text remain reachable.

### Prepare the target and camera

An **inner corner** is an intersection surrounded by four squares inside the
board. Nine by six inner corners therefore needs ten by seven squares; do not
enter or print a nine-by-six-square target. The adjacent inner-corner spacing is
one square side. Measure several squares along both printed directions with a
ruler after printing at 100% without fit-to-page. For the supplied target, both
spacings must be 25 mm. This wizard has no square-size entry field: reprint a
mismatched target rather than assuming the solver knows its measured dimensions.
Keep the target flat on a rigid backing and preserve its white border.

Keep the camera fixed and move the board. Set a useful focus, even lighting and
an exposure that keeps the stationary grid sharp without glare or deep shadows.
Keep resolution, focus, zoom, exposure and other image-processing settings fixed;
disable automatic changes where the backend allows. Restart capture after
changing settings. A profile from a different focus/zoom or optical setup needs
fresh calibration even if device identity and resolution happen to match.

### Try this sequence

1. Begin with the board near the image centre, facing the camera and large enough
   to read every intersection. Hold it still for Capture view, then inspect the
   stored corner overlay.
2. Move the **board centre** toward the top, bottom, left and right image regions,
   then toward each of the four image-corner regions. These are regions of the
   camera image, not squares of the printed target. Keep all outer squares and
   the white border visible. If an edge-region placement would clip the target,
   move slightly farther away first; do not try to put the target centre on the
   literal image boundary.
3. Try nearer and farther distances to change the board's apparent size. The
   closer view must remain fully visible and the farther view must remain sharp,
   sufficiently large and readable. A count of ten nearly identical views is
   not useful diversity.
4. Turn the board within the image, then add out-of-plane tilts about **both**
   board axes as described below. Alternate which edge comes toward the camera.
   Do not use nearly edge-on, clipped, moving/blurred or unreadable views.
5. Follow the next-capture suggestion for missing measured diversity, but also
   inspect X/Y tilts yourself. Once measured minimums pass, calculate intrinsics
   and inspect aggregate/per-view reprojection errors. Thresholds and low error
   alone do not establish physical accuracy; validate with real equipment in
   [#179](https://github.com/Krzys2002/Mini_Robot/issues/179).

### Board axes and rotations

These are axes attached to the board, separate from image coordinates or a
future tabletop world frame. For the supplied landscape target:

| Axis | Direction on the board | What the operator does |
| --- | --- | --- |
| X | Along the nine-column inner-corner direction, across the long edge | Tilt about X: bring the top edge toward the camera, then try the bottom edge |
| Y | Along the six-row inner-corner direction, across the short edge | Tilt about Y: bring the left edge toward the camera, then try the right edge |
| Z | Perpendicular to the board face | Rotate about Z: turn the board clockwise/counterclockwise within the image |

Use gentle, readable X/Y tilts that show perspective, rather than flattening
one board dimension into a nearly edge-on sliver. The unmarked target can have
reversed corner ordering; these axis descriptions guide movement and are not a
claim that the software knows which physical edge is positive. **X/Y tilt angles
are not measured or certified by capture progress.** In-plane angle diversity is
not a substitute for both out-of-plane tilts.

### Interpret progress and hints

- **Views** counts accepted observations only. The minimum is a collection gate;
  the maximum bounds storage. Removing a view recomputes the remaining coverage.
- **Image regions** counts occupied board-centre cells in a 3 × 3 image grid.
  The default requires three different cells. It does not measure full corner
  coverage, board tilt or a well-conditioned camera model. Explore edges and
  corners even after the minimum is satisfied.
- **Scale spread** is max minus min of the square root of the inner-corner
  quadrilateral area divided by image area. It is dimensionless. For example,
  4% and 9% area produce scale values 0.20 and 0.30, hence a spread of 0.10;
  the default minimum is 0.08. It is neither a measured distance nor a tilt angle.
- **In-plane rotation** is the largest first-row angle difference modulo 180°.
  The default minimum is 0.20 rad (about 11.5°); this does not estimate X/Y tilt.
- Hints first suggest a missing image region, then scale, then in-plane angle,
  then additional distinct views. They are suggestions, not new acceptance
  gates. Once minimums pass, they ask for visual X/Y-tilt and overlay inspection.
  At the view limit with insufficient diversity, remove a similar view and
  replace it. A cancelled/new-source session starts guidance from the beginning.

| Capture feedback | Action before trying again |
| --- | --- |
| Full chessboard not found | Show every outer square and border, improve lighting/focus, reduce extreme tilt |
| Board clipped or too close to edge | Move inward or slightly farther away; keep the outer border visible too |
| Blurry or low contrast | Hold still, improve focus/light, avoid glare; keep settings fixed after adjusting/restarting |
| Too small | Move closer while keeping the whole target visible |
| Redundant view | Change position, distance, in-plane rotation or X/Y tilt; retry after the board is still |
| Repeated/old/stale frame | Wait for fresh camera input or reconnect/start the camera |
| View or memory limit | Remove a less useful view; if lowering resolution is necessary, restart the session |
| Resolution changed | Restart at one fixed delivered resolution |
| Detector failure | Read the reported error, restore readable input and retry; preserve persistent failures for diagnosis |

The sharpness number is a Laplacian-variance/contrast heuristic in the target
bounds, not a physical focus measurement. Redundancy compares corner positions,
including reversed ordering; it does not infer a measured 3D orientation.

## Quality gates and diversity

`CalibrationCaptureConfig` centralizes the injectable thresholds. Defaults:

| Gate | Default |
| --- | --- |
| Full grid | Exactly 54 finite, row-major inner corners |
| Edge clearance | All inner corners at least 4 pixels from image boundaries |
| Minimum board size | Inner-corner quadrilateral covers at least 2% of image area |
| Sharpness | Grayscale Laplacian variance in target bounding rectangle ≥ 40 |
| Redundancy | RMS corner displacement / image diagonal ≥ 0.025 against every accepted view |
| Completion | At least 10 views, 3 distinct centre cells in a 3 × 3 grid, scale spread ≥ 0.08, rotation spread ≥ 0.20 rad |
| Storage | At most 24 accepted views and 128 MiB of retained frame storage |
| Input size | At most 4 megapixels, neither dimension above 4096 |
| Input freshness | Selected raw frame no more than 2 s old at capture request |

Redundancy comparison also considers reversed corner order, accounting for the
unmarked chessboard's 180° ambiguity. Scale is the square root of inner-board
area / image area. Rotation is the first row's heading, compared modulo pi.
Repeated sequence numbers and captures no newer than an accepted frame are
rejected. Invalid layouts, incomplete boards, mixed resolutions, insufficient
sharpness, clipped inner corners and memory/view limits yield visible feedback.
A full outer-border check is an operator responsibility: inner-corner bounds do
not by themselves prove the outer squares and border are wholly visible.

The fixed target in this version uses row-major metric object points with the
first inner corner at (0, 0, 0), x along columns, y along rows and z = 0.
These coordinates describe the target plane, not the future tabletop world frame.

## Architecture and lifecycle

- `micro_robot_calibration`: Qt/OpenCV-free target geometry, quality/diversity
  policy, bounded observations and explicit Idle/Collecting/Complete/Cancelled
  state. `ICalibrationDetector` is the image-processing boundary.
- `micro_robot_calibration_opencv`: portable OpenCV detector using
  `findChessboardCornersSB`. OpenCV 4 uses calib3d; OpenCV 5 moved detection to
  objdetect. Frame pixels are read-only and Gray8/RGB/RGBA padded rows are supported.
- `CalibrationController`: GUI-thread application adapter with an injected
  detector, solver and profile store on one worker; a separate worker serializes
  optional optical flow and preview remapping. Acquisition retains the newest frame, full
  detection retains its exact reusable frame/result and the held overlay. Each path
  has at most one in-flight job and replaceable newest input.
  Full live detection pauses during capture/solving/profile I/O. Preview remapping
  can run alongside detection on the other worker. Capture stays
  enabled during full live detection and may wait behind that one OpenCV call;
  no additional full live calls can get ahead of it. Corrected previews remap the
  current image and held raw coordinates using the
  same compatible profile. Held positions remain fixed until the next result;
  tracked positions follow the current frame only after successful flow. Tracked
  previews run flow before remapping to avoid remapping a frame twice.
  Cancellation/shutdown invalidate results immediately without GUI waits. OpenCV
  itself is synchronous and cannot be interrupted mid-call. Frames/corners are
  plain domain values; widget code does not perform detection.
- `CalibrationWizard`: Qt presentation, corner overlays, review and action routing.
  It contains no detection or acceptance algorithm.

DesktopController forwards immutable frames and source/stream availability without
changing camera ownership or bypassing `ICameraSource`. Stop requests, source
changes and disconnect/fault events clear the session. Resolution changes clear
it on the next frame. Generation tokens discard any result already in flight.
Restart/cancel, review/live transitions and wizard hide/destruction invalidate
in-flight live results. The target is immutable for a controller's lifetime; a
new controller is required for another target. Restart/cancel can invalidate a
job without blocking the GUI. Shutdown asks the
workers to quit, and the main event loop drains camera acquisition and both
calibration workers before destruction. Normal destruction therefore does not wait on detection.

## Build and verification

Desktop builds include the detector automatically. To run detector tests without
Qt, configure with `-DMICRO_ROBOT_BUILD_DESKTOP=OFF
-DMICRO_ROBOT_BUILD_CALIBRATION=ON`. To omit OpenCV entirely, disable desktop,
replay and calibration. The pure capture library/tests remain available.

```sh
cmake --preset macos
cmake --build --preset macos --parallel 3
ctest --preset macos -R '^(Calibration|QtCalibration|OpenCvCalibrationUndistorter)'
ctest --preset macos
```

Deterministic tests generate chessboards in memory, including rotated, scaled,
perspective-transformed, padded/color, blurred, partial and blank images. Session
tests cover quality rejection, duplicates, diversity, limits and transitions.
Offscreen UI tests exercise capture/review/completion, remove/recapture,
restart/cancel, stale input, source and resolution changes, worker execution,
bounded jobs and discarding cancelled results.

### Manual acceptance procedure

1. Print and measure the target, select a physical camera and open the wizard.
2. Capture a diverse set without external scripts. Inspect every overlay.
3. Try blur, partial/clipped board, small board and repeated view. Confirm feedback.
4. Review earlier views, remove one, recapture it, then finish after all diversity gates pass.
5. Test cancel/restart, camera stop/removal/source change, and shutdown during detection.
6. Repeat UI checks on Linux X11/Wayland and later Windows as available.

Physical camera/printed-target acceptance, Linux GUI behavior and Windows have
not been verified by the automated tests. Physical acceptance is tracked separately
in [#179](https://github.com/Krzys2002/Mini_Robot/issues/179). Record actual physical
outcomes there; do not infer them from passing deterministic tests.

OpenCV reference: [chessboard detector documentation](https://docs.opencv.org/4.10.0/d9/d0c/group__calib3d.html).

## Verification record (2026-10-19)

On macOS with Apple Clang 21, Qt 6 and OpenCV 5.0.0:

- Desktop Debug configure/build: passed, including the production application.
- `ctest --test-dir build/issue-53 --output-on-failure --no-tests=error`:
  **187/187 passed**, including 20 new calibration tests and platform-boundary checks.
- Qt/OpenCV-free configure/build with desktop, replay and standalone calibration
  disabled: passed. `ctest --test-dir build/issue-53-core --output-on-failure
  --no-tests=error`: **123/123 passed**.
- Standalone detector configure/build with desktop/replay disabled and calibration
  enabled: passed. `ctest --test-dir build/issue-53-headless -R '^Calibration'
  --output-on-failure --no-tests=error`: **12/12 passed**, without Qt.
- The offscreen wizard was rendered and inspected for controls, review selector,
  progress feedback and accepted-corner overlay. The UI regression test also
  verifies that accepted green corner markers reach the rendered preview.
- A detector-to-session test completes the default ten-view capture policy from
  varied generated board images. This is deterministic evidence, not a physical
  camera/printed-board acceptance result.

Physical target/camera capture, Linux X11/Wayland GUI checks, Windows runtime,
and an OpenCV 4 build were not run. The source/CMake compatibility branch for
OpenCV 4 remains for Linux CI to validate. Intrinsics, distortion and reprojection
error calculation remain #54; profile persistence remains #55.

## Live preview processing policy

Full detection is visibility-gated, with a 200 ms minimum start interval, one
in-flight job and no pending live-detection queue. Acquisition retains only the
newest input. Capture can wait behind that single full OpenCV call, while GUI
cancellation/shutdown invalidate results asynchronously. The synchronous OpenCV
call cannot be interrupted mid-call. The two workers drain before destruction.

`ICalibrationCornerTracker` exposes only frames and image points. Its OpenCV
adapter uses sparse pyramidal Lucas–Kanade flow in both directions, on the
preview worker. The provisional policy hides the whole grid if any point fails
status, finite/bounds checks, 1 px forward/backward consistency, 20-level patch
error, or movement above 40% of the minimum seed spacing. It rejects steps above
150 ms and estimates more than 500 ms of capture time after the full detection.
These conservative limits can hide valid rapid motion; repeating chessboard
texture can still fool optical flow. Tracked points are never saved observations.

Hold intentionally uses older detected coordinates over newer video and marks
them cyan. Track draws estimates only on the frame they were calculated for.
Matched advances pixels and full-detection points together, including the initial
sample. All modes use the same raw/corrected pixel-centre mapping. A mode change
invalidates old-mode detection, flow and remap completions without changing the
captured dataset or resuming a paused review. Only Track is disabled if an injected
configuration has no tracker. OpenCV's `video` component is required for desktop/
calibration builds; `core` remains independent of Qt/OpenCV.

## Performance and software verification for #242 (2026-11-10)

Baseline `11f73a2` called `ICalibrationDetector::detect` only from Capture, once per
explicit request; `receiveFrame` performed no chessboard detection. The separate
`VisionController` continuously runs AprilTag detection on its own worker, with
one active job and one replaceable newest pending frame. Its results cannot be
reused as chessboard corners. No continuous chessboard bottleneck existed.

Synthetic measurements used macOS 26.7.2 arm64, AppleClang 21, Qt 6.12.0, OpenCV
5.1.0 and Debug builds. Four detector calls per image, including the cold call,
measured the following full-resolution durations:

| Image | Complete board duration (ms) | Blank duration (ms) |
| --- | --- | --- |
| 640 × 480 | 8.95–13.32 | 6.21–6.63 |
| 1280 × 720 | 22.07–24.50 | 19.80–21.55 |
| 1920 × 1080 | 49.94–53.48 | 46.34–48.02 |

These measurements support a provisional 5 Hz full-detection cap; they are not a
hard processing deadline or representative of every real negative scene.

The final offscreen audit completed 27 scenarios: three preview modes × hidden/
raw/corrected display × VGA/720p/1080p. A synthetic board alternated by 2 px at a
requested 33 ms input interval, with the real AprilTag worker active. Each
scenario lasted roughly two seconds. All visible scenarios ran nine full
detections; hidden scenarios ran none. Hold/Matched ran no optical flow. No
sequence rewind occurred. Representative 1080p observations were:

| Mode / display | Video frames / inputs | Full detections | Flow calls / mean ms |
| --- | --- | --- | --- |
| Hold / raw | 61 / 61 | 9 | 0 / — |
| Track / raw | 60 / 60 | 9 | 67 / 2.75 |
| Track / corrected | 59 / 61 | 9 | 65 / 1.89 |
| Matched / raw | 9 / 60 | 9 | 0 / — |
| Matched / corrected | 10 / 61 | 9 | 0 / — |

Corrected startup may briefly present a raw image before a compatible synthetic
profile is loaded. A pending final corrected frame may be discarded on hide/
shutdown. Some frames receive both an estimate and a later correction, so overlay
updates and flow calls are not distinct-frame counts. GUI heartbeat observations
and earlier synthetic measurements showed no sustained GUI stall; they do not
establish native input/display latency limits. Rendering/remapping/flow can still
skip intermediate frames under load. This is not a guarantee of camera FPS on
arbitrary equipment.

Deterministic tests cover sample reuse, exact-frame full redetection on Capture,
quality/diversity rules, bounded work, current-frame flow and analytical
Brown–Conrady corner remapping. UI tests cover all three modes, switching while
flow is gated, review preservation, dialog reopen, camera/resolution/restart
invalidation, missing detections, cancellation and both-worker shutdown.

After local integration into `feat/242-live-calibration-overlays` at `8696dc9`:

- Root macOS build passed; CTest passed 475 tests, with one opt-in detector timing
  test skipped (476 discovered).
- Root core build/CTest passed 280 tests without Qt/OpenCV.
- The standalone `Mini_Robot/` macOS entry point was rebuilt for the same code
  revision and passed 475 tests with the same one opt-in skip.
- Initial simultaneous core/desktop runs encountered a failure and a skip in
  unchanged `V4l2IdentityTest` cases. Separate full reruns passed. The intermittent
  cause was not established; no camera-identity source was changed.
- `git diff --check` passed. Unrelated IDE edits were excluded from commits.

Commands executed from the configured repository root:

```sh
cmake --build --preset macos --parallel 3
ctest --preset macos --parallel 3
cmake --build --preset core --parallel 3
ctest --preset core --parallel 3
```

Run the desktop/core suites separately. Existing GoogleTest source was supplied
through `FETCHCONTENT_SOURCE_DIR_GOOGLETEST` when configuring the local presets.
The standalone `Mini_Robot/` entry point uses separate build directories and the
same presets. Reproduce the explicit audit from the repository root:

```sh
cmake --build build/macos --target micro_robot_calibration_audit --parallel 3
QT_QPA_PLATFORM=offscreen build/macos/Mini_Robot/tests/micro_robot_calibration_audit
MICRO_ROBOT_CALIBRATION_AUDIT=1 build/macos/Mini_Robot/tests/micro_robot_calibration_detector_tests --gtest_filter=CalibrationDetectorTest.PerformanceAudit
```

The audit executable is excluded from the default build. The timed detector test
is opt-in and skipped by normal CTest; it asserts correctness without a
machine-dependent timing threshold. Runtime debug logs record full-detection
frame sequence/duration and `queued_live=0`; flow and AprilTag workers independently
record their processing durations.

Physical camera/native-display checks, real focus/blur and rapid board motion,
Linux X11/Wayland, Windows and OpenCV 4 execution remain **NOT RUN** for #242.
Real acceptance belongs to [#179](https://github.com/Krzys2002/Mini_Robot/issues/179).
Record setup, revision, preview/pixel mode, screenshots and PASS/FAIL/NOT RUN per
scenario. Existing physical evidence for older revisions does not validate this
change. No optical accuracy, physical motor safety or complete motion loop is
claimed. Intermediate experiment details remain available in Git history.

## Automatic review validation (#243)

Deterministic `CalibrationReviewTest` cases inject a monotonic clock and deliver
Qt timeout events without waiting five seconds. They cover accepted/rejected/
throwing-detector results, the 4999/5000 ms boundary, replacing an interval, queued
old timeouts, explicit review/live during detection, invalidation, missing input,
shutdown/destruction, retained UI feedback, and delayed corrected presentation.
Physical camera and native-display acceptance belongs to #179 and is **NOT RUN**
for this change. Software timing tests do not establish native rendering latency.

Verification executed on macOS arm64 with AppleClang 21, Qt 6.12.0 and OpenCV 5.1.0:

- Root `cmake --preset macos` using an existing GoogleTest checkout, followed by
  `cmake --build --preset macos --parallel 3` and
  `ctest --preset macos --parallel 3`: **484 passed, 1 opt-in performance audit
  skipped**, zero failures (485 discovered, including nine review tests).
- Root `cmake --preset core` using the same GoogleTest source, followed by
  `cmake --build --preset core --parallel 3` and
  `ctest --preset core --parallel 3`: **280 passed** without Qt/OpenCV.
- `git diff --check`: passed. Initial review-fixture observer lifetime failures
  were corrected before the final full passing run.

No root/standalone build integration changed; standalone execution was not
repeated for #243. Physical camera/native display, Linux/Windows and OpenCV 4
execution remain **NOT RUN**. This is software evidence only.

## Guidance validation (#244, 2026-11-11)

The #244 child was tested on approved PR #253 head
`5eb25a90ad3bac0a87ba6901b6f149967a08b390` (itself based on #252).
On macOS arm64, AppleClang 21, Qt 6.12.0 and OpenCV 5.1.0:

- Root `cmake --preset core` with the existing GoogleTest checkout, then
  `cmake --build --preset core --parallel 3` and
  `ctest --preset core --parallel 3`: **285 passed**, without Qt/OpenCV.
- Root `cmake --preset macos` with that checkout, then
  `cmake --build --preset macos --parallel 3` and
  `ctest --preset macos --parallel 3`: **492 passed, 1 opt-in performance audit
  skipped**, zero failures (493 discovered).
- Five new `CalibrationGuidanceTest` cases cover accepted-only region coverage,
  rejected/removal/reset transitions, scale/angle priorities, count/readiness and
  view-limit replacement. Three `CalibrationGuidanceUiTest` cases cover hints,
  help, retained rejection feedback and small-window scrolling/fixed actions.
- `QT_QPA_PLATFORM=offscreen QT_SCALE_FACTOR=1.5` and `=2` runs of
  `micro_robot_tests --gtest_filter='CalibrationGuidanceUiTest.*'`: **3/3 passed
  at each scale**. Rendered 520 × 480 logical-pixel help/primary-action views were
  visually inspected at both scales. Qt printed a font-alias startup notice;
  it did not affect the passing checks.
- `git diff --check`: passed. No build integration changed; standalone execution
  was not repeated for #244. The task worktree had no usable graph, so source
  and tests were inspected directly.

Physical camera/printed target, native-display behavior, Linux/Windows and
OpenCV 4 execution: **NOT RUN**. The placement/rotation guidance follow-up in
[#179](https://github.com/Krzys2002/Mini_Robot/issues/179) owns setup and per-scenario
physical evidence, separate from the software acceptance above. Earlier #179
results do not establish acceptance of this revision's guidance.
