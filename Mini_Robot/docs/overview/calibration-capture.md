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
for five seconds, then resumes live preview automatically. The interval begins at
presentation, including completion of corrected-preview processing; no sleep blocks
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
