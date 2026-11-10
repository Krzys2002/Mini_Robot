# Intrinsic calibration target capture

Issue #53 implements observation capture. Issue #54 adds the
[intrinsics solver and quality reporting](intrinsic-calibration.md).
Calibration-profile persistence (#55) and tabletop homography (#57) remain separate.
No AprilTag identity or WorldModel updates occur during this workflow.

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
quality/diversity rules without adding an observation. **Cyan dots are held or tracked
corners**, without a fresh assessment of the current image. The yellow ring identifies the
first corner. Visually
check alignment with the printed intersections. Use the accepted-view selector
to inspect any stored view. **Capture view** selects the displayed sample (raw
source pixels even in corrected mode), reusing a full detection only when it
belongs to that exact frame. Held corners are never saved for a newer image: Capture runs a full-resolution
detection on its selected frame unless that exact frame already has a full result.
All existing quality checks remain. Neither holding nor tracking the overlay adds observations.
After capture/review, click **Live view** before selecting another view. A sample
older than two seconds is rejected; wait for a fresh sample. Integer image
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
There is no disk export; see the intrinsic-calibration guide for result handling.

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
| Input freshness | Latest frame no more than 2 s old at capture request |

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
cmake --build build/issue-53 --parallel 3
ctest --test-dir build/issue-53 -R '^Calibration' --output-on-failure --no-tests=error
ctest --test-dir build/issue-53 --output-on-failure --no-tests=error
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

## Live overlay audit for #242 (2026-11-10)

Baseline `11f73a2` calls `ICalibrationDetector::detect` only from Capture, once per
explicit request; `receiveFrame` does no chessboard detection. The separate
`VisionController` continuously runs AprilTag detection on its own worker, with
one active job and one replaceable newest pending frame. These results cannot
be reused as chessboard corners. No continuous chessboard bottleneck existed.

Synthetic measurements on macOS 26.7.2 arm64, AppleClang 21, Qt 6.12.0 and OpenCV
5.1.0 (Debug) support a provisional 200 ms minimum live start interval. A run of
four detections per image, including the cold first call, measured:

| Image | Complete board duration (ms) | Blank duration (ms) |
| --- | --- | --- |
| 640 × 480 | 8.95–13.32 | 6.21–6.63 |
| 1280 × 720 | 22.07–24.50 | 19.80–21.55 |
| 1920 × 1080 | 49.94–53.48 | 46.34–48.02 |

An offscreen wizard audit then drove synthetic board frames at a requested 33 ms
interval while the real AprilTag worker remained active. Each scenario lasted
roughly two seconds. The visible wizard painted actual paired samples; a GUI
heartbeat requested every 5 ms measured event-loop scheduling gaps:

| Image / wizard | Input frames | Chessboard calls / mean / max ms | AprilTag calls / mean / max ms | Max GUI gap ms |
| --- | --- | --- | --- | --- |
| 640 × 480 hidden | 65 | 0 / — / — | 65 / 3.90 / 6.78 | 6.72 |
| 640 × 480 visible | 60 | 9 / 20.70 / 30.34 | 60 / 3.86 / 12.47 | 7.46 |
| 1280 × 720 hidden | 62 | 0 / — / — | 62 / 4.71 / 8.74 | 7.83 |
| 1280 × 720 visible | 61 | 9 / 36.45 / 41.45 | 61 / 3.62 / 12.80 | 6.48 |
| 1920 × 1080 hidden | 61 | 0 / — / — | 61 / 5.84 / 7.92 | 7.65 |
| 1920 × 1080 visible | 61 | 9 / 65.90 / 73.98 | 61 / 5.15 / 12.75 | 6.35 |

There was no observed GUI stall in this short workload. Detection cost grows with
resolution, so running it for every incoming frame would waste processing time.
The cap bounds demand without changing full-resolution capture/quality logic.
Live work has zero pending jobs; a gated-worker regression feeds 100 frames and
verifies only one detection starts. Another test queues one capture behind that
job and verifies exactly two total calls and the selected frame. Solving/profile
I/O/remapping inhibit new live dispatch. Slower OpenCV calls reduce sampling rate;
the cap is not a hard detector deadline or a promise of native input latency.

Reproduce from the repository root after configuring `macos`:

```sh
cmake --build build/macos --target micro_robot_calibration_audit --parallel 3
QT_QPA_PLATFORM=offscreen build/macos/Mini_Robot/tests/micro_robot_calibration_audit
MICRO_ROBOT_CALIBRATION_AUDIT=1 build/macos/Mini_Robot/tests/micro_robot_calibration_detector_tests --gtest_filter=CalibrationDetectorTest.PerformanceAudit
```

The audit executable is excluded from the default build. The timed detector test
is opt-in and skipped by normal CTest; it has correctness assertions and no
machine-dependent timing threshold. Runtime debug logs record live frame sequence,
detection duration and `queued_live=0`. AprilTag logs independently record frame
sequence and processing duration.

Validation on the implementation branch: root core configure/build/CTest passed
280/280; root macOS configure/build/CTest had 458 passes, zero failures and one
opt-in audit skip (459 discovered); standalone Mini_Robot core configure/build/
CTest passed 280/280. Deterministic tests cover sample/result reuse, exact selected
capture while a worker is occupied, camera/resolution/restart invalidation,
hidden/destructed dialogs, GUI event processing during gated detection, shutdown,
and analytical Brown–Conrady corner remapping. This extends the earlier software
record without replacing its physical evidence.

Physical-camera live alignment/performance: **NOT RUN**. Native macOS display,
Linux X11/Wayland, Windows and OpenCV 4 execution: **NOT RUN**. Synthetic board and
blank images do not represent every real negative scene, blur, focus or lens.
Real capture/live evidence belongs to [#179](https://github.com/Krzys2002/Mini_Robot/issues/179),
including setup, revision, raw/corrected mode, GUI responsiveness and shutdown.
No optical accuracy, bounded physical stopping or complete motion loop is claimed.

## Camera-rate tracking experiment for #242

Historical measurements for the first optical-flow experiment. Tracking is now
available alongside Hold through the selector; see the current mode notes below.
It was requested on branch `feat/242-camera-rate-tracked-preview`, based on working commit `c0b4031` on
`feat/242-live-calibration-overlays`. The working branch is preserved. The earlier
5 Hz paired-sample audit above records that baseline; this branch replaces its
held-image behavior with camera-rate raw video and worker-side tracking.

`ICalibrationCornerTracker` exposes only frames and image points, without Qt or
OpenCV types. `OpenCvCalibrationCornerTracker` uses sparse pyramidal Lucas–Kanade
flow in both directions. Its provisional defaults hide the entire grid if any
point fails status, finite/bounds checks, 1 px forward/backward consistency,
20-level patch error, or movement greater than 40% of the minimum seed spacing.
Frame-to-frame gaps above 150 ms are rejected; controller estimates cannot extend
beyond 500 ms of capture time since a full detection. These conservative limits
can hide valid overlays during rapid motion; they are not measured physical
accuracy guarantees. Repeating chessboard texture can still fool optical flow.
A missing/failed full detection or failed flow removes the tracking anchor until
a new full detection arrives. Every input clears old points before presenting
new raw pixels. Worker completions may add points only to that exact current
frame; old results may advance the internal anchor but never rewind video.

Cyan estimates are presentation-only. A full detection on the selected raw frame
is required before accepting a capture; its measured sharpness and full existing
quality/diversity checks remain authoritative. Review/capture pauses live updates.
Camera/resolution changes, cancellation, restart and hidden/destroyed dialogs
invalidate both full-detection and flow work. Shutdown emits `stopped()` once,
after both workers finish. OpenCV calls themselves remain non-interruptible.

The audit executable now drives a board alternating by 2 px between frames at a
requested 33 ms interval, with real AprilTag processing active. It tests hidden,
raw-visible and undistorted-visible modes at three resolutions, about two seconds
each. Undistortion uses a synthetic compatible profile, not an optical calibration.
Measurements on the same macOS/Qt/OpenCV setup as above:

| Image / mode | Video frames / inputs | Distinct frames with tracked overlay | Flow mean / max ms |
| --- | --- | --- | --- |
| 640 × 480 raw | 59 / 59 | 58 | 0.86 / 1.15 |
| 640 × 480 corrected | 62 / 62 | 60 | 0.85 / 1.39 |
| 1280 × 720 raw | 61 / 61 | 60 | 2.16 / 4.78 |
| 1280 × 720 corrected | 61 / 61 | 59 | 1.51 / 2.25 |
| 1920 × 1080 raw | 60 / 60 | 58 | 3.03 / 4.43 |
| 1920 × 1080 corrected | 60 / 61 | 56 | 1.87 / 3.10 |

Every visible scenario ran nine full detections; hidden scenarios ran neither
chessboard detection nor flow. No sequence rewind occurred. A pending final
corrected frame can be discarded when the audit hides/stops the wizard. Some
frames receive both an estimate and a later correction; update count is distinct
from the number of camera frames. The largest 5 ms heartbeat gap was 11.73 ms.
These are short offscreen observations, not native input/display latency limits.
The initial experiment remapped some corrected frames twice, reducing 1080p
overlay coverage; tracking before remapping removed that avoidable duplicate work.

The audit invocation is unchanged from the preceding section. The experiment adds
OpenCV's `video` component only to calibration/desktop builds; `core` still avoids
Qt/OpenCV. Deterministic additions cover translations in padded Gray/RGB/RGBA,
occlusion/ambiguous motion, invalid inputs/policy, full-rate raw presentation,
estimate expiry, exact-frame full redetection on Capture, capture while flow is
blocked, flow failure, source/restart/teardown invalidation and both-worker drain.

Physical camera/native display checks, fast real board movement, real focus/blur,
Linux/Windows and OpenCV 4 execution remain **NOT RUN**. Existing #179 physical
acceptance is not marked complete by this experiment. At that stage, the experimental branch was
not published or substituted for the working branch.

Experiment validation: root core and standalone core configure/build/CTest each
passed 280 tests. Root macOS and standalone Mini_Robot macOS configure/build/CTest
each had 467 passes, zero failures and one opt-in detector-audit skip (468 tests
discovered). The nine-scenario moving-board audit ran explicitly and completed.
Both entry points used the existing GoogleTest source checkout through
`FETCHCONTENT_SOURCE_DIR_GOOGLETEST`, without a dependency download. Commands:

```sh
cmake --preset core -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=/path/to/existing/googletest
cmake --build --preset core --parallel 3
ctest --preset core --parallel 3
cmake --preset macos -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=/path/to/existing/googletest
cmake --build --preset macos --parallel 3
ctest --preset macos --parallel 3
# Run the macOS sequence again from Mini_Robot/ for the standalone entry point.
# Standalone core was also checked with a separate explicit build directory:
cmake -S Mini_Robot -B build/standalone-core -G Ninja -DMICRO_ROBOT_BUILD_DESKTOP=OFF -DMICRO_ROBOT_BUILD_REPLAY=OFF -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=/path/to/existing/googletest
cmake --build build/standalone-core --parallel 3
ctest --test-dir build/standalone-core --parallel 3 --output-on-failure
```

## Held-overlay revision

This historical revision (commit `3a5705d`) kept the last full detection's
corner positions fixed until the next detection completes. Optical flow and its
extra OpenCV `video` dependency have been removed. The preceding optical-flow
section is historical evidence for commit `2547c66`, not current behavior.

Video continues on every incoming frame. A delayed detection replaces the overlay
over the newest image without rewinding video. A failed/empty detection clears
it. Camera/resolution changes, restart/cancel, review/live transitions and dialog
hide/teardown also clear it. Cyan identifies an overlay held from an older frame.
In undistorted mode, held raw coordinates are transformed using the same profile
as the current image. Their positions stay fixed for a fixed profile and preview
mode. This deliberately relaxes #242's current-frame preview requirement on this
experimental branch: moving the board can make the displayed corners misalign
until detection catches up. The original paired-frame branch at `c0b4031` remains
unchanged. This display choice does not weaken capture: only a full detection of
the selected raw frame can supply saved points and quality checks.

The nine-scenario moving-board audit completed with zero sequence rewinds and
nine full detections in each roughly two-second visible scenario. Raw video
presented every input frame: VGA 59/59, 720p 61/61, 1080p 61/61. Corrected video
presented 62/62, 61/61 and 60/61 respectively. The pending final corrected frame
may be discarded when hiding/stopping. Largest GUI heartbeat gap: 12.49 ms.
These synthetic measurements do not establish physical/native-display behavior.

Root and standalone macOS builds/tests each passed 462 tests with one opt-in
audit skipped (463 discovered). Root core passed 280 tests on the final rerun.
An initial core run failed the unchanged camera-identity test
`V4l2IdentityTest.RejectsMissingSerialAndDuplicatePhysicalIdentity`; the isolated
four-test identity suite and subsequent full core run passed. Its transient cause
was not established; no unrelated camera-identity source was changed.
Validation used the same configure/build/CTest and explicit audit commands listed
above. Four held-overlay UI tests cover fixed coordinates while video advances,
replacement by a delayed result, clearing on missing detection/source change,
exact-frame capture instead of cached points, and both-worker shutdown.
Physical camera/native display, Linux/Windows and OpenCV 4 remain **NOT RUN**.

## Selectable overlay modes

At commit `3f606b7`, the branch combined held and optical-flow versions. **Live overlay**
in the calibration wizard selects **Hold until next detection** (default) or
**Track between detections**. Both use camera-rate raw video, bounded work and
full detection at up to 5 Hz. Hold intentionally allows older corner positions
on newer images; Track uses the guarded optical-flow policy recorded above and
hides obsolete/uncertain estimates. Cyan denotes held or tracked points. Capture
continues to require a full detection and quality assessment of its selected raw
frame in either mode. Switching does not change stored observations or resume a
paused review. The preference survives dialog close/reopen in this application
session, but is not saved into profiles or application settings.

Mode changes invalidate old-mode full-detection, tracking and remap completions.
The production composition injects the tracker; the selector is disabled for
injected configurations without tracking support. OpenCV's `video` dependency is
again enabled for calibration/desktop builds; core remains Qt/OpenCV-free.

Root core passed 280 tests. Root and standalone macOS builds/tests each passed
473 tests with one opt-in detector-audit skip (474 discovered). Tests cover both
modes, switching through the UI, switching while flow is gated, preserving the
captured dataset/paused review and retaining the selection across dialog reopen.
The expanded offscreen audit completed 18 moving-board scenarios: both overlay
modes, hidden/raw/corrected display, VGA/720p/1080p and concurrent AprilTag work.
Every visible scenario ran nine full detections in about two seconds. Hold ran
zero optical-flow calls; Track ran flow only while visible. There were no sequence
rewinds. At 1080p, Hold raw video delivered 61/61 frames and Track raw 60/60;
corrected mode delivered 60/61 in each case. Track flow averaged 1.14–2.76 ms in
visible scenarios. These timings are synthetic, not physical/native-display
acceptance. Physical camera, native macOS display, Linux/Windows and OpenCV 4
checks remain NOT RUN. Validation commands are the same as those recorded above;
original working branch `feat/242-live-calibration-overlays` remains at `c0b4031`.

## Three-mode integration into the main #242 branch

The selector now includes the original **Matched image and corners (up to 5 Hz)**
preview alongside Hold (default) and Track. Matched advances image and detected
corners as a pair and reuses that exact full result for Capture. Its initial image
also stays fixed while the first detection runs, preventing a delayed first result
from rewinding video. Hold and Matched remain available when no optical-flow
adapter is injected; only Track is disabled. Mode changes preserve paused review,
captured observations and profiles, and invalidate old-mode worker completions.

Before local integration, root macOS tests passed 475 cases with one opt-in audit
skip (476 discovered); root core passed 280. New cases exercise Matched through the
selector without a tracker, image/corner pairing while newer input arrives,
exact-sample Capture reuse, and switching from gated flow into Matched without
an obsolete estimate replacing the matched sample. The audit expanded to 27
scenarios (three preview modes × hidden/raw/corrected × three resolutions), all
completed with zero sequence rewinds. Visible Matched scenarios performed nine
full detections in roughly two seconds and presented nine paired overlays; other
modes retained camera-rate raw video. No optical-flow calls ran in Hold/Matched.
The same audit/build commands recorded above apply. Local integration targets
`feat/242-live-calibration-overlays`, not repository `master`; no GitHub merge or
publication is implied. Physical/native-display and other-platform limitations
remain NOT RUN as recorded above.
