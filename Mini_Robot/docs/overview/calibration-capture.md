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
The visible dialog samples the current camera at up to 5 Hz, with one live
detection in flight and no queued live jobs. It holds each completed sample until
the next result is ready, keeping pixels and corners together. Click **Start /
restart capture** and **Capture view** while holding the board still and in focus.
Each usable, distinct view is accepted automatically.

Green dots show usable live samples or accepted captured inner corners. Red dots
show rejected detections when corners are available. Live evaluation uses the same
quality/diversity rules without adding an observation. The yellow ring identifies the first corner. Visually
check alignment with the printed intersections. Use the accepted-view selector
to inspect any stored view. **Capture view** selects the displayed sample (raw
source pixels even in corrected mode), reusing its detection when available.
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
  detector and a single detection worker. Acquisition retains only the newest
  frame; a separate paired sample/result supports live presentation and capture.
  No live work is dispatched while detection, solving, profile I/O or remapping
  is active. Capture stays enabled during live detection and may wait behind the
  one already running OpenCV call; no additional live calls can get ahead of it.
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
worker to quit, and the main event loop drains both capture and detector workers
before destruction. Normal destruction therefore does not wait on detection.

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
