# Intrinsic camera model and reprojection quality

Issue #54 adds solving to the [chessboard capture workflow](calibration-capture.md).
After collecting diverse accepted views, click **Calculate intrinsics**. The
existing worker calculates a model without blocking camera preview or the GUI.
Tilt the board out of plane as well as varying position, distance and rotation:
the capture diversity gates alone do not prove the calibration is well conditioned.

The dialog shows `fx`, `fy`, `cx`, `cy` in pixels and aggregate reprojection RMS.
Each accepted-view selector entry shows its frame sequence and individual RMS;
select it to inspect the corresponding corner overlay. The displayed warning
threshold defaults to **1.00 pixel**. Aggregate RMS or any individual RMS strictly
above this threshold produces an advisory warning. Inspect high-error views and
restart capture with a better dataset. A warning does not discard a finite,
otherwise valid model. No automatic outlier removal occurs.

Calculation failure or invalid output produces a rejection message and exposes
no model. Retry calculation or restart capture. Closing a completed dialog keeps
its observations/result in memory. Restart, cancellation, camera stop/loss,
camera identity change or resolution change clears the result. Application exit
also discards the capture result, but explicitly saved
[calibration profiles](calibration-profiles.md) survive restart. The profile guide
covers exact camera/resolution selection and the optional undistorted verification
preview. Workspace mapping remains separate.

## Model and API

`IntrinsicCalibration.hpp` is part of the Qt/OpenCV-free
`micro_robot_calibration` library. It defines owned, copyable values for services:

- `IntrinsicCalibrationInput`: camera identity, resolution, target and corner-only
  observations, without retained image buffers. `intrinsicCalibrationInput()`
  snapshots a completed `CalibrationCapture`.
- `IIntrinsicCalibrationSolver`: synchronous worker-only solver interface.
- `IntrinsicCalibration`: row-major 3 × 3 matrix
  `[fx,0,cx; 0,fy,cy; 0,0,1]`, five dimensionless distortion coefficients in
  **`k1,k2,p1,p2,k3`** order, aggregate error and per-frame errors/counts.
- `validateIntrinsicCalibration()`: public model/result validation for consumers,
  including future profile loading. Empty string means valid; otherwise it
  returns an explanatory error. This is not profile serialization.

`OpenCvIntrinsicCalibrationSolver` in `micro_robot_calibration_opencv` uses
`calibrateCamera` with the standard five-coefficient Brown-Conrady pinhole model.
Fisheye, rational, thin-prism and tilted-sensor models are unsupported. Extrinsic
poses are temporary solver values; target coordinates use metres with the origin
at the first inner corner, X along columns, Y along rows and Z = 0. They do not
define the tabletop world frame.

Each frame's RMS is `sqrt(sum(du² + dv²) / corner_count)`. Aggregate RMS is the
same calculation across all corners in all frames, equivalently the point-weighted
quadratic mean of per-frame RMS. It is not the average of per-frame errors or
error per image axis. Errors are recomputed with `projectPoints` from fitted
intrinsics, distortion and temporary board poses. See the
[OpenCV camera calibration API](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html).

`CalibrationController` injects the solver at composition, snapshots corners on
the GUI thread, runs one job on its existing worker, and validates the returned
model and its camera/frame metadata before publishing it via `changed()`.
`result()` exposes the optional validated value on the GUI thread; copy it before
handing it to another service/worker. A generation token discards stale results
after restart/cancel/input changes. Shutdown drains an in-flight solver while
the event loop remains active.

## Validation policy

`IntrinsicCalibrationConfig` centralizes injectable thresholds. Defaults:

| Check | Policy |
| --- | --- |
| Camera metadata | Nonempty identity, positive width/height |
| Matrix | All finite; canonical zero skew and homogeneous row |
| Focal lengths | `fx/width`, `fy/height` each in [0.1, 10] |
| Focal aspect | `max(fx/fy, fy/fx)` ≤ 2 |
| Principal point | Inside calibrated image bounds |
| Distortion | Exactly five finite coefficients, each absolute value ≤ 10 |
| Error metadata | At least three uniquely identified views, ≥ 4 points each, finite nonnegative RMS |
| Aggregate | Matches weighted per-view RMS within relative/absolute 1e-6 |
| Quality warning | Any per-view or aggregate RMS > 1 pixel |

The solver accepts 3–24 complete grids at the capture workflow's supported
resolution (≤ 4 megapixels, each dimension ≤ 4096). It rejects invalid targets,
non-finite/out-of-image corners, repeated frame IDs and collapsed/collinear board
geometry before calling OpenCV. OpenCV exceptions become failed outcomes.
These plausibility checks are practical bounds, not a proof of observability,
lens invertibility or physical accuracy. A low fit error alone does not certify
the lens model. Validate against independent physical observations before use.

## Verification

```sh
cmake -S . -B build/issue-54 -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build/issue-54 --parallel 3
ctest --test-dir build/issue-54 --output-on-failure --no-tests=error
```

Set `CMAKE_PREFIX_PATH` to the local Qt installation if needed. For offline
GoogleTest sources use `FETCHCONTENT_SOURCE_DIR_GOOGLETEST` as described in the
[test guide](../development/testing.md).

Pure tests check malformed matrices/distortion/errors, configurable thresholds,
warning boundaries, weighted error consistency and owned capture snapshots.
Solver tests analytically project a known camera through varied tilted board
poses without OpenCV projection helpers, recover all five distortion terms and
intrinsics, and identify a deliberately noisy frame. Offscreen UI tests check
worker execution, quality text, per-view errors, failures, invalid returned
models, bounded work and cancellation/source changes/shutdown while solving.

Manual/hardware checks still required: use a measured flat printed target with a
physical camera, capture varied tilts, inspect RMS and overlays, recapture after
an intentionally poor dataset and compare independent image points. No physical
camera accuracy is claimed. Linux X11/Wayland, Windows runtime and an OpenCV 4
build must be verified in their respective environments.

### Verification record (2026-10-20)

On macOS with Apple Clang 21, Qt 6 and OpenCV 5.0.0:

- Desktop Debug configure/build in `build/issue-54`: passed.
- `ctest --test-dir build/issue-54 --output-on-failure --no-tests=error`:
  **200/200 passed**.
- Qt/OpenCV-free configure/build in `build/issue-54-core` (desktop, replay and
  standalone calibration OFF): passed; unfiltered CTest **130/130 passed**.
- Standalone configure/build in `build/issue-54-headless` (desktop/replay OFF,
  calibration ON): passed; `ctest --test-dir build/issue-54-headless -L calibration
  --output-on-failure --no-tests=error`: **20/20 passed** without Qt.
- Offscreen completed-calibration dialog rendered and inspected: aggregate RMS,
  threshold, focal/principal values, per-view error selector and corners visible.
  A UI test also checks that the settled quality label fits its allocated height.
- Physical camera/printed-board accuracy, Linux GUI, Windows runtime and OpenCV 4
  were not run; the manual procedure above records the remaining checks.
