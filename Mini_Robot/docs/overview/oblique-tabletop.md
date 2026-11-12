# Side-mounted planar tabletop calibration

[#247](https://github.com/Krzys2002/Mini_Robot/issues/247) audits the implemented
workflow for side views. A projective homography supports an oblique **plane**;
that fact and synthetic tests do not establish physical camera acceptance.
[The tabletop procedure](tabletop-homography.md) still requires measured references,
compatible intrinsics, explicit physical axes and operator review before activation.

## Source audit and demonstrated defect

| Boundary | Finding and software evidence |
| --- | --- |
| Point acquisition (`WorkspaceCalibrationDialog.cpp`) | Clicks undo display letterboxing/scaling to native undistorted pixel centres. Paired physical labels determine axes; image/world perimeter winding may differ. There is no overhead-only screen-axis sort. Use visible references, not guessed occluded corners. |
| Undistortion (`OpenCvCalibrationUndistorter.cpp`, `OpenCvMetricObservation.cpp`) | Frozen images and marker corners use the exact bound K at native resolution, without crop/resize/new K. The lens mapping does not assume a plane orientation. Strong perspective plus radial/tangential distortion is tested at near/far points. |
| Estimation (`OpenCvWorkspaceHomographySolver.cpp`) | Homogeneous DLT/SVD and all-point projective refinement retain the projective denominator; they do not substitute an affine map or global pixels/metre scale. Exact four-point and overdetermined oblique recovery are tested. The existing singular-value-ratio rejection now gives recovery advice. |
| Bounds (`Workspace.cpp`, `TabletopHomography.cpp`) | Bounds are physical world extents, independent of camera screen directions. The integrated #246 frame uses explicit signed extents and an editable origin. Hull area and membership diagnostics use normalized world differences and remain translation invariant. |
| Projection (`TabletopHomography.cpp`) | H is normalized before projection. Singular/horizon points and horizons crossing the unordered fitting hull are rejected. That hull does not establish accuracy outside its measured region. |
| Overlays (`WorkspaceCalibrationDialog.cpp`, `VisionDebugView.cpp`) | Native pixels are placed through aspect-preserving viewport conversion; image +v is down. The world convention is a legend, not a claim that world +Y always points upward in the camera image. Offscreen picking/alignment tests are software evidence only. |
| Metric consumer (`MetricObservation.cpp`) | Centre uses the projective diagonal intersection; heading uses transformed marker corners. **Defect fixed:** the horizon-side check previously used unnormalised H while projection used normalised H. An equivalent finite H scaled by `1.7e308` overflowed the former check and discarded a valid metric tag. The regression failed before the fix and passes with shared homogeneous normalization semantics. This was a numerical defect, not a reproduced physical side-camera failure. |

No additional overhead-only algorithm defect was demonstrated in this audit.
Camera placement, physical correspondence correctness, lens-model quality and
native platform behavior still require separate measurements.

## Coverage and recovery feedback

A solve now reports the convex fitting hull as a fraction of the declared table
area and native image area, plus the number of Check references outside either
the image or world fitting hull. Checks remain held out of estimation; their
residuals and extrapolation status are separate facts. A zero synthetic residual
outside the hull does not establish extrapolation accuracy. No accuracy threshold
is invented, and coverage fractions are not confidence or uncertainty estimates.

A side-view table can occupy a small image area while its physical references
cover the entire table. Inspect **workspace coverage**, near/far visibility and
independent residuals together; do not reject valid oblique geometry solely for
small image coverage. Clustered references give limited support: spread measured
references across both axes, near/far edges and interior, and add independent
checks. Parts hidden by table edges, obstacles or tags have no observed reference;
do not guess their image locations.

Degenerate/nearly collinear fitting sets, ill-conditioned SVD and horizons crossing
the fitting region have actionable rejection messages. Raise or reposition an
almost edge-on camera, improve near/far separation in the image and verify point
pairing/plane consistency. More samples on one narrow line do not fix geometry.
A stable matrix determinant alone is not an accuracy guarantee. Legacy profiles without acquisition records cannot restore fitting coverage;
independent rechecks remain necessary. The integrated [#248 acquisition records](workspace-chessboard.md)
retain references from which support can be derived. The separate [#249 diagnostics](workspace-diagnostics.md)
define a conditional spatial uncertainty model; coverage fractions alone remain
unrelated to confidence or accuracy.

## Plane, motion and height limits

The table must be flat. Keep camera, table, focus/zoom, resolution and lens binding
fixed after calibration. Moving either camera or table changes the mapping even
when profile metadata still matches. Recalibrate/recheck and explicitly activate
a replacement. Undistortion removes the modelled lens distortion, not parallax.

H maps an image ray to its intersection with the calibrated table plane. A raised
object or tag is generally projected to a different table position. In the test
camera at `(0.75,-1,0.5)` metres, the point `(0.3,1)` metres at 50 mm above the table
maps to approximately `(0.25,1.2222)` m in the plane. The deterministic regression
checks the independently derived ray/plane intersection and an error above 0.2 m.
Actual errors depend on camera pose, height and point location; no universal
correction or elevated-tag accuracy is supplied. This is not arbitrary 3D
reconstruction. Mounting height and marker-plane offset must be recorded for
physical metric-pose acceptance (#224).

## Deterministic construction and physical evidence

`tests/calibration/ObliqueWorkspaceFixtures.hpp` defines an independent pinhole
world→image construction: camera `(0.75,-1,0.5)` m looks along world +Y, table is
1.5 × 2.4 m, native image is 1000 × 800, and K has fx=400, fy=410, cx=500, cy=300.
Near/far depths are 1/3.4 m, giving a 3.4× horizontal scale ratio. Fitting corners
are separate from held-out near/far edge, side-edge and interior positions.
Tests recover that plane, preserve physical axis signs and metric tag headings,
exercise distortion, reject edge-on/horizon cases and expose height parallax.
They use exact synthetic geometry, not measured camera accuracy.

Run core and host presets, including `TabletopObliqueTest`,
`OpenCvWorkspaceHomographyObliqueTest`, `MetricObservationObliqueTest`,
`OpenCvMetricObservationObliqueTest` and `WorkspaceObliqueUiTest`.
Physical side-camera/native-display checks are owned by
[#261](https://github.com/Krzys2002/Mini_Robot/issues/261) and are **NOT RUN**.
That issue requires pose/setup, intrinsics/resolution, raw correspondences,
independent surveyed references, residuals, uncertainty, revisions and explicit
PASS/FAIL/NOT RUN evidence. #259 retains guided-corner UI commissioning, #224
retains physical metric tags, and #63 retains benchmarking.
