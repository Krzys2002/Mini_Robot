# Workspace scale, error maps and conditional uncertainty

After explicitly activating a tabletop transform, open **Scale / error maps…**
from tabletop calibration. The diagram is a sampled world-coordinate map with
+X right and +Y up, including signed coordinates from an interior origin. It is
not a rectified camera image. Choose world-X scale, world-Y scale, interpolated
survey error in mm, that error as a percentage of the workspace diagonal, or a
conditional pointwise prediction radius in mm. Hover for the actual sampled
coordinates and values. Grey cells mean unavailable; colours do not fill missing
or extrapolated regions. The legend reports the current field's units and range.

Green squares locate fitted references; yellow circles locate held-out Checks
whose independence has not been confirmed; white crosses locate explicitly
surveyed Checks. All reference positions are drawn, with at most 32 ID labels.
The virtual table retains all rows and shows fit/Check roles, metric coordinates,
absolute residuals, percentages and source placement/corner IDs. Fitted residuals
are diagnostics, not independent accuracy measurements. A held-out board corner
can still share spacing, anchor, orientation or image errors with fitting data.

## Evidence and operator procedure

1. Keep camera/lens/table fixed. Acquire and save a complete calibration and its
   linked [acquisition record](workspace-chessboard.md), including held-out Checks.
2. Independently survey the **stored world coordinates** of the Checks to be used
   as ground truth. Correct and reverify/recalculate the calibration acquisition
   beforehand when needed. Merely selecting a row does not replace generated
   board coordinates with independently measured coordinates.
3. Select eligible Check rows and confirm independence only when justified by the
   physical measurement procedure. This enables interpolation inside the surveyed
   world-reference hull. Fit rows cannot be selected. No outliers are silently
   removed; selection/method must be recorded without outcome-driven cherry-picking.
4. Prediction additionally requires at least six Checks with well-constrained
   two-dimensional coverage, explicit residual-model assumptions, a positive
   **additional per-axis standard uncertainty** (UI mm; core metres), and a method
   describing the survey, uncertainty budget and model justification. The floor
   is added to residual variance; it is not an inferred equipment specification
   or a substitute for a justified model. The implementation caps it at 1000 m.
5. Apply the evidence. Edits take effect on Apply; maps retain their last applied
   evidence until then. Settings are session-only and belong to this exact active
   calibration/acquisition. Reopening the dialog preserves applied settings;
   replacing or invalidating the active calibration clears them.
6. Export the current snapshot to JSON to retain samples, masks, observations,
   evidence and complete calibration identity. Keep separate physical measurements
   and actual model-validation results with the experiment record.

With legacy profiles or missing acquisition evidence, geometric scale may still
be shown inside the declared workspace/native image. Fit coverage is explicitly
unknown, no survey interpolation is available and prediction is unavailable.
An exact four-point fit without independent Checks never implies zero uncertainty.

## Geometry, errors and interpolation

`H` maps native-K undistorted pixel centres `(u,v)` to world metres `(x,y)`.
Let `G` be its inverse, with `D = g6*x + g7*y + g8`, and `u,v = G(x,y)` after
homogeneous division. The analytic inverse-map Jacobian is

```
J = [[g0-u*g6, g1-u*g7],
     [g3-v*g6, g4-v*g7]] / D
scaleX = norm(J column 0) / 100  [px/cm]
scaleY = norm(J column 1) / 100  [px/cm]
```

These are directional differential scales along the world axes, not a single
perspective-independent scalar or a finite 1 cm chord length. Homogeneous matrix
scale does not change them. Invalid/nonfinite/horizon mappings, native-image
exclusions and world-bound exclusions are unavailable. When acquisition exists,
geometric support additionally requires both the measured fit-world convex hull
and the fit-image convex hull. No fitting-hull extrapolation is represented as
supported. Optical distortion/display resizing does not enter these derivatives.

For a stored paired reference, `e = H(image) - surveyedWorld` in metres. Absolute
error is `norm(e)`. Relative percentage is `100*norm(e)/hypot(width,height)`, labelled
**% of workspace diagonal**. This denominator does not move with the origin. A
diagonal at or below 1e-9 m or a nonfinite calculation makes the percentage
unavailable rather than clamping it or dividing by a point coordinate near zero.

The absolute spatial estimate uses inverse-distance-squared interpolation of
confirmed survey error magnitudes in normalized world coordinates
`((x-minX)/width,(y-minY)/height)`. At a reference it uses that observed magnitude;
elsewhere it is an interpolation, not an actual measurement or uncertainty bound.
It is masked outside the confirmed surveyed-world hull and geometric support.

## Reusable prediction contract for distance reporting

`WorkspaceDiagnostics` is an immutable snapshot with complete profile, acquisition,
survey settings, observations, support hulls, model and sampled values. Core
`workspacePredictionRadius(snapshot, q, coverage)` returns an optional **metre
radius around the unchanged H coordinate q**. Absence is an explicit unavailable
result, not zero. No bias correction silently changes world coordinates.

The conditional model uses independently surveyed Check residuals as responses and
**H-mapped coordinates** as predictors. This permits a subsequent image endpoint
to query the model using its observed H coordinate, without knowing its true
location. For each axis independently, normalize `q` by the signed workspace
minimum/extents and set `phi(q) = [1, qxNormalized, qyNormalized]`. Then

```
B = inverse(Phi' Phi) Phi' E       # E columns are X/Y residuals
nu = numberOfChecks - 3
s_axis² = SSE_axis / nu + additionalStandardUncertaintyMeters²
h(q) = phi(q) inverse(Phi' Phi) phi(q)'
mu_axis(q) = phi(q) B_axis
axisRadius = abs(mu_axis(q)) + t(1-alpha/4, nu) * sqrt((1+h(q))*s_axis²)
radius = hypot(axisRadiusX, axisRadiusY)
```

The `1+h` term predicts an individual new residual rather than just the mean
field. The absolute mean term bounds error around the original H result. Student-t
quantiles are evaluated by bounded inversion of the incomplete beta CDF. The
3-parameter normalized design must be invertible, its Frobenius condition estimate
must be at most 1e8, and the mapped survey hull must occupy at least 1e-4 of the
workspace area. These numerical/coverage gates do not establish model adequacy.
Prediction is available only inside the **H-mapped survey hull** and geometric
support. That hull can differ slightly from the physical-reference hull used by
the interpolation map; both are exported.

For displayed pointwise coverage 0.95, `alpha=0.05`; two equal-tailed axis intervals
use Bonferroni, so their joint coverage is at least 95% **under the model**. X/Y
correlation does not require independent axes. This is not simultaneous coverage
of all grid cells, a worst-case physical bound, or proof that assumptions hold.
For #250 the API also supports per-endpoint coverage 0.975. The sum of the two
endpoint radii gives a conservative at-least-95% pair/distance bound by the union
inequality and triangle inequality, without assuming independent endpoint errors.
Endpoint-selection uncertainty must be accounted for separately by that consumer;
fit RMS or interpolated magnitudes are never substituted for missing prediction.

Assumptions are substantive: conditional on a fixed active H and fixed observed
predictors, independently surveyed residual observations must follow an adequately
affine mean field with independent Gaussian, constant-variance scatter. Cross-axis
correlation is allowed; correlation **between survey observations**, unmodelled
shared camera/target bias, uncertain predictors, non-affine spatial effects,
plane-height mismatch and changed conditions can invalidate coverage. Operators
must justify their procedure rather than tick a checkbox to obtain a number.
Positive variance inflation prevents an exact synthetic residual fit from yielding
a zero radius; it cannot make a false model physically valid.

The general individual-response prediction formula is documented in
[NIST's linear-regression example](https://pages.nist.gov/ml_uncertainty_py/toy_regression.html).
The axis and endpoint combinations use the
[NIST Bonferroni inequality](https://itl.nist.gov/div898/handbook/prc/section4/prc473.htm).
The distinction between uncertainty, correlation and coverage follows the
[BIPM JCGM 100 guide](https://www.bipm.org/documents/20126/2071204/JCGM_100_2008_E.pdf).
The chosen spatial model and gates are application design decisions, not a
physical calibration certificate supplied by those references.

## Bounded work, invalidation and export

The default grid is 41×25 (1025 nodes); both dimensions must be 2–65, giving at most
4225 nodes. References retain #248's 4096 Fit + 4096 Check limits. No rows/samples
are truncated. Validation, hulls, regression, interpolation, map sampling and disk
I/O execute on a serialized Qt worker, with one running job and one replaceable
latest map request. Rendering is bounded by these node/reference limits. No live
frame raster is queued for diagnostics.

The calibration controller owns the diagnostics service so closing/reopening the
map dialog does not lose applied evidence or spawn more workers. Active camera,
resolution, K/distortion, H, origin/frame or bounds replacement clears maps and
survey settings, fences late results and starts a fresh geometric map. Draft
calibration edits do not relabel the still-active calibration. Repeated requests
clear the displayed snapshot immediately; stale worker results cannot reinstall
old maps. Shutdown participates in the application's existing worker-stop checks.

Export schema `mini-robot-workspace-diagnostics-v1` is atomic JSON (`QSaveFile`, no
direct-write fallback), capped at 16 MiB. It contains canonical lossless profile
text, canonical acquisition text when available, full optics/camera/frame/bounds/H
identity, survey IDs as strings, method/assumptions/floor, observed pixels/world
and H-mapped coordinates/residuals/roles, all support hulls, fitted model and all
row-major sample values/masks/units. Null means unavailable. It records model and
coverage meaning; the file is an export, not a new calibration profile or automatic
settings importer. An export in progress retains its original immutable identity
if calibration changes meanwhile; it is not relabelled as current. Failures preserve
the previous destination file and report the error.

## Validation and limits

Deterministic tests cover affine/perspective Jacobians, extreme homogeneous scale,
signed-origin invariance, horizon/native/fit/survey masks, missing/collinear survey
evidence, positive floors, Student-t reference values, synthetic bias/noise,
conditional prediction with correlated axes, full dataset/grid capacity, worker
coalescing/invalidation, exceptions, export parsing and offscreen UI/model behavior.
Physical acceptance is owned by [#265](https://github.com/Krzys2002/Mini_Robot/issues/265);
all new physical-camera/native-display checks are NOT RUN.
Actual commands/results and native-display gaps are recorded in
[testing](../development/testing.md). Physical independent scale/error/model-coverage
acceptance remains separate from synthetic tests and from the broader
[#63 benchmark](https://github.com/Krzys2002/Mini_Robot/issues/63) and
[#224 metric pose acceptance](https://github.com/Krzys2002/Mini_Robot/issues/224).
No physical accuracy or general 95% coverage is established by this implementation.
