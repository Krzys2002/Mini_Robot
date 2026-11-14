# Two-point tabletop distance

With a compatible, explicitly active workspace calibration, open **Measure
distance…** from tabletop calibration. This is a planar calibration diagnostic;
it does not move robots, measure arbitrary 3D distance or correct elevated objects.

## Operator procedure

1. Keep camera, lens and table fixed. Select **Freeze measurement frame**. A fresh
   input is undistorted on the existing calibration worker into a separate native-K
   measurement image. This capture preserves calibration references, placements,
   acquisition records and draft results. The dialog identifies the camera,
   resolution, active frame identity and frozen sequence.
2. In **Frozen undistorted camera**, select A and B by clicking. The selected-endpoint
   control lets you replace either point; the first A pick advances to B. Cyan A,
   magenta B and their line appear on the image. Inspect mapped world coordinates
   in mm and the corresponding native undistorted pixels below each endpoint.
3. Edit mapped-world X/Y in mm and use **Apply coordinates**, or pick in the
   **World diagram (+Y up)**. The diagram uses the same signed world frame; it is
   not a rectified camera image. Switching between these two supported views
   preserves the exact world endpoints, distance and selection evidence. Numeric
   edits and new picks reset that endpoint's previous selection bound/justification.
4. The planar distance is displayed in cm and mm. **Clear** removes one endpoint;
   **Clear both** removes both. An accepted re-freeze request clears old picks
   before producing a new image. A rejected stale-frame request does not relabel
   an old frozen image as new.
5. Selection uncertainty initially means **unknown**. For each endpoint, enter a
   written justification, explicitly confirm a known maximum disk, choose native
   undistorted pixels or mapped-world millimetres, and enter its radius. These
   are deterministic bounds in the observed pixel/H-world domain, not standard
   deviations or an automatically assumed click precision. Include pointer/display
   quantization, feature selection and the actual procedure in the justification.
   Zero needs exact-endpoint evidence. Changing units clears the old bound and
   justification; it never silently reinterprets the number.
6. Calibration uncertainty comes from the active
   [workspace diagnostics model](workspace-diagnostics.md), including its explicitly
   confirmed independent survey evidence, positive additional standard uncertainty
   and justified conditional residual assumptions. Enable/validate that evidence
   through **Scale / error maps…**. If it is missing, or either selection bound is
   unknown/unjustified/unsupported, the distance remains visible with an explicit
   **uncertainty unavailable** reason. Fit RMS is never substituted for it.

Native input and world edits must have supported centres. Invalid, nonfinite,
horizon or out-of-domain picks/edits are rejected with feedback and the previous
endpoint remains unchanged. A supported centre whose *selection footprint* leaves
coverage can retain a distance but has unavailable uncertainty. Legacy profiles
without acquisition hulls use only declared bounds/native geometry, explicitly
label unknown fit coverage and cannot supply the independent survey model.
Numerical coordinate/display digits are not demonstrated physical accuracy.

## Pixel and frame conventions

The selectable camera image is **undistorted using the bound native camera K**,
with its original full dimensions, no crop or mirror. A raw lens-distorted selector
is not offered. Raw `(u,v)` must be undistorted before applying H; neither display
coordinates nor an enlarged/cropped image's pixel coordinates can be used directly.

For a centred letterboxed rectangle `(left,top,width,height)` displaying native
image dimensions `(W,H)`, logical view picks map to native pixel-centre coordinates:

```
u = (viewX-left)*W/width - 0.5
v = (viewY-top)*H/height - 0.5
viewX = left + (u+0.5)*width/W
viewY = top  + (v+0.5)*height/H
```

Device-pixel ratio is not applied a second time. Native values must lie in
`[0,W-1] × [0,H-1]`; letterboxing is not selectable. The boundary check admits only
32 machine-epsilon units scaled by native extent for arithmetic roundoff and
clamps only that roundoff, not real outside picks. World-diagram selection uses
continuous workspace fractions, with world +Y opposite the screen-down axis.

The controller retains one separate immutable measurement raster, with no frame
queue. Freeze admission uses the existing 2 s fresh-input limit and shared worker
busy gate. Exact source sequence/time/dimensions/pixel format/storage metadata are
checked on undistortion output. The optical/profile generation fences late work.
A camera, K/distortion, resolution or explicitly active H/frame/origin/bounds
replacement clears the measurement frame and endpoints. Merely editing or loading
a *draft* does not change a measurement of the still-active calibration. Shutdown
uses the existing calibration worker lifecycle; no distance worker is added.

## Distance and selection propagation

`WorkspaceDistance.hpp/.cpp` has no Qt, OpenCV or hardware dependency. It accepts
two observed world endpoints and selection evidence, and returns either an
invalid-measurement error or distance with optional uncertainty and a reason.
Internal distances and world radii are metres; pixel radii are explicitly tagged.
The selection-method text is bounded to 512 bytes. The UI radius editors accept
up to 1e6 native pixels or 1000 m; bounds outside actual support remain unavailable.
Numeric coordinate editors expand to accommodate the active workspace instead of
clamping a larger loaded frame to the ordinary tabletop editing range.

The central result is `d = hypot(xB-xA,yB-yA)` without bias correction. It does not
divide by distance, so zero and near-zero separations have deliberate finite
behavior. There is no relative percentage or linearized norm gradient at zero.

For a native-pixel disk of declared radius `r`, write `H` as numerator matrix `A`,
denominator row `c'` and constant terms. At native centre `p`, with mapped centre
`q=f(p)` and homogeneous denominator `den`, the exact projective difference is

```
f(p+delta)-f(p) = (A-q*c')*delta / (den+c'*delta)
selectionWorldRadius = r * spectralNorm(A-q*c') / (abs(den)-r*norm(c))
```

The denominator margin must stay strictly positive beyond roundoff. Otherwise
the selection bound is unavailable. This encloses the entire mapped native disk;
it does not use just a local Jacobian approximation. A declared mapped-world
radius directly supplies `selectionWorldRadius`.

The resulting world disk must fit entirely inside geometric support and, for a
prediction bound, the H-mapped independent survey hull. Support is checked with
linear half-plane disk margins through the projective inverse, including its
horizon, native rectangle, measured fit-world hull and fit-image hull. Boundary
sampling of a few directions is not used as proof. The circular enclosure can be
more conservative than the true projective image of a disk; an unavailable result
can therefore reflect this enclosure rather than every actual perturbed point
being unsupported.

## Calibration prediction over the whole footprint

The consumer reuses #249's exact fitted model, variance floor, support and
97.5%-per-endpoint Student-t critical value. It does not introduce a second
calibration uncertainty estimator. At normalized design vector `phi(q)`, let
`M=(Phi'Phi)^-1` and let `D` map a physical world displacement to changes in the
normalized design vector (zero intercept row, diagonal `1/width,1/height`). For a
world selection disk radius `r`:

```
h(q) = phi(q)' M phi(q)
hMax <= (sqrt(h(q)) + r*sqrt(lambdaMax(D' M D)))²
absMeanAxisMax <= abs(phi(q)' betaAxis) + r*norm(D' betaAxis)
axisRadius = absMeanAxisMax + endpointCritical*sqrt((1+hMax)*varianceAxis)
calibrationRadius = hypot(axisRadiusX,axisRadiusY)
```

These conservative analytic bounds include spatial variation of the predicted
mean and leverage throughout the footprint. Evaluating calibration uncertainty
only at the centre and adding a selection radius would not account for that
variation. At radius zero this agrees with the point endpoint contract.

Finally,

```
R = selectionRadiusA + selectionRadiusB + calibrationRadiusA + calibrationRadiusB
interval = [max(0,d-R), d+R]
```

The displayed ±R and interval have **conditional at-least-95% coverage for a fixed
endpoint pair under the calibration model and valid declared selection bounds**:
97.5% endpoint prediction envelopes combine by the union inequality, and distance
error is bounded by the triangle inequality. Endpoint calibration errors do not
need to be independent. Common errors may cancel in actual length, so the sum is
conservative; it is not independent-error root-sum-of-squares propagation.
Selection bounds are deterministic assumptions, not extra Gaussian observations.

This is not simultaneous coverage for every possible pair or adaptively chosen
pairs selected using validation residuals. Survey dependence, non-affine effects,
errors in predictors, unmodelled shared camera/target effects, changing scene/plane
conditions and an unjustified selection budget can invalidate the coverage claim.
All #249 assumptions and physical limitations still apply. Unknown evidence never
becomes a zero bound. No field is silently extrapolated beyond support.

## Validation and physical evidence

Core tests cover known lengths/directions, perspective and signed origins, zero and
near-zero separation, pixel-centre/letterbox/resize round trips, invalid centres,
method/selection gates, projective horizons, image-fit hull constraints, exact
endpoint-envelope sums and enclosing perturbations over entire circles. UI/service
tests cover separate frozen-frame ownership, preserved calibration evidence,
responsive/stale/malformed/late worker behavior, view switching, numeric edits,
selection units/justifications, re-freeze/clear, pending evidence, active-vs-draft
origin changes and larger valid loaded coordinate ranges.

See [recorded verification](../development/testing.md#two-point-workspace-distance-250)
for actual commands/results and platform gaps. Independent physical lengths,
selection-budget justification, actual coverage and native-display behavior belong
to [#268](https://github.com/Krzys2002/Mini_Robot/issues/268); calibration-model
validation remains [#265](https://github.com/Krzys2002/Mini_Robot/issues/265), related
to #63/#224. **All new physical-camera/native-display checks: NOT RUN.** Offscreen
widgets and synthetic math do not establish physical accuracy or actual coverage.

### Tabletop distance scope

Camera/world measurements are distances on the active tabletop z=0 plane; results
retain the world frame and exact calibration revision. They do not measure elevated
axle/body-centre separation. Conditional distance intervals and two-point agreement
do not certify the independently-per-X/Y ±5 mm robot-position target (#224/#63).
Actual height-aware localization belongs to #270/#271/#272; those issues do not
depend on this diagnostic. Physical distance acceptance (#268) remains NOT RUN.
