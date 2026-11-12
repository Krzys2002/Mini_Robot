# Camera-to-table homography calibration

Issue [#57](https://github.com/Krzys2002/Mini_Robot/issues/57) estimates and saves
a planar image-to-world transform. It builds on the [workspace model](workspace.md)
and [intrinsic profiles](calibration-profiles.md). Metric observation conversion and tracking exist as separate components; this
calibration workflow does not move robots or establish a complete control loop. Explicitly activated profiles install measured workspace bounds
in the authoritative WorldModel. Draft edits leave that calibration intact; camera
or optical incompatibility invalidates the active calibration and its bounds. H remains
in the calibration service and is supplied to the implemented metric observation consumer;
a complete perception-to-motion loop is not established.

## Coordinate convention

The input is **undistorted native-resolution pixels using the bound intrinsic
camera matrix K**. Raw camera frames use Brown–Conrady distortion; the worker
undistorts a frozen frame with the existing OpenCV adapter, retaining the original
K, dimensions, pixel format and timestamps. It does not crop, resize, mirror or
choose a new camera matrix. Preview scaling/letterboxing is presentation only;
clicks map back to the frozen image's native pixel coordinates. Integer coordinates
are pixel centres, matching OpenCV: the centre of the top-left pixel is (0,0).
The displayed raster edges correspond to -0.5 and width/height minus 0.5; picking
subtracts 0.5 and marker rendering adds it. Numeric centres range from zero to
width/height minus one. Black borders
from undistortion are not physical reference features.

World points use measured **metres** in the explicit [workspace frame](workspace.md).
The operator labels physical +X/+Y edges and may choose a corner or interior origin.
World +X is right and +Y up in the intended world view, independently of screen orientation. Image u increases right and v increases down; therefore
corresponding image/world perimeter windings commonly differ. H is row-major:

```text
[x', y', w'] = H * [u, v, 1]
world x = x' / w', world y = y' / w'
```

Never fit H to raw distorted coordinates and later apply it to undistorted points,
or vice versa. Future consumers must undistort raw detections using this exact
stored optical model once, then use H. The geometry projection helper performs no
lens correction and rejects singular matrices, non-finite results and horizon
points. It does not establish a safe extrapolation region or permission to control.

## Operator procedure

1. Select/start the intended camera and load or calculate its compatible intrinsic
   calibration in **Camera calibration…**. Open **Tabletop / workspace calibration…**
   from that dialog. A stable physical camera identity is required for save/load;
   session-only identities may estimate a transform but cannot persist one.
2. Enter an explicit workspace identity and measured positive width/height in
   metres. No physical table dimensions are assumed. Keep camera, lens settings,
   resolution and table fixed throughout acquisition and later use.
3. Press **Freeze / reset references**. A fresh frame (at most two seconds old) is
   remapped on the worker. Point selection always refers to that immutable frame;
   incoming frames do not move its reference features. Freezing again clears all
   old correspondences while retaining the draft transform for verification.
4. Select C1 (physical base), C2 (+X neighbour), C3 (opposite), C4 (+Y neighbour).
   Review the cyan numbered perimeter, axis arrows and measured edge lengths.
   Use **Undo last corner**, numeric corner cells, or **Reselect selected corner**
   followed by an image click to correct mistakes. Crossed, duplicate, concave or
   collinear boundaries cannot be confirmed. The application never sorts corners
   by their positions on screen.
   Choose a corner origin, or measured origin offsets from C1 along +X/+Y in metres.
   Interior origins deliberately create negative coordinates on part of the table.
   Press **Confirm boundary / axes** before adding extra references. The four
   generated paired boundary rows are locked; correct them through the corner
   controls. Add up to 4,096 total fitting references, including those four corners.
   After **Calculate**, **Pick origin after Calculate** permits a mapped image
   pick inside the workspace. Changing the origin clears draft results and all
   fit/check references; confirm the boundary again and recalculate. A picked
   origin is transform-derived, not independent ground truth.
5. Add independent interior/reference points and tick **Check only**. Up to 4,096
   check points are held out of estimation. They must not repeat fitting points
   or each other. All image points must lie inside the native image, and all world
   points inside the declared measured bounds. Numeric cells use a decimal dot.
6. Press **Calculate**. Invalid input is rejected with an actionable message.
   Inspect fit and held-out RMS/maximum Euclidean errors, in metres. Four exact
   correspondences can fit almost perfectly even when the physical calibration
   is wrong. No check points is explicitly reported as missing validation, not as
   zero independent error. No project-specific physical accuracy threshold is
   invented; compare errors to an approved experiment requirement.
7. **Save transform** atomically stores the draft profile. Saving does not activate
   it. **Load transform** loads an exact workspace/camera/resolution key, rejects
   optical mismatches and updates the draft dimensions. It does not replace the
   active calibration. To recheck a loaded transform, freeze a fresh frame, enter
   independent points with **Check only** selected, then press **Verify checks**.
   New points default to Check when a draft transform exists. Verification reports
   metric errors without refitting or changing H; an invalid check is rejected.
8. Review fit/check errors against the experiment's approved accuracy requirement,
   then press **Activate transform** to install the draft. Activation is an explicit
   operator decision; missing checks and the absence of an automatic accuracy
   threshold are visible warnings. A finite fit alone never establishes physical
   acceptance. Keep the previous active calibration until a replacement is accepted.

Identity, dimension, origin and corner edits invalidate the draft and clear its
references. Additional fitting-point edits invalidate draft H. Active calibration
replacement clears old WorldModel measurements and pending vision diagnostics. Check-point
edits retain H for verification. Camera loss/change, delivered resolution changes
and optical-model changes clear both draft and active transforms and their bounds;
repeated mismatched frames produce a single notification per mismatch episode.
This remains deliberate because reconnection does not prove unchanged camera/table
placement. The service remembers workspace identity/dimensions across a hiccup and
reopening the dialog, so the operator can reload and recheck without retyping them.
It does not automatically re-arm a transform after a camera interruption.
Late worker completion cannot restore an old result. Capture, solving and disk I/O
are serialized off the GUI thread; shutdown drains the worker asynchronously.
Closing the dialog preserves active and draft calibrations and session references.

## Estimation and persistence

The Qt/OpenCV-free domain owns paired coordinates, geometry validation, projection,
residual evaluation and profile parsing. A dimension-scaled homogeneous DLT/SVD
estimate using coordinates relative to the explicit workspace minimum supplies a double-precision starting fit and explicit conditioning guard;
this scaling is not Hartley centroid/isotropic normalization. The OpenCV adapter
also computes `cv::findHomography(image, world, 0)`, using every fitting pair and
refining geometric error in destination metres. It selects that candidate only if
its measured RMS is lower than the double-precision estimate (OpenCV may internally
round inputs). No RANSAC silently discards pairs. Invalid geometry, unresolved fits
and horizons crossing the convex hull of the unordered fitting set are rejected.
Checking the denominator sign at every sample is equivalent to checking hull
vertices; interior and edge samples do not impose perimeter-order restrictions.
Check points never influence estimation. No extrapolation accuracy outside the
measured fitting region is established.

The workspace profile remains separate from the intrinsic profile. Strict v3 text
stores dimensions, explicit minimum X/Y, frame identity, optional ordered boundary,
`undistorted_native_k`, optional nine-element H, complete bound intrinsics and an optional acquisition-record ID.
V1 profiles migrate with exactly zero minimums and unchanged H/intrinsics, retaining
their original lower-left coordinates. V2 files retain signed bounds/frame/H unchanged. Saves write v3; unknown versions are rejected.
See [frame/persistence details and rollback limits](workspace.md#profile-versions-and-compatibility).
Complete v3 acquisitions persist fit/check pairs, measured placements and original
detections in a linked sidecar. Legacy and reset/recheck-only sessions do not invent
missing provenance. Residuals are recomputed from retained inputs, not stored as
physical accuracy evidence. See [flat-board acquisition](workspace-chessboard.md). Loaded boundary coordinates are stored acquisition references;
freezing another image does not prove unchanged camera/table placement. Independent
Check points may verify a loaded H without selecting corners again; use **Reselect
all corners** to begin a new fit instead.

The filesystem adapter uses hashed, length-prefixed workspace/camera/resolution
keys under the injected per-user `workspace` directory. User IDs never become path
components. QSaveFile atomic replacement has no direct-write fallback. Read/write
errors remain visible and failed saves preserve the previous profile. Physical
camera placement and workspace identity still require operator verification;
matching metadata does not prove unchanged extrinsics.

## Verification and manual commissioning

Deterministic tests cover affine/perspective maps, overdetermined/noisy fits,
independent check errors, invalid geometry, horizon/conditioning failures,
strict serialization, atomic store recreation and optical mismatch, dialog point
entry, asynchronous invalidation, stale frames and worker failure feedback.

```sh
ctest --test-dir build/macos -R 'TabletopHomographyTest|WorkspaceProfileTest|OpenCvWorkspaceHomographyTest|QtWorkspaceProfileStoreTest|WorkspaceCalibrationUiTest' --output-on-failure --no-tests=error
```

Physical camera/table accuracy and new native-display checks are **NOT RUN**.
[#259](https://github.com/Krzys2002/Mini_Robot/issues/259) owns the guided-corner/origin
physical acceptance; [#224](https://github.com/Krzys2002/Mini_Robot/issues/224) retains
printed-tag metric pose acceptance. To commission,
measure additional independent tabletop references, freeze the production camera
at its intended size/focus, check axis signs and metric errors across the fitting
region, restart the application and explicitly reload, then repeat the independent
checks. Move the camera deliberately to demonstrate why metadata alone cannot
revalidate placement. Record reference measurement uncertainty, camera/lens/device
identity, resolution, table dimensions, correspondences, residuals and revisions.
Native macOS, Linux X11/Wayland and Windows dialog/input checks and OpenCV 4 execution
remain separate unverified platform checks; offscreen tests do not establish them.

## Repeated flat-board placements

The [workspace chessboard guide](workspace-chessboard.md) describes the shared
9×6 inner-corner target, measured anchor/orientation and explicit ordering review.
Use **Freeze next board frame** to keep the dataset between placements; the
original reset/recheck action clears its displayed references. Large reference
tables use a model, worker validation/verification and consistent 4,096-point
role limits. V3 profiles link immutable acquisition snapshots; preserve profile
and sidecar together. Physical large-dataset/target accuracy is NOT RUN in #263.

## Spatial scale/error diagnostics

The active transform exposes [directional px/cm, survey error maps and a conditional
prediction contract](workspace-diagnostics.md). Independent survey confirmation and
model assumptions are explicit; fit residuals are not an accuracy guarantee.
Exports retain the complete calibration/acquisition identity and sampled masks.
Physical acceptance is owned by [#265](https://github.com/Krzys2002/Mini_Robot/issues/265),
with all new physical/native-display checks NOT RUN.

## Two-point diagnostic

An explicitly active calibration also supports [two-point planar distance](workspace-distance.md)
in a separate frozen undistorted camera image or the signed world diagram. Measurement
freeze retains calibration data; world endpoints persist across view-only changes.
Conditional ± combines surveyed calibration evidence and justified selection disks;
missing evidence produces distance with uncertainty unavailable. Physical acceptance
is tracked in [#268](https://github.com/Krzys2002/Mini_Robot/issues/268), all new checks NOT RUN.

## Side-mounted views and fit support

See the [side-mounted source audit and limitations](oblique-tabletop.md) for the
known-geometry tests and demonstrated numerical fix. A projective H handles an
oblique plane without changing physical axis labels. Use visible, independently
measured references across near/far edges and both axes; raise an almost edge-on
camera and never guess occluded reference locations.

Solve feedback separates workspace fitting-hull coverage, image coverage and
extrapolated Check references from metric residuals. Small image coverage alone
is expected in some side views. Fit coverage is not an accuracy/confidence bound. Linked acquisition records can
restore the underlying references; legacy profiles without those records cannot. Neither bounds
membership nor a low residual establishes accuracy outside the fitting hull.

Keep camera/table/lens settings fixed. Markers above the calibrated plane have
height-dependent parallax; undistortion does not correct it and this method does
not reconstruct arbitrary 3D. Physical side-camera acceptance is NOT RUN and
belongs to #261; printed-tag metric acceptance remains #224.
