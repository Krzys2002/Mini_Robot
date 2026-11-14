# Camera-to-table homography calibration

Issue [#57](https://github.com/Krzys2002/Mini_Robot/issues/57) estimates and saves
a planar image-to-world transform. It builds on the [workspace model](workspace.md)
and [intrinsic profiles](calibration-profiles.md). Robot detection/pose conversion
and tracking are separate work in #58 and later issues; this workflow does not
move robots or infer poses. Explicitly activated profiles install measured workspace bounds
in the authoritative WorldModel. Draft edits leave that calibration intact; camera
or optical incompatibility invalidates the active calibration and its bounds. H remains
in the calibration service until future observation consumers are implemented.

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

World points use measured **metres**, origin at the tabletop's lower-left, +X to
the right and +Y up. Image u increases right and v increases down; therefore
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
3. Press **Freeze undistorted frame**. A fresh frame (at most two seconds old) is
   remapped on the worker. Point selection always refers to that immutable frame;
   incoming frames do not move its reference features. Freezing again clears all
   old correspondences while retaining the draft transform for verification.
4. Click reference points or press **Add point**, then enter each paired u/v pixel
   coordinate and measured x/y position. Use 4–64 paired fitting points in any
   order, including rectangular corners, edge midpoints and interior references.
   Each pair must identify the same physical reference in both frames; duplicate
   image/world coordinates and collinear/degenerate layouts are rejected. Verify
   physical labels; fitting residuals alone cannot detect every mistaken pairing.
5. Add independent interior/reference points and tick **Check only**. Up to 64
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

Identity, dimensions and fitting-point edits invalidate only the draft. Check-point
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
estimate supplies a double-precision starting fit and explicit conditioning guard;
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

The workspace profile remains separate from the intrinsic profile. Version 1 uses
strict `micro_robot_workspace 1` text, quoted workspace identity, metric dimensions,
`undistorted_native_k`, optional nine-element H and a complete quoted intrinsic
profile. Decimal doubles use `max_digits10` and the classic locale, preserving
exact optical compatibility across save/load. Unknown versions/conventions,
malformed/truncated/oversized data, non-finite or singular transforms, invalid
intrinsics and trailing data are rejected. An absent H represents uncalibrated
metadata; the UI never treats it as a usable loaded transform. Acquisition points
and residuals are session diagnostics, not persisted measurement evidence.

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

Physical camera/table accuracy is **NOT RUN** for this implementation. To commission,
measure additional independent tabletop references, freeze the production camera
at its intended size/focus, check axis signs and metric errors across the fitting
region, restart the application and explicitly reload, then repeat the independent
checks. Move the camera deliberately to demonstrate why metadata alone cannot
revalidate placement. Record reference measurement uncertainty, camera/lens/device
identity, resolution, table dimensions, correspondences, residuals and revisions.
Native macOS, Linux X11/Wayland and Windows dialog/input checks and OpenCV 4 execution
remain separate unverified platform checks; offscreen tests do not establish them.

## Side-mounted views and fit support

See the [side-mounted source audit and limitations](oblique-tabletop.md) for the
known-geometry tests and demonstrated numerical fix. A projective H handles an
oblique plane without changing physical axis labels. Use visible, independently
measured references across near/far edges and both axes; raise an almost edge-on
camera and never guess occluded reference locations.

Solve feedback separates workspace fitting-hull coverage, image coverage and
extrapolated Check references from metric residuals. Small image coverage alone
is expected in some side views. Fit coverage is session-only and is not an
accuracy/confidence bound; loaded profiles cannot restore it. Neither bounds
membership nor a low residual establishes accuracy outside the fitting hull.

Keep camera/table/lens settings fixed. Markers above the calibrated plane have
height-dependent parallax; undistortion does not correct it and this method does
not reconstruct arbitrary 3D. Physical side-camera acceptance is NOT RUN and
belongs to #261; printed-tag metric acceptance remains #224.

### Tabletop and elevated robot planes

The commissioning envelope to test is a 1×0.5 m workspace with a fixed camera
0.5–1 m above the table and 0.1–0.3 m outside its long edge, 40 mm footprint tags
at approximately 30–70 mm **measured** height. This is intended setup, not physical
acceptance. A tabletop homography maps z=0. An elevated parallel tag centre and
an axle/body centre are distinct points. Per-robot heights, tag-to-body offsets
and heading/mount transforms remain necessary; [#270](https://github.com/Krzys2002/Mini_Robot/issues/270)
owns that conversion, [#271](https://github.com/Krzys2002/Mini_Robot/issues/271)
owns reference/survey identity, and [#272](https://github.com/Krzys2002/Mini_Robot/issues/272)
owns timing. These successors do not depend on the map/distance diagnostic stack.

Physical side-camera acceptance (#261) remains NOT RUN.
