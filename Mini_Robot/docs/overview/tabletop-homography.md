# Camera-to-table homography calibration

Issue [#57](https://github.com/Krzys2002/Mini_Robot/issues/57) estimates and saves
a planar image-to-world transform. It builds on the [workspace model](workspace.md)
and [intrinsic profiles](calibration-profiles.md). Robot detection/pose conversion
and tracking are separate work in #58 and later issues; this workflow does not
move robots or infer poses. Accepted profiles install measured workspace bounds
in the authoritative WorldModel; invalidation clears those bounds. H remains
in the calibration service until future observation consumers are implemented.

## Coordinate convention

The input is **undistorted native-resolution pixels using the bound intrinsic
camera matrix K**. Raw camera frames use Brown–Conrady distortion; the worker
undistorts a frozen frame with the existing OpenCV adapter, retaining the original
K, dimensions, pixel format and timestamps. It does not crop, resize, mirror or
choose a new camera matrix. Preview scaling/letterboxing is presentation only;
clicks map back to the frozen image's native pixel coordinates. Black borders
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
   old correspondences and the old estimate.
4. Click reference points or press **Add point**, then enter each paired u/v pixel
   coordinate and measured x/y position. Use 4–64 fitting points forming a convex
   perimeter, in matching order. Start at any labelled physical point and proceed
   around the perimeter; either winding is allowed. No duplicate, crossed,
   collinear or interior fitting vertices are accepted. A rotated correspondence
   assignment can still describe a mathematical transform: verify physical labels,
   do not rely only on fitting residuals to detect a mistaken association.
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
7. **Save transform** atomically stores the accepted workspace profile. **Load
   transform** explicitly loads its exact workspace/camera/resolution key and
   rejects an optical-model mismatch. Loaded measured dimensions replace the
   form's dimensions. A loaded transform has no current frozen correspondences or
   re-measured residuals; inspect camera placement and use independent measurements
   before commissioning. Reload is explicit because image identity cannot detect
   a moved camera or table.

Editing identity, dimensions or points invalidates the estimate before it can be
saved. Camera loss/change, delivered resolution changes and optical-model changes
also clear the frozen frame and result. Late worker completion cannot restore an
old result. Capture, solving and disk I/O are serialized off the GUI thread;
shutdown drains the worker asynchronously. Closing the dialog leaves the service's
current profile available during the session; it does not declare physical acceptance.

## Estimation and persistence

The Qt/OpenCV-free domain owns paired coordinates, ordering validation, projection,
residual evaluation and profile parsing. The injected OpenCV adapter solves a
normalized homogeneous DLT system with SVD: image coordinates are normalized by
native dimensions, world coordinates by measured extents. Rank/conditioning checks
reject unresolved fits. A horizon crossing inside the convex fitting polygon is
rejected. All fitting points contribute; no RANSAC silently removes operator pairs.
Residuals are evaluated in world metres. Check points never influence estimation.
No extrapolation accuracy outside the measured fitting region is established.

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
independent check errors, invalid geometry/ordering, horizon/conditioning failures,
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
