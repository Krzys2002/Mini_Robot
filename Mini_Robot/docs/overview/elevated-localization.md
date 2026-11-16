# Elevated parallel-tag localization

Issue #270 adds an explicit height-aware profile upgrade. Legacy v1/v2/v3
profiles still map undistorted pixels with their original H and world frame.
Their measurements are labelled **tabletop tag centre only**; they are not
corrected axle/body poses. The calibration boundary remains independent of
tracking, planning, RobotManager and transport. No controller is added.

## Measurement procedure

Keep camera, lens focus/zoom, resolution, table and world frame fixed. Establish
and independently check intrinsics and the tabletop frame first. In Workspace
calibration, Load or Calculate a draft, then open **Elevated tag / axle-body
geometry…**. This is a measured-survey import and review workflow, not automatic
physical commissioning.

1. Measure a flat reference surface parallel to the table. Enter its corner
   surface height above tabletop z=0, board thickness and backing thickness in
   metres. Surface height must include both thicknesses and any stand-off.
   All survey rows must be on this same plane. A bare-table survey explicitly
   uses zero surface height and zero thicknesses. Unknown is not zero: do not
   confirm the measurements if they have not been measured.
2. Retain raw rasters, camera identity, calibration revision, surveyed physical
   point identities, survey method, instrument uncertainty and independent truth
   evidence. Enter their evidence ID/method in the dialog. Use the existing
   compatible **undistorted native-K** frozen frame to read image coordinates.
   Do not use resized preview coordinates, raw distorted pixels, or detections
   from another frame/placement. Reference world x/y use the existing signed
   frame; they do not shift its origin or enlarge travel bounds.
3. Paste at least four `Fit` and four independently surveyed `Check` rows:
   `Fit u_px v_px x_m y_m`, or `Check u_px v_px x_m y_m`. Spread both groups
   over near/far edges and both axes. Checks are excluded from estimation and
   candidate selection. Reused/generated board locations can share anchor,
   orientation, spacing and height errors; a Check label alone is not proof of
   independent physical truth. Retain those limitations in the survey evidence.
4. Enter each robot's actual parallel tag height and tag-to-axle/body transform:
   `RobotId height_m body_forward_m body_left_m body_heading_rad "mount_revision" "measurement_method"`.
   IDs are application RobotIds, **not** tag dictionary IDs: the default identity
   map is tag IDs 0–15 to RobotIds 1–16. The identity map is unchanged.
   Translation is from tag centre to axle/body centre, expressed in tag-heading
   axes: +forward follows the decoded tag front and +left is 90° CCW. Heading
   offset adds to tag heading. Measure actual mounting height, translation and
   yaw; nominal 30/50/70 mm examples are not defaults or inferred measurements.
   A robot with no configured measured mount produces no corrected measurement.
5. Select maximum reference reprojection error (pixels) and held-out absolute
   error on **each** world axis (metres). Defaults are 2 px and 0.005 m; these are
   configurable survey rejection limits, not certified runtime accuracy bounds.
   Confirm that heights/thicknesses/mounts and independent checks were measured.
   **Estimate and validate draft** runs off the GUI thread. Rejected work has no
   corrected draft; improve measurements/geometry rather than accepting a pose
   ambiguity or guessing missing dimensions.
6. Review the retained survey, mounts and result. Close the survey dialog and
   explicitly **Activate**, then **Save** in Workspace calibration. Load also
   requires explicit Activate. Draft text edits invalidate pending draft work;
   they do not mutate an already active snapshot. After changing physical camera
   or mounting geometry, stop acquisition/use of the old configuration until a
   newly measured draft is activated. Input/K/resolution changes clear active
   calibration. Activation clears old world/tracking measurements and fences
   pending vision results. New calibration solves allocate fresh revisions;
   changed mounting measurements require a new mounting revision.

A survey has at most 256 references and 64 mounts. Oversized imports fail as a
whole; pasted rows are never silently truncated. The persisted profile remains
bounded at 128 KiB, including the survey. The 4 MiB v3 acquisition sidecar and its
original references/placement evidence are retained separately and unchanged.

## Geometry and validation

OpenCV's planar IPPE estimator receives measured Fit references and calibrated K,
with **no distortion** because these image points are already undistorted. It
estimates both planar pose candidates on a canonical zero-height plane, then
converts translation back to the measured reference-surface height. It does not
decompose or infer camera pose from the runtime tabletop H.

The stored transform is `X_camera = R_world_to_camera * X_world + t`.
Camera axes are +X right, +Y down, +Z forward; world +Z is above the table, with
world X/Y and signed origin unchanged. Positive heading is CCW about world +Z
from +X. Camera world position is `C = -transpose(R) * t`. Reject non-finite or
non-orthonormal/reflected rotations, cameras below the reference surface or tag
planes, and references behind the camera. Candidate ranking uses Fit RMS only.
If two physically admissible solutions differ by less than 0.25 px of Fit RMS,
reject the unresolved ambiguity. An absent alternate-gap field means all other
IPPE candidates failed the above-table/positive-depth tests; it does not mean a
survey was independently certified. Require every reference to pass the chosen
pixel limit and every held-out Check to pass the chosen x **and** y limits.
The stored survey is rechecked when loading/using the profile.

At runtime, the existing OpenCV converter undistorts each native tag's four
corners **once**. Domain code computes the projective centre, rotates each camera
ray with `transpose(R)`, and intersects it with the measured robot plane:
`P = C + ((height - C.z) / direction.z) * direction`. Reject horizontal,
backwards, non-finite or unsupported rays. Derive tag heading from the transformed
front/back corners, rotate the measured planar translation by tag heading, then
add the measured heading offset and wrap the angle before tracker publication.
All distances are metres, angles radians, capture times monotonic.

Support policy requires all undistorted tag corners inside the native image, and
all corrected tag corners, tag centre **and body centre** inside the existing
workspace rectangle. This is a conservative tag-support policy, not a physical
body-footprint, obstacle or control-clearance check. Tags whose support crosses
the boundary are rejected even if their body centre is inside. Survey residuals
do not guarantee accuracy between references or outside their distribution.
Tilted/nonparallel tags and nonplanar surfaces are outside this model.

## Revisions and persistence

The v4 extension contains the measured survey (including reference-surface and
board/backing heights), pose, candidate separation, calibration revision and
per-RobotId measured mounts/revisions/methods. It binds to the exact canonical
legacy profile: camera/device, resolution, K/distortion, tabletop H, signed bounds,
frame ID and immutable acquisition ID. A mismatched binding rejects the profile;
geometry is never silently rescaled or relabelled. V1/v2/v3 import creates no
extrinsics or mounting geometry. Without an explicit validated upgrade the
writer continues to emit v3 tabletop semantics. Older software cannot read v4;
retain old profile backups and their sidecars for rollback.

Adding/changing the extension does not rewrite the acquisition record or make it
claim a later camera/body transform. Store/controller binding comparisons use the
unchanged tabletop portion when checking a v4 profile against its original v3
sidecar. The sidecar's immutable byte/collision protections remain in force.
Runtime measurement metadata records whether its reference is tabletop tag or
axle/body centre, plus camera, world-frame, calibration and mounting revisions.
The worker snapshots the complete profile; changing the active calibration
invalidates its generation, clears tracker measurements, and drops late results.
The single WorldModel still owns Pose2D; no 3D state or independent pose authority
is introduced. Future common-world multi-camera fusion (#108) remains separate.

## Integration with pending diagnostics (#267/#269/#273)

Those optional diagnostics/distance PRs are not prerequisites of #270. Their
current diagnostics implementation compares the *complete* runtime profile with
the acquisition profile. A v4 extension therefore needs a narrow integration
adaptation when those siblings become available: in `WorkspaceDiagnostics.cpp`,
compare `tabletopProfileBinding(acquisition->profile)` with
`tabletopProfileBinding(d->profile)`. Keep the runtime v4 profile and original
v3 acquisition separate; do not overwrite the latter to make a comparison pass.
Diagnostic scale/error maps and distance uncertainty remain **tabletop z=0**,
not propagated robot-body accuracy. A combined-tree regression must verify
unchanged acquisition bytes, retained v4 geometry and unchanged tabletop scales.
This branch stays on approved parent #264; it does not import those optional PRs.

## Verification and physical acceptance

Deterministic suites cover oblique near/far/edge/interior geometry, 30/50/70 mm
planes, off-centre/rotated mounts, signed origins, z=0/H equivalence and homogeneous
H scaling, Brown-Conrady corner undistortion, multiple heights, invalid geometry,
planar ambiguity, bounded surveys, migration/sidecar preservation and worker fences.
They do not establish camera-loaded throughput or physical accuracy.

[Issue #224](https://github.com/Krzys2002/Mini_Robot/issues/224) already owns static
physical elevated axle/body checks; [#63](https://github.com/Krzys2002/Mini_Robot/issues/63)
owns quantitative and moving benchmarks, including survey uncertainty and update
availability. All new physical checks are **NOT RUN**. Commission the recorded
1.0×0.5 m workspace with camera 0.5–1.0 m high and 0.1–0.3 m outside the long edge,
1080p/30 FPS (or separately calibrated 720p), actual 30–70 mm mounts, and timestamped
independent x/y errors. No ±5 mm guarantee, current-time prediction guarantee,
complete perception-to-motion loop or production motor-safety claim follows from
this implementation. Reference monitoring and timing instrumentation remain
#271/#272.
