# Tabletop workspace and calibration model

The authoritative `WorldModel` owns metric robot state and optional workspace
bounds. The guided calibration introduced by [#246](https://github.com/Krzys2002/Mini_Robot/issues/246)
extends the original #56/#57 lower-left frame without changing existing physical
coordinates when old profiles are loaded.

## Physical frame

Select four physical corners in this explicit order: **C1 base, C2 adjacent along
+X, C3 opposite, C4 adjacent along +Y**. Enter the measured C1→C2 length W and
C1→C4 length H in metres. The workspace is rectangular in physical space; its
camera projection can be a rotated or oblique convex quadrilateral. Screen order
never determines physical axes. Both image windings are accepted. Duplicate,
crossed, concave, collinear, non-finite and out-of-image boundaries are rejected.
A wrong physical label can still produce a mathematical fit: verify the labels
and signs against independent references.

Choose an origin at any corner or enter its measured offsets `(ox, oy)` from C1
along the labelled +X/+Y edges. Offsets must lie in `[0,W] × [0,H]`. Coordinates
then occupy `[-ox, W-ox] × [-oy, H-oy]`. An interior origin produces negative X
to its left and negative Y below it in the intended world view. A preliminary
calculation also allows picking an origin on the frozen image; that mapped pick
is an estimate, not an independently measured reference.

Heading remains zero along +X and positive counterclockwise toward +Y. Internal
units are metres/radians; image coordinates are native-K undistorted pixels.
The table is a plane; no elevated-object or robot-motion accuracy follows from
this coordinate contract.

`WorkspaceBounds(width,height)` preserves the legacy zero-minimum rectangle.
The three-argument overload stores an explicit minimum plus positive dimensions;
finite, representable extents are required. `contains()` includes the boundary.
Profile validation additionally requires `(0,0)` to lie inside that rectangle.
A bounds check is not robot-footprint clearance or a motion safety policy.

## World state and presentation

`WorldModel::setWorkspaceBounds()` configures dimensions only and does not clamp
or reinterpret poses. `setWorkspaceFrame(bounds, frameId)` explicitly installs an
active frame and clears old measurements/timestamps/tracking metadata while
preserving registered robot identities. Detached snapshots carry the bounds and
optional frame identity. The desktop uses this operation on explicit calibration
activation; camera/optical invalidation clears active bounds and measurements.
The vision controller separately fences pending results, clears previews/batches,
and invalidates its tracker through the existing bridge.

Draft corner, dimension, origin or workspace-identity edits clear draft fit/check
references and results, allocate a new frame identity, and reject late worker
completion. The previous active calibration remains installed until explicit
replacement or camera/optical invalidation. Additional fitting-reference edits
invalidate draft H; Check-only edits retain H for independent verification.

World-view fractions use `u=(x-minX)/W`, `v=1-(y-minY)/H`, with top-left `(0,0)`
and bottom-right `(1,1)`. Apply viewport scale and padding at the UI boundary.
Preserve physical aspect ratio. Out-of-bounds inputs are rejected, never clamped.
`workspaceToView` and `viewToWorkspace` are presentation conversions, not camera
calibration. Image +v points down; camera overlays must project the physical axes
rather than treating world +Y as screen up.

## Profile versions and compatibility

`WorkspaceCalibrationProfile` owns workspace identity, explicit metric bounds,
exact camera/resolution/K/distortion binding, optional image→world H, frame ID and
optional ordered native-K boundary references and a lightweight acquisition ID.
Guided frames receive UUIDs; frame IDs must be interpreted together with the workspace/optical binding and H, not as
a global promise of unchanged physical placement. Recalculating H in the same
physical frame still requires diagnostic invalidation on activation.

The strict `micro_robot_workspace 2` format adds `minimum`, quoted `frame`, and
`boundary` fields after dimensions. Optional boundary storage is four ordered
`u v` pairs. H maps directly to signed world coordinates; the origin is not an
extra presentation-only translation. Doubles round-trip with `max_digits10` and
the classic locale. The active profile retains its 128 KiB bound. V3 adds a quoted
`acquisition` ID
linking a separate bounded record of references/placements; no acquisition arrays
are copied into runtime profiles. See [flat-board acquisition](workspace-chessboard.md).

The reader also accepts strict v1 files: minimums become exactly zero, H and
intrinsics remain unchanged, frame ID becomes `legacy-lower-left`, and boundary
references remain absent. V2 files retain their exact frame and H; neither older
version invents acquisition
provenance. Saving writes v3. Nothing is silently recentered or
mirrored. Unknown versions, missing frame/acquisition fields, malformed/oversized data,
invalid bounds/boundaries, incompatible optical models, singular/non-finite H and
trailing data remain rejected. Older software cannot read v3: retain original
v1/v2 files compatible with the intended rollback version. Profile storage keys
and atomic replacement remain unchanged; v3 also requires its linked sidecar.

See [the operator procedure](tabletop-homography.md),
[Workspace.hpp](../../src/world/Workspace.hpp),
[WorkspaceCalibration.hpp](../../src/calibration/WorkspaceCalibration.hpp), and
[WorkspaceProfile.cpp](../../src/calibration/WorkspaceProfile.cpp).

## Verification limits

Core tests cover signed extents/view round trips, boundary validation in both
windings, strict profile migration and downstream metric consistency. OpenCV tests
cover solving with signed origins and reversed physical axis assignment. Offscreen
UI tests cover guidance, edits, picking, save/load and stale-work invalidation.
They establish software behavior, not physical camera accuracy or native-display
acceptance. Physical reference and printed-tag evidence belongs to the separate
[#259](https://github.com/Krzys2002/Mini_Robot/issues/259) (corner/origin workflow) and
[#224](https://github.com/Krzys2002/Mini_Robot/issues/224) (metric tags); all new physical
checks remain NOT RUN until executed.

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
