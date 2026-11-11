# Flat-board workspace acquisition

[#248](https://github.com/Krzys2002/Mini_Robot/issues/248) reuses the intrinsic
`CalibrationTarget` and `ICalibrationDetector` for measured **planar workspace**
references. The default is the same 9×6 inner-corner target, nominal spacing 25 mm.
Enter the actual measured spacing if printing differs. Keep the board flat on the
calibrated plane; the tilted views used to solve lens intrinsics are a different
procedure. This workflow does not estimate an unmeasured board displacement or
physical orientation from the image.

## Operator procedure

1. Select compatible intrinsics, configure the signed workspace and complete the
   [four-corner/origin review](workspace.md). The workspace corners C1–C4 are
   distinct from the chessboard's logical inner-corner indices 0–53.
2. Expand **Flat tabletop chessboard acquisition**. Use **Freeze next board frame**
   to freeze a fresh undistorted image **while retaining the dataset**. Keep the
   camera, table, resolution and optics fixed; move only the board between
   placements. **Freeze / reset references** starts the original corner/recheck
   session and clears its displayed dataset, so use the placement-specific action
   for repeated board acquisition.
3. Press **Detect exact frozen frame**. Only a complete detector result for that
   exact immutable native-K image is reviewed. Held/live-tracked preview corners
   are never calibration observations. No frame history/raster is retained per
   placement; original pixel measurements and source sequence/steady-clock ticks
   are recorded instead.
4. Choose an **anchor inner-corner index**, enter its measured absolute world X/Y
   in metres, and enter the angle of increasing board columns from world +X
   (degrees in the UI, radians internally). Explicitly choose whether increasing
   rows is +90° or −90° from increasing columns. One point and spacing alone do not
   determine orientation. Confirm physical labels against the board, not screen
   positions.
5. Alternatively, select an existing reference row, press **Use selected
   reference**, enable **Measured offset from reference**, and enter measured
   offsets in world X/Y. The record retains reference ID, its original coordinate
   snapshot, entered offset and resolved anchor. Correcting/removing that reference
   later does not silently move an already imported placement.
6. Press **Review measured geometry**. The labelled board zoom is a presentation
   crop only: coordinates remain full-resolution native undistorted pixels.
   Its red arrow indicates +columns, yellow +rows and magenta circle the anchor.
   Click a labelled corner or change the index to choose another anchor, then
   re-enter/check its measured position and review again. Review indices,
   **Reverse detector columns/rows**, axis signs and world positions. Rectangular
   chessboards do not supply unique physical corner identity; reversals/symmetries
   require operator confirmation and are never silently resolved from screen order.
7. Review the 54 candidate rows and inclusion mask. Duplicate image/world
   references and points outside the measured workspace are explicitly excluded
   with reasons. Exclude further rows if needed. The originals, original automatic
   review result and final mask remain in provenance. Choosing an independent
   **Check-only placement** holds those points out of fitting; otherwise they are
   Fit references. Confirm physical identity, axes, ordering and mask, then import.
   Imports exceeding a role/count limit are rejected atomically, never truncated.
8. Repeat with another measured placement and a new placement freeze/detection.
   The stored placement anchor/orientation is not inferred from an earlier image.
   Imported image/world values and roles remain editable in the reference table;
   hover their provenance for original measurements and placement metadata.
   Removing a reference updates its placement mask while retaining the original
   detection. Corrections do not overwrite the original detector points or measured
   placement parameters. Placement controls configure the next reviewed import;
   existing imported rows are corrected directly in the table.
9. **Calculate**, review fit/held-out residuals and explicitly **Activate**.
   **Verify checks** validates Check edits without refitting H. Batch validation,
   detection, solving and verification run on the serialized worker. Geometry
   changes/camera loss fence late work. Draft acquisition edits preserve the active
   transform and its immutable evidence until explicit replacement. After Check
   edits, renewed validation (Calculate, Verify or successful Save) establishes the
   complete draft snapshot needed for activation.
10. **Save transform** writes the profile and linked acquisition record. Loading
    a complete record restores references, masks and placements; freeze the next
    placement frame to continue it. The original reset/recheck workflow can verify
    a loaded transform using session-only Check points; those session-only points
    do not replace its saved acquisition. The UI states this distinction. Save
    retains the original fit provenance in that mode.

“Check” means held out of the fit. Shared printed spacing, placement anchors,
orientation and reference chains can correlate errors. A grid point marked Check
is not automatically an independently surveyed ground-truth measurement or a
confidence bound. Keep physical accuracy evidence and its measurement uncertainty
separate; this issue supplies no uncertainty/precision guarantee.

## Capacity and the requested 1,525 locations

| Resource | Bound / behavior |
| --- | --- |
| Fit / Check roles | 4–4,096 Fit; 0–4,096 Check; at most 8,192 total |
| Stored placements | 128 per dataset, including their original 54 detections/masks |
| Active profile | Existing 128 KiB text bound; no acquisition point arrays |
| Acquisition sidecar | Strict 4 MiB byte bound, with reference/placement counts checked before count-driven array growth |
| Reference UI | `QTableView` / `WorkspaceReferenceModel`, without per-cell widgets for thousands of rows |
| Context overlay | At most 2,048 extra-reference markers and 64 labels, explicitly labelled; every dataset row remains available |
| Board review | All 54 logical indices in a separate labelled zoom/table |
| Worker | One serialized operation; no unbounded detection/solve queue or retained raster history |

For 1.50 m × 0.60 m at 0.025 m spacing, inclusive indices are `0..60` and `0..24`:
**61×25 = 1,525 locations**. A 9×6 inner-corner board covers 8×5 spacing intervals.
The deterministic acquisition test uses column anchors at indices
`0,8,16,24,32,40,48,52` and row anchors `0,5,10,15,19`: 40 overlapping placements.
Existing boundary corners count once; overlapping locations are explicitly masked,
not silently truncated. The resulting record contains exactly 1,525 unique fitting
locations, all original detections and each placement's measured geometry. Separate
solver tests fit 1,525 and 4,096 points with independent held-out references.

This is a software demonstration, not a physical lattice survey. The complete
printed board extends beyond its first/last inner corners. For edge placements,
use a measured workspace within a larger flat plane or proper coplanar support;
do not tilt/overhang it out of the plane. Record backing/paper height and actual
spacing/flatness. More fit points do not establish independent accuracy.

Duplicate checking uses ordered spatial buckets in normalized image/world
coordinates, with neighbouring cells checked at the existing 1e−8 duplicate
tolerance. Large inputs avoid all-pairs scans. Bounds snapping during board
arithmetic is restricted to machine roundoff at a boundary (16 epsilon-scaled
units); physically outside candidates remain outside for explicit exclusion.

The DLT matrix is 2N×9. Four points require the complete 8×9 right nullspace, so
that case retains `FULL_UV`. Larger cases use thin U, 2N×9, rather than quadratic
2N×2N storage. Diagnostic residual arrays use the same role limits. Data validation
and original provenance validation do not run per marker; runtime profiles remain
small and observation conversion does not copy the acquisition arrays.

## Profile v3 and acquisition record v1

The v3 workspace profile adds a quoted `acquisition` ID after the v2 frame/boundary
fields. V1 profiles retain exactly their zero minimums and original H; v2 profiles
retain their signed frame and H. Neither migration invents missing acquisition
provenance. Unknown versions, missing/mismatched linked records and malformed data
are rejected. Saves write v3; older applications cannot read it, so preserve old
files for rollback.

The linked record stores the exact profile/H/optical/frame binding, current paired
rows and Fit/Check/boundary roles, stable row/source IDs, measured target spacing,
anchor index/position, column angle, row direction, reversals, relative-reference
snapshot/offset, notes, source frame sequence/steady-clock ticks, original native
pixel detections, automatic review results and inclusion mask. Logical mask/source
indices use the explicit row/column ordering; the detector array retains its
original order. Current row values plus those originals preserve manual
corrections. Decimal doubles use `max_digits10` and the classic locale.

Records reproduce solver inputs and placement interpretation; they do not contain
raster images for re-running detection. Retain raw/undistorted images and physical
measurement evidence separately when commissioning. Steady-clock ticks are not
UTC timestamps. An acquisition ID is an immutable store key, not a cryptographic
signature of physical correctness.

`QtWorkspaceProfileStore` hashes workspace/camera/resolution and acquisition IDs
into safe filenames. It writes a new immutable acquisition snapshot first, then
atomically replaces the small profile using `QSaveFile`; direct-write fallback is
disabled. Existing record IDs may be reused only for identical bytes. A nonblocking
lock rejects conflicting writers. Failed saves preserve the previous profile and
its still-unmodified record. A completed snapshot left by a later profile-write
failure remains as evidence. New saved datasets retain old snapshots; no automatic
history deletion occurs. Manage disk retention deliberately, and copy the whole
workspace profile directory or the profile **together with its linked sidecar**
when backing up/exporting. Each artifact has the byte/count bounds above.

Application active/draft acquisition snapshots are immutable and distinct. The
active snapshot survives draft edits and is cleared on camera/optical invalidation.
Its lightweight profile carries only the link. Valid metadata is not physical
accuracy; future diagnostics must assess the evidence's support and correlation.

## Verification and measured limits

Core tests cover anchors/orientations/reversals, explicit relative snapshots,
invalid data, spatial duplicates, the exact 40-placement lattice, count/byte limits,
strict format/migration and lossless provenance. Qt/OpenCV tests cover the real
shared detector through workspace pair generation, 1,525/4,096-point solving,
immutable/missing/mismatched sidecars, multi-placement lifecycle, stale workers,
manual edits, active/draft evidence, zoom picking and large table models.

The opt-in isolated process audit is reproducible from a root macOS build:

```sh
python3 tools/calibration/audit_workspace_solver.py --binary build/macos/Mini_Robot/tests/micro_robot_calibration_detector_tests
```

It compares the legacy full-U decomposition if the old cap were merely raised,
thin decompositions and the complete production solver. RSS includes process and
linked-library overhead; elapsed times include startup unless explicitly printed
as solve time. Matrix dimensions establish the allocation difference independently
of process RSS. Results are host observations, not portable timing guarantees.
The audit case is deliberately skipped in normal CTest and run explicitly.

Physical target/workspace accuracy and new native-display checks are **NOT RUN**
and owned by [#263](https://github.com/Krzys2002/Mini_Robot/issues/263).
Related corner/origin commissioning is #259; physical side-camera evidence is #261,
metric tags #224 and benchmarking #63. No elevated-object correction, arbitrary 3D
reconstruction, completed motion loop or production motor safety is established.

### Recorded software run (2026-11-11)

Code revision `4e2f22d`, on the approved #260 parent `ea81b7a`: macOS arm64,
AppleClang 21, Qt 6.12 and OpenCV 5.1. Root core and standalone core each passed
294 tests. The root desktop run passed 484 cases with one opt-in SVD audit skipped
(485 selected, zero failures); the audit was then run explicitly in five isolated
processes. Eight acquisition UI cases passed at both 1.5× and 2×. These results do
not include the separately published #247 branch.

| Isolated audit case | Peak RSS | Process elapsed | Additional allocation/solve evidence |
| --- | ---: | ---: | --- |
| Legacy full U, 1,525 points | 167.95 MiB | 0.130 s | U 3,050×3,050 = 74,420,000 bytes |
| Thin U, 1,525 points | 25.69 MiB | 0.095 s | U 3,050×9 = 219,600 bytes |
| Thin U, 4,096 points | 26.75 MiB | 0.096 s | U 8,192×9 = 589,824 bytes |
| Complete production solver, 1,525 points | 27.80 MiB | 0.132 s | Solve 34.55 ms; independent Check recovered |
| Complete production solver, 4,096 points | 30.25 MiB | 0.188 s | Solve 95.44 ms; independent Check recovered |

Table-model show plus next event delivery measured 14/14 ms for 1,525/8,192 rows
at 1.5× and 17/19 ms at 2× in the offscreen test. This is a synthetic one-shot
observation, not native input-latency or camera-load acceptance. Board zoom and
fixed primary action rows were visually reviewed offscreen. Physical/native checks
remain NOT RUN in #263.
