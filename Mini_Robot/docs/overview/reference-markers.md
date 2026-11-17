# Surveyed reference markers and placement consistency

Issue #271 adds automatic acquisition of surveyed tag36h11 references and
bounded monitoring for [height-aware body localization](elevated-localization.md).
References have their own registry and never become robot tracks. The existing
manual/chessboard tabletop and v4 localization workflows remain available.
This is an implementation of commissioning/consistency checks; physical accuracy,
monitoring reliability and camera-loaded throughput remain unverified.

## Commission a registry

Start with a compatible, measured height-aware draft and per-robot mounts. In
Workspace calibration open **Surveyed references / drift monitoring…**. Paste
one measured marker per line using decimal points:

```
Fit|Check|Monitor id height_m black_edge_m standard_uncertainty_m independent_0_or_1 "installation_revision" "survey_revision" "method/evidence_ID" x0_m y0_m x1_m y1_m x2_m y2_m x3_m y3_m
```

- Family is currently tag36h11, matching the detector. IDs are dictionary IDs;
  default robot tags 0–15 are reserved for RobotIds 1–16. Assign disjoint reference
  IDs. Duplicate IDs, unsupported families and conflicting robot/reference roles
  are rejected. Each reference has exactly one role: **Fit**, **independent
  Check**, or **Monitor**. Runtime conversion also checks the actual robot map.
- All four coordinates are absolute measured world X/Y in decoded TL, TR, BR, BL
  order, clockwise when viewed from world +Z. Do not reorder corners by their
  screen positions. A cyclic rotation is valid only when the survey identifies
  those decoded corners in that physical orientation. Square geometry must agree
  with the measured black-edge size within 1 µm plus 2%; this is an input sanity
  tolerance, not a physical accuracy bound.
- Measure each marker's actual corner-surface height above tabletop z=0,
  including its substrate/support. Different references may have different
  heights. Each marker is parallel to the table; tilted/bent references are
  unsupported. The black-edge size excludes quiet margins. Record positive
  measured standard uncertainty, installation and survey revisions, method,
  independent evidence ID, and shared-systematic limitations. Missing values
  must not be replaced with guessed nominal values.
- References can lie outside the 1×0.5 m travel rectangle. Their survey domain
  is separate from travel geometry: registry editing never expands planner,
  body-support or WorldModel bounds. The old manual/chessboard homography
  correspondences retain their original in-bounds validation; the new 3D
  reference solver uses explicit surveyed positions without that restriction.
- At least three **visible marker centres** in each of Fit and Check must have
  a convex hull containing every travel-rectangle corner. Four well-distributed
  perimeter markers per role are usually needed for a rectangular setup. A single
  square, a narrow cluster, or many corners from one location cannot establish
  workspace support. Preserve usable camera coverage and sampling: every
  included raw/undistorted edge is at least 4 native pixels; Fit and Check centre
  spreads each span at least 5% of image width and height and have a triangle of
  at least 0.25% of native image area. These are conditioning/sampling rejection
  gates, not detector-success or ±5 mm guarantees.
- Check markers must declare independent survey. They are never used to estimate
  the camera pose or rank candidate solutions. Monitor markers are also excluded
  from fitting and independent-Check acceptance. Reusing a ruler, anchor,
  substrate or reference frame can correlate errors across all roles; role
  labels and more samples do not remove that shared uncertainty.

Confirm the measurements and **Apply registry draft**. This allocates a new
registry revision and clears its prior commissioning validation; changed physical
positions/heights require new installation and survey revisions. Changed survey
size, uncertainty or method requires a new survey revision. Policy-only changes
also require explicit revalidation/activation before use.

Stream the exact matching camera. The existing detector supplies the newest
bounded reference snapshot at the configured cadence. The dialog retains one
row per registry member, including **Included**, **Missing/occluded**,
**Duplicate identity**, **Invalid corners**, and **Unsupported ray/image** reasons.
Raw and once-undistorted native pixels remain reviewable. No preview coordinates,
image resizing, stale placement inference, or extra image queue is used.

Use **Fit latest references**, review the result, then explicitly **Activate**
and **Save** in Workspace calibration. Acquisition must be from the last two
seconds, the exact K/distortion/resolution/camera and current world/registry
revision. Edits and source changes fence late observations and solver results.
The frozen commissioning snapshot records source calibration revision and input
frame sequence/generation/capture ticks without relabelling them as the newly
solved revision. Applying/editing a draft does not mutate the active calibration.
After physically moving hardware, stop using the old setup until revalidation.

The extrinsics backend uses measured 3D Fit corners: IPPE for a common-height
plane, SQPnP when Fit heights differ. Each reference uses its own measured height
for projection, ray intersection, cheirality and held-out per-axis validation.
Identical X/Y at different measured Z are distinct 3D references; duplicate 3D
points or duplicate image points are rejected. The solver preserves tabletop H,
world axes/frame identity and measured robot mounting transforms. No automatic
refit or world-frame change occurs during runtime monitoring.

## Monitoring policy and states

Default policy: assess at most every **200 ms**, expire evidence after **750 ms**,
require **3** consecutive widespread failing assessments over at least **400 ms**,
with **2 px** maximum corner reprojection residual or **0.005 m** absolute residual
on either world axis. Exceeding either limit makes a monitor marker an outlier.
Cadence, age, duration, count and both limits are editable; durations are bounded
between 20 ms and 10 s, evidence age/duration cannot be shorter than cadence, and
at least two drift samples are required. Choose limits from commissioned sensor
noise and survey uncertainty; these defaults are not physical acceptance bounds.
Survey uncertainty and method are retained, not silently converted into a
confidence interval or used to excuse deterministic threshold violations.

Only **Monitor** role markers provide runtime consistency evidence. Their visible
centres must surround the travel rectangle. States are:

| State | Meaning and robot measurement policy |
| --- | --- |
| Unvalidated | No fresh assessment after activation/input change; withhold. |
| Consistent | Visible monitor support surrounds workspace and every included monitor is within limits; permit timestamped robot measurements while evidence is fresh. Physical accuracy is not certified. |
| Individual outlier | One isolated failing monitor (or a minority with sufficient remaining support). Inspect its physical placement/survey. Permit only if the remaining good monitor centres still surround the workspace; otherwise withhold. |
| Suspected drift | Widespread or unresolved multi-reference inconsistency; immediately withhold while evaluating persistence. |
| Confirmed drift | At least 75% of visible monitors fail, at least three failing centres surround the workspace, and count/duration persistence passes. Latch withholding until explicit fresh revalidation/activation. |
| Insufficient evidence | Missing/occluded, malformed, stale, expired or spatially inadequate monitoring. Withhold; this proves neither camera movement nor calibration health. |

“Confirmed drift” means persistent **global placement inconsistency**. Translation,
rotation and scale-like residual patterns are supported; equal residual direction
is not required. The data alone cannot distinguish camera/table movement from
shared reference-support movement or a correlated survey error. It must not be
presented as proof that the camera moved. An isolated moved marker alone cannot
confirm global drift. Lost/invalid evidence, including detector failures between scheduled assessments,
breaks a pending persistence run;
gaps longer than maximum evidence age reset it. Confirmed drift survives later
healthy-looking frames, occlusion and source restart for the same configuration.

Monitoring uses the existing serialized vision worker and newest-frame handoff.
Between assessments it reuses only a still-fresh immutable report, while robot
corners continue at the normal vision cadence. A 100 ms GUI watchdog invalidates
tracking on expiry even if no further camera frame arrives; publication also
checks evidence age after worker latency. This is a supervisory validity gate,
not a physical motor stop/watchdog guarantee. Application event-loop stalls and
unknown upstream capture delay remain limitations; #272 owns timing quality.

Temporary occlusion/expiry may recover on fresh, consistent spatial support if no
drift was confirmed; geometry is unchanged and no refit occurs.

To recover from confirmed drift, acquire fresh references, explicitly **Fit latest references**, review
held-out results, then **Activate**. A new successful geometry revision resets the
latched monitor; merely seeing references again, adjusting a draft, restarting
input or pressing Activate on an unchanged active calibration does not silently
clear it. The old active profile remains unchanged until replacement. Monitor
results never overwrite commissioning evidence or refit geometry.

A rejected fit retains the previous geometry/mount context so detector acquisition
continues and corrected observations can be retried without reloading. That
context is retry input only: Activate and Save stay disabled until a successful
solve (or explicit replacement of the draft). The active profile is unchanged.

## Persistence and boundaries

Profiles with a registry use schema **v5**. They contain bounded surveyed geometry,
uncertainty/method/revisions/roles, policy and an immutable corner-only commissioning
snapshot, with exact camera/world/registry bindings and the validated localization
revision. Nested localization format v2 records explicit per-corner reference
heights; legacy format v1 keeps its common reference-surface height meaning.
Legacy workspace v1/v2/v3/v4 profiles do not invent references, uncertainty or
monitoring evidence. Without this extension, the writer retains the corresponding
v3 tabletop/v4 localization semantics. Older software cannot read v5: retain
old profiles and sidecars for rollback.

Capacity is 32 references (128 detected corners) with a bounded 128 KiB runtime
profile. Oversized imports fail atomically; pasted rows are not truncated. V3
manual/chessboard acquisition sidecars remain immutable and separate. Exact
legacy tabletop binding comparisons strip both derived localization and reference
extensions, preserving old records byte-for-byte. Changing K/resolution/frame/H
rejects obsolete reference bindings. Changed mounting/geometry still fences all
old robot measurements through the existing vision generation and tracker reset.
Reference snapshots contain no retained rasters or unbounded history; keep raw
images and independent survey records in the external experiment evidence.

The detector/camera boundary, WorldModel Pose2D, travel bounds, planning,
RobotManager and transport contracts remain unchanged. The optional pending
#267/#269/#273 diagnostics integration still needs its known tabletop-binding
comparison adaptation from #270. Its maps/distances remain tabletop z=0 and
must not claim propagated robot-body accuracy. This issue does not add those
optional PRs to its approved #286 parent chain.

## Validation and remaining physical work

Deterministic suites cover disjoint identities/roles, outside-travel references,
canonical corner rotations, differing heights, native distortion, spatial/image
conditioning, occlusion, isolated outliers, translated/rotated camera geometry,
latched drift and explicit recovery, cadence/newest-frame coalescing, no-frame
expiry, late snapshot fences, strict v5 roundtrip and immutable v3 sidecars.
Synthetic coordinates and offscreen UI checks do not establish physical results.

Existing owners already include the new physical scenarios:
[#261](https://github.com/Krzys2002/Mini_Robot/issues/261) for reference occlusion,
individual/camera movement and recovery; [#224](https://github.com/Krzys2002/Mini_Robot/issues/224)
for static robot/reference separation and axle/body validity; and
[#63](https://github.com/Krzys2002/Mini_Robot/issues/63) for quantitative localization,
drift/recovery and performance evidence. All new physical checks remain
**NOT RUN**. Record actual revisions, fixed 1×0.5 m workspace, camera 0.5–1 m high
and 0.1–0.3 m outside the long edge, 1080p/30 FPS (or independently calibrated
720p), measured 30–70 mm robot mounts, survey uncertainty, raw observations,
per-axis errors and rejection/status timelines. No ±5 mm, commissioned update
rate, current-time prediction, physical stopping or closed-loop safety claim is made.
