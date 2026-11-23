# Localization timing, quality and provenance

Issue #272 instruments the existing camera → detector/converter worker → tracker
→ authoritative WorldModel path. It covers both tabletop and elevated body
localization (#270) and includes surveyed reference monitoring (#271) in the
conversion timing.

## Timestamp contract

`Frame::capturedAt` is the original observation timestamp for compatibility. It
does **not** by itself establish sensor exposure. `FrameTiming` separately stores
optional `receivedAt`, optional steady-domain `exposureAt`, observation-time
source, exposure clock-mapping description/uncertainty and whether upstream
buffering is known. Missing data stays missing, including for legacy producers.

- Qt timestamps native-frame receipt before image conversion. V4L2 timestamps
  successful retrieval before RGB conversion. Both use receipt estimates, keep
  exposure unavailable and report unknown upstream buffering. Neither maps a
  device clock or claims hardware timestamps.
- Replay retains its virtual mapped media observation timeline and separately
  records virtual delivery time. Simulation labels its supplied clock explicitly.
  Neither is physical exposure evidence.
- A future adapter may supply exposure only with an explicit justified mapping
  into the application's steady clock and a nonnegative uncertainty. Exposure
  must not follow receipt. A mapped-exposure observation must use that exposure
  time; mismatched/future bindings are rejected. A raw device timestamp is never
  implicitly cast to a steady timestamp.

`MeasurementProvenance` follows detections, robot observations, metric
measurements and detached WorldModel tracking snapshots. It records source,
sequence, input generation, native size, original observation time/timing, world
frame and the exact serialized calibration binding shared immutably across
measurements. Elevated body measurements preserve the robot's mounting revision.
Tabletop tags have no mounting revision. The full profile identity is distinct
from the acquisition or localization revision on the metric measurement.
Stop, source, resolution, calibration and converter changes fence old results
and reset session statistics. Gate edits affect subsequent measurements and
preserve existing poses and statistics. Binding changes prevent blending
old filtered geometry or differentiating velocity across revisions.

WorldModel `poseUpdatedAt`, provenance observation time and raw/filtered pose time
remain the observation time. `publishedAt` records the owner-thread application
boundary separately. Filtering never advances the measurement clock. There is
no predicted-current pose, horizon or prediction covariance in this feature;
those values are explicitly unavailable. Reuse the existing tracker/estimators
for future prediction rather than creating another pose authority.

## Report definitions

Open **Vision debug** and select **Copy snapshot** to copy the current report; the owner-thread service
also exposes `localizationReport(now)` as a Qt/OpenCV-free value. Diagnostics
continue updating with debug-image preparation disabled. All latency fields have
sample count and nearest-rank p50/p95/p99 in milliseconds, over at most the last
256 valid samples per field:

| Field | Interval |
| --- | --- |
| Queue | Vision intake/enqueue to detector worker start, including newest-pending wait |
| Detection | Worker detector call/result preparation |
| Conversion | Metric conversion; includes reference monitoring when a compatible converter integrates it |
| Debug preparation | Worker matched-image overlay preparation, when enabled |
| Logging calls | Owner per-frame debug log call cost, when enabled; logger filtering/flush settings matter |
| Delivery/observers | Synchronous owner descriptions and result/debug observers; overlaps publication and diagnostics |
| World publication | Bridge quality gates and tracker/WorldModel application; excludes later display refresh |
| Receipt to output | Known source receipt to completion of an accepted WorldModel update |
| Exposure to output | Verified mapped exposure to accepted output; unavailable for current live adapters |
| Observation age | Original observation time to accepted output completion, with timestamp semantics retained |
| Overlay paint / diagnostics CPU | Actual GUI paint/diagnostic formatting work, not screen scanout latency |
| Instrumentation update CPU | Statistics update cost; diagnostic CPU also includes report calculation |

Durations overlap: do not sum them. Owner-event-loop delivery delay and other
unattributed delay remain included in receipt-to-output, not falsely assigned to
the detector. Unknown/invalid clocks yield unavailable metrics rather than zero
latency. Historic distributions do not imply a currently fresh pose.

Input FPS uses delivered receipt timestamps (Vision intake when source receipt
is absent). Per-robot useful Hz counts **only tracker-applied new measurements**,
using output completion times; camera/detector output rate is separate. Rates
use at most 256 timestamps within five seconds, `(n-1)/(now-first)`, so gaps and
silence reduce the rate; fewer than two samples are unavailable, and an expired
window reports zero for a previously active stream. Per-robot accepted/rejected
counts, latest inter-update gap and current observation age are available in the
report. Missing tags are counted for robot IDs seen earlier in the current
input generation, on each frame processed without a detector error; absence/occlusion is not a pose, and cause of absence is unclassified.
Detector errors are counted separately and never called missing-tag evidence.

The worker still owns one active job plus one newest pending frame. Replaced
pending frames, source-sequence gaps, malformed frames, duplicate/out-of-order
input, detector errors and metric-unavailable frames have separate counters.
Sequence gaps are observable source delivery losses, not a complete sensor-drop
count: native/device buffering and drops before sequence assignment can remain
unknown. Do not add overlapping counters to infer a total physical loss rate.

## Quality gates and load comparison

The dialog exposes shortest-edge sampling in native pixels and a maximum original
observation age in milliseconds; zero disables those gates. The service's
`setQualityGates` additionally supports an optional detector-native minimum margin.
OpenCV does not supply that margin: enabling a margin gate rejects unavailable
values explicitly. Invalid configurations are rejected. Gates run once in the
tracking bridge immediately before tracker ingestion. Rejections retain tag/robot identity, minimum edge
and a named reason; tracker rejections retain their own reason in diagnostics.
Family, unassigned ID, duplicate identity, invalid geometry, workspace support,
sampling, margin, age and clock failures are distinct.

Sampling gates, marker-size quality and age-weighted reliability are heuristics,
not calibrated probabilities, millimetre covariance or permission to move.

Debug/logging toggles start fresh statistics epochs and fence in-flight old-epoch
cost samples without clearing tracked poses. Disabled overlays cannot be restored
by a late worker. Compare toggles with the same camera,
profile, tag layout, resolution and commissioned recording. Separately compare
dialog hidden/shown (paint/diagnostic costs) and reference monitoring off/on once
#271 is integrated, recording its registry/policy and conversion costs. External
workspace-diagnostic or other logging load must be recorded with the experiment;
it is not magically attributed to reference monitoring. No sleeps or synthetic
rate throttling are used to obtain the nominal update target.

## Validation ownership

Deterministic tests cover unknown/invalid/mapped exposure, exact stage percentiles,
bounded samples, per-robot rejection/absence/cadence, expiry, raw-versus-filtered
time, source/frame/calibration/mount/generation changes, replay semantics,
overload/newest-frame release and worker shutdown with instrumentation enabled.
Disabled feature costs remain unavailable, not fabricated zero timings.

Physical **15 useful updates/s target, minimum 10**, latency/error/occlusion/load
comparisons and the 1080p/720p side-camera setup are owned by
[#63](https://github.com/Krzys2002/Mini_Robot/issues/63), with physical age/velocity
diagnostics in [#236](https://github.com/Krzys2002/Mini_Robot/issues/236).
Those issues already contain the required setup/evidence/checklists; no duplicate
issue is needed. All added physical checks remain **NOT RUN**. Neither camera
30 FPS nor synthetic tests establish commissioned useful cadence, axle/body
accuracy, current-time ±5 mm, physical stopping or closed-loop safety.
