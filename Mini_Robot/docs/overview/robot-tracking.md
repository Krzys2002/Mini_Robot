# Robot tracking and observation age

`RobotTracker` accepts metric world-frame poses and steady-clock capture times on
its owner thread. `WorldModel` remains the pose authority; its tracking metadata
and `tracking(id, now)` expose status and observation age without reading a hidden
clock. Unknown means no tracked measurement. Fresh means age below `staleAfter`,
stale means age at least `staleAfter` and below `lostAfter`, and lost means age
at least `lostAfter` or explicit input invalidation. If the supplied time precedes
capture, status is conservatively lost and age is unavailable.

The tracker rejects non-finite, future, expired, duplicate and out-of-order
measurements. A new valid observation registers an unknown ID; a newer observation
can restore a lost track. An invalidated input marks tracks lost immediately and
rejects measurements captured at or before invalidation. An empty successful
frame lets previous observations age normally. Camera, calibration or converter
invalidation reaches the tracker through the metric batch boundary. A direct
non-tracker pose update clears tracking metadata, so it cannot masquerade as a
fresh observation. Snapshots preserve metadata, but consumers must evaluate age
at their current monotonic time; a copied status cannot age by itself.

Defaults are 250 ms stale and 1000 ms lost, configurable through
[application settings](configuration.md#tracking-age). They are provisional,
not physical safety thresholds. Historical poses remain stored after loss; use
tracking status and age to decide whether a pose is usable. Tracking does not enable robot motion or an operational safety state machine.
Physical camera/tag localization remains unverified (see #224).

## Source ownership and future control gate

The production `connectVisionTracking` bridge has one exclusive VisionController
owner per RobotTracker/WorldModel tracking set. That controller owns the selected
camera, discards obsolete generations and invalidates all its tracks on source,
resolution, calibration, stop and shutdown changes. The signal's sourceId is
currently diagnostic, not a fusion key. Connecting multiple camera controllers to
this bridge is unsupported: one source's invalidation would erase another's
tracks. Multiple cameras require source-specific provenance, clocks/calibration,
per-source invalidation and a defined arbitration/fusion authority first.

Before autonomous pose-based commands are enabled, their supervisor must evaluate
`world.tracking(robotId, now)` at every submission and control tick, using current
steady-clock time. Only Fresh permits new autonomous Velocity. Unknown, Stale,
Lost, missing calibration or source invalidation must inhibit new Velocity,
cancel unsent autonomous motion and request ordinary Stop on the transition.
Invalidation takes effect immediately, without waiting for an age timer. A fresh
measurement alone must not restart a stopped task: explicit task rearming and
fresh command intent are required. E-stop/Fault retain their stronger independent
latches and acknowledged recovery requirements. Commands already handed to the
radio require robot-local expiry/watchdog enforcement; a desktop Stop is not proof
of physical stopping. The existing thresholds are provisional and require measured
latency/accuracy and stopping-budget acceptance before autonomous use.

This is the required policy for the future autonomous command boundary (#77),
not a claim that a pose-based command supervisor is implemented. Current manual
control does not consume tracked poses and remains governed by desktop/robot
safety independently. No autonomous controller is enabled by these changes.

## Pose smoothing

The tracker accepts `PoseFilterConfig` with position and heading alpha in (0,1].
`[tracking] position_alpha` and `heading_alpha` configure the desktop; defaults
are 1 (raw behavior). Smaller values suppress jitter with additional lag.
Position uses exponential interpolation; heading uses the shortest angular arc.
Raw pose and continuous unwrapped heading are retained in tracking metadata in
WorldModel snapshots. Consecutive rotation must be less than pi for unwrapping.
New, lost, invalidated and externally replaced tracks initialize from the next
accepted sample. Rejected measurements do not advance filter state.
Synthetic tests cover wrap crossing, reset and stationary jitter. Physical camera
noise and lag tuning remain unverified.

## Motion and reliability

Tracking metadata exposes optional world vx/vy (m/s) and omega (rad/s), computed
from consecutive raw measurements and continuous heading. Using raw differences
keeps motion independent of pose smoothing lag; camera noise can affect velocity.
Intervals below 1 ms have no derivative; a gap at least `lostAfter`, invalidation,
first observation or external replacement resets the estimate. Duplicate/older
samples are rejected. Non-finite derivative results are unavailable.

`world.tracking(id, now)` supplies current age and a reliability score:
`quality * max(0, 1 - age/lostAfter)`, zero when lost/invalidated. Metric batches use
`minimumEdgePixels/(minimumEdgePixels+10)` as a geometry heuristic. Unknown quality
uses age alone; raw quality remains optional in the snapshot. This is not a
calibrated probability or a controller safety authorization. The GUI shows age,
reliability and motion; consumers must still gate on fresh tracking at current time.
Synthetic stationary, constant-motion, wrap and dt tests run without hardware.
Physical velocity accuracy and confidence calibration remain unverified.
