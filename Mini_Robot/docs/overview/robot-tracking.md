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
tracking status and age to decide whether a pose is usable. This issue does not
implement filtering, velocity, robot motion or an operational safety state machine.
Physical camera/tag localization remains unverified (see #224).
