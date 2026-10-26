# Vision module

The [detector worker](../../vision/apriltag-detection.md) acquires no devices and
processes one frame plus one newest pending frame. The
[observation model](../../vision/robot-observation.md) maps IDs, rejects ambiguous
measurements and derives pixel geometry without creating robot state.

The [debug overlay](../../vision/debug-overlay.md) pairs each result with its
exact immutable raw image and worker-prepared geometry. Its separate Qt dialog
presents IDs, centers, front arrows, age/quality and detector/output cadence;
widgets do not perform perception or calibrated pose conversion.

Tests cover core pairing/cadence and deterministic rendering, rotation, resizing,
source invalidation, bounded frame ownership, lifecycle and high-DPI presentation.
Physical overlay acceptance remains [#211](https://github.com/Krzys2002/Mini_Robot/issues/211),
with all physical checks NOT RUN. Printed-tag detection commissioning remains #191.
Workspace transforms, metric robot poses and tracking remain separate issues.
