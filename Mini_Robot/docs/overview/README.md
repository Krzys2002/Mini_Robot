# Project overview

- [Architecture](architecture.md): application modules, interfaces, ownership,
  domain units, threading boundaries, and current capability mapped to M1–M8.
- [Source map](source-map.md): every implemented source file and its CMake target.
- [Platform boundaries](platform-boundaries.md): OS-dependency audit, layer rules,
  device-discovery pattern, backend replacement, and enforcement checks.
- [Camera discovery](camera-discovery.md): system inventory, opaque IDs, refresh, and verification limits.
- [Calibration capture](calibration-capture.md): chessboard target, quality gates, worker wizard and observation review.
- [Camera streaming](camera-streaming.md): native start/stop, failure reporting, and worker lifecycle.
- [Camera selection](camera-selection.md): selectable devices, source factory, and active-source replacement.
- [Camera source contract](camera-source.md): lifecycle, frames, and subscriptions.
- [Recorded-video replay](recorded-video.md): local files, virtual timing, EOF, and fixtures.
- [AprilTag selection](../vision/apriltag-selection.md): marker size, identity mapping, camera constraints, and printable sheets.
- [Transport contract](transport.md): commands, telemetry, and async ownership.
- [Differential-drive model](differential-drive.md): SI equations, wheel targets, signs, and bounds.
- [WorldModel](world-model.md): registration, timestamp ordering, and snapshots.
- [Simulation](simulation.md): virtual time and deterministic fake behavior.
- [Desktop demo](desktop-demo.md): controls, composition, and shutdown.
- [Logging](logging.md): logger/channel API, record format, sinks, and log file location.
- [Configuration](configuration.md): settings file format, location, defaults, and diagnostics.
- [Milestone roadmap](../planning/MILESTONES.md): planned capability progression.
- [Build and test guide](../development/testing.md): command-line verification.

Implementation constraints and platform policy are defined in
[AGENTS.md](../../AGENTS.md).
