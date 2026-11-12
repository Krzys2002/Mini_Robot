# Project overview

- [Current capability and integration limits](capabilities.md): implemented components, remaining workflow and evidence owners.

- [Architecture](architecture.md): application modules, interfaces, ownership,
  domain units, threading boundaries, and current capability mapped to M1–M8.
- [Source map](source-map.md): every implemented source file and its CMake target.
- [Platform boundaries](platform-boundaries.md): OS-dependency audit, layer rules,
  device-discovery pattern, backend replacement, and enforcement checks.
- [Camera discovery](camera-discovery.md): system inventory, opaque IDs, refresh, and verification limits.
- [Calibration capture](calibration-capture.md): chessboard target, quality gates, worker wizard and observation review.
- [Intrinsic calibration](intrinsic-calibration.md): camera model, solver, validation and reprojection quality.
- [Camera streaming](camera-streaming.md): native start/stop, failure reporting, and worker lifecycle.
- [Camera backends](camera-backends.md): Linux V4L2/OpenCV and macOS Qt backends, capture resolution, and failure handling.
- [Camera selection](camera-selection.md): selectable devices, source factory, and active-source replacement.
- [Camera source contract](camera-source.md): lifecycle, frames, and subscriptions.
- [Recorded-video replay](recorded-video.md): local files, virtual timing, EOF, and fixtures.
- [AprilTag selection](../vision/apriltag-selection.md): marker size, identity mapping, camera constraints, and printable sheets.
- [Transport contract](transport.md): commands, telemetry, and async ownership.
- [Vision module](modules/vision.md): observation pipeline and matched-frame debug presentation.
- [AprilTag frame detection](../vision/apriltag-detection.md): pixel observations, bounded worker, processing time and replay.
- [BLE connection lifecycle](ble-connection.md): opt-in Qt Central, discovery, GATT readiness, cancellation and hardware acceptance.
- [BLE messages](ble-messages.md): bounded command writes, telemetry validation, sequences, diagnostics and adapter E-stop.
- [Communication module](modules/communication.md): RobotManager sessions and backend isolation.
- [Manual robot control](manual-robot-control.md): discovery/selection, connection, bounded hold-to-drive, Stop/E-stop, validation and physical limits.
- [Robot communication](robot-communication.md): platform-neutral robot discovery,
  Qt Bluetooth review for Linux/macOS, control isolation, and mock transport tests.
- [Workspace distance](workspace-distance.md): two endpoints in frozen camera/world views, selection bounds and conditional distance intervals.
- [Workspace diagnostics](workspace-diagnostics.md): directional px/cm, independent survey error maps, bounded exports and conditional prediction contract.
- [Workspace chessboard acquisition](workspace-chessboard.md): flat-board anchors/orientation, repeated placements, 1,525-location coverage, capacity and v3 provenance.
- [Tabletop homography](tabletop-homography.md): paired undistorted image/world points, residuals, workspace profile save/load and commissioning.
- [CodeCell C6 hardware profile and limits (#154)](../testing/codecell-c6-hardware-154.md):
  verified board/motor facts, unresolved safety blockers, and the bench envelope
  used before powered motion acceptance.
- [Differential-drive model](differential-drive.md): SI equations, wheel targets, signs, and bounds.
- [World module](modules/world.md): robot registry, tabletop frame, metric bounds and workspace-calibration model.
- [WorldModel behavior](world-model.md): registration, poses and snapshots.
- [Robot tracking](robot-tracking.md): measurement age, loss and reappearance.
- [Simulation](simulation.md): virtual time and deterministic fake behavior.
- [Desktop demo](desktop-demo.md): controls, composition, and shutdown.
- [Logging](logging.md): logger/channel API, record format, sinks, and log file location.
- [Configuration](configuration.md): settings file format, location, defaults, and diagnostics.
- [Milestone roadmap](../planning/MILESTONES.md): planned capability progression.
- [Build and test guide](../development/testing.md): command-line verification.

Implementation constraints and platform policy are defined in
[AGENTS.md](../../AGENTS.md).
