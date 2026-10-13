# Project overview

- [Architecture](architecture.md): application modules, interfaces, ownership,
  domain units, threading boundaries, and current capability mapped to M1–M8.
- [Source map](source-map.md): every implemented source file and its CMake target.
- [Camera discovery](camera-discovery.md): system inventory, opaque IDs, refresh, and verification limits.
- [Camera source contract](camera-source.md): lifecycle, frames, and subscriptions.
- [Transport contract](transport.md): commands, telemetry, and async ownership.
- [WorldModel](world-model.md): registration, timestamp ordering, and snapshots.
- [Simulation](simulation.md): virtual time and deterministic fake behavior.
- [Desktop demo](desktop-demo.md): controls, composition, and shutdown.
- [Logging](logging.md): logger/channel API, record format, sinks, and log file location.
- [Configuration](configuration.md): settings file format, location, defaults, and diagnostics.
- [Milestone roadmap](../planning/MILESTONES.md): planned capability progression.
- [Build and test guide](../development/testing.md): command-line verification.

Implementation constraints and platform policy are defined in
[AGENTS.md](../../AGENTS.md).
