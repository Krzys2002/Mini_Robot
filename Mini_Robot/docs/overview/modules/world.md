# World module

The composition root owns one `WorldModel`; services receive it by injection and
UI observes detached state. It stores robot records and optional metric workspace
bounds, with snapshots preserving ownership and timestamp semantics.

- [WorldModel behavior](../world-model.md): registration, explicit pose updates,
  monotonic ordering and detached snapshots.
- [Tabletop workspace](../workspace.md): physical origin, axes, heading, SI bounds,
  world-view presentation, versioned workspace-calibration data and test coverage.

Workspace profiles are separate calibration data. Current dimensions do not imply
calibrated localization, collision clearance, fresh tracking or safe control.
Homography estimation/persistence and metric robot observations remain #57/#58.
