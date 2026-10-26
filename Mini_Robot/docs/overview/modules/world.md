# Authoritative world state

Milestone scope: M1 / M2. [Module index](README.md).

## Capability and boundaries

One non-copyable WorldModel owns registered robots, timestamped world poses and
optional metric workspace bounds. Registration creates unknown pose. Updates
reject unknown IDs, non-finite values and stale timestamps; queries/snapshots
return detached values. Changing bounds never clamps or rewrites poses. Filtering,
prediction, tracking freshness and automatic poses are not registry roles.

The tabletop frame uses a measured lower-left origin, +X right, +Y up, zero
heading along +X and positive CCW rotation, in metres/radians. Workspace profiles
are separate versioned calibration data bound to exact camera/resolution/optical
parameters. Current dimensions do not imply calibrated localization, collision
clearance, fresh tracking or safe control. Homography estimation/persistence and
metric robot observations remain #57/#58.

## Behavior guides

- [WorldModel contract](../world-model.md)
- [Tabletop frame, bounds and workspace-calibration model](../workspace.md)

## Verification and remaining acceptance

World tests cover registry changes, unknown poses, finite coordinates, timestamp
ordering and snapshot ownership without GUI/hardware. Workspace tests cover valid
dimensions, closed containment, layout conversion and unchanged poses; profile
tests cover versions, optical compatibility and scale-invariant homography checks.
The behavior guides own detailed test selection and physical acceptance limits.
