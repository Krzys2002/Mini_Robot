# Motion model and supervisory control

Milestone scope: M4. [Module index](README.md).

## Capability and boundaries

Pure differential-drive kinematics convert body velocities and wheel targets in
SI units, with inverse conversion, signs and limits. A stored demo pose is not
external feedback. Closed-loop scheduling, tracking and safety transitions remain
separate issues. Fast wheel regulation, watchdog and motor stop remain robot-side.

## Behavior guides

- [Differential-drive model](../differential-drive.md)

## Verification and remaining acceptance

The control suite checks analytical kinematics, inverse round trips, signs,
bounds and invalid inputs without GUI/hardware. Tracking/stopping behavior needs
later integration/physical experiments.
