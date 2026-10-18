# Differential-drive kinematics

[DifferentialDrive.hpp](../../src/control/DifferentialDrive.hpp) exposes pure,
reentrant C++20 calculations in `micro_robot_control`. It links only the common
types and transport domain API, with no Qt, BLE, camera, or device backend.
It reuses `Pose2D` and the existing `VelocityCommand`; it does not keep robot state
or send commands. `WorldModel` remains authoritative.

## Model and signs

The ideal planar model assumes equal wheel radii and rolling without lateral slip.
The pose reference is the midpoint of the drive axle. Use the calibrated tabletop
world origin; +X points right and +Y up in the world map viewed from above.
Heading theta is zero along +X and increases counterclockwise.
Body velocity `v` is forward in metres/second and `omega` is counterclockwise in
radians/second, matching the [transport contract](transport.md) and
[protocol](../protocol/robot-protocol.md). `unicycleDerivative` returns:

```text
dx/dt     = v cos(theta)       [m/s]
dy/dt     = v sin(theta)       [m/s]
dtheta/dt = omega              [rad/s]
```

It evaluates instantaneous derivatives, without time integration, angle wrapping,
pose mutation, or a controller. Input headings may be unnormalized radians.

`DifferentialDriveGeometry` supplies wheel radius `r` and wheel-centre separation
`b` (`wheelTrack`), both in metres. No physical defaults are assumed. Wheel angular
targets `qL`, `qR` are in rad/s; positive on either side rolls the robot forward,
regardless of motor wiring or mounting orientation:

```text
qL = (v - omega b/2) / r
qR = (v + omega b/2) / r

v     = r (qL + qR) / 2
omega = r (qR - qL) / b
```

For example, with `r = 0.02 m`, `b = 0.10 m`:

| v (m/s) | omega (rad/s) | qL (rad/s) | qR (rad/s) | Motion |
| --- | --- | --- | --- | --- |
| 0.20 | 0 | 10 | 10 | Forward |
| -0.20 | 0 | -10 | -10 | Reverse |
| 0 | 2 | -5 | 5 | Counterclockwise rotation |
| 0 | -2 | 5 | -5 | Clockwise rotation |
| 0.20 | 2 | 5 | 15 | Counterclockwise arc |

## Bounds and failure behavior

`toBoundedWheelAngularVelocity` takes an explicit symmetric maximum wheel angular
speed `Q` in rad/s. It computes both wheel targets, then multiplies them by
`min(1, Q / max(abs(qL), abs(qR)))` for a nonzero peak. This preserves signs and
curvature (and scales `v` and `omega` together); it does not clip each wheel
independently. Stationary input stays stationary. A zero limit returns zero wheels.
For the arc above, `Q = 7.5 rad/s` yields `(2.5, 7.5) rad/s`, equivalent to
`v = 0.10 m/s`, `omega = 1 rad/s`.

All functions return `std::optional` and are `noexcept`. Nonfinite inputs,
nonfinite intermediate conversion values/results, nonpositive geometry, and
negative/nonfinite limits return `nullopt`. A nonrepresentable unbounded
conversion is rejected even if a limit is supplied. Callers must handle failure
before issuing motion; failure is not a valid motion target. Every value in a
supplied pose must be finite, including its position.

These helpers are available to future desktop controllers/simulation without
changing the current demo or transport behavior. They do not validate tracking,
connection state, command age, or E-stop state. Freshness, body-command limits,
and scheduling belong to their respective application services. Physical wheel
PID, PWM mapping, actuator limits, motor polarity, and the watchdog remain on the
ESP32-C6. The geometry helper alone does not authorize motion.

## Verification

`micro_robot_control_tests` runs without Qt or physical hardware, covering
straight/reverse motion, both rotation signs, arcs/pivots, inverse conversion,
world-frame derivatives, bounds, invalid data and overflow. Platform boundary
checks cover the new library. No hardware test is required for the pure model;
physical geometry, polarity and wheel response must be verified when integrating
real robots. Linux/Windows runtime validation is separate from local macOS tests.
