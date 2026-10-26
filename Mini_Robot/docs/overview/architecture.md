# Current architecture by milestone

This page defines stable system boundaries and common conventions. Capability
status and feature verification belong to the [module pages](modules/README.md)
and their behavior guides. The [roadmap](../planning/MILESTONES.md) owns milestone
scope and exit criteria; live progress belongs to GitHub issues.

## Milestone status

Use the module index for the implemented subset of M1–M4. The roadmap defines
M1–M8 requirements, including later planning/coordination work. A type, interface,
successful build or closed issue alone does not establish physical acceptance.

## M1 — Implemented foundation

### Modules and dependencies

Module pages explain capability and ownership. The [source map](source-map.md)
explains how to generate the inventory for a configured build. Dependencies come
from CMake; adding sources/targets does not require another hand-maintained list.

### Current runtime data flow

The [desktop module](modules/desktop.md) and [desktop guide](desktop-demo.md) own
current composition and demo behavior. Preserve this dependency direction:

```text
CameraSource -> Vision / Calibration -> RobotTracker -> WorldModel
             -> Planning / Task -> Trajectory / Supervisory Control
             -> RobotManager -> RobotTransport -> ESP32-C6
```

### Camera and transport boundaries

Acquisition is separate from perception. Planning/control depend on domain-facing
interfaces, never concrete camera/radio APIs or widgets. [Camera](modules/camera.md)
and [communication](modules/communication.md) own their contracts and adapters.
Simulation/replay use compatible interfaces; device addresses and protocol handles
stay inside adapters.

### Simulation and authoritative state

One application-level [WorldModel](modules/world.md) owns current robot state.
Detections/telemetry are measurements, not competing authoritative poses.
[Simulation](modules/simulation.md) is a replaceable backend, not physical
acceptance or collision-safe coordination.

### Ownership, threading, and shutdown

GUI objects stay on the GUI thread. Worker results cross an explicit asynchronous
boundary. Queues and frame handoffs are bounded and prefer newest useful data.
Backend/model references outlive consumers. Immutable frames can be retained;
borrowed event values must be copied. Callbacks stay prompt and non-throwing.

Shutdown requests stop/disconnect, drains workers without blocking the GUI event
loop, then destroys objects in dependency order. Concrete lifecycle belongs to the
owning module. Safety decisions never use indefinitely stale state.

### M1 completion gaps

Acceptance gaps belong to module verification guidance, rather than duplicated
shared tables. Hardware/platform checks require executed evidence; compilation
is not physical acceptance.

## Domain conventions

These contracts apply across modules:

- `RobotId` is an explicit unsigned 32-bit application identity with comparison
  and hashing, independent of hardware addresses. Zero is valid; missing identity
  uses an optional value.
- `Pose2D` uses metres for x/y and radians for heading. In a top-down world view,
  +x is right and +y is up; heading is zero along +x and positive counterclockwise.
  The physical origin will be established by M2 workspace calibration. Image
  coordinates have a separate top-left origin with +u right and +v down.
- `Meters` and `Radians` are explicit value wrappers. They do not normalize
  angles, validate finite values, or provide dimensional arithmetic. Code that
  needs a canonical angle calls `normalizeAngle`, which wraps to (−π, π]. Convert
  display/protocol units at boundaries; WorldModel validates accepted poses.
- Timestamp and duration use steady-clock types. Epoch is unspecified and valid;
  it is not wall time, a persisted media offset, or a remote device clock. Missing
  observations use optional timestamps. State timestamps do not imply freshness.

Definitions are in [src/common](../../src/common). Configured source ownership
and dependencies are available through the [generated inventory](source-map.md).

## M2–M8 — Planned architectural growth

The roadmap owns future scope and sequencing. As issue-scoped features are added:

- Transform measurements into a common metric frame before tracking/control.
- Separate path geometry, timing, tracking control and multi-robot coordination.
- Keep fast wheel regulation, command watchdog and safe motor stop on robots.
- Isolate additional transports, cameras and depth sensors behind interfaces.
- Retain macOS/Linux portability and the Windows target without claiming unrun checks.

## Verification and maintenance

Update the owning module/behavior guide for feature changes. Edit this page when
shared boundaries or conventions change. Recipes live in the
[testing guide](../development/testing.md); ownership and merge order are in
[documentation maintenance](../development/documentation.md).
