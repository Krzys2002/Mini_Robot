# RobotManager sessions and command routing

Issue [#67](https://github.com/Krzys2002/Mini_Robot/issues/67) adds the Qt-free
application service in [RobotManager.hpp](../../src/app/RobotManager.hpp) and
[RobotManager.cpp](../../src/app/RobotManager.cpp), target
`micro_robot_robot_manager`. The [composition root](../../src/app/main.cpp)
injects an `IRobotTransport`, the authoritative `WorldModel` and an optional log
channel. The service coordinates connection lifetimes and keeps communication
state by `RobotId`; it never exposes radio addresses, GATT objects or BLE tokens.

## Ownership and identity

One manager exclusively controls its injected transport's requests and weak event
subscription. The composition root owns the actual transport and must keep it,
WorldModel and the logger alive longer than the manager. Copy/move are disabled.
All calls/events run on the same owning thread; the service introduces no worker,
polling loop, hidden clock or unbounded event queue.

`addRobot(id, registeredAt)` explicitly creates a session and registers an unknown
pose in WorldModel if needed. Existing world registration/poses remain unchanged.
Duplicate IDs, shutdown-time additions and adoption of already active backend
sessions are refused. Unknown callbacks never create sessions or world records.
The adapter privately resolves each `RobotId` to its endpoint and verifies identity;
for BLE, duplicate advertisements or mismatched Robot Info remain adapter errors.

`RobotSession` is a detached communication-state value: connection state/time,
connect intent, pending removal, latest telemetry and last error. `session(id)`
returns an optional copy; `sessions()` returns a copy ordered by RobotId. These
values contain no robot poses. WorldModel remains the sole pose registry, and
telemetry cannot create a camera pose or establish fresh tracking. The current
service uses one selected transport for its robot set; mixed-radio routing is not
introduced.

## Requests and state

| API | Behavior |
| --- | --- |
| `connect(id)` | Explicit request for a registered robot; repeated current connect intent is idempotent; waits for terminal state before a new connection after disconnect |
| `disconnect(id)` | Cancels connection intent, clears active telemetry immediately, asks the adapter to cancel/disconnect; retains configured session and world pose |
| `removeRobot(id)` | Removes an idle session immediately; otherwise requests disconnect and retains the record until terminal acknowledgement |
| `sendCommand(id, command)` | Routes unchanged command metadata/payload to that RobotId only when connected and not removing/shutting down |
| `stop(id, at, validFor)` | Routes a semantic `StopCommand` with caller-supplied monotonic issue time and validity; not an E-stop latch |
| `supportsEmergencyStop()` / `requestEmergencyStop(id)` | Optional dedicated backend latch; records persistent intent, blocks Velocity and permits ordinary Stop; returns submission only |
| `shutdown()` | Idempotently cancels/disconnects every managed robot and refuses new connections/commands |
| `isStopped()` | All remaining published sessions are terminal and have no connect intent |

Unrequested Connecting/Connected events are intentionally cancelled, including
late completion after cancellation; they never authorize commands. Disconnecting
progress does not submit another disconnect request. A new connect during teardown
returns false; the desktop reports the refusal to the operator. This intentionally
requires test/manual backends to request a connection before publishing Connected.

Cancellation before a backend starts can leave it already Disconnected, with no
terminal event to emit (for example simulation connect+disconnect before
`advanceTo`). Removal can finish immediately when both manager and adapter already
publish a terminal state. Events for removed IDs are ignored. Native request
cancellation and old radio-token rejection remain the adapter's responsibility.
Removing a communication session deliberately retains its WorldModel record;
retirement of robot observations is a separate explicit world operation.

`CommandSubmission` distinguishes `Submitted`, `UnknownRobot`, `NotConnected`,
`ShuttingDown` and `EmergencyStopped`. Submitted means handed to the adapter, never
radio delivery or
execution. The adapter still validates finite setpoints/domain versions/positive
validity and rechecks freshness at dispatch. Stop is an ordinary transport command;
it does not bypass the link or replace the robot watchdog or future domain E-stop
policy. The manager generates no autonomous motion and supplies no tracking-based
permission policy; future supervisory control must use calibrated fresh poses.

Shutdown is deliberately connection teardown, not a motion-stop protocol. It
does not send Stop; after shutdown begins command submission is refused. Physical
stopping relies on robot-side stop-on-disconnect/watchdog behavior. A Stop queued
immediately before disconnect can be cancelled by an adapter, so an orderly stop
requires a separate delivery/acknowledgement policy before teardown (#69–#72).
No physical stop guarantee is made here.

The service lives in `src/app` because it coordinates application ownership and
WorldModel registration; the communication module describes its responsibility,
not a requirement to depend on Qt or to live with radio adapters.

## Telemetry, errors and observers

Callbacks update the matching session before notifying the weak observer through
`IRobotTransportEvents`. Unknown IDs are ignored. Only connected, requested,
non-removing sessions accept telemetry. Receipt timestamps before the current
connection or at/before the last accepted sample are ignored; unsupported domain
versions and non-finite optional measurements report an error without replacing a
valid sample. Non-finite measurements use `InvalidTelemetry`, while
`BackendFailure` remains a backend-operation failure. Unknown measurement fields remain optional; zero measurements remain
zero. Sequence metadata is copied for diagnostics, not interpreted as an ordering
or acknowledgement contract.

A terminal state clears active telemetry and connect intent; another robot's data
is untouched. Fault is quiescent and permits an explicit later `connect`, without
an automatic reconnect policy. The last error is diagnostic and does not invent a
connection transition; errors older than the current connection transition are
ignored. Connection timestamps cannot move backwards; the adapter
must already report the callback's state. Per-session history is bounded to one
telemetry sample and one latest error; no experiment aggregation is added.

Observers may detach/replace themselves or request disconnect/removal in a
callback. Map references do not survive notification, and retained backend sinks
are invalidated before manager destruction. Do not destroy the manager/transport
from inside a callback. Before destroying the actual transport, call shutdown and
drain its owning event loop or simulation clock until terminal state, outside
callbacks. The manager destructor requests disconnect and detaches safely; it
cannot block awaiting native radio completion.

## Desktop and backend selection

`DesktopController` receives the manager, not `IRobotTransport`. It forwards
connect/disconnect actions and presents one selected robot's telemetry/status;
other sessions stay separately queryable through the manager. Closing the
application requests manager-wide shutdown and waits for every managed session,
including non-selected robots, plus camera and perception/calibration workers.

The composition root injects `SimulationTransport` by default or `BleTransport`
with `--ble` in a BLE-enabled build. Neither the manager nor widgets link concrete
transports. Simulation's explicit `advanceTo` stays in composition/tests. Existing
BLE phase diagnostics remain adapter-local; command and telemetry paths use the
same service API for both implementations.

## Verification and limits

[RobotManagerTest](../../tests/app/RobotManagerTest.cpp) exercises identity
isolation, command/Stop routing and unchanged metadata, stale/incompatible
telemetry, explicit unknown values, fault/retry, session retirement, cancellation,
observer replacement/reentrancy, retained-sink destruction, shutdown and simulation
adapter validation. It runs with the `core` label without Qt/OpenCV or hardware.

[BleTransportTest](../../tests/transport/BleTransportTest.cpp) runs the manager
against two fake-radio endpoints with the real BLE adapter: verified identity,
command routing, decoded telemetry, asynchronous removal and complete shutdown.
Desktop tests exercise selected-robot isolation and all-session shutdown. Build
with `MICRO_ROBOT_BUILD_BLE=ON` for fake BLE coverage; no radio hardware is used.

At the maintainer's request, physical RobotManager/two-robot interoperability is
tracked separately in [#209](https://github.com/Krzys2002/Mini_Robot/issues/209).
**All physical checks are NOT RUN.** That procedure reuses the lower-level adapter/
protocol acceptance in [#189](https://github.com/Krzys2002/Mini_Robot/issues/189).
Native Linux BlueZ and
Windows execution are not established by these deterministic tests. Manual-motion
UI and the dedicated E-stop handoff are described in
[manual robot control](manual-robot-control.md). Broader safety/recovery policy,
automatic reconnect and measurements remain #70–#72; no M7 coordination is implemented here.
