# Robot transport contract

Source: [IRobotTransport.hpp](../../src/transport/IRobotTransport.hpp) and
[TransportTypes.hpp](../../src/transport/TransportTypes.hpp).

`IRobotTransport` is a Qt-free asynchronous boundary. Application `RobotId`
values are independent of BLE addresses, UUIDs, or Zigbee handles. Only the
[simulation backend](simulation.md) currently implements it in production source.
This interface is not a serialized wire protocol or a BLE implementation.

## Lifecycle and ownership

| Operation | Contract |
| --- | --- |
| `setEventSink(weak_ptr)` | Weak subscription; an empty sink detaches. |
| `connectionState(id)` | Unknown IDs return `Disconnected`. |
| `connect(id)` | Prompt, idempotent request; completion is a state event. |
| `disconnect(id)` | Cancels pending connect; completion is a state event. |
| `sendCommand(id, command)` | Submission only; returning does not confirm delivery or execution. |

`ConnectionState` includes Disconnected, Connecting, Connected, Disconnecting,
and Fault. Calls and callbacks are serialized on the owning thread. Requests do
not dispatch events inline. Event handlers must be short and non-throwing;
borrowed telemetry/error values must be copied if retained or queued. Owners
disconnect active robots, await terminal events asynchronously, and destroy the
backend on its owning thread outside a callback. Future workers must marshal
results back before GUI delivery. No internal queue or thread is provided by the
interface itself.

## Commands, telemetry, and errors

`RobotCommand` holds either `StopCommand` or `VelocityCommand`. Velocity uses
metres/second forward in the robot body frame and radians/second counterclockwise.
`issuedAt` is local monotonic time; `validFor` must be positive. The default command
has a Stop payload but zero validity, so callers must fill in timing explicitly.
Backends copy retained commands and validate finite setpoints and freshness again
before dispatch. Disconnected or invalid submissions report an error event.

`RobotTelemetry` contains RobotId, local receipt time, and optional battery voltage,
linear velocity, and angular velocity. Missing values are unknown. Telemetry is
not an authoritative pose update: pose storage remains in [WorldModel](world-model.md).
`TransportError` carries identity, operation, portable error code, message, and
local timestamp. Portable codes are Unavailable, NotConnected, InvalidCommand,
and BackendFailure.

Sequence numbers, acknowledgements, encoding, and remote clock conversion are
future adapter/protocol responsibilities. No GATT layout, command saturation,
robot motor controller, or real safety watchdog is implemented here.

[Interface tests](../../tests/interfaces/InterfacesTest.cpp) exercise abstract
substitution, copied event values, weak subscriptions, and virtual destruction.
[Simulation tests](../../tests/simulation/SimulationTest.cpp) verify the concrete
fake's cancellation and dispatch-time freshness rules.
