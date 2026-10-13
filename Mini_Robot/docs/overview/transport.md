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
and Fault. State is per robot, and state queries reflect transitions before the
event runs. Connecting/Disconnecting are optional pending states; simulation
transitions directly. Disconnected/Fault are quiescent for that robot, with no
pending work. Connect may attempt recovery from Fault; disconnect on a quiescent
Fault may preserve it. Errors are diagnostic and do not implicitly change state;
terminal failures also publish Fault. Calls and callbacks are serialized on the owning thread. Requests do
not dispatch events inline. Event handlers must be short and non-throwing;
borrowed telemetry/error values must be copied if retained or queued. Owners
disconnect active robots, await terminal events asynchronously, and destroy the
backend on its owning thread outside a callback. Future workers must marshal
results back before GUI delivery. No internal queue or thread is provided by the
interface itself. The interface is not thread-safe; serialize all access, including
sink replacement. Callbacks may request connect/disconnect/send or replace the
sink, without nested event dispatch. Replacing a sink affects subsequent events,
while the current callback keeps its recipient alive. An expired sink suppresses
delivery without changing backend state. No blocking device I/O belongs on the GUI
thread; future GUI composition needs an asynchronous worker bridge.

## Commands, telemetry, and errors

`RobotCommand` holds either `StopCommand` or `VelocityCommand`. Velocity uses
metres/second forward in the robot body frame and radians/second counterclockwise.
`issuedAt` is local monotonic time; `validFor` must be positive. The default command
has a Stop payload but zero validity, so callers must fill in timing explicitly.
Backends copy retained commands and validate finite setpoints and freshness again
before dispatch. The accepted time window is `issuedAt <= now` with elapsed age
strictly less than `validFor`; expiry at the exact deadline is rejected. This
applies to Stop too. Disconnected or invalid submissions report an error event.

`RobotTelemetry` contains RobotId, local receipt time, and optional battery voltage,
linear velocity, and angular velocity. Missing values are unknown. Telemetry is
not an authoritative pose update: pose storage remains in [WorldModel](world-model.md).
`TransportError` carries identity, operation, portable error code, message, and
local timestamp. Portable codes are Unavailable, NotConnected, InvalidCommand,
BackendFailure, and UnsupportedVersion. An UnsupportedVersion error identifies
SendCommand or ReceiveTelemetry and includes the robot ID and local timestamp.

## Version and sequence reservation

Commands and telemetry carry an owned `TransportMessageMetadata` value:

| Field | Domain rule |
| --- | --- |
| `modelVersion` | Unsigned 16-bit domain schema version; defaults to `currentTransportModelVersion` (1). Only 1 is currently supported. Zero and unknown versions are rejected. |
| `sequence` | Optional unsigned 64-bit reserved message identifier. Empty means unassigned/unknown; zero and the maximum value are valid assigned values. |

`isSupportedTransportModelVersion()` provides the shared version check. Adapters
must reject unsupported command versions asynchronously with UnsupportedVersion
before accepting a command. Incoming telemetry must be converted into a supported
domain version before publication; if that is impossible, report UnsupportedVersion
for ReceiveTelemetry and drop it. Future schema changes extend domain types and
their version handling independently of GUI or concrete radio APIs.

Metadata is appended with defaults, preserving existing aggregate callers.
Backends copy metadata with retained commands and events. Sequence is reserved
without ordering, deduplication, acknowledgement, or freshness semantics: callers
may omit it, and a lower/repeated value alone does not invalidate a command.
Do not use sequence as a substitute for `issuedAt`/`validFor` or treat telemetry's
sequence as acknowledgement of a command.

These are domain values, not a wire header. [Issue #64](https://github.com/Krzys2002/Mini_Robot/issues/64)
will define wire versions, sequence allocation/session scope/wraparound, correlation
and acknowledgements, encoding, byte order, remote clock conversion, and BLE GATT.
Adapters may map a wire version into domain version 1; they must not assume the
two numbers coincide. No GATT layout, command saturation, robot motor controller,
or real safety watchdog is implemented by #44.

Simulation preserves submitted metadata in `lastSentCommand()`, rejects unsupported
versions without replacing the prior accepted command, and emits version-1
telemetry with an unassigned sequence. It does not allocate sequences or simulate
wire ordering. Existing accepted intent still expires normally after a rejected
submission; this is not a real robot safety mechanism.

```cpp
using namespace micro_robot;
using namespace std::chrono_literals;
RobotCommand command{VelocityCommand{0.1, 0.2}, now, 100ms};
command.metadata.sequence = 42; // optional reserved identifier
transport.sendCommand(RobotId{7}, command); // IRobotTransport&, no radio details
```

[Interface tests](../../tests/interfaces/InterfacesTest.cpp) exercise abstract
substitution, copied event values, weak subscriptions, and virtual destruction.
[Simulation tests](../../tests/simulation/SimulationTest.cpp) verify the concrete
fake's cancellation and dispatch-time freshness rules.
[Domain tests](../../tests/transport/TransportTypesTest.cpp) verify metadata
defaults/copying, valid sequence extremes, and explicit version support without
Qt or a backend. Simulation tests also cover unsupported-version diagnostics,
metadata preservation, exclusive expiry, and extreme monotonic timestamps.

## Issue scope and verification

Issue #44 supplies the transport/domain boundary; its blocker #25 supplies RobotId
and monotonic time. Tests use simulation and a separate test transport through
the same interface. No physical BLE/Zigbee implementation or hardware test is
required for this boundary. Linux and Windows builds/hardware remain unverified
on the macOS development host.

Build normally using the [test guide](../development/testing.md), then select:

```sh
ctest --test-dir build/debug -R '^(TransportDomainTest|TransportInterfaceTest|SimulationTransportTest|BackendSubstitutionTest)\.' --output-on-failure --no-tests=error
```
