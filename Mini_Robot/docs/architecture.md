# Current Phase 0 architecture

The desktop target uses Qt Widgets through `micro_robot_ui`. Common domain types
are provided by the header-only C++20 `micro_robot_common` target, which has no Qt
or hardware dependencies. `micro_robot_common_tests` links only common types and
GoogleTest; the separate `micro_robot_tests` target covers the Qt window.

## Domain conventions

- `RobotId` is an explicit unsigned 32-bit application identifier with value
  comparison and hashing. It is not a hardware address. Zero is valid; absence
  should be represented with `std::optional<RobotId>`.
- `Pose2D` stores `x` and `y` as `Meters`, and `heading` as `Radians`. In the
  top-down world map, +x points right and +y points up; heading starts along +x
  and increases counterclockwise. The physical origin will be set by calibration
  in a later phase. Image pixels are a separate coordinate system.
- `Meters` and `Radians` wrap a `double` with explicit construction and explicit
  access through `.value`. They default to zero and do not perform validation,
  dimensional arithmetic, or angle normalization. Convert display/protocol units
  at their respective boundaries.
- `Timestamp` and `Duration` use `std::chrono::steady_clock`. Use chrono duration
  arithmetic for elapsed time and freshness. Its epoch is unspecified; these
  timestamps are not wall-clock dates or portable persisted/device timestamps.
  Missing timestamps should use `std::optional<Timestamp>`.

## Asynchronous interfaces

`micro_robot_camera_api` and `micro_robot_transport_api` are header-only targets
depending only on common types. Their abstract interfaces and event sinks are
Qt-free. Deterministic fake backends implement these contracts in simulation.

`ICameraSource` exposes start/stop requests, source information, state, and frame,
state-change, and error events. Frames carry a sequence, local capture timestamp,
dimensions, pixel format, stride, and shared immutable owned storage. Producers
must not reuse retained buffers; delivery queues must be bounded, preferring the
latest frame. Qt/OpenCV image conversions belong in future adapters.

`IRobotTransport` exposes per-robot connect/disconnect, connection state, command
submission, and telemetry/state/error events. Commands carry stop or velocity
intent, a local issue timestamp, and a positive freshness duration. This defines
data contracts only, not control behavior. Telemetry uses local receipt timestamps
and optional measurements; absent fields are unknown, not zero. Wire formats,
radio addresses, protocol counters, and device clocks remain backend details.
Submitting a command does not confirm robot delivery or execution.

The application owns backends with `std::unique_ptr` and owns event sinks with
`std::shared_ptr`. Backends retain only `std::weak_ptr` sinks and lock them for each
dispatch. A callback temporarily retains its recipient. Empty subscriptions
detach; expired recipients are skipped. Reference arguments must be copied when
retained; shared immutable frames can be retained directly.

Each backend is thread-affine: public calls and event dispatch are serialized on
its owning thread. Requests return promptly and events are deferred. Future Qt
worker implementations marshal results to that thread and bridge events to the
GUI with queued signals/slots. Event sinks are plain C++ adapters, not shared-owned
GUI widgets. Use receiver-context Qt connections to handle GUI receiver lifetimes.
Callbacks must be short and non-throwing, and must not destroy the backend.

Stop/disconnect cancel pending startup/connection requests. Repeated requests are
idempotent. Owners await terminal state events asynchronously before destroying
backends on their owning thread; implementations must cancel outstanding work
and deliver no callbacks after destruction. A Stopped/Disconnected/Fault terminal
state must mean that the relevant stream/session has quiesced. No busy waiting or
blocking device I/O belongs on the GUI thread.

Interface tests use test-only doubles to check polymorphic dispatch, event data,
frame retention, weak subscriptions, and virtual destruction. They cannot prove
future backend scheduling/validation behavior; concrete backends must test those
contract requirements when implemented.

## Deterministic backends

`micro_robot_simulation` contains `FakeCameraSource` and `SimulationTransport`,
with no Qt, camera, radio, or network dependency. Composition/test code drives
each backend with `advanceTo(Timestamp)`. A future Qt adapter can advance virtual
time from a timer; consumers continue to use only the abstract interfaces.
Neither implementation reads wall-clock time, starts threads, or sleeps.
Nondecreasing timestamps are required; recursive advancement is rejected.
Requests coalesce and emit no inline callbacks. State changes are deliberately
direct (Stopped/Streaming or Disconnected/Connected), without simulated delays.

The camera emits at most one 8x6 Gray8 frame per advancement. Its configurable
positive cadence and pattern seed produce repeatable pixels from sequence and
pixel index. Missed intervals are skipped rather than queued. Frames are stamped
with the supplied virtual capture time; retained pixel buffers are immutable and
independently owned. Restarting resets cadence but preserves increasing sequence
numbers. Stop cancels pending start and prevents further frame delivery.

The transport processes robots in ascending ID order, with one pending command
per robot. New commands supersede pending ones; disconnect cancels pending
commands and clears accepted state. Commands are copied and validated at dispatch
for connection, positive validity interval, finite setpoints, and freshness;
future-issued and expired commands report InvalidCommand. The deadline is
exclusive: elapsed time equal to validFor is stale. lastSentCommand() provides
test/composition inspection of the last accepted command in the current session.

Telemetry is emitted once per new virtual timestamp for each connected robot.
It reports a fixed 3.7 V battery and echoes accepted velocity intent while fresh;
stop or expired intent reports zero velocity. This is architectural test data,
not robot physics or a real watchdog. Newly created sessions during a callback
wait until the next advancement. Callback stop/disconnect requests suppress frame
or telemetry delivery for that session. Weak subscriptions may expire or detach
without retaining recipients. Backend instances cannot be copied or moved;
transfer ownership through unique_ptr instead.

Simulation tests cover deterministic replay, cancellation, freshness, retained
frame ownership, bounded pending data, callback handling, and interface-based
consumer substitution with separate test-only implementations. The simulation
test executable links no Qt libraries. The desktop is not wired to simulation yet.

World model, real hardware backends, logging, and configuration remain unimplemented.
