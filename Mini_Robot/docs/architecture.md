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
test executable links no Qt libraries.

## World model

`micro_robot_world` depends only on common domain types. `WorldModel` owns a private
registry keyed by `RobotId`; ordered storage makes snapshots deterministic for
multiple robots. It performs no capture, communication, tracking/filtering,
planning, or rendering.

Robots are registered explicitly with a caller-supplied monotonic timestamp.
`RobotState` records identity, registration time, optional world pose, and optional
pose update time. Unknown pose remains absent rather than defaulting to the origin.
Registration does not imply a camera observation or transport connection.
Duplicate registration leaves existing state unchanged. Removal affects only the
requested ID; re-registration creates a fresh record with no previous pose.

`updatePose` stores finite world coordinates and heading as supplied, without
normalization or smoothing. It rejects unknown IDs, non-finite components, updates
before registration, and updates at or before an existing pose timestamp. The
first pose may have the same timestamp as registration. `PoseUpdateResult` exposes
the result; rejected updates do not mutate state. Each robot has independent
timestamp ordering, with no global observation timestamp requirement.

Use the model on one owning thread, with application orchestration as the writer.
Lookups return optional value copies; `WorldSnapshot` owns a full registry copy.
Neither can modify authoritative state or become invalidated by later updates.
These values can be delivered to the GUI through queued application-layer updates.
The model reads no clocks and contains no Qt callbacks or internal worker threads.

World-model unit tests link only the common/world targets and GoogleTest. Real
hardware backends, logging, and configuration remain unimplemented.

## Phase 0 desktop integration

`main.cpp` is the composition root: it owns FakeCameraSource, SimulationTransport,
WorldModel, the injected DesktopController, MainWindow, and a simulation QTimer.
Only composition code knows concrete backend classes. The controller target links
Qt Core, interface targets, and WorldModel; the UI target adds Qt Widgets and has
no dependency on simulation or hardware implementations.

MainWindow contains buttons and labels. Button signals connect to controller
slots; controller signals update source state, frame metadata, connection status,
telemetry, and a description of stored world state. No device-state polling or
business logic lives in the widgets. Frames are described, not rendered as video;
camera discovery and real camera selection remain Phase 1 work.

Connection events distinguish an established connection from an active pending
attempt. The UI offers disconnect/cancel while Connecting or Disconnecting; demo
pose production starts only after Connected. Initialization is idempotent and
cannot return a shutdown controller to Ready. Tests exercise these transitions
using alternative manual backends injected into the same controller and window.

The controller owns a plain event sink, with weak subscriptions from the backends.
Sources, model, and controller share the GUI thread in Phase 0. Dependencies
outlive the controller, whose destructor detaches both subscriptions. Qt receiver
contexts manage signal connection lifetimes. Future worker implementations must
marshal their events to the controller's owning thread before using this sink.

Startup registers Robot 1 with an unknown pose, leaving camera and transport idle.
The timer advances simulation every 100 ms; source callbacks drive UI changes.
While connected, composition code supplies a static pose (0.25 m, 0.15 m, 0 rad)
with the current monotonic timestamp. This demo observation is independent of
frames and telemetry; it demonstrates WorldModel storage without tracking or
physics. Disconnection keeps the last stored pose and displays inactive telemetry.

On application exit the timer stops, controller shutdown requests stop/disconnect,
and composition code advances both fakes once to drain terminal state events.
Only then are widgets, controller, model, and backends destroyed. UI integration
tests drive virtual time explicitly, exercise buttons/signals/state displays, and
check shutdown and subscription lifetimes without physical hardware.

## Phase 0 review status

The root CMake file manages dependencies and build options; source and test
directories define targets. `MICRO_ROBOT_BUILD_DESKTOP` defaults to ON. Setting it
to OFF omits Qt discovery, the controller, widgets, desktop executable, and GUI
tests; common, interface, world, and simulation tests then configure/build without
Qt installed. Unused GoogleMock targets are disabled in the GoogleTest fallback.
GoogleTest may be supplied as an installed package or through the pinned fallback;
an offline build may set `FETCHCONTENT_SOURCE_DIR_GOOGLETEST` to cached sources.

The standard library owns domain state and simulation buffers. Qt owns widgets
and layouts through parent relationships; raw widget aliases are non-owning.
Backends use weak event subscriptions, and application declaration order keeps
referenced dependencies alive through controller teardown. All Phase 0 callbacks
are serialized on the GUI thread; the controller and WorldModel are not thread
safe. Worker-thread acquisition/transport and a queued event bridge are future
work, not implied support in this implementation.

Logging and configuration-file loading are still missing Phase 0 requirements.
The FakeCameraConfig constructor parameters are not a configuration-loading
subsystem, and Qt diagnostic messages are not application startup/shutdown logs.
No logger, configuration loader, or related tests were added during this review.
Windows compatibility follows the portable code/CMake boundaries, but Windows
builds and deployment packaging have not been verified.
