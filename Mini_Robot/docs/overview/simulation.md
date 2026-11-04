# Deterministic camera and transport backends

Source: [FakeCameraSource.hpp](../../src/simulation/FakeCameraSource.hpp),
[FakeCameraSource.cpp](../../src/simulation/FakeCameraSource.cpp),
[FakeCameraSourceFactory.hpp](../../src/simulation/FakeCameraSourceFactory.hpp),
[FakeCameraSourceFactory.cpp](../../src/simulation/FakeCameraSourceFactory.cpp),
[SimulationTransport.hpp](../../src/simulation/SimulationTransport.hpp), and
[SimulationTransport.cpp](../../src/simulation/SimulationTransport.cpp).
Both implement the [camera](camera-source.md) and [transport](transport.md)
boundaries without Qt, physical devices, networking, or robot physics.

## Driving virtual time

Requests set pending intent. Events occur only during `advanceTo(Timestamp)` on
the owning thread; neither backend reads a clock, starts a worker, or sleeps.
Time may stay equal or increase. Backward time throws `std::invalid_argument`;
recursive advancement throws `std::logic_error`. Instances are non-copyable and
non-movable. Keep an event sink alive with shared ownership if events are needed;
backends keep only weak subscriptions. Callbacks may request stop/disconnect,
but must not throw, destroy the backend, or call advanceTo recursively.

The desktop composition uses a QTimer and passes `MonotonicClock::now()` to the
simulated transport; the fake camera factory is used by alternate compositions and tests.
[Tests](../../tests/simulation/SimulationTest.cpp) instead supply fixed timestamps.
Consumers depend on interfaces; only composition/tests drive concrete fake time.

## Fake camera

`FakeCameraConfig` defaults to source ID `fake-camera`, display name `Fake camera`,
33 ms interval, and seed 0. A non-positive interval throws at construction.
Frames are always 8 by 6 Gray8, stride 8, with owned immutable storage. For each
pixel index, the value is `(patternSeed + sequence + index) % 256`.

Start/stop requests coalesce and transition directly between Stopped and Streaming
at advancement. Cancelling a pending start emits a quiescent state acknowledgement
even if the state was still Stopped. At most one frame is emitted per call; missed intervals are
skipped. Sequence begins at 0, can have gaps, and remains increasing across
restarts. A repeated advancement in the same cadence tick emits no extra frame.
Frames use the supplied capture timestamp. Retained frames survive later advances
and source destruction. A stop requested in the Streaming callback suppresses
that call's frame. This fake does not decode recorded media or simulate camera
permissions, disconnection, configurable resolution, or optics.

`FakeCameraSourceFactory` implements [ICameraSourceFactory](camera-selection.md)
for deterministic tests and alternate compositions. It supports any device with an ID
and creates a fake source per selection, named `<device name> (simulated feed)`
with source ID `simulated:<device id>`; the configured interval and seed apply to
every source. Its `advanceTo(now)` advances each live created source in creation
order. Sources unregister on destruction and must not outlive the factory.

## Simulated transport

`SimulationTransportConfig` defaults to a zero telemetry interval, 3.7 V battery,
maximum linear/angular speeds of 32.767 m/s and rad/s (the v1 representable range),
and a 500 ms maximum command validity. Set lower simulated actuator limits when
modelling a particular robot. Invalid/non-finite limits, limits above the v1 range,
validity outside 1–65535 ms, negative cadence, or non-positive/non-finite battery
voltage throw `std::invalid_argument`. The desktop sets telemetry cadence from
`simulation.telemetry_interval_ms` (see [configuration](configuration.md)).

Robots are processed in ascending RobotId order. Connect/disconnect requests
coalesce, with direct Disconnected/Connected transitions at advancement. Each
robot has one pending command: latest wins, except a queued Stop takes precedence
over later motion, matching BLE. Disconnect cancels pending commands and clears
active motion/session state. New sessions created in callbacks wait until the next
advancement. Connect explicitly recovers a quiescent Fault.

Both current `RobotCommand` alternatives (Stop and Velocity) use the same
`IRobotTransport` / [RobotManager](robot-manager.md) boundary as BLE. Desktop
composition already selects simulation or BLE; no command conversion is required
in UI/control consumers. Emergency stop uses the separate transport operation: it
latches, clears pending and active motion, reports zero velocity and emergency-stop
state, and rejects later velocity commands even after reconnect. No clear operation
is currently exposed.

Commands require a connected, desired-connected destination, supported domain
version, positive validity, `issuedAt <= now`, and at least one whole millisecond
of remaining validity at dispatch, matching the current BLE encoder. Velocity
components must be finite and within configured limits, with no silent clamping.
They are rounded to mm/s and mrad/s as a v1 device would receive. Disconnected
submissions report NotConnected; invalid timing/setpoints report InvalidCommand;
unsupported versions report UnsupportedVersion. Rejections preserve the existing
setpoint and never refresh its watchdog. Stop is freshness-checked on the desktop
side, but its accepted zero setpoint never expires on the simulated device.

The simulated watchdog uses `min(floor(remaining validity / 1 ms), configured
maximum validity)` from dispatch time. At or after its deadline, it clears motion,
sets the diagnostic `commandExpired` flag, and reports zero velocity. The next
accepted Stop or Velocity clears the flag. Advance virtual time to exercise this
without sleeps. A large time jump emits the resulting current state at the supplied
timestamp, rather than backfilling missed samples.

Caller command metadata is retained without ordering/deduplication, as required
by the domain contract. Independently, the adapter allocates a wrapping 16-bit
command sequence per accepted dispatch. `robotStatus(id)` exposes simulated
Idle/Moving state, command-expired flag, and last accepted adapter sequence to test
harnesses. It returns empty outside Connected. This is diagnostic device state,
not a pose registry or new UI/domain status API. `lastSentCommand(id)` remains a
copied historical record and may exist after expiry; it is not proof of actuation.

Telemetry publishes configured battery and simulated setpoint velocities through
the existing domain fields. It carries an independent monotonically increasing
sequence, starting at zero per session, rather than echoing caller metadata or
acknowledging a command. Periodic samples obey `telemetryInterval` and are emitted
at most once per distinct timestamp; Idle/Moving transitions and watchdog expiry
bypass cadence, including at an already sampled timestamp. Disconnect requested
in a callback suppresses telemetry for that session.

### Reproducing failures without hardware

`simulateDisconnect(id)` and `simulateFault(id)` queue respectively Disconnected
and Fault plus a BackendFailure/Disconnect error at the next advancement. They
clear pending and accepted motion, watchdogs and session counters, suppress that
robot's telemetry, and leave other robots unaffected. Repeated pending injections
coalesce. Recovery requires explicit `connect(id)`; it starts stopped with no
accepted command. There is no automatic reconnect or replay of previous motion.

```cpp
transport.sendCommand(id, {VelocityCommand{0.1, 0.0}, Timestamp{1s}, 100ms});
transport.advanceTo(Timestamp{1s});
transport.advanceTo(Timestamp{1100ms}); // watchdog: Idle, expired, zero velocity
transport.simulateFault(id);           // deferred, no inline callbacks
transport.advanceTo(Timestamp{1101ms}); // Fault + diagnostic error
transport.connect(id);
transport.advanceTo(Timestamp{1102ms}); // Connected, stopped, new session
```

There is no movement integration, motor dynamics, radio delay/loss, wire encoding,
or physical acknowledgement. `robotStatus` describes simulated device acceptance
only. The simulator does not validate physical stopping or BLE delivery. Manual
motion UI (#69), supervisory safety policy (#70), and hardware acceptance remain
owned by their respective issues. Linux/Windows runtime and physical hardware
checks must be reported separately from deterministic macOS tests.

### Communication log

An optional `LogChannel` passed at construction records simulated traffic with
the robot ID; the desktop composition uses the `simulation` module. Connect and
disconnect requests, submitted and accepted commands, and telemetry samples are
Debug records; connection state changes are Info; rejected commands are Warning
with the error code. Numbers use fixed three-decimal, locale-independent text, so
the same inputs produce the same messages. Without a channel nothing is logged.
Higher-level code still observes only `IRobotTransport` events.

```cpp
#include "simulation/FakeCameraSource.hpp"
#include "simulation/SimulationTransport.hpp"
#include <chrono>
#include <cassert>

int main()
{
    using namespace micro_robot;
    using namespace std::chrono_literals;
    FakeCameraSource camera;
    SimulationTransport transport;
    const RobotId id{1};
    camera.start();
    transport.connect(id);
    camera.advanceTo(Timestamp{1s});
    transport.advanceTo(Timestamp{1s});
    assert(camera.state() == CameraState::Streaming);
    assert(transport.connectionState(id) == ConnectionState::Connected);
    transport.sendCommand(id, {VelocityCommand{0.1, 0.2}, Timestamp{1s}, 100ms});
    transport.advanceTo(Timestamp{1050ms});
    assert(transport.lastSentCommand(id));
    camera.stop();
    transport.disconnect(id);
    camera.advanceTo(Timestamp{1100ms});
    transport.advanceTo(Timestamp{1100ms});
    assert(camera.state() == CameraState::Stopped);
    assert(transport.connectionState(id) == ConnectionState::Disconnected);
}
```
