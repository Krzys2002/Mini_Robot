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

`SimulationTransportConfig` defaults to a zero telemetry interval and a 3.7 V
battery. A negative interval, or a non-finite or non-positive voltage, throws
`std::invalid_argument` at construction. The desktop composition sets the interval
from `simulation.telemetry_interval_ms` (see [configuration](configuration.md)).

Robots are processed in ascending RobotId order. Connect/disconnect requests
coalesce, with direct Disconnected/Connected transitions at advancement.
Each robot has one pending command: the most recent submission wins. Disconnect
cancels pending commands and clears accepted state when an active session ends.
New sessions created in callbacks wait until the next advancement.

A command is accepted only when connected and desired-connected, with finite
velocity components, supported domain model version, positive validity, `issuedAt <= now`, and
`now - issuedAt < validFor`. Equal-to-deadline and future-issued commands are
invalid. Disconnected submissions report NotConnected; invalid timing/setpoints
report InvalidCommand. Unsupported domain versions report UnsupportedVersion
without replacing the accepted command. Optional sequence metadata is copied
without allocation, ordering, or deduplication. `lastSentCommand(id)` is an optional copied record of the
last accepted command; it may still exist after expiry and is not proof of
current actuation.

Telemetry is emitted at most once per distinct timestamp per connected robot: on
the first advancement of a session, then whenever at least `telemetryInterval`
has elapsed since that robot's previous sample (zero means every new timestamp).
It reports the configured battery voltage and echoes fresh accepted velocity
intent; Stop or expired intent reports zero velocity. Telemetry uses the current domain model version and an unassigned
sequence; it does not acknowledge the command's sequence. Disconnect requested in a callback suppresses that session's
telemetry. There is no movement integration, protocol encoding, radio latency,
command acknowledgement, local actuator limit, or real robot watchdog.

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
