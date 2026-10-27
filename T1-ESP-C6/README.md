# Legacy HTTP control prototype

This Arduino/CodeCell experiment is not the desktop's production BLE endpoint.
The production runtime, authorization/session model, command protocol, bounded
stop/deceleration policy and electrical safety acceptance remain #158/#160/#162
and their prerequisites. Do not infer motor, watchdog or E-stop interoperability
from the separate connection-only BLE fixture in PR #213.

## Local safety correction

The existing 300 ms command validity is now checked independently by the actuator
task, at every existing 20 ms scheduler cycle, including the exact expiry boundary.
Every accepted command gets a generation; computed motor output carries that
generation. Fresh incoming traffic cannot keep an old computed output alive if
control work stalls. Command handoff contention produces zero output rather than
an unbounded wait. The 32-bit millis subtraction is wrap-safe; expired intent
is invalidated and cannot revive on a later clock wrap.

E-stop latches outside the command mutex. The actuator task observes it each
cycle; it is not a measured immediate electrical-disable guarantee. Clear zeros
pending intent and output, invalidates pre-clear computations and requires a new
post-clear command. A concurrent Stop during clear stays latched. Commands during
E-stop/fault are refused; initialization/task allocation failures latch a fault
and do not start normal motion. This remains a software gate: actuator-task,
CPU, driver or electrical failure requires the production hardware safety work.

Only the motor task writes actuators after setup. Gate checks and motor writes
are serialized with command/clear handoffs. The control task retains legacy IMU
and target experiments; this patch does not validate their motion accuracy or
make HTTP an authenticated production control channel.

## Browser command ownership

The page sends periodic direct commands only during an active joystick gesture.
Pointer release submits zero once; unrelated pointer release/idle refresh does not
replace target mode. Motion HTTP requests are serialized, and mode changes cancel
queued old intent. E-stop bypasses the motion queue. Clear waits for prior stop
and old requests to finish, then requires a new joystick gesture. Merely loading
the page issues no motion command. The one-shot experimental target still expires
after the existing validity window; this change does not silently extend it.

## Verification

From the repository root: `pio run -d T1-ESP-C6` builds the firmware; no upload is
part of this command. Record the resolved PlatformIO/Arduino/CodeCell versions,
because the inherited prototype dependency URLs are not immutable.

From `Mini_Robot`: build a normal CMake configuration and run
`ctest --test-dir build/macos -R 'ActuatorSafety|PrototypeController' --output-on-failure --no-tests=error`.
The host target compiles the actual controller against deterministic clock,
semaphore, scheduler and CodeCell fakes; it tests stalled control, exact expiry,
pre-clear snapshots, contention and fault handling. Pure gate tests include wrap.
Run `node Mini_Robot/tests/firmware/browser-control.test.js` from the repository
root for five asynchronous browser command-ordering scenarios.

Firmware compilation and deterministic tests do not establish physical stopping.
Bench tests still need a safely restrained robot, output measurements around
expiry/E-stop/clear, deliberately stalled control work, reset/power-failure checks
and the measured limits selected by #156. All such hardware checks for this patch
are NOT RUN. No production firmware issue is closed by these focused corrections.
