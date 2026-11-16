# CodeCell C6 v1 codec and command validation

Status: implemented by [#161](https://github.com/Krzys2002/Mini_Robot/issues/161) on
top of the [production runtime](firmware-runtime.md) (#158),
[identity provisioning](firmware-provisioning.md) (#159) and the
[bounded drive](firmware-drive.md) (#160). It is the robot side of
[protocol version 1](../protocol/robot-protocol.md) (#64). The desktop side is
[message handling](ble-messages.md) (#66).

This is portable C++20 with no BLE, GATT, motor or clock calls. The
[BLE peripheral](firmware-ble.md) (#163) owns one `CommandSession` and feeds it every
Command and Emergency Stop write; telemetry publication is
[#164](https://github.com/Krzys2002/Mini_Robot/issues/164). Host tests and a compile check
show that the bytes and rules are right. They do not show physical stopping.

## Files

| Header | Contents |
| --- | --- |
| [`protocol/V1Wire.hpp`](../../firmware/codecell-c6/include/protocol/V1Wire.hpp) | Frozen v1 constants: types, exact lengths, offsets, flags, sentinels, result codes, bounded little-endian access |
| [`protocol/V1Codec.hpp`](../../firmware/codecell-c6/include/protocol/V1Codec.hpp) | `decodeCommand()` (validation steps 1–4), Velocity validity/range checks, wire-to-SI conversion, Telemetry/CommandResult/Robot Info encoders |
| [`protocol/V1Sequence.hpp`](../../firmware/codecell-c6/include/protocol/V1Sequence.hpp) | `isNewer()` serial comparison, per-session `lastAccepted` (`CommandSequenceState`) and the robot transmit counter |
| [`protocol/V1CommandSession.hpp`](../../firmware/codecell-c6/include/protocol/V1CommandSession.hpp) | One session: a written frame becomes at most one authority call and exactly one CommandResult |
| [`docs/protocol/fixtures/v1-frames.txt`](../protocol/fixtures/v1-frames.txt) | Shared byte vectors, read by both the firmware and the desktop tests |

## Wire to domain mapping on the robot

| Wire (v1) | Robot domain | Rule |
| --- | --- | --- |
| Velocity `linear` (int16 mm/s) | `BodyVelocity::linearMetresPerSecond` | `raw / 1000`, after validation |
| Velocity `angular` (int16 mrad/s) | `BodyVelocity::angularRadiansPerSecond` | `raw / 1000`, counter-clockwise positive |
| Velocity `validForMs` (uint16) | `std::chrono::milliseconds` to `IMotionCommandSink::velocity()` | `1..maxCommandValidityMs`. The deadline is measured from the receipt time on the robot clock |
| Receipt | `runtime::Instant` | Stamped by the BLE adapter at callback entry (#163), never desktop time |
| Header `sequence` | Not passed to the authority | Stays in `CommandSession`. The authority sees no wire field |
| Stop / EmergencyStop / ClearEmergencyStop | `stop()` / `emergencyStop()` / `clearEmergencyStop()` | One call per accepted frame |
| Telemetry `state`, CommandResult `state` | `runtime::MotionState` | Same numeric values. An impossible value is reported as `Fault` |
| Telemetry `lastAcceptedSequence`, flag bit 2 | `CommandSequenceState::lastAccepted()` | Filled in by the session, never by the caller |
| Telemetry battery, measured velocity | `std::optional` in wire units | Empty becomes `0xFFFF` / `0x8000`. A known value equal to a sentinel is refused. `measuredToWire()` rounds half away from zero and never clamps |
| Telemetry `uptimeMs` | Robot monotonic uptime | Diagnostic only, wraps |
| Robot Info | `provisioning::RobotInfoValues` (#159) plus a `FirmwareVersion` | Protocol range is always exactly `1..1`. The firmware version is a build input (#164) |

The advertised limits and the enforced limits come from one provisioned record.
`commandLimits(robotInfoValues(config))` gives the session its integer limits, and
`runtimeConfigFor(config)` gives the authority the same values in SI
(`mm / 1000`). A test checks every advertised maximum from 1 to 32767 at its
boundaries, plus every int16 value for several maxima. In all of them the integer
rule `abs(raw) <= max` and the authority's SI comparison agree.

## Validation order

`CommandSession::onCommandWrite()` applies the specified order to every Command
characteristic write:

1. Fewer than 2 bytes: `MalformedLength`, echoing type `0` and sequence `0`.
2. Byte 0 is not `0x01`: `UnsupportedVersion`. Byte 1 is echoed, and bytes 2–3 when present.
3. The type is not `0x01`–`0x04` (reserved, `0x00` or robot-to-desktop): `UnknownMessageType`.
4. The length is not exactly the type's length: `MalformedLength`. Trailing bytes are never ignored.
5. Then, in this order:
   - **sequence**: Velocity and ClearEmergencyStop must be newer than
     `lastAccepted`, otherwise `StaleSequence`;
   - **state**: `Fault` gives `RobotFault`, and the latch gives `EmergencyStopLatched`;
   - **validity**: `InvalidValidity`;
   - **range**: `OutOfRange`.

The authority keeps every motion decision. It checks state atomically inside the
actuator gate. A Velocity whose validity or range is invalid never reaches it. The
session then reports a state rejection first, using the runtime status at that
moment. A concurrent transition can change only which rejection is reported, never
motion. A rejected frame changes neither motion, the deadline nor `lastAccepted`.

The authority (#158) caps a too-long validity at the ceiling. v1 instead rejects
`validForMs > maxCommandValidityMs`. The session therefore rejects such a frame
before the authority sees it, and the authority's cap stays as defence in depth.

| Authority outcome | v1 result |
| --- | --- |
| `Accepted` | `Accepted` |
| `RejectedFault` | `RobotFault` |
| `RejectedEmergencyStopLatched` | `EmergencyStopLatched` |
| `RejectedNotReady` | `RobotFault` (v1 has no "not ready" code; see below) |
| `RejectedInvalid` | `InvalidValidity` |
| `RejectedOutOfRange` | `OutOfRange` |
| `RejectedSessionNotReady` | `RobotFault` (never sent: v1 sends no result before the session is ready) |

`RejectedNotReady` means the runtime has not finished start-up, or is in
maintenance or shutting down. The robot is refusing motion to protect itself, so it is reported as
`RobotFault`, which the desktop already treats as a rejection. This is a firmware
mapping, not a protocol change. `RejectedSessionNotReady` (#162) is the last state
check: a Velocity or ClearEmergencyStop without the current session token. The session
reports it in the same precedence position for invalid frames (`isOpen()`).

## Sequence rules

- `isNewer(a, b)` is true when `0 < (uint16)(a − b) < 32768`. Equal values are not
  newer, and neither are values exactly 32768 apart (in either direction). The step
  from 65535 to 0 is newer.
- The first Stop, Velocity or ClearEmergencyStop accepted in a session is accepted
  without comparison.
- Stop is always executed. A stale Stop is `Accepted` but does not move `lastAccepted`.
- EmergencyStop is never sequence-checked and never changes `lastAccepted`.
- Telemetry and CommandResult share one robot counter, which starts at 0 per
  session. `open()` and `end()` restart both counters (#162). `open()` also obtains the
  session token from the authority, and `end()` reports the end of the session, which
  stops as for Stop ([sessions](firmware-safety.md#sessions)). The
  [BLE peripheral](firmware-ble.md#session-lifecycle) calls them on CCCD and connection events (#163).

## Emergency Stop characteristic and JSON

`onEmergencyStopWrite()` calls `emergencyStop()` before it parses anything. A write
of any length or content therefore latches, including a zero-length write, a
malformed frame, a future version or JSON. The result is always `Accepted` with
type `0x03`. It echoes the frame's sequence only for a well-formed v1
EmergencyStop, and `0` otherwise.

The codec has no JSON parser. On the Command characteristic, a JSON object starts
with `{` (`0x7B`), so it is rejected at step 2 as `UnsupportedVersion` and never
moves the robot. No serial JSON mirror is implemented. The USB maintenance console
(#159) has no motion command, so serial input cannot bypass this path.

## Bounds

- Every function takes a `std::span` and reads at most the exact v1 frame length
  (10 bytes for a command). It never reads past the span, whatever the input
  length. A test shows that bytes beyond the longest frame never change a verdict,
  even in a 64 KiB write.
- No allocation: frames are `std::array`, and results are `std::optional`. The
  decoder and encoders are `constexpr`. The tests evaluate one decode at compile
  time, and a counting allocator measures zero allocations across decode, session,
  authority and telemetry.
- `CommandSession` is not thread-safe. The BLE adapter owns one per connection and
  calls it from its single callback context.
- Compiled with the pinned ESP32-C6 toolchain (`riscv32-esp-elf-g++` 14.2.0,
  `-std=gnu++20 -Os -fno-exceptions -Werror`), a probe that calls both write paths
  is 1,060 bytes of text with no data or bss.

## Not in this issue

- GATT characteristics, CCCDs, detecting session readiness and calling `open()`/`end()`
  (Velocity ignored before both CCCDs are enabled), advertising and disconnect
  handling: the [BLE peripheral](firmware-ble.md) (#163).
- Telemetry scheduling, battery and diagnostics content and the firmware version
  bytes: #164. Since #162, `telemetry()` takes `state` and `commandExpired` from the
  runtime status, so they always match the safety state.
- Powered and radio checks: #238 and #166.

## Verification

Host tests (`core` label, in
[`tests/`](../../firmware/codecell-c6/tests/ProtocolCodecTest.cpp)):

- `ProtocolVectorsTest`, `ProtocolCodecTest`:
  - every shared vector decodes or encodes byte for byte;
  - every truncation and trailing extension of every command;
  - every version × type byte at eight lengths;
  - robot-to-desktop types written to Command;
  - JSON;
  - signed extrema and the `0x8000` sentinel;
  - validity/range boundaries and their order;
  - sentinel collisions;
  - measurement rounding;
  - Robot Info validation and its provisioned source.
- `ProtocolSequenceTest`:
  - `isNewer()` exhaustively over the whole 16-bit distance space at seven base
    values;
  - equal, older, wrap and half-range cases;
  - the first-command, stale-Stop and E-stop exceptions;
  - reset;
  - the wrapping transmit counter.
- `ProtocolSessionTest`, `ProtocolRuntimeTest`:
  - the precedence matrix;
  - exactly one result per frame on the shared counter;
  - header echoes;
  - outcome mapping;
  - the E-stop characteristic for any payload;
  - the wire sequence kept out of the authority;
  - with the real runtime: the advertised limits are the enforced ones, and
    rejected frames keep state, output, deadline and `lastAccepted`;
  - latch, clear, fault and JSON.
- `ProtocolMutationTest`, `ProtocolFuzzTest`:
  - every single-bit flip and every byte value at every offset, against an
    independent table-driven reference decoder;
  - the read bound and zero allocations;
  - a reproducible run of 200,000 random writes (seed 161) on both characteristics
    against the real runtime. Inputs are short, exact, long and oversized, each in
    an exact-size heap buffer. A rejected frame must never change motion state, the
    drive or `lastAccepted`, and every write to the E-stop characteristic must latch.

Sanitizer run, as listed in the [firmware guide](../../firmware/codecell-c6/README.md#host-sanitizers-and-fuzz-run):
`-DCODECELL_FIRMWARE_HOST_SANITIZERS=ON` (AddressSanitizer and
UndefinedBehaviorSanitizer). `CODECELL_PROTOCOL_FUZZ_ITERATIONS` and
`CODECELL_PROTOCOL_FUZZ_SEED` change the bounded defaults.

The desktop `BleProtocolVectorsTest` (in `micro_robot_ble_contract_tests`) reads the
same file:

- the desktop encoder must reproduce every command vector that is within the
  published Robot Info limits, and reject the others rather than clamp them;
- the desktop decoder must decode every robot vector to its fields.
