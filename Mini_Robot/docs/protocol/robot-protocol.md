# Robot message protocol, version 1

Status: specification for [issue #64](https://github.com/Krzys2002/Mini_Robot/issues/64).
The desktop [BLE lifecycle](../overview/ble-connection.md) validates Robot Info
and establishes GATT readiness. Desktop [message handling](../overview/ble-messages.md)
implements frame encoding/decoding and writes. ESP32-C6 firmware and physical
command-delivery acceptance remain outstanding. The BLE carriage of
these frames is defined in [BLE GATT contract](ble-gatt.md).

This document defines the bytes exchanged between the desktop and one robot. It
does not change the domain types in
[TransportTypes.hpp](../../src/transport/TransportTypes.hpp): the
[transport contract](../overview/transport.md) stays radio- and encoding-free, and
only a protocol adapter behind `IRobotTransport` uses this format.

Key words: **must**/**must not** are mandatory for a conforming implementation;
**should** is a recommendation whose deviation must be justified.

## Roles and identity

| Term | Meaning |
| --- | --- |
| Desktop | Central/supervisor. Sends commands, receives telemetry and command results. |
| Robot | ESP32-C6 peripheral. Executes commands, owns motor control, watchdog, and local limits. |
| Session | One connection between one desktop and one robot. Sequence state is per session. |
| `robotId` | Unsigned 32-bit application identity, equal to the domain `RobotId` value. |

- Each robot stores its `robotId` in non-volatile memory at provisioning time.
  The operator must keep IDs unique within the arena. `robotId` is independent of
  BLE addresses, platform peripheral UUIDs, or Zigbee short addresses.
- The authoritative identity is the `robotId` field of the
  [Robot Info](#robot-info-record) record, read once per session before any
  command. Advertised names are only discovery hints.
- Frames do not repeat `robotId`: a session is point-to-point, so every frame on
  it belongs to the identified robot. A future multi-robot gateway (for example
  Zigbee) must add its own addressing outside these frames.

## Encoding rules

- All multi-byte integers are **little-endian**. Signed integers are two's complement.
- Frames are packed with no padding or alignment bytes.
- Every frame has an exact length for its message type. Shorter or longer frames
  are malformed; there are no optional trailing fields in version 1.
- One transport write or notification carries exactly one frame. Frames are never
  concatenated or fragmented. Every version 1 frame and the Robot Info record are
  at most 20 bytes, which fits the default BLE ATT payload without MTU negotiation.
- Reserved bits must be sent as zero and must be ignored on receipt.

### Scaled units

| Wire field | Type | Unit | Domain value | Unknown sentinel |
| --- | --- | --- | --- | --- |
| Linear velocity | `int16` | 1 mm/s | `metres/second = raw / 1000` | `0x8000` (-32768) in telemetry only |
| Angular velocity | `int16` | 1 mrad/s | `radians/second = raw / 1000` | `0x8000` (-32768) in telemetry only |
| Battery voltage | `uint16` | 1 mV | `volts = raw / 1000` | `0xFFFF` |
| Durations | `uint16` | 1 ms | `std::chrono::milliseconds` | none |
| Robot uptime | `uint32` | 1 ms | diagnostic only, never converted to `Timestamp` | none |

Sign conventions follow the domain: positive linear velocity is forward in the
robot body frame, and positive angular velocity is counterclockwise seen from above.

Desktop encoding converts SI values with round-half-away-from-zero
(`std::lround(value * 1000)`). It must reject a command with `InvalidCommand`
before sending if a value is non-finite, rounds outside the `int16` range, or
exceeds the robot's advertised maximum. The adapter must not clamp silently;
saturation belongs to the controller. In commands, `-32768` is out of range,
because every advertised maximum is at most 32767.

## Frame header

Every frame starts with the same 4-byte header:

| Offset | Size | Field | Rule |
| --- | --- | --- | --- |
| 0 | 1 | `version` | Protocol version. Version 1 is `0x01`. |
| 1 | 1 | `type` | Message type, see [message types](#message-types). |
| 2 | 2 | `sequence` | `uint16`, sender's per-session counter. |

Bytes 0 and 1 keep this meaning in every future protocol version, so any receiver
can identify and reject a frame it does not understand. The protocol version is
not the domain `modelVersion`. The adapter maps wire version 1 to domain model
version 1 explicitly and must not assume the numbers are equal.

## Message types

| Type | Name | Direction | Total length | Carried on |
| --- | --- | --- | --- | --- |
| `0x01` | Stop | desktop to robot | 4 | Command |
| `0x02` | Velocity | desktop to robot | 10 | Command |
| `0x03` | EmergencyStop | desktop to robot | 4 | Emergency Stop (also honoured on Command) |
| `0x04` | ClearEmergencyStop | desktop to robot | 4 | Command |
| `0x80` | Telemetry | robot to desktop | 18 | Telemetry |
| `0x81` | CommandResult | robot to desktop | 9 | Command Result |

Types `0x05`–`0x7F` are reserved for future desktop-to-robot messages and
`0x82`–`0xFF` for robot-to-desktop messages. Type `0x00` is never valid.

### Stop (`0x01`)

Header only. The robot sets its velocity setpoint to zero, decelerates within
its local limits, and enters `Idle`. A Stop never expires and needs no validity.
It maps from the domain `StopCommand`.

### Velocity (`0x02`)

| Offset | Size | Field | Rule |
| --- | --- | --- | --- |
| 4 | 2 | `linear` | `int16`, mm/s, `abs(linear) <= maxLinear` |
| 6 | 2 | `angular` | `int16`, mrad/s, `abs(angular) <= maxAngular` |
| 8 | 2 | `validForMs` | `uint16`, `1 <= validForMs <= maxCommandValidityMs` |

This is a supervisory body-frame setpoint, as in the domain `VelocityCommand`. The robot converts it
to wheel targets and runs the motor control loop. The command stays in force
until it is superseded or expires. See [freshness](#freshness-and-watchdog).

### EmergencyStop (`0x03`)

Header only. The robot immediately disables motor drive outputs without a ramp,
latches the `EmergencyStop` state, and drops any active setpoint. The robot never
rejects this message for sequence, validity, or state reasons. See
[emergency stop](#emergency-stop).

### ClearEmergencyStop (`0x04`)

Header only. This message releases the latch and moves the robot to `Idle` with
a zero setpoint. Motion does not resume: a new Velocity command is required. If
the robot is not latched, the message is accepted as a no-op.

### Telemetry (`0x80`)

| Offset | Size | Field | Rule |
| --- | --- | --- | --- |
| 4 | 1 | `state` | [Robot state](#robot-states) |
| 5 | 1 | `flags` | Bit 0 `commandExpired`, bit 1 `batteryLow`, bit 2 `hasAcceptedCommand`, bits 3–7 reserved |
| 6 | 2 | `lastAcceptedSequence` | Newest accepted desktop sequence this session; `0` and meaningless while `hasAcceptedCommand` is clear |
| 8 | 4 | `uptimeMs` | `uint32` robot monotonic uptime; wraps after about 49.7 days |
| 12 | 2 | `batteryMillivolts` | `uint16`, `0xFFFF` if unknown |
| 14 | 2 | `measuredLinear` | `int16` mm/s, `0x8000` if unknown |
| 16 | 2 | `measuredAngular` | `int16` mrad/s, `0x8000` if unknown |

`commandExpired` is set when the watchdog ends motion, and it stays set until
the next accepted Velocity or Stop. `hasAcceptedCommand` is set once the session
has a `lastAccepted` value. The robot sends telemetry every
`telemetryPeriodMs` while the session is ready, and also immediately after any
state change.

### CommandResult (`0x81`)

| Offset | Size | Field | Rule |
| --- | --- | --- | --- |
| 4 | 2 | `commandSequence` | Sequence of the command being answered, `0` if unreadable |
| 6 | 1 | `commandType` | Type of that command, `0` if unreadable |
| 7 | 1 | `result` | [Result code](#result-codes) |
| 8 | 1 | `state` | Robot state after processing |

While the session is ready, the robot sends exactly one CommandResult per
received command frame, including rejected frames. The transport defines when a
session is ready ([BLE](ble-gatt.md#session-establishment)). Delivery is best
effort, so a disconnect can lose it.

### Robot Info record

This record is read once per session (see [BLE GATT](ble-gatt.md#characteristics)).
It is a plain record without the frame header, and its byte 0 versions its layout.

| Offset | Size | Field | Rule |
| --- | --- | --- | --- |
| 0 | 1 | `infoFormat` | `0x01`; frozen meaning in all versions |
| 1 | 1 | `minProtocolVersion` | Lowest supported protocol version; frozen offset |
| 2 | 1 | `maxProtocolVersion` | Highest supported protocol version; frozen offset |
| 3 | 4 | `robotId` | `uint32` application identity |
| 7 | 3 | `firmwareVersion` | major, minor, patch bytes; diagnostic only |
| 10 | 2 | `maxLinear` | `uint16` mm/s, `1..32767` |
| 12 | 2 | `maxAngular` | `uint16` mrad/s, `1..32767` |
| 14 | 2 | `maxCommandValidityMs` | `uint16` watchdog ceiling, `1..65535` |
| 16 | 2 | `telemetryPeriodMs` | `uint16` nominal telemetry period, `1..65535` |

`infoFormat` 1 is exactly 18 bytes. The desktop must reject other lengths, an
unknown `infoFormat`, `minProtocolVersion > maxProtocolVersion`, or zero limits.

## Robot states

| Value | State | Motion | Leaves by |
| --- | --- | --- | --- |
| `0` | `Idle` | Motors stopped, ready for commands | Velocity, EmergencyStop, local fault |
| `1` | `Moving` | Executing a fresh Velocity setpoint | Stop, expiry, disconnect, EmergencyStop, fault |
| `2` | `EmergencyStop` | Drive disabled and latched | ClearEmergencyStop only |
| `3` | `Fault` | Drive disabled by robot self-protection | Firmware-defined recovery or reset; not clearable by desktop in v1 |

Other values are reserved. The desktop must treat an unknown state as `Fault`
for safety decisions.

The robot boots into `Idle` with outputs disabled. The emergency-stop latch is
held in RAM and is not kept across a robot reset. The desktop's own E-stop state
is the operator-facing authority. After any reconnect, the desktop must send
EmergencyStop first if its safety state is still in E-stop (see
[BLE session safety](ble-gatt.md#session-safety)).

## Sequence numbers

- Desktop and robot each own one `uint16` transmit counter per session. Each
  starts at `0` for the first frame after a connection and increments by one per
  frame sent, modulo 65536. Retransmitting a frame uses a new sequence number.
- Newer-than is serial-number comparison: `a` is newer than `b` when
  `0 < (uint16_t)(a - b) < 32768`. Equal or older values are not newer.
- The robot keeps `lastAccepted` for desktop sequences. It is unset at session start.
  - The first Stop, Velocity, or ClearEmergencyStop accepted in a session is
    accepted without comparison.
  - After that, Velocity and ClearEmergencyStop must be newer than `lastAccepted`.
    Otherwise they are rejected with `StaleSequence`.
  - Stop is always executed, because stopping is always safe. It advances
    `lastAccepted` only when it is newer.
  - EmergencyStop is never sequence-checked and never changes `lastAccepted`.
  - A rejected frame never changes `lastAccepted`.
- The robot uses one counter for both Telemetry and CommandResult. The desktop
  drops a robot frame whose sequence is not newer than the last one it accepted.
  It counts sequence gaps as lost notifications for communication measurements.
- Both sides reset all sequence state when the session ends. The desktop must
  discard any command queued for a previous session.

### Acknowledgement and errors

- A transport-level write confirmation means only that the robot's stack
  received the bytes. Acceptance is reported only by CommandResult with
  `result = Accepted`, correlated by `commandSequence`.
- The desktop must not retransmit a Velocity frame. It sends a fresh command
  with a new validity window instead. EmergencyStop is retransmitted until
  accepted (see [emergency stop](#emergency-stop)).
- Telemetry's `lastAcceptedSequence` is a cumulative status, not a per-command
  acknowledgement.

### Result codes

| Value | Name | Meaning |
| --- | --- | --- |
| `0x00` | `Accepted` | Frame applied, including a Stop with a stale sequence |
| `0x01` | `UnsupportedVersion` | Byte 0 is not a version this session speaks |
| `0x02` | `UnknownMessageType` | Type is reserved, robot-to-desktop, or not allowed on this characteristic |
| `0x03` | `MalformedLength` | Length does not match the message type |
| `0x04` | `StaleSequence` | Sequence not newer than `lastAccepted` |
| `0x05` | `InvalidValidity` | `validForMs` is `0` or above `maxCommandValidityMs` |
| `0x06` | `OutOfRange` | Velocity magnitude exceeds advertised maximum |
| `0x07` | `EmergencyStopLatched` | Velocity rejected while latched |
| `0x08` | `RobotFault` | Velocity or ClearEmergencyStop rejected in `Fault` |

Other values are reserved. The desktop must treat an unknown result code as a
rejection.

## Rejecting unknown or invalid input

A rejected frame has no effect on motion: an existing Velocity setpoint keeps
running only until its own expiry. The robot checks frames in this order:

1. Fewer than 2 bytes: `MalformedLength`. Echo `commandType = 0` and `commandSequence = 0`.
2. Byte 0 is not a supported version: `UnsupportedVersion`. Echo byte 1 as type,
   and bytes 2–3 as sequence when present. The reply is always a version 1
   frame, because byte 0 and byte 1 meanings are frozen.
3. Type is unknown, or is not a desktop-to-robot type: `UnknownMessageType`.
4. Length differs from the table: `MalformedLength`.
5. Type-specific sequence, state, validity, and range checks, in that order.

On the desktop:

- Unsupported Robot Info (format, version range, length): do not publish
  `Connected`. Report `UnsupportedVersion` for `Connect`, then disconnect.
- Robot frame with an unsupported version: drop it and report
  `UnsupportedVersion` for `ReceiveTelemetry`.
- Unknown type, wrong length, or reserved state/result value: drop the frame and
  report `BackendFailure` for `ReceiveTelemetry`. If the robot repeats this, the
  desktop should disconnect.
- Rejected CommandResult: report a `TransportError` for `SendCommand`
  (see [domain mapping](#domain-mapping)).

## Freshness and watchdog

The two clocks are not synchronized. Freshness therefore travels as a relative
validity window, and each side measures it on its own monotonic clock.

Desktop rules:

- Immediately before the write, the adapter re-checks domain freshness (`issuedAt <= now`
  and elapsed `< validFor`). It then encodes the remaining window:
  `validForMs = floor((issuedAt + validFor - now) / 1 ms)`, clamped down to
  `maxCommandValidityMs`. If the result is `0`, the command is stale. Reject it
  locally with `InvalidCommand` and do not send it.
- While a robot should keep moving, the desktop should send a new Velocity at an
  interval no longer than half the validity window, so that one lost frame does
  not stop the robot.

Robot rules, which form the command watchdog:

- On accepting a Velocity, the robot sets
  `deadline = receiptTime + validForMs` on its monotonic clock. This replaces any
  previous deadline.
- When `now >= deadline` (expiry exactly at the deadline, matching the domain
  rule), the robot ends motion as for Stop and enters `Idle`. It also sets
  `commandExpired` and sends telemetry immediately.
- `maxCommandValidityMs` is the hard ceiling. No setpoint can outlive it, even if
  the desktop and link fail together.
- Telemetry, CommandResult, or rejected frames never extend a deadline.

Radio latency is not subtracted, so the robot may act on a setpoint for up to one
transmission latency after the desktop's deadline. The validity ceiling and the
disconnect stop bound this delay.

On the desktop side, if no robot frame arrives within three `telemetryPeriodMs`
periods, the link should be treated as stale. The safety layer decides the
response ([#70](https://github.com/Krzys2002/Mini_Robot/issues/70)).

### Illustrative defaults

These values are starting points for bring-up. They belong in configuration or
firmware build settings, not as literals scattered through modules.

| Parameter | Owner | Default |
| --- | --- | --- |
| `maxCommandValidityMs` | Robot (advertised) | 500 ms |
| `telemetryPeriodMs` | Robot (advertised) | 100 ms |
| Velocity `validFor` | Desktop | 250 ms |
| Velocity resend period while moving | Desktop | 100 ms |
| Telemetry-silence threshold | Desktop | 3 × `telemetryPeriodMs` |
| E-stop retry period | Desktop | 100 ms |

## Emergency stop

- The robot must act on an EmergencyStop as soon as it is received. It
  disables outputs, latches, and sends a CommandResult and then telemetry
  showing `EmergencyStop`.
- The robot accepts the message on the dedicated Emergency Stop path regardless of
  sequence, validity, robot state, or planner activity on the desktop. On that
  path even an unparsable payload latches the robot (see [BLE GATT](ble-gatt.md#emergency-stop-characteristic)).
- While latched, the robot rejects Velocity with `EmergencyStopLatched`. Stop is
  accepted with no effect, and EmergencyStop is accepted again. Only
  ClearEmergencyStop releases the latch.
- The desktop retries EmergencyStop every retry period until it receives an
  `Accepted` result or telemetry with `state = EmergencyStop`, or until the session
  ends (which also stops the robot).
- The desktop sends ClearEmergencyStop only on an explicit operator action. It
  must never send it as a side effect of reconnecting, task cancellation, or a
  control loop.

## Domain mapping

The adapter owns this mapping. No domain type contains a wire field.

| Domain | Wire version 1 |
| --- | --- |
| `StopCommand` | Stop (`0x01`) |
| `VelocityCommand` (m/s, rad/s) | Velocity (`0x02`), scaled to mm/s and mrad/s |
| `RobotCommand::issuedAt` and `validFor` | Remaining `validForMs` computed at send time |
| `metadata.modelVersion = 1` | Wire `version = 1` (explicit mapping table) |
| `RobotCommand::metadata.sequence` | Not transmitted. The adapter allocates wire sequences and may log the pairing. |
| `RobotTelemetry::receivedAt` | Desktop receipt time. Never derived from `uptimeMs`. |
| `batteryVoltageVolts` | `batteryMillivolts / 1000`, empty for `0xFFFF` |
| `linearMetersPerSecond`, `angularRadiansPerSecond` | `measured* / 1000`, empty for `0x8000` |
| `RobotTelemetry::metadata.sequence` | Robot sequence unwrapped to 64 bits within the session; restarts per session and is not an acknowledgement |
| Robot `state`, `flags`, `uptimeMs`, `lastAcceptedSequence` | No domain field yet. Adapter logs them; later issues may add domain safety state. |
| EmergencyStop, ClearEmergencyStop | No domain command yet. The BLE adapter has an E-stop primitive; [#70](https://github.com/Krzys2002/Mini_Robot/issues/70) defines the domain request and explicit clear. |

| CommandResult `result` | Domain `TransportErrorCode` for `SendCommand` |
| --- | --- |
| `UnsupportedVersion` | `UnsupportedVersion` |
| `StaleSequence`, `InvalidValidity`, `OutOfRange`, `EmergencyStopLatched`, `RobotFault` | `InvalidCommand`, with the wire name in the message |
| `UnknownMessageType`, `MalformedLength`, unknown code | `BackendFailure` (desktop encoder defect or mismatch) |

## Example frames

Hex bytes in transmission order. The limits in the Robot Info example are illustrative.

| Frame | Bytes |
| --- | --- |
| Velocity, seq 258, 0.150 m/s, -0.500 rad/s, 250 ms | `01 02 02 01 96 00 0C FE FA 00` |
| Stop, seq 259 | `01 01 03 01` |
| EmergencyStop, seq 260 | `01 03 04 01` |
| Telemetry, seq 7, `Moving`, last 258, uptime 123456 ms, 3.912 V, 0.148 m/s, angular unknown | `01 80 07 00 01 04 02 01 40 E2 01 00 48 0F 94 00 00 80` |
| CommandResult, seq 8, for Velocity 258, `Accepted`, `Moving` | `01 81 08 00 02 01 02 00 01` |
| Robot Info, ID 7, firmware 0.1.0, 300 mm/s, 6000 mrad/s, 500 ms, 100 ms | `01 01 01 07 00 00 00 00 01 00 2C 01 70 17 F4 01 64 00` |

Implementations should use these frames as encoder and decoder test vectors.

## JSON bring-up representation

The BLE wire format is binary from version 1 onwards, because every frame must
fit in a 20-byte ATT payload without MTU negotiation. JSON is defined only as a
readable 1:1 mirror for:

- the robot's USB serial console during firmware bring-up;
- desktop and firmware logs;
- test fixtures.

It is never parsed from a BLE characteristic. A JSON write starts with `{`
(`0x7B`), so the Command characteristic rejects it as `UnsupportedVersion`.
On the Emergency Stop characteristic, any write latches, JSON included.

Each frame is a single-line UTF-8 JSON object. Values use the same integers and
scaled units as the binary fields, so conversion is lossless and validation is
identical. Unknown keys, missing keys, and wrong value types are rejected.

| Binary | JSON key | JSON value |
| --- | --- | --- |
| `version`, `type`, `sequence` | `v`, `type`, `seq` | integer, `stop`/`velocity`/`estop`/`clear_estop`/`telemetry`/`result`, integer |
| Velocity fields | `linear_mm_s`, `angular_mrad_s`, `valid_for_ms` | integers |
| Telemetry fields | `state`, `command_expired`, `battery_low`, `last_seq`, `uptime_ms`, `battery_mv`, `linear_mm_s`, `angular_mrad_s` | `idle`/`moving`/`estop`/`fault`, booleans, integers. Unknown measurements are `null`; `last_seq` is `null` while `hasAcceptedCommand` is clear. |
| CommandResult fields | `cmd_seq`, `cmd_type`, `result`, `state` | integer, type name, snake-case result name, state name |

```json
{"v":1,"type":"velocity","seq":258,"linear_mm_s":150,"angular_mrad_s":-500,"valid_for_ms":250}
{"v":1,"type":"result","seq":8,"cmd_seq":258,"cmd_type":"velocity","result":"accepted","state":"moving"}
```

## Version evolution

- Version 1 is fixed. Any new field, message type, unit, or semantic change
  requires a new protocol version, a matching JSON update, and a new
  `maxProtocolVersion` on robots that support it.
- These invariants hold in every version: header bytes 0 (`version`) and 1 (`type`),
  Robot Info bytes 0–2, little-endian byte order, and the EmergencyStop
  characteristic's latch-on-any-write behaviour.
- A version 1 robot supports exactly version 1. A future robot that supports more
  than one version must add an explicit version-selection step at session start
  before it sends telemetry. Until then the desktop speaks version 1 and refuses
  robots whose range excludes 1.
