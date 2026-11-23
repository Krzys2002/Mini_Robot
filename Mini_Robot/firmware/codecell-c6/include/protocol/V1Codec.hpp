#pragma once
// Firmware-side protocol version 1 codec (#161): decodes untrusted desktop-to-robot
// command frames and encodes the robot-to-desktop Telemetry and CommandResult frames and
// the Robot Info record, byte for byte as in docs/protocol/robot-protocol.md (#64).
//
// Pure functions with no motor, BLE, clock or allocation. Each call does a fixed amount
// of work: it reads at most the exact v1 frame length, and nothing past the span it was
// given, however long or short the input is. JSON is not a wire format here: a BLE
// write that starts with '{' is rejected as an unsupported version like any other
// non-v1 byte 0. The desktop codec (#66) is the other endpoint; both test against the
// shared vectors in docs/protocol/fixtures/v1-frames.txt.
#include "protocol/V1Wire.hpp"
#include "provisioning/RobotIdentity.hpp"
#include "runtime/RuntimeTypes.hpp"
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace codecell::protocol::v1 {

// ---- Desktop-to-robot commands --------------------------------------------------------

// Velocity payload in wire units. Range and validity limits are checked after the
// sequence and state checks (validation step 5), not by the decoder.
struct VelocityFields {
    std::int16_t linearMillimetresPerSecond{0};
    std::int16_t angularMilliradiansPerSecond{0};
    std::uint16_t validForMilliseconds{0};

    friend constexpr bool operator==(const VelocityFields&, const VelocityFields&) = default;
};

// A frame that passed validation steps 1-4: supported version, a desktop-to-robot type
// and the exact length. `velocity` is meaningful only for MessageType::Velocity.
struct CommandFrame {
    MessageType type{MessageType::Stop};
    std::uint16_t sequence{0};
    VelocityFields velocity{};

    friend constexpr bool operator==(const CommandFrame&, const CommandFrame&) = default;
};

// Outcome of steps 1-4. Exactly one of `frame` or a rejection is present; a rejection
// carries the fields the CommandResult must echo.
struct CommandDecodeResult {
    std::optional<CommandFrame> frame;
    ResultCode rejection{ResultCode::Accepted};
    std::uint8_t echoType{kUnreadableType};
    std::uint16_t echoSequence{kUnreadableSequence};
};

// Desktop-to-robot types only; robot-to-desktop and reserved types are not commands.
[[nodiscard]] constexpr std::optional<std::size_t> commandLength(std::uint8_t type) noexcept
{
    switch (static_cast<MessageType>(type)) {
    case MessageType::Stop: return kStopLength;
    case MessageType::Velocity: return kVelocityLength;
    case MessageType::EmergencyStop: return kEmergencyStopLength;
    case MessageType::ClearEmergencyStop: return kClearEmergencyStopLength;
    case MessageType::Telemetry:
    case MessageType::CommandResult: return std::nullopt;
    }
    return std::nullopt;
}

// Validation steps 1-4 in the specified order ("Rejecting unknown or invalid input"):
// 1. fewer than 2 bytes: MalformedLength, echo type 0 and sequence 0;
// 2. byte 0 not version 1: UnsupportedVersion, echo byte 1 and the sequence when present;
// 3. not a desktop-to-robot type: UnknownMessageType;
// 4. not the exact length for the type: MalformedLength.
[[nodiscard]] constexpr CommandDecodeResult decodeCommand(std::span<const std::uint8_t> bytes) noexcept
{
    CommandDecodeResult result;
    if (bytes.size() < kMinimumReadableLength) {
        result.rejection = ResultCode::MalformedLength;
        return result;
    }
    result.echoType = bytes[kTypeAt];
    if (bytes.size() >= kHeaderLength) result.echoSequence = readU16(bytes, kSequenceAt);
    if (bytes[kVersionAt] != kProtocolVersion) {
        result.rejection = ResultCode::UnsupportedVersion;
        return result;
    }
    const auto length = commandLength(bytes[kTypeAt]);
    if (!length) {
        result.rejection = ResultCode::UnknownMessageType;
        return result;
    }
    if (bytes.size() != *length) {
        result.rejection = ResultCode::MalformedLength;
        return result;
    }
    CommandFrame frame;
    frame.type = static_cast<MessageType>(bytes[kTypeAt]);
    frame.sequence = readU16(bytes, kSequenceAt);
    if (frame.type == MessageType::Velocity) {
        frame.velocity.linearMillimetresPerSecond = readI16(bytes, kVelocityLinearAt);
        frame.velocity.angularMilliradiansPerSecond = readI16(bytes, kVelocityAngularAt);
        frame.velocity.validForMilliseconds = readU16(bytes, kVelocityValidForAt);
    }
    result.frame = frame;
    return result;
}

// ---- Validity and range limits (step 5, after sequence and state) ---------------------

// The limits a robot advertises in Robot Info and must enforce.
struct CommandLimits {
    std::uint16_t maxLinearMillimetresPerSecond{0};
    std::uint16_t maxAngularMilliradiansPerSecond{0};
    std::uint16_t maxCommandValidityMilliseconds{0};
};

[[nodiscard]] constexpr CommandLimits commandLimits(const provisioning::RobotInfoValues& info) noexcept
{
    return {info.maxLinearMillimetresPerSecond, info.maxAngularMilliradiansPerSecond,
            info.maxCommandValidityMilliseconds};
}

// `1 <= validForMs <= maxCommandValidityMs`.
[[nodiscard]] constexpr bool validityAllowed(std::uint16_t validForMilliseconds, const CommandLimits& limits) noexcept
{
    return validForMilliseconds >= kMinValidForMilliseconds
        && validForMilliseconds <= limits.maxCommandValidityMilliseconds;
}

// `abs(raw) <= maximum`, computed in 32 bits so that -32768 (the telemetry "unknown"
// sentinel) is out of range for every representable maximum.
[[nodiscard]] constexpr bool magnitudeWithin(std::int16_t raw, std::uint16_t maximum) noexcept
{
    const std::int32_t value = raw;
    return (value < 0 ? -value : value) <= static_cast<std::int32_t>(maximum);
}

// The type-specific Velocity checks that do not depend on robot state, in the specified
// order: validity, then range. Accepted when both pass.
[[nodiscard]] constexpr ResultCode checkVelocityFields(const VelocityFields& velocity, const CommandLimits& limits) noexcept
{
    if (!validityAllowed(velocity.validForMilliseconds, limits)) return ResultCode::InvalidValidity;
    if (!magnitudeWithin(velocity.linearMillimetresPerSecond, limits.maxLinearMillimetresPerSecond)
        || !magnitudeWithin(velocity.angularMilliradiansPerSecond, limits.maxAngularMilliradiansPerSecond))
        return ResultCode::OutOfRange;
    return ResultCode::Accepted;
}

// Wire to SI at the domain boundary: metres (radians) per second = raw / 1000.
[[nodiscard]] constexpr runtime::BodyVelocity toBodyVelocity(const VelocityFields& velocity) noexcept
{
    return {velocity.linearMillimetresPerSecond / kMilliUnitsPerUnit,
            velocity.angularMilliradiansPerSecond / kMilliUnitsPerUnit};
}

// ---- Robot-to-desktop frames ----------------------------------------------------------

using TelemetryBytes = std::array<std::uint8_t, kTelemetryLength>;
using CommandResultBytes = std::array<std::uint8_t, kCommandResultLength>;
using RobotInfoBytes = std::array<std::uint8_t, kRobotInfoLength>;

// Robot state byte. A value outside the four v1 states cannot come from the runtime; if
// it ever did, the robot reports Fault, which the desktop must assume for unknown states.
[[nodiscard]] constexpr std::uint8_t stateByte(runtime::MotionState state) noexcept
{
    switch (state) {
    case runtime::MotionState::Idle: return kStateIdle;
    case runtime::MotionState::Moving: return kStateMoving;
    case runtime::MotionState::EmergencyStop: return kStateEmergencyStop;
    case runtime::MotionState::Fault: return kStateFault;
    }
    return kStateFault;
}

// Telemetry values in wire units. Unknown values are empty and become the v1 sentinels;
// nothing here is measured by this codec. The CodeCell C6 Drive has no velocity source, so the
// image always sends them unknown (#164, telemetry/TelemetryInputs.hpp).
struct TelemetryValues {
    runtime::MotionState state{runtime::MotionState::Idle};
    bool commandExpired{false};
    bool batteryLow{false};
    std::optional<std::uint16_t> lastAcceptedSequence;  // Empty: hasAcceptedCommand clear.
    std::uint32_t uptimeMilliseconds{0};                // Robot monotonic uptime, wraps.
    std::optional<std::uint16_t> batteryMillivolts;
    std::optional<std::int16_t> measuredLinearMillimetresPerSecond;
    std::optional<std::int16_t> measuredAngularMilliradiansPerSecond;
};

// Encodes one Telemetry frame. nullopt when a known value equals its "unknown"
// sentinel (0xFFFF mV, -32768 mm/s or mrad/s), which the desktop could not tell apart
// from unknown; the caller must report such a value as unknown instead.
[[nodiscard]] constexpr std::optional<TelemetryBytes> encodeTelemetry(std::uint16_t sequence,
                                                                      const TelemetryValues& values) noexcept
{
    if ((values.batteryMillivolts && *values.batteryMillivolts == kUnknownBattery)
        || (values.measuredLinearMillimetresPerSecond && *values.measuredLinearMillimetresPerSecond == kUnknownVelocity)
        || (values.measuredAngularMilliradiansPerSecond
            && *values.measuredAngularMilliradiansPerSecond == kUnknownVelocity))
        return std::nullopt;
    TelemetryBytes out{};
    out[kVersionAt] = kProtocolVersion;
    out[kTypeAt] = static_cast<std::uint8_t>(MessageType::Telemetry);
    writeU16(out, kSequenceAt, sequence);
    out[kTelemetryStateAt] = stateByte(values.state);
    std::uint8_t flags = 0;  // Reserved bits 3-7 are sent as zero.
    if (values.commandExpired) flags |= kFlagCommandExpired;
    if (values.batteryLow) flags |= kFlagBatteryLow;
    if (values.lastAcceptedSequence) flags |= kFlagHasAcceptedCommand;
    out[kTelemetryFlagsAt] = flags;
    writeU16(out, kTelemetryLastAcceptedAt, values.lastAcceptedSequence.value_or(kNoAcceptedSequence));
    writeU32(out, kTelemetryUptimeAt, values.uptimeMilliseconds);
    writeU16(out, kTelemetryBatteryAt, values.batteryMillivolts.value_or(kUnknownBattery));
    writeI16(out, kTelemetryLinearAt, values.measuredLinearMillimetresPerSecond.value_or(kUnknownVelocity));
    writeI16(out, kTelemetryAngularAt, values.measuredAngularMilliradiansPerSecond.value_or(kUnknownVelocity));
    return out;
}

// SI measurement to the telemetry wire unit with round-half-away-from-zero (the rule the
// desktop uses for commands). Non-finite values and values that round outside
// [-32767, 32767] are not representable: empty, so they are reported as unknown rather
// than clamped.
[[nodiscard]] inline std::optional<std::int16_t> measuredToWire(double siValue) noexcept
{
    // Values at or above this magnitude round to more than 32767 counts.
    constexpr double kFirstUnrepresentable = static_cast<double>(kMaxAdvertisedVelocity) + 0.5;
    if (!std::isfinite(siValue)) return std::nullopt;
    const double scaled = siValue * kMilliUnitsPerUnit;
    if (!(std::abs(scaled) < kFirstUnrepresentable)) return std::nullopt;
    return static_cast<std::int16_t>(std::lround(scaled));
}

struct CommandResultValues {
    std::uint16_t commandSequence{kUnreadableSequence};
    std::uint8_t commandType{kUnreadableType};
    ResultCode result{ResultCode::Accepted};
    runtime::MotionState state{runtime::MotionState::Idle};  // State after processing.
};

[[nodiscard]] constexpr CommandResultBytes encodeCommandResult(std::uint16_t sequence,
                                                               const CommandResultValues& values) noexcept
{
    CommandResultBytes out{};
    out[kVersionAt] = kProtocolVersion;
    out[kTypeAt] = static_cast<std::uint8_t>(MessageType::CommandResult);
    writeU16(out, kSequenceAt, sequence);
    writeU16(out, kResultCommandSequenceAt, values.commandSequence);
    out[kResultCommandTypeAt] = values.commandType;
    out[kResultCodeAt] = static_cast<std::uint8_t>(values.result);
    out[kResultStateAt] = stateByte(values.state);
    return out;
}

// ---- Robot Info record ----------------------------------------------------------------

// Diagnostic firmware version bytes. The image's version is ble::kRobotInfoFirmwareVersion
// (#164); this codec does not choose one.
struct FirmwareVersion {
    std::uint8_t major{0};
    std::uint8_t minor{0};
    std::uint8_t patch{0};
};

// Encodes infoFormat 1 from the provisioned values (#159). nullopt when a value is not
// representable in the record: a zero limit or telemetry period, or a velocity maximum
// above 32767. The protocol range is always exactly version 1.
[[nodiscard]] constexpr std::optional<RobotInfoBytes> encodeRobotInfo(const provisioning::RobotInfoValues& info,
                                                                      const FirmwareVersion& firmware) noexcept
{
    if (info.maxLinearMillimetresPerSecond == 0 || info.maxLinearMillimetresPerSecond > kMaxAdvertisedVelocity
        || info.maxAngularMilliradiansPerSecond == 0 || info.maxAngularMilliradiansPerSecond > kMaxAdvertisedVelocity
        || info.maxCommandValidityMilliseconds == 0 || info.telemetryPeriodMilliseconds == 0)
        return std::nullopt;
    RobotInfoBytes out{};
    out[kInfoFormatAt] = kRobotInfoFormat;
    out[kInfoMinVersionAt] = kMinProtocolVersion;
    out[kInfoMaxVersionAt] = kMaxProtocolVersion;
    writeU32(out, kInfoRobotIdAt, info.robotId);
    out[kInfoFirmwareAt] = firmware.major;
    out[kInfoFirmwareAt + 1] = firmware.minor;
    out[kInfoFirmwareAt + 2] = firmware.patch;
    writeU16(out, kInfoMaxLinearAt, info.maxLinearMillimetresPerSecond);
    writeU16(out, kInfoMaxAngularAt, info.maxAngularMilliradiansPerSecond);
    writeU16(out, kInfoMaxValidityAt, info.maxCommandValidityMilliseconds);
    writeU16(out, kInfoTelemetryPeriodAt, info.telemetryPeriodMilliseconds);
    return out;
}
} // namespace codecell::protocol::v1
