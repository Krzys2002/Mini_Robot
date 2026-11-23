#include "transport/ble/BleProtocol.hpp"
#include "transport/ble/BleWire.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace micro_robot {
namespace {
using ble_wire::read16;
void append16(std::vector<std::uint8_t>& b, std::uint16_t value)
{
    b.push_back(static_cast<std::uint8_t>(value));
    b.push_back(static_cast<std::uint8_t>(value >> 8));
}
std::vector<std::uint8_t> header(std::uint8_t type, std::uint16_t sequence)
{
    std::vector<std::uint8_t> b{ble_wire::version, type}; // Explicit domain-v1 -> wire-v1 mapping.
    append16(b, sequence);
    return b;
}
std::optional<double> signedMeasurement(std::uint16_t raw)
{
    if (raw == ble_wire::unknownMeasurement) return {};
    // Portable signed conversion, independent of host endianness/representation.
    const int value = raw > ble_wire::maximumSigned ? static_cast<int>(raw) - ble_wire::signedModulus : raw;
    return value / ble_wire::milliPerUnit;
}
} // namespace
BleEncodeResult encodeBleCommand(const RobotCommand& command, const BleRobotInfo& limits,
                                 std::uint16_t sequence, Timestamp now)
{
    const auto invalid = [](const char* message) { return BleEncodeResult{{}, TransportErrorCode::InvalidCommand, message}; };
    if (!isSupportedTransportModelVersion(command.metadata.modelVersion))
        return {{}, TransportErrorCode::UnsupportedVersion, "Unsupported transport domain model version"};
    // Stop removes motion and has no wire validity field. Never age it out.
    if (std::holds_alternative<StopCommand>(command.payload))
        return {EncodedBleCommand{header(ble_wire::stop, sequence)}, TransportErrorCode::InvalidCommand, {}};
    if (!limits.maximumValidityMilliseconds || !limits.maximumLinearMillimetersPerSecond
        || limits.maximumLinearMillimetersPerSecond > ble_wire::maximumSigned || !limits.maximumAngularMilliradiansPerSecond
        || limits.maximumAngularMilliradiansPerSecond > ble_wire::maximumSigned)
        return invalid("Invalid advertised BLE command limits");
    if (command.validFor <= Duration::zero() || command.issuedAt > now)
        return invalid("Command validity is nonpositive or issued in the future");
    const auto issued = command.issuedAt.time_since_epoch().count();
    const auto current = now.time_since_epoch().count();
    if (issued < 0 && current > std::numeric_limits<Duration::rep>::max() + issued)
        return invalid("Command is stale");
    const auto elapsed = now - command.issuedAt;
    if (elapsed >= command.validFor) return invalid("Command is stale");
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(command.validFor - elapsed).count();
    if (remaining < ble_wire::minimumValidityMilliseconds) return invalid("Command has less than one millisecond of validity remaining");
    EncodedBleCommand frame;
    const auto& velocity = std::get<VelocityCommand>(command.payload);
    frame.bytes = header(ble_wire::velocity, sequence);
    const double linear = velocity.linearMetersPerSecond * ble_wire::milliPerUnit;
    const double angular = velocity.angularRadiansPerSecond * ble_wire::milliPerUnit;
    if (!std::isfinite(linear) || !std::isfinite(angular)
        || std::abs(linear) > limits.maximumLinearMillimetersPerSecond
        || std::abs(angular) > limits.maximumAngularMilliradiansPerSecond)
        return invalid("Velocity is non-finite or exceeds advertised limits; no clamping applied");
    append16(frame.bytes, static_cast<std::uint16_t>(std::lround(linear)));
    append16(frame.bytes, static_cast<std::uint16_t>(std::lround(angular)));
    append16(frame.bytes, static_cast<std::uint16_t>(std::min<std::int64_t>(remaining, limits.maximumValidityMilliseconds)));
    frame.withResponse = false;
    return {std::move(frame), TransportErrorCode::InvalidCommand, {}};
}
EncodedBleCommand encodeBleEmergencyStop(std::uint16_t sequence)
{
    return {header(ble_wire::emergencyStop, sequence), BleEndpoint::EmergencyStop, true};
}
EncodedBleCommand encodeBleEmergencyClear(std::uint16_t sequence)
{
    return {header(ble_wire::emergencyClear, sequence), BleEndpoint::Command, true};
}
BleDecodeResult decodeBleNotification(BleEndpoint endpoint, std::span<const std::uint8_t> b)
{
    const auto invalid = [](const char* message) { return BleDecodeResult{{}, TransportErrorCode::BackendFailure, message}; };
    if (b.size() < ble_wire::identifyingPrefixLength) return invalid("Malformed BLE notification header");
    if (b[ble_wire::versionOffset] != ble_wire::version) return {{}, TransportErrorCode::UnsupportedVersion, "Unsupported BLE wire version"};
    const bool telemetry = endpoint == BleEndpoint::Telemetry;
    if ((!telemetry && endpoint != BleEndpoint::CommandResult) || b[ble_wire::typeOffset] != (telemetry ? ble_wire::telemetry : ble_wire::commandResult))
        return invalid("Unknown or wrong-channel BLE message type");
    if (b.size() != (telemetry ? ble_wire::telemetryLength : ble_wire::resultLength)) return invalid("Malformed BLE notification length");
    const auto state = b[telemetry ? ble_wire::telemetryStateOffset : ble_wire::resultStateOffset];
    if (state > static_cast<unsigned>(BleRobotState::Fault)) return invalid("Unknown robot state; treat as Fault");
    DecodedBleFrame frame;
    frame.sequence = read16(b, ble_wire::sequenceOffset);
    if (telemetry) {
        BleTelemetryFrame value;
        value.state = static_cast<BleRobotState>(state);
        value.commandExpired = (b[ble_wire::flagsOffset] & ble_wire::expiredFlag) != 0;
        value.batteryLow = (b[ble_wire::flagsOffset] & ble_wire::batteryLowFlag) != 0;
        if (b[ble_wire::flagsOffset] & ble_wire::hasSequenceFlag) value.lastAcceptedSequence = read16(b, ble_wire::lastSequenceOffset);
        value.uptimeMilliseconds = ble_wire::read32(b, ble_wire::uptimeOffset);
        if (const auto battery = read16(b, ble_wire::batteryOffset); battery != ble_wire::unknownBattery) value.batteryVolts = battery / ble_wire::milliPerUnit;
        value.linearMetersPerSecond = signedMeasurement(read16(b, ble_wire::linearOffset));
        value.angularRadiansPerSecond = signedMeasurement(read16(b, ble_wire::angularOffset));
        frame.payload = value;
    } else {
        if (b[ble_wire::resultCodeOffset] > ble_wire::robotFault) return invalid("Unknown command result code; treated as rejection");
        frame.payload = BleCommandResultFrame{read16(b, ble_wire::resultSequenceOffset), b[ble_wire::resultTypeOffset], b[ble_wire::resultCodeOffset], static_cast<BleRobotState>(state)};
    }
    return {std::move(frame), TransportErrorCode::BackendFailure, {}};
}
TransportErrorCode bleRejectionCode(std::uint8_t result) noexcept
{
    if (result == ble_wire::unsupportedVersion) return TransportErrorCode::UnsupportedVersion;
    if (result == ble_wire::emergencyStopLatched) return TransportErrorCode::EmergencyStopLatched;
    if (result == ble_wire::robotFault) return TransportErrorCode::RobotFault;
    if (result >= ble_wire::staleSequence && result <= ble_wire::outOfRange) return TransportErrorCode::InvalidCommand;
    return TransportErrorCode::BackendFailure;
}
const char* bleResultName(std::uint8_t result) noexcept
{
    constexpr const char* names[]{"Accepted", "UnsupportedVersion", "UnknownMessageType", "MalformedLength",
        "StaleSequence", "InvalidValidity", "OutOfRange", "EmergencyStopLatched", "RobotFault"};
    return result <= ble_wire::robotFault ? names[result] : "UnknownResult";
}
std::optional<BleSequenceUpdate> BleReceiveSequence::accept(std::uint16_t sequence)
{
    if (!last_) { last_ = sequence; unwrapped_ = sequence; return BleSequenceUpdate{unwrapped_, 0}; }
    const auto difference = static_cast<std::uint16_t>(sequence - *last_);
    if (!difference || difference >= ble_wire::sequenceHalfRange || unwrapped_ > std::numeric_limits<std::uint64_t>::max() - difference) return {};
    last_ = sequence;
    unwrapped_ += difference;
    return BleSequenceUpdate{unwrapped_, static_cast<std::uint16_t>(difference - 1)};
}
} // namespace micro_robot
