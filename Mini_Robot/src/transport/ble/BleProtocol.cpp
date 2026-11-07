#include "transport/ble/BleProtocol.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace micro_robot {
namespace {
void append16(std::vector<std::uint8_t>& b, std::uint16_t value)
{
    b.push_back(static_cast<std::uint8_t>(value));
    b.push_back(static_cast<std::uint8_t>(value >> 8));
}
std::vector<std::uint8_t> header(std::uint8_t type, std::uint16_t sequence)
{
    std::vector<std::uint8_t> b{1, type}; // Explicit domain-v1 -> wire-v1 mapping.
    append16(b, sequence);
    return b;
}
std::uint16_t read16(std::span<const std::uint8_t> b, std::size_t i)
{
    return static_cast<std::uint16_t>(b[i] | (static_cast<std::uint16_t>(b[i + 1]) << 8));
}
std::optional<double> signedMeasurement(std::uint16_t raw)
{
    if (raw == 0x8000) return {};
    // Portable signed conversion, independent of host endianness/representation.
    const int value = raw > 32767 ? static_cast<int>(raw) - 65536 : raw;
    return value / 1000.0;
}
} // namespace
BleEncodeResult encodeBleCommand(const RobotCommand& command, const BleRobotInfo& limits,
                                 std::uint16_t sequence, Timestamp now)
{
    const auto invalid = [](const char* message) { return BleEncodeResult{{}, TransportErrorCode::InvalidCommand, message}; };
    if (!isSupportedTransportModelVersion(command.metadata.modelVersion))
        return {{}, TransportErrorCode::UnsupportedVersion, "Unsupported transport domain model version"};
    if (!limits.maximumValidityMilliseconds || !limits.maximumLinearMillimetersPerSecond
        || limits.maximumLinearMillimetersPerSecond > 32767 || !limits.maximumAngularMilliradiansPerSecond
        || limits.maximumAngularMilliradiansPerSecond > 32767)
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
    if (remaining < 1) return invalid("Command has less than one millisecond of validity remaining");
    EncodedBleCommand frame;
    const auto* velocity = std::get_if<VelocityCommand>(&command.payload);
    frame.bytes = header(velocity ? 2 : 1, sequence);
    if (velocity) {
        const double linear = velocity->linearMetersPerSecond * 1000;
        const double angular = velocity->angularRadiansPerSecond * 1000;
        if (!std::isfinite(linear) || !std::isfinite(angular)
            || std::abs(linear) > limits.maximumLinearMillimetersPerSecond
            || std::abs(angular) > limits.maximumAngularMilliradiansPerSecond)
            return invalid("Velocity is non-finite or exceeds advertised limits; no clamping applied");
        append16(frame.bytes, static_cast<std::uint16_t>(std::lround(linear)));
        append16(frame.bytes, static_cast<std::uint16_t>(std::lround(angular)));
        append16(frame.bytes, static_cast<std::uint16_t>(std::min<std::int64_t>(remaining, limits.maximumValidityMilliseconds)));
        frame.withResponse = false;
    }
    return {std::move(frame), TransportErrorCode::InvalidCommand, {}};
}
EncodedBleCommand encodeBleEmergencyStop(std::uint16_t sequence)
{
    return {header(3, sequence), BleEndpoint::EmergencyStop, true};
}
EncodedBleCommand encodeBleEmergencyClear(std::uint16_t sequence)
{
    return {header(4, sequence), BleEndpoint::Command, true};
}
BleDecodeResult decodeBleNotification(BleEndpoint endpoint, std::span<const std::uint8_t> b)
{
    const auto invalid = [](const char* message) { return BleDecodeResult{{}, TransportErrorCode::BackendFailure, message}; };
    if (b.size() < 2) return invalid("Malformed BLE notification header");
    if (b[0] != 1) return {{}, TransportErrorCode::UnsupportedVersion, "Unsupported BLE wire version"};
    const bool telemetry = endpoint == BleEndpoint::Telemetry;
    if ((!telemetry && endpoint != BleEndpoint::CommandResult) || b[1] != (telemetry ? 0x80 : 0x81))
        return invalid("Unknown or wrong-channel BLE message type");
    if (b.size() != (telemetry ? 18U : 9U)) return invalid("Malformed BLE notification length");
    const auto state = b[telemetry ? 4 : 8];
    if (state > 3) return invalid("Unknown robot state; treat as Fault");
    DecodedBleFrame frame;
    frame.sequence = read16(b, 2);
    if (telemetry) {
        BleTelemetryFrame value;
        value.state = static_cast<BleRobotState>(state);
        value.commandExpired = (b[5] & 1) != 0;
        value.batteryLow = (b[5] & 2) != 0;
        if (b[5] & 4) value.lastAcceptedSequence = read16(b, 6);
        value.uptimeMilliseconds = static_cast<std::uint32_t>(b[8]) | (static_cast<std::uint32_t>(b[9]) << 8)
            | (static_cast<std::uint32_t>(b[10]) << 16) | (static_cast<std::uint32_t>(b[11]) << 24);
        if (const auto battery = read16(b, 12); battery != 0xffff) value.batteryVolts = battery / 1000.0;
        value.linearMetersPerSecond = signedMeasurement(read16(b, 14));
        value.angularRadiansPerSecond = signedMeasurement(read16(b, 16));
        frame.payload = value;
    } else {
        if (b[7] > 8) return invalid("Unknown command result code; treated as rejection");
        frame.payload = BleCommandResultFrame{read16(b, 4), b[6], b[7], static_cast<BleRobotState>(state)};
    }
    return {std::move(frame), TransportErrorCode::BackendFailure, {}};
}
TransportErrorCode bleRejectionCode(std::uint8_t result) noexcept
{
    if (result == 1) return TransportErrorCode::UnsupportedVersion;
    if (result == 7) return TransportErrorCode::EmergencyStopLatched;
    if (result == 8) return TransportErrorCode::RobotFault;
    if (result >= 4 && result <= 6) return TransportErrorCode::InvalidCommand;
    return TransportErrorCode::BackendFailure;
}
const char* bleResultName(std::uint8_t result) noexcept
{
    constexpr const char* names[]{"Accepted", "UnsupportedVersion", "UnknownMessageType", "MalformedLength",
        "StaleSequence", "InvalidValidity", "OutOfRange", "EmergencyStopLatched", "RobotFault"};
    return result <= 8 ? names[result] : "UnknownResult";
}
std::optional<BleSequenceUpdate> BleReceiveSequence::accept(std::uint16_t sequence)
{
    if (!last_) { last_ = sequence; unwrapped_ = sequence; return BleSequenceUpdate{unwrapped_, 0}; }
    const auto difference = static_cast<std::uint16_t>(sequence - *last_);
    if (!difference || difference >= 32768 || unwrapped_ > std::numeric_limits<std::uint64_t>::max() - difference) return {};
    last_ = sequence;
    unwrapped_ += difference;
    return BleSequenceUpdate{unwrapped_, static_cast<std::uint16_t>(difference - 1)};
}
} // namespace micro_robot
