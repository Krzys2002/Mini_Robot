#pragma once

#include "common/RobotId.hpp"
#include "common/Time.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <variant>

namespace micro_robot {

enum class ConnectionState { Disconnected, Connecting, Connected, Disconnecting, Fault };

// Version of these domain values, independent of any future wire protocol.
inline constexpr std::uint16_t currentTransportModelVersion{1};

struct TransportMessageMetadata {
    std::uint16_t modelVersion{currentTransportModelVersion};
    // Reserved message identifier, not an acknowledgement or ordering guarantee.
    // Empty means unassigned/unknown; zero is a valid assigned value. Future
    // protocol adapters define allocation, session scope, and wire mapping.
    std::optional<std::uint64_t> sequence;
};

[[nodiscard]] inline constexpr bool isSupportedTransportModelVersion(std::uint16_t version) noexcept
{
    return version == currentTransportModelVersion;
}

struct StopCommand {};

// Semantic setpoint only; no controller, motor mapping, or packet encoding.
struct VelocityCommand {
    double linearMetersPerSecond{}; // Positive forward in the robot frame.
    double angularRadiansPerSecond{}; // Positive counterclockwise.
};

struct RobotCommand {
    std::variant<StopCommand, VelocityCommand> payload{StopCommand{}};
    Timestamp issuedAt{};
    Duration validFor{}; // Must be set positive; local freshness, not a wire clock.
    TransportMessageMetadata metadata{};
};

struct RobotTelemetry {
    RobotId robotId;
    Timestamp receivedAt; // Desktop receipt time, not a remote device clock.
    std::optional<double> batteryVoltageVolts;
    std::optional<double> linearMetersPerSecond;
    std::optional<double> angularRadiansPerSecond;
    TransportMessageMetadata metadata{};
};

enum class TransportOperation { Connect, Disconnect, SendCommand, ReceiveTelemetry };
enum class TransportErrorCode {
    Unavailable, NotConnected, InvalidCommand, BackendFailure, UnsupportedVersion, InvalidTelemetry
};

struct TransportError {
    RobotId robotId;
    TransportOperation operation;
    TransportErrorCode code;
    std::string message;
    Timestamp occurredAt;
};

} // namespace micro_robot
