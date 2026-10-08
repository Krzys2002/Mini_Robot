#pragma once

#include "common/RobotId.hpp"
#include "common/Time.hpp"

#include <optional>
#include <string>
#include <variant>

namespace micro_robot {

enum class ConnectionState { Disconnected, Connecting, Connected, Disconnecting, Fault };

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
};

struct RobotTelemetry {
    RobotId robotId;
    Timestamp receivedAt; // Desktop receipt time, not a remote device clock.
    std::optional<double> batteryVoltageVolts;
    std::optional<double> linearMetersPerSecond;
    std::optional<double> angularRadiansPerSecond;
};

enum class TransportOperation { Connect, Disconnect, SendCommand, ReceiveTelemetry };
enum class TransportErrorCode { Unavailable, NotConnected, InvalidCommand, BackendFailure };

struct TransportError {
    RobotId robotId;
    TransportOperation operation;
    TransportErrorCode code;
    std::string message;
    Timestamp occurredAt;
};

} // namespace micro_robot
