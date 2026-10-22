#pragma once
#include "transport/TransportTypes.hpp"
#include "transport/ble/BleTypes.hpp"

#include <span>
#include <vector>

namespace micro_robot {
enum class BleEndpoint { Command, EmergencyStop, Telemetry, CommandResult };
enum class BleRobotState : std::uint8_t { Idle, Moving, EmergencyStop, Fault };
struct EncodedBleCommand {
    std::vector<std::uint8_t> bytes;
    BleEndpoint endpoint{BleEndpoint::Command};
    bool withResponse{true}; // ATT receipt only, never application acceptance.
};
struct BleEncodeResult {
    std::optional<EncodedBleCommand> frame;
    TransportErrorCode code{TransportErrorCode::InvalidCommand};
    std::string error;
};
[[nodiscard]] BleEncodeResult encodeBleCommand(const RobotCommand& command, const BleRobotInfo& limits,
                                              std::uint16_t sequence, Timestamp now);
[[nodiscard]] EncodedBleCommand encodeBleEmergencyStop(std::uint16_t sequence);
struct BleTelemetryFrame {
    BleRobotState state{};
    bool commandExpired{}, batteryLow{};
    std::optional<std::uint16_t> lastAcceptedSequence;
    std::uint32_t uptimeMilliseconds{}; // Remote clock: diagnostic only.
    std::optional<double> batteryVolts, linearMetersPerSecond, angularRadiansPerSecond;
};
struct BleCommandResultFrame {
    std::uint16_t commandSequence{};
    std::uint8_t commandType{}, result{};
    BleRobotState state{};
};
struct DecodedBleFrame {
    std::uint16_t sequence{};
    std::variant<BleTelemetryFrame, BleCommandResultFrame> payload;
};
struct BleDecodeResult {
    std::optional<DecodedBleFrame> frame;
    TransportErrorCode code{TransportErrorCode::BackendFailure};
    std::string error;
};
// Exact-length, channel, version and enum validation precedes any state update.
[[nodiscard]] BleDecodeResult decodeBleNotification(BleEndpoint endpoint, std::span<const std::uint8_t> bytes);
[[nodiscard]] TransportErrorCode bleRejectionCode(std::uint8_t result) noexcept;
[[nodiscard]] const char* bleResultName(std::uint8_t result) noexcept;
struct BleSequenceUpdate { std::uint64_t unwrapped{}; std::uint16_t gap{}; };
// One shared robot counter for Telemetry and Command Result. Reset per session.
class BleReceiveSequence final {
public:
    [[nodiscard]] std::optional<BleSequenceUpdate> accept(std::uint16_t sequence);
private:
    std::optional<std::uint16_t> last_;
    std::uint64_t unwrapped_{};
};
} // namespace micro_robot
