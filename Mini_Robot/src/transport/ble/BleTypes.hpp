#pragma once

#include "common/RobotId.hpp"

#include <chrono>
#include <array>
#include <optional>
#include <span>
#include <string>

namespace micro_robot {
// A scan-local opaque identifier, never an application RobotId. Addresses can be
// absent on macOS; advertised IDs are only hints until Robot Info is validated.
struct BleDevice {
    std::string id;
    std::string name;
    std::optional<std::string> address;
    std::optional<RobotId> advertisedId;
};
enum class BlePhase {
    Disconnected, Scanning, Connecting, DiscoveringServices,
    DiscoveringCharacteristics, ReadingIdentity, EnablingNotifications,
    Ready, Disconnecting, Fault
};
struct BleTransportConfig {
    std::chrono::milliseconds scanTimeout{10000};
    std::chrono::milliseconds phaseTimeout{10000};
    std::chrono::milliseconds disconnectTimeout{3000};
    std::size_t maximumDevices{128};
};
struct BleRobotInfo {
    RobotId robotId{0};
    std::uint16_t maximumLinearMillimetersPerSecond{};
    std::uint16_t maximumAngularMilliradiansPerSecond{};
    std::uint16_t maximumValidityMilliseconds{};
    std::uint16_t telemetryPeriodMilliseconds{};
};
// Robot Info v1 only. No frame/telemetry codec (#66). On failure output is empty.
struct BleInfoResult {
    std::optional<BleRobotInfo> info;
    bool unsupportedVersion{};
    std::string error;
};
struct BleCharacteristicCapabilities {
    bool present{}, read{}, write{}, writeWithoutResponse{}, notify{}, cccd{};
};
// Robot Info, Command, Emergency Stop, Telemetry, Command Result, in UUID order.
[[nodiscard]] std::string validateBleGattLayout(const std::array<BleCharacteristicCapabilities, 5>& layout);
[[nodiscard]] std::optional<RobotId> advertisedBleRobotId(const std::string& name);
[[nodiscard]] BleInfoResult validateBleRobotInfo(std::span<const std::uint8_t> bytes, RobotId expected);
[[nodiscard]] const char* blePhaseName(BlePhase phase) noexcept;
} // namespace micro_robot
