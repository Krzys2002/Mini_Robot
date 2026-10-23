#include "transport/ble/BleTypes.hpp"

#include <charconv>

namespace micro_robot {
std::string validateBleGattLayout(const std::array<BleCharacteristicCapabilities, 5>& layout)
{
    constexpr const char* names[]{"Robot Info", "Command", "Emergency Stop", "Telemetry", "Command Result"};
    for (std::size_t i = 0; i < layout.size(); ++i) {
        const auto& c = layout[i];
        const bool valid = c.present && (i != 0 || c.read)
            && ((i != 1 && i != 2) || (c.write && c.writeWithoutResponse))
            && (i != 3 || (c.read && c.notify && c.cccd))
            && (i != 4 || (c.notify && c.cccd));
        if (!valid) return std::string{"Missing characteristic, required properties or CCCD: "} + names[i];
    }
    return {};
}
std::optional<RobotId> advertisedBleRobotId(const std::string& name)
{
    if (!name.starts_with("MR-") || name.size() == 3) return {};
    std::uint32_t id{};
    const auto [end, error] = std::from_chars(name.data() + 3, name.data() + name.size(), id);
    if (error != std::errc{} || end != name.data() + name.size()) return {};
    return RobotId{id};
}
BleInfoResult validateBleRobotInfo(std::span<const std::uint8_t> b, RobotId expected)
{
    if (b.size() != 18 || b[0] != 1 || b[1] > b[2] || b[1] > 1 || b[2] < 1)
        return {{}, true, "Unsupported Robot Info length, format or protocol version range"};
    const auto u16 = [&b](std::size_t i) {
        return static_cast<std::uint16_t>(b[i] | (static_cast<std::uint16_t>(b[i + 1]) << 8));
    };
    const auto id = static_cast<std::uint32_t>(b[3]) | (static_cast<std::uint32_t>(b[4]) << 8)
        | (static_cast<std::uint32_t>(b[5]) << 16) | (static_cast<std::uint32_t>(b[6]) << 24);
    if (id != expected.value()) return {{}, false, "Robot Info identity differs from the requested RobotId"};
    BleRobotInfo info{RobotId{id}, u16(10), u16(12), u16(14), u16(16)};
    if (!info.maximumLinearMillimetersPerSecond || info.maximumLinearMillimetersPerSecond > 32767
        || !info.maximumAngularMilliradiansPerSecond || info.maximumAngularMilliradiansPerSecond > 32767
        || !info.maximumValidityMilliseconds || !info.telemetryPeriodMilliseconds)
        return {{}, false, "Robot Info contains invalid actuator/watchdog/telemetry limits"};
    return {info, false, {}};
}
const char* blePhaseName(BlePhase phase) noexcept
{
    switch (phase) {
    case BlePhase::Disconnected: return "Disconnected";
    case BlePhase::Scanning: return "Scanning";
    case BlePhase::Connecting: return "Connecting";
    case BlePhase::DiscoveringServices: return "DiscoveringServices";
    case BlePhase::DiscoveringCharacteristics: return "DiscoveringCharacteristics";
    case BlePhase::ReadingIdentity: return "ReadingIdentity";
    case BlePhase::EnablingNotifications: return "EnablingNotifications";
    case BlePhase::Ready: return "Ready";
    case BlePhase::Disconnecting: return "Disconnecting";
    case BlePhase::Fault: return "Fault";
    }
    return "Unknown";
}
} // namespace micro_robot
