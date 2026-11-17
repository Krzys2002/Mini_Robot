#include "transport/ble/BleTypes.hpp"
#include "transport/ble/BleWire.hpp"

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
    constexpr auto prefix = ble_wire::advertisedNamePrefix;
    if (!name.starts_with(prefix) || name.size() == prefix.size()) return {};
    std::uint32_t id{};
    const auto [end, error] = std::from_chars(name.data() + prefix.size(), name.data() + name.size(), id);
    if (error != std::errc{} || end != name.data() + name.size()) return {};
    return RobotId{id};
}
BleInfoResult validateBleRobotInfo(std::span<const std::uint8_t> b, RobotId expected)
{
    using namespace ble_wire;
    // The advertised version range must include the only version this desktop speaks.
    if (b.size() != robotInfoLength || b[infoFormatOffset] != robotInfoFormat
        || b[minimumVersionOffset] > b[maximumVersionOffset] || b[minimumVersionOffset] > version
        || b[maximumVersionOffset] < version)
        return {{}, true, "Unsupported Robot Info length, format or protocol version range"};
    const auto id = read32(b, robotIdOffset);
    if (id != expected.value()) return {{}, false, "Robot Info identity differs from the requested RobotId"};
    BleRobotInfo info{RobotId{id}, read16(b, maximumLinearOffset), read16(b, maximumAngularOffset),
        read16(b, maximumValidityOffset), read16(b, telemetryPeriodOffset)};
    if (!info.maximumLinearMillimetersPerSecond || info.maximumLinearMillimetersPerSecond > maximumSigned
        || !info.maximumAngularMilliradiansPerSecond || info.maximumAngularMilliradiansPerSecond > maximumSigned
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
