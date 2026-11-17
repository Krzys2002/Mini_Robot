#pragma once
// Read-only identity and limits for Robot Info, advertising and diagnostics (#159).
// Both come from the one ValidatedRobotConfig the runtime started with, so the
// advertised name, the Robot Info record and the enforced validity ceiling cannot
// disagree. Byte encoding of Robot Info is protocol::v1::encodeRobotInfo (#161); GATT
// and advertising are the BLE peripheral (#163). A robot without a validated record has no identity
// to publish and must not advertise (docs/overview/firmware-provisioning.md).
#include "provisioning/RobotConfigRecord.hpp"
#include <array>
#include <charconv>
#include <cstddef>
#include <cstdint>

namespace codecell::provisioning {

// Robot Info fields that the configuration owns, in v1 wire units. The format byte and
// protocol range are fixed by the v1 codec (#161); the firmware version is the image's
// (ble::kRobotInfoFirmwareVersion, #164).
struct RobotInfoValues {
    RobotId robotId{0};
    std::uint16_t maxLinearMillimetresPerSecond{0};
    std::uint16_t maxAngularMilliradiansPerSecond{0};
    std::uint16_t maxCommandValidityMilliseconds{0};
    std::uint16_t telemetryPeriodMilliseconds{0};

    friend bool operator==(const RobotInfoValues&, const RobotInfoValues&) = default;
};

[[nodiscard]] constexpr RobotInfoValues robotInfoValues(const ValidatedRobotConfig& config) noexcept
{
    const auto& raw = config.raw();
    return {raw.robotId, raw.maxLinearMillimetresPerSecond, raw.maxAngularMilliradiansPerSecond,
            raw.maxCommandValidityMilliseconds, raw.telemetryPeriodMilliseconds};
}

// Scan-response Complete Local Name "MR-<robotId in decimal>" (ble-gatt.md). The
// longest is "MR-4294967295": 13 characters plus the terminator.
inline constexpr std::array<char, 3> kAdvertisedNamePrefix{'M', 'R', '-'};
inline constexpr std::size_t kMaxRobotIdDigits = 10;
inline constexpr std::size_t kAdvertisedNameCapacity = kAdvertisedNamePrefix.size() + kMaxRobotIdDigits + 1;
using AdvertisedName = std::array<char, kAdvertisedNameCapacity>;

[[nodiscard]] inline AdvertisedName advertisedName(RobotId robotId) noexcept
{
    AdvertisedName name{};
    std::size_t at = 0;
    for (const char c : kAdvertisedNamePrefix) name[at++] = c;
    // Always fits: the capacity holds every uint32 in decimal plus the terminator.
    const auto [end, error] = std::to_chars(name.data() + at, name.data() + name.size() - 1, robotId);
    (void)error;
    *end = '\0';
    return name;
}
} // namespace codecell::provisioning
