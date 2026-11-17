#pragma once
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

// Independent desktop v1 definitions; shared byte fixtures guard firmware parity.
// Layouts follow docs/protocol/robot-protocol.md. Never include firmware headers here.
namespace micro_robot::ble_wire {
inline constexpr std::uint8_t version = 1;
inline constexpr std::uint8_t stop = 1, velocity = 2, emergencyStop = 3, emergencyClear = 4;
inline constexpr std::uint8_t telemetry = 0x80, commandResult = 0x81;
inline constexpr std::uint8_t accepted = 0, unsupportedVersion = 1, staleSequence = 4,
    outOfRange = 6, emergencyStopLatched = 7, robotFault = 8;
inline constexpr std::size_t telemetryLength = 18, resultLength = 9, typeOffset = 1,
    sequenceOffset = 2, telemetryStateOffset = 4, resultStateOffset = 8,
    flagsOffset = 5, lastSequenceOffset = 6, batteryOffset = 12,
    linearOffset = 14, angularOffset = 16, resultSequenceOffset = 4,
    resultTypeOffset = 6, resultCodeOffset = 7;
inline constexpr std::uint8_t expiredFlag = 1, batteryLowFlag = 2, hasSequenceFlag = 4;
inline constexpr std::uint16_t unknownBattery = 0xffff, unknownMeasurement = 0x8000;
inline constexpr int maximumSigned = 32767, signedModulus = 65536;
inline constexpr unsigned sequenceHalfRange = 32768;
inline constexpr double milliPerUnit = 1000.0;

// Frame header: version, type, little-endian uint16 sequence.
inline constexpr std::size_t versionOffset = 0, headerLength = 4;
// Version and type must be readable before channel and length checks.
inline constexpr std::size_t identifyingPrefixLength = typeOffset + 1;
inline constexpr std::size_t uptimeOffset = 8; // Telemetry uint32 uptimeMs.
// Velocity validForMs rule: 1 <= validForMs <= maxCommandValidityMs.
inline constexpr std::int64_t minimumValidityMilliseconds = 1;

// Robot Info record: no frame header; byte 0 versions its layout.
inline constexpr std::uint8_t robotInfoFormat = 1;
inline constexpr std::size_t robotInfoLength = 18, infoFormatOffset = 0,
    minimumVersionOffset = 1, maximumVersionOffset = 2, robotIdOffset = 3,
    maximumLinearOffset = 10, maximumAngularOffset = 12, maximumValidityOffset = 14,
    telemetryPeriodOffset = 16;
// Scan-response Complete Local Name: prefix followed by the decimal robotId.
inline constexpr std::string_view advertisedNamePrefix = "MR-";

// Layout consistency; the documented byte values are pinned by the shared fixtures.
static_assert(sequenceOffset + sizeof(std::uint16_t) == headerLength);
static_assert(uptimeOffset + sizeof(std::uint32_t) == batteryOffset);
static_assert(angularOffset + sizeof(std::uint16_t) == telemetryLength);
static_assert(resultStateOffset + sizeof(std::uint8_t) == resultLength);
static_assert(telemetryPeriodOffset + sizeof(std::uint16_t) == robotInfoLength);

// Little-endian field reads. Callers have already validated the exact length.
[[nodiscard]] inline std::uint16_t read16(std::span<const std::uint8_t> b, std::size_t i)
{
    return static_cast<std::uint16_t>(b[i] | (static_cast<std::uint16_t>(b[i + 1]) << 8));
}
[[nodiscard]] inline std::uint32_t read32(std::span<const std::uint8_t> b, std::size_t i)
{
    return static_cast<std::uint32_t>(b[i]) | (static_cast<std::uint32_t>(b[i + 1]) << 8)
        | (static_cast<std::uint32_t>(b[i + 2]) << 16) | (static_cast<std::uint32_t>(b[i + 3]) << 24);
}
} // namespace micro_robot::ble_wire
