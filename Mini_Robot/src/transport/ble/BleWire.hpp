#pragma once
#include <cstddef>
#include <cstdint>

// Independent desktop v1 definitions; shared byte fixtures guard firmware parity.
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
} // namespace micro_robot::ble_wire
