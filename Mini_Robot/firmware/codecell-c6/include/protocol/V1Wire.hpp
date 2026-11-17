#pragma once
// Protocol version 1 wire constants (#161), transcribed from
// docs/protocol/robot-protocol.md (#64). Version 1 is frozen: these values define the
// bytes, not the domain. Portable C++20 with no Arduino, ESP-IDF, BLE or motor types.
//
// Byte access is bounded: every reader takes a std::span and an offset that the codec
// has already checked against the exact frame length, so nothing reads outside the
// buffer it was given.
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>

namespace codecell::protocol::v1 {

// ---- Header (every frame) -------------------------------------------------------------
inline constexpr std::uint8_t kProtocolVersion = 0x01;
inline constexpr std::size_t kVersionAt = 0;
inline constexpr std::size_t kTypeAt = 1;
inline constexpr std::size_t kSequenceAt = 2;
inline constexpr std::size_t kHeaderLength = 4;
// The smallest input whose version and type bytes can be read (validation step 1).
inline constexpr std::size_t kMinimumReadableLength = 2;
// Half of the uint16 sequence space: serial-number comparison is ambiguous at this distance.
inline constexpr std::uint16_t kSequenceHalfRange = 0x8000;

// ---- Message types --------------------------------------------------------------------
enum class MessageType : std::uint8_t {
    Stop = 0x01,
    Velocity = 0x02,
    EmergencyStop = 0x03,
    ClearEmergencyStop = 0x04,
    Telemetry = 0x80,
    CommandResult = 0x81,
};
// Echoed in a CommandResult when the type byte could not be read.
inline constexpr std::uint8_t kUnreadableType = 0x00;
inline constexpr std::uint16_t kUnreadableSequence = 0x0000;

// ---- Exact frame lengths --------------------------------------------------------------
inline constexpr std::size_t kStopLength = kHeaderLength;
inline constexpr std::size_t kVelocityLength = 10;
inline constexpr std::size_t kEmergencyStopLength = kHeaderLength;
inline constexpr std::size_t kClearEmergencyStopLength = kHeaderLength;
inline constexpr std::size_t kTelemetryLength = 18;
inline constexpr std::size_t kCommandResultLength = 9;
inline constexpr std::size_t kRobotInfoLength = 18;
// Every v1 frame and the Robot Info record fit the default 20-byte ATT payload.
inline constexpr std::size_t kMaxAttPayload = 20;
static_assert(kVelocityLength <= kMaxAttPayload && kTelemetryLength <= kMaxAttPayload
              && kCommandResultLength <= kMaxAttPayload && kRobotInfoLength <= kMaxAttPayload);

// ---- Velocity (0x02) offsets ----------------------------------------------------------
inline constexpr std::size_t kVelocityLinearAt = 4;
inline constexpr std::size_t kVelocityAngularAt = 6;
inline constexpr std::size_t kVelocityValidForAt = 8;
// validForMs must be at least this; the upper bound is the advertised maxCommandValidityMs.
inline constexpr std::uint16_t kMinValidForMilliseconds = 1;

// ---- Telemetry (0x80) offsets and flags -----------------------------------------------
inline constexpr std::size_t kTelemetryStateAt = 4;
inline constexpr std::size_t kTelemetryFlagsAt = 5;
inline constexpr std::size_t kTelemetryLastAcceptedAt = 6;
inline constexpr std::size_t kTelemetryUptimeAt = 8;
inline constexpr std::size_t kTelemetryBatteryAt = 12;
inline constexpr std::size_t kTelemetryLinearAt = 14;
inline constexpr std::size_t kTelemetryAngularAt = 16;
inline constexpr std::uint8_t kFlagCommandExpired = 0x01;
inline constexpr std::uint8_t kFlagBatteryLow = 0x02;
inline constexpr std::uint8_t kFlagHasAcceptedCommand = 0x04;
// lastAcceptedSequence while hasAcceptedCommand is clear.
inline constexpr std::uint16_t kNoAcceptedSequence = 0x0000;

// ---- CommandResult (0x81) offsets -----------------------------------------------------
inline constexpr std::size_t kResultCommandSequenceAt = 4;
inline constexpr std::size_t kResultCommandTypeAt = 6;
inline constexpr std::size_t kResultCodeAt = 7;
inline constexpr std::size_t kResultStateAt = 8;

// ---- Robot Info record (no frame header) ----------------------------------------------
inline constexpr std::uint8_t kRobotInfoFormat = 0x01;
inline constexpr std::size_t kInfoFormatAt = 0;
inline constexpr std::size_t kInfoMinVersionAt = 1;
inline constexpr std::size_t kInfoMaxVersionAt = 2;
inline constexpr std::size_t kInfoRobotIdAt = 3;
inline constexpr std::size_t kInfoFirmwareAt = 7;
inline constexpr std::size_t kInfoMaxLinearAt = 10;
inline constexpr std::size_t kInfoMaxAngularAt = 12;
inline constexpr std::size_t kInfoMaxValidityAt = 14;
inline constexpr std::size_t kInfoTelemetryPeriodAt = 16;
// A v1 robot supports exactly version 1.
inline constexpr std::uint8_t kMinProtocolVersion = kProtocolVersion;
inline constexpr std::uint8_t kMaxProtocolVersion = kProtocolVersion;

// ---- Scaled units and sentinels -------------------------------------------------------
// Linear velocity in 1 mm/s, angular in 1 mrad/s: SI value = raw / 1000.
inline constexpr double kMilliUnitsPerUnit = 1000.0;
// Measured velocity unknown (telemetry only). In commands this value is out of range.
inline constexpr std::int16_t kUnknownVelocity = std::numeric_limits<std::int16_t>::min();
inline constexpr std::uint16_t kUnknownBattery = 0xFFFF;
// Largest magnitude a Robot Info maximum (and so any accepted command) may have.
inline constexpr std::uint16_t kMaxAdvertisedVelocity = 32767;

// ---- Robot states (byte values of runtime::MotionState) -------------------------------
inline constexpr std::uint8_t kStateIdle = 0;
inline constexpr std::uint8_t kStateMoving = 1;
inline constexpr std::uint8_t kStateEmergencyStop = 2;
inline constexpr std::uint8_t kStateFault = 3;

// ---- Result codes ---------------------------------------------------------------------
enum class ResultCode : std::uint8_t {
    Accepted = 0x00,
    UnsupportedVersion = 0x01,
    UnknownMessageType = 0x02,
    MalformedLength = 0x03,
    StaleSequence = 0x04,
    InvalidValidity = 0x05,
    OutOfRange = 0x06,
    EmergencyStopLatched = 0x07,
    RobotFault = 0x08,
};

[[nodiscard]] constexpr const char* toString(ResultCode code) noexcept
{
    switch (code) {
    case ResultCode::Accepted: return "Accepted";
    case ResultCode::UnsupportedVersion: return "UnsupportedVersion";
    case ResultCode::UnknownMessageType: return "UnknownMessageType";
    case ResultCode::MalformedLength: return "MalformedLength";
    case ResultCode::StaleSequence: return "StaleSequence";
    case ResultCode::InvalidValidity: return "InvalidValidity";
    case ResultCode::OutOfRange: return "OutOfRange";
    case ResultCode::EmergencyStopLatched: return "EmergencyStopLatched";
    case ResultCode::RobotFault: return "RobotFault";
    }
    return "Unknown";
}

// ---- Little-endian byte access --------------------------------------------------------
// Callers pass offsets already validated against the exact frame length.
[[nodiscard]] constexpr std::uint16_t readU16(std::span<const std::uint8_t> bytes, std::size_t at) noexcept
{
    return static_cast<std::uint16_t>(bytes[at] | (static_cast<std::uint16_t>(bytes[at + 1]) << 8));
}
// Two's complement decoding without implementation-defined narrowing.
[[nodiscard]] constexpr std::int16_t readI16(std::span<const std::uint8_t> bytes, std::size_t at) noexcept
{
    constexpr std::int32_t kModulus = 0x10000;
    constexpr std::uint16_t kSignBit = 0x8000;
    const std::uint16_t raw = readU16(bytes, at);
    return static_cast<std::int16_t>(raw >= kSignBit ? static_cast<std::int32_t>(raw) - kModulus
                                                     : static_cast<std::int32_t>(raw));
}
template <std::size_t N>
constexpr void writeU16(std::array<std::uint8_t, N>& out, std::size_t at, std::uint16_t value) noexcept
{
    static_assert(N >= 2);
    out[at] = static_cast<std::uint8_t>(value & 0xFFU);
    out[at + 1] = static_cast<std::uint8_t>(value >> 8);
}
template <std::size_t N>
constexpr void writeI16(std::array<std::uint8_t, N>& out, std::size_t at, std::int16_t value) noexcept
{
    writeU16(out, at, static_cast<std::uint16_t>(value));  // Modular conversion: well defined.
}
template <std::size_t N>
constexpr void writeU32(std::array<std::uint8_t, N>& out, std::size_t at, std::uint32_t value) noexcept
{
    static_assert(N >= 4);
    for (std::size_t i = 0; i < 4; ++i) out[at + i] = static_cast<std::uint8_t>((value >> (8 * i)) & 0xFFU);
}
} // namespace codecell::protocol::v1
