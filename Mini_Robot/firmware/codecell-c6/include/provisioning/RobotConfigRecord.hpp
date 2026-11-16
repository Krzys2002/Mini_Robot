#pragma once
// Versioned, validated board/configuration record (#159). Portable C++20: no Arduino,
// ESP-IDF, NVS or radio types may appear here.
//
// The record holds the persistent RobotId, the calibrated actuator data that #160/#167
// will use, the limits advertised in Robot Info and the hardware capabilities the robot
// requires. Nothing here has a usable default: every field of a default RobotConfig is
// zero, which fails validation, so an unset value can never arm the outputs. The only
// exception is the RobotId, where every uint32 value (including 0) is valid; the
// maintenance path therefore requires it to be set and confirmed explicitly.
//
// Stored layout, little-endian and packed (docs/overview/firmware-provisioning.md):
//   0  magic "MRCF"         4  schemaVersion u16      6  recordLength u16
//   8  generation u32       12 schema payload         end-4 CRC-32 of all preceding bytes
// The header and the trailing CRC keep these positions in every schema version, so the
// integrity and age of a record can be checked even when its payload is not understood.
#include "TimingBudget.hpp"
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace codecell::provisioning {

// Application identity, equal to the desktop domain RobotId and the v1 Robot Info robotId.
// Every value is valid; uniqueness within the arena is an operator duty (see the guide).
using RobotId = std::uint32_t;

// Board the record was calibrated for. Only the CodeCell C6 Drive (#154) exists today.
enum class BoardProfile : std::uint8_t { CodeCellC6Drive = 1 };

// Hardware capabilities a record can require. The values only name hardware listed in
// the #154 hardware profile; they do not claim that the hardware is validated.
namespace capability {
inline constexpr std::uint8_t kDifferentialDrive = 1U << 0;  // Dual H-bridge wheel drive.
inline constexpr std::uint8_t kInertialSensor = 1U << 1;     // On-board BNO085 IMU, local use only.
inline constexpr std::uint8_t kDefined = kDifferentialDrive | kInertialSensor;
// What this image can serve. The IMU is not wired (#158), so a record requiring it is refused.
inline constexpr std::uint8_t kSupportedByFirmware = kDifferentialDrive;
} // namespace capability

// Per-wheel direction correction for the drive adapter (#160).
namespace polarity {
inline constexpr std::uint8_t kLeftReversed = 1U << 0;
inline constexpr std::uint8_t kRightReversed = 1U << 1;
inline constexpr std::uint8_t kDefined = kLeftReversed | kRightReversed;
} // namespace polarity

// DriveCell `Drive(direction, power_percent)` full scale (#154 hardware profile).
inline constexpr std::uint8_t kDriveOutputPercentFullScale = 100;
// Robot Info maxLinear/maxAngular range (robot-protocol.md, Robot Info record).
inline constexpr std::uint16_t kMaxAdvertisedVelocityRaw = 32767;
// Unit factors between the stored integers and SI values.
inline constexpr double kMicrometresPerMetre = 1'000'000.0;
inline constexpr double kMillimetresPerMetre = 1'000.0;
inline constexpr double kMilliradiansPerRadian = 1'000.0;
inline constexpr std::uint64_t kMicrometresPerMillimetre = 1'000;
inline constexpr std::uint64_t kMilliradiansPerRadianInt = 1'000;

// Raw record values in their stored units. Use ValidatedRobotConfig to read them.
struct RobotConfig {
    RobotId robotId{0};
    std::uint8_t boardProfile{0};             // BoardProfile value.
    std::uint8_t requiredCapabilities{0};     // capability:: bits.
    std::uint8_t drivePolarity{0};            // polarity:: bits.
    std::uint8_t maxDriveOutputPercent{0};    // Approved per-motor output bound at the actuator write.
    std::uint32_t wheelDiameterMicrometres{0};
    std::uint32_t trackWidthMicrometres{0};   // Wheel-centre distance.
    std::uint16_t maxWheelSpeedMillimetresPerSecond{0};  // Wheel surface speed at maxDriveOutputPercent.
    std::uint16_t maxLinearMillimetresPerSecond{0};      // Advertised Robot Info maxLinear.
    std::uint16_t maxAngularMilliradiansPerSecond{0};    // Advertised Robot Info maxAngular.
    std::uint16_t maxCommandValidityMilliseconds{0};     // Advertised and enforced validity ceiling.
    std::uint16_t telemetryPeriodMilliseconds{0};        // Advertised nominal telemetry period.

    friend bool operator==(const RobotConfig&, const RobotConfig&) = default;
};

enum class RecordError : std::uint8_t {
    None = 0,
    // Integrity of a stored record.
    Truncated,           // Shorter than the header plus CRC.
    BadMagic,
    LengthMismatch,      // recordLength differs from the stored size or from the schema size.
    ChecksumMismatch,    // Torn or corrupted write.
    UnsupportedVersion,  // Intact, but this image does not understand the schema version.
    ConflictingRecords,  // Two intact records claim the same generation with different content.
    // Content of a record or of a provisioning draft.
    UnsupportedBoard,
    ReservedCapabilityBits,
    DriveCapabilityMissing,
    CapabilityNotInFirmware,
    ReservedPolarityBits,
    DriveOutputOutOfRange,      // 1..100 percent.
    WheelDiameterZero,
    TrackWidthZero,
    WheelSpeedZero,
    LinearLimitOutOfRange,      // 1..32767 mm/s.
    AngularLimitOutOfRange,     // 1..32767 mrad/s.
    LinearLimitAboveWheelSpeed, // The drive cannot reach the advertised maxLinear.
    AngularLimitAboveWheelSpeed,// The drive cannot reach the advertised maxAngular on the spot.
    ValidityOutOfRange,         // 1..kCommandValidityCeiling (#156) ms.
    TelemetryPeriodMismatch,    // Must equal the budgeted kTelemetryPeriod (#156).
    // State of the A/B store (#159 review). Appended so the earlier values keep their numbers.
    CommittedRecordDamaged,     // A slot is damaged and no interrupted provisioning write explains it.
    FactoryResetIncomplete,     // A factory reset started and has not finished erasing both slots.
    ProvisioningIncomplete,     // A provisioning write was cut and no armable record preceded it.
};

[[nodiscard]] constexpr const char* toString(RecordError error) noexcept
{
    switch (error) {
    case RecordError::None: return "None";
    case RecordError::Truncated: return "Truncated";
    case RecordError::BadMagic: return "BadMagic";
    case RecordError::LengthMismatch: return "LengthMismatch";
    case RecordError::ChecksumMismatch: return "ChecksumMismatch";
    case RecordError::UnsupportedVersion: return "UnsupportedVersion";
    case RecordError::ConflictingRecords: return "ConflictingRecords";
    case RecordError::UnsupportedBoard: return "UnsupportedBoard";
    case RecordError::ReservedCapabilityBits: return "ReservedCapabilityBits";
    case RecordError::DriveCapabilityMissing: return "DriveCapabilityMissing";
    case RecordError::CapabilityNotInFirmware: return "CapabilityNotInFirmware";
    case RecordError::ReservedPolarityBits: return "ReservedPolarityBits";
    case RecordError::DriveOutputOutOfRange: return "DriveOutputOutOfRange";
    case RecordError::WheelDiameterZero: return "WheelDiameterZero";
    case RecordError::TrackWidthZero: return "TrackWidthZero";
    case RecordError::WheelSpeedZero: return "WheelSpeedZero";
    case RecordError::LinearLimitOutOfRange: return "LinearLimitOutOfRange";
    case RecordError::AngularLimitOutOfRange: return "AngularLimitOutOfRange";
    case RecordError::LinearLimitAboveWheelSpeed: return "LinearLimitAboveWheelSpeed";
    case RecordError::AngularLimitAboveWheelSpeed: return "AngularLimitAboveWheelSpeed";
    case RecordError::ValidityOutOfRange: return "ValidityOutOfRange";
    case RecordError::TelemetryPeriodMismatch: return "TelemetryPeriodMismatch";
    case RecordError::CommittedRecordDamaged: return "CommittedRecordDamaged";
    case RecordError::FactoryResetIncomplete: return "FactoryResetIncomplete";
    case RecordError::ProvisioningIncomplete: return "ProvisioningIncomplete";
    }
    return "Unknown";
}

// Content rules. Ranges come from the v1 protocol and the #156 budget; no physical
// limit is invented here: the calibrated values themselves are provisioned per robot.
[[nodiscard]] constexpr RecordError validate(const RobotConfig& config) noexcept
{
    if (config.boardProfile != static_cast<std::uint8_t>(BoardProfile::CodeCellC6Drive))
        return RecordError::UnsupportedBoard;
    if ((config.requiredCapabilities & ~capability::kDefined) != 0) return RecordError::ReservedCapabilityBits;
    if ((config.requiredCapabilities & capability::kDifferentialDrive) == 0) return RecordError::DriveCapabilityMissing;
    if ((config.requiredCapabilities & ~capability::kSupportedByFirmware) != 0)
        return RecordError::CapabilityNotInFirmware;
    if ((config.drivePolarity & ~polarity::kDefined) != 0) return RecordError::ReservedPolarityBits;
    if (config.maxDriveOutputPercent == 0 || config.maxDriveOutputPercent > kDriveOutputPercentFullScale)
        return RecordError::DriveOutputOutOfRange;
    if (config.wheelDiameterMicrometres == 0) return RecordError::WheelDiameterZero;
    if (config.trackWidthMicrometres == 0) return RecordError::TrackWidthZero;
    if (config.maxWheelSpeedMillimetresPerSecond == 0) return RecordError::WheelSpeedZero;
    if (config.maxLinearMillimetresPerSecond == 0 || config.maxLinearMillimetresPerSecond > kMaxAdvertisedVelocityRaw)
        return RecordError::LinearLimitOutOfRange;
    if (config.maxAngularMilliradiansPerSecond == 0
        || config.maxAngularMilliradiansPerSecond > kMaxAdvertisedVelocityRaw)
        return RecordError::AngularLimitOutOfRange;
    // Straight motion: both wheels at the body speed.
    if (config.maxLinearMillimetresPerSecond > config.maxWheelSpeedMillimetresPerSecond)
        return RecordError::LinearLimitAboveWheelSpeed;
    // Rotation on the spot: wheel speed = omega * track / 2. In stored units
    // (mrad/s, um, mm/s): omega * track <= 2 * wheel * 1000 um/mm * 1000 mrad/rad.
    const std::uint64_t needed = std::uint64_t{config.maxAngularMilliradiansPerSecond} * config.trackWidthMicrometres;
    const std::uint64_t available = 2 * std::uint64_t{config.maxWheelSpeedMillimetresPerSecond}
        * kMicrometresPerMillimetre * kMilliradiansPerRadianInt;
    if (needed > available) return RecordError::AngularLimitAboveWheelSpeed;
    if (config.maxCommandValidityMilliseconds == 0
        || std::chrono::milliseconds{config.maxCommandValidityMilliseconds} > timing::kCommandValidityCeiling)
        return RecordError::ValidityOutOfRange;
    if (std::chrono::milliseconds{config.telemetryPeriodMilliseconds} != timing::kTelemetryPeriod)
        return RecordError::TelemetryPeriodMismatch;
    return RecordError::None;
}

// Read-only view of a configuration that passed validate(). It can only be obtained
// through validated(), so holding one is proof that the values are usable. Accessors
// return SI units (metres, radians, seconds) as required at domain boundaries.
class ValidatedRobotConfig {
public:
    [[nodiscard]] static constexpr std::optional<ValidatedRobotConfig> validated(const RobotConfig& config) noexcept
    {
        if (validate(config) != RecordError::None) return std::nullopt;
        return ValidatedRobotConfig{config};
    }

    [[nodiscard]] constexpr const RobotConfig& raw() const noexcept { return config_; }
    [[nodiscard]] constexpr RobotId robotId() const noexcept { return config_.robotId; }
    [[nodiscard]] constexpr bool leftWheelReversed() const noexcept
    {
        return (config_.drivePolarity & polarity::kLeftReversed) != 0;
    }
    [[nodiscard]] constexpr bool rightWheelReversed() const noexcept
    {
        return (config_.drivePolarity & polarity::kRightReversed) != 0;
    }
    [[nodiscard]] constexpr std::uint8_t maxDriveOutputPercent() const noexcept { return config_.maxDriveOutputPercent; }
    [[nodiscard]] constexpr double wheelDiameterMetres() const noexcept
    {
        return config_.wheelDiameterMicrometres / kMicrometresPerMetre;
    }
    [[nodiscard]] constexpr double trackWidthMetres() const noexcept
    {
        return config_.trackWidthMicrometres / kMicrometresPerMetre;
    }
    [[nodiscard]] constexpr double maxWheelSpeedMetresPerSecond() const noexcept
    {
        return config_.maxWheelSpeedMillimetresPerSecond / kMillimetresPerMetre;
    }
    [[nodiscard]] constexpr double maxLinearMetresPerSecond() const noexcept
    {
        return config_.maxLinearMillimetresPerSecond / kMillimetresPerMetre;
    }
    [[nodiscard]] constexpr double maxAngularRadiansPerSecond() const noexcept
    {
        return config_.maxAngularMilliradiansPerSecond / kMilliradiansPerRadian;
    }
    [[nodiscard]] constexpr std::chrono::milliseconds maxCommandValidity() const noexcept
    {
        return std::chrono::milliseconds{config_.maxCommandValidityMilliseconds};
    }
    [[nodiscard]] constexpr std::chrono::milliseconds telemetryPeriod() const noexcept
    {
        return std::chrono::milliseconds{config_.telemetryPeriodMilliseconds};
    }

private:
    constexpr explicit ValidatedRobotConfig(const RobotConfig& config) noexcept : config_(config) {}
    RobotConfig config_;
};

// ---- Stored layout ----
inline constexpr std::array<std::uint8_t, 4> kRecordMagic{'M', 'R', 'C', 'F'};
inline constexpr std::uint16_t kSchemaVersion = 1;
inline constexpr std::size_t kHeaderBytes = 12;
inline constexpr std::size_t kChecksumBytes = 4;
inline constexpr std::size_t kSchemaV1PayloadBytes = 26;
inline constexpr std::size_t kRecordV1Bytes = kHeaderBytes + kSchemaV1PayloadBytes + kChecksumBytes;
// Read buffer for any version; a larger stored value is reported as LengthMismatch.
inline constexpr std::size_t kMaxRecordBytes = 64;
static_assert(kRecordV1Bytes <= kMaxRecordBytes);

using RecordBytes = std::array<std::uint8_t, kRecordV1Bytes>;

// CRC-32 (IEEE 802.3, reflected 0xEDB88320, as used by zlib).
[[nodiscard]] constexpr std::uint32_t crc32(std::span<const std::uint8_t> bytes) noexcept
{
    constexpr std::uint32_t kPolynomial = 0xEDB88320U;
    std::uint32_t crc = 0xFFFFFFFFU;
    for (const auto byte : bytes) {
        crc ^= byte;
        for (int bit = 0; bit < 8; ++bit) crc = (crc >> 1) ^ (kPolynomial & (0U - (crc & 1U)));
    }
    return ~crc;
}

namespace detail {
constexpr void put16(std::span<std::uint8_t> out, std::size_t at, std::uint16_t value) noexcept
{
    out[at] = static_cast<std::uint8_t>(value);
    out[at + 1] = static_cast<std::uint8_t>(value >> 8);
}
constexpr void put32(std::span<std::uint8_t> out, std::size_t at, std::uint32_t value) noexcept
{
    put16(out, at, static_cast<std::uint16_t>(value));
    put16(out, at + 2, static_cast<std::uint16_t>(value >> 16));
}
[[nodiscard]] constexpr std::uint16_t get16(std::span<const std::uint8_t> in, std::size_t at) noexcept
{
    return static_cast<std::uint16_t>(in[at] | (in[at + 1] << 8));
}
[[nodiscard]] constexpr std::uint32_t get32(std::span<const std::uint8_t> in, std::size_t at) noexcept
{
    return get16(in, at) | (std::uint32_t{get16(in, at + 2)} << 16);
}

// Schema 1 payload offsets (absolute).
inline constexpr std::size_t kVersionAt = 4;
inline constexpr std::size_t kLengthAt = 6;
inline constexpr std::size_t kGenerationAt = 8;
inline constexpr std::size_t kRobotIdAt = 12;
inline constexpr std::size_t kBoardAt = 16;
inline constexpr std::size_t kCapabilitiesAt = 17;
inline constexpr std::size_t kPolarityAt = 18;
inline constexpr std::size_t kDriveOutputAt = 19;
inline constexpr std::size_t kWheelDiameterAt = 20;
inline constexpr std::size_t kTrackWidthAt = 24;
inline constexpr std::size_t kWheelSpeedAt = 28;
inline constexpr std::size_t kMaxLinearAt = 30;
inline constexpr std::size_t kMaxAngularAt = 32;
inline constexpr std::size_t kValidityAt = 34;
inline constexpr std::size_t kTelemetryAt = 36;
inline constexpr std::size_t kChecksumV1At = 38;
static_assert(kChecksumV1At + kChecksumBytes == kRecordV1Bytes);
} // namespace detail

// Encodes the current schema. Callers validate first; encoding never alters values.
[[nodiscard]] constexpr RecordBytes encode(const RobotConfig& config, std::uint32_t generation) noexcept
{
    using namespace detail;
    RecordBytes out{};
    for (std::size_t i = 0; i < kRecordMagic.size(); ++i) out[i] = kRecordMagic[i];
    put16(out, kVersionAt, kSchemaVersion);
    put16(out, kLengthAt, static_cast<std::uint16_t>(kRecordV1Bytes));
    put32(out, kGenerationAt, generation);
    put32(out, kRobotIdAt, config.robotId);
    out[kBoardAt] = config.boardProfile;
    out[kCapabilitiesAt] = config.requiredCapabilities;
    out[kPolarityAt] = config.drivePolarity;
    out[kDriveOutputAt] = config.maxDriveOutputPercent;
    put32(out, kWheelDiameterAt, config.wheelDiameterMicrometres);
    put32(out, kTrackWidthAt, config.trackWidthMicrometres);
    put16(out, kWheelSpeedAt, config.maxWheelSpeedMillimetresPerSecond);
    put16(out, kMaxLinearAt, config.maxLinearMillimetresPerSecond);
    put16(out, kMaxAngularAt, config.maxAngularMilliradiansPerSecond);
    put16(out, kValidityAt, config.maxCommandValidityMilliseconds);
    put16(out, kTelemetryAt, config.telemetryPeriodMilliseconds);
    put32(out, kChecksumV1At, crc32(std::span<const std::uint8_t>{out}.first(kChecksumV1At)));
    return out;
}

struct DecodedRecord {
    RecordError error{RecordError::Truncated};
    // Set once the header, length and CRC are intact, even if the version or content
    // is unusable: the generation then still orders it against the other slot.
    std::optional<std::uint32_t> generation{};
    RobotConfig config{};  // Meaningful only when error == None.
};

// Integrity first (magic, length, CRC), then version, then content. A version other
// than kSchemaVersion is never reinterpreted: a future schema adds its own decoder here
// and migrates by re-validating into the current RobotConfig.
[[nodiscard]] constexpr DecodedRecord decode(std::span<const std::uint8_t> bytes) noexcept
{
    using namespace detail;
    DecodedRecord result;
    if (bytes.size() < kHeaderBytes + kChecksumBytes) return result;
    for (std::size_t i = 0; i < kRecordMagic.size(); ++i) {
        if (bytes[i] != kRecordMagic[i]) {
            result.error = RecordError::BadMagic;
            return result;
        }
    }
    if (get16(bytes, kLengthAt) != bytes.size()) {
        result.error = RecordError::LengthMismatch;
        return result;
    }
    const std::size_t checksumAt = bytes.size() - kChecksumBytes;
    if (crc32(bytes.first(checksumAt)) != get32(bytes, checksumAt)) {
        result.error = RecordError::ChecksumMismatch;
        return result;
    }
    result.generation = get32(bytes, kGenerationAt);
    if (get16(bytes, kVersionAt) != kSchemaVersion) {
        result.error = RecordError::UnsupportedVersion;
        return result;
    }
    if (bytes.size() != kRecordV1Bytes) {
        result.error = RecordError::LengthMismatch;
        return result;
    }
    RobotConfig& c = result.config;
    c.robotId = get32(bytes, kRobotIdAt);
    c.boardProfile = bytes[kBoardAt];
    c.requiredCapabilities = bytes[kCapabilitiesAt];
    c.drivePolarity = bytes[kPolarityAt];
    c.maxDriveOutputPercent = bytes[kDriveOutputAt];
    c.wheelDiameterMicrometres = get32(bytes, kWheelDiameterAt);
    c.trackWidthMicrometres = get32(bytes, kTrackWidthAt);
    c.maxWheelSpeedMillimetresPerSecond = get16(bytes, kWheelSpeedAt);
    c.maxLinearMillimetresPerSecond = get16(bytes, kMaxLinearAt);
    c.maxAngularMilliradiansPerSecond = get16(bytes, kMaxAngularAt);
    c.maxCommandValidityMilliseconds = get16(bytes, kValidityAt);
    c.telemetryPeriodMilliseconds = get16(bytes, kTelemetryAt);
    result.error = validate(c);
    return result;
}
} // namespace codecell::provisioning
