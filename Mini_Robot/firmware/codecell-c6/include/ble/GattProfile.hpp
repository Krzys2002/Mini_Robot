#pragma once
// Protocol v1 GATT layout, advertising payloads and link-parameter hints of the ESP32-C6
// peripheral (#163; contract: docs/protocol/ble-gatt.md, #64). Portable C++20: no NimBLE,
// ESP-IDF or Arduino types. The NimBLE adapter (src/NimBlePeripheral.hpp) builds the GATT
// server and the advertising from these values only, and the host tests check them byte
// for byte against the specification.
#include "TimingBudget.hpp"
#include "protocol/V1Codec.hpp"
#include "provisioning/RobotIdentity.hpp"
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>

namespace codecell::ble {

// ---- UUIDs ----------------------------------------------------------------------------
// All v1 UUIDs share one project base; only the first 32-bit field differs. Fixed for v1.
inline constexpr const char* kServiceUuid = "4d520001-1f2a-4032-8415-63a5fef8a0dc";

inline constexpr std::size_t kUuidTextLength = 36;  // 8-4-4-4-12 hexadecimal digits
inline constexpr std::size_t kUuid128Length = 16;
// A 128-bit UUID in over-the-air (little-endian) byte order, as in advertising data.
using Uuid128Bytes = std::array<std::uint8_t, kUuid128Length>;

namespace detail {
[[nodiscard]] constexpr std::optional<std::uint8_t> hexDigit(char c) noexcept
{
    if (c >= '0' && c <= '9') return static_cast<std::uint8_t>(c - '0');
    if (c >= 'a' && c <= 'f') return static_cast<std::uint8_t>(c - 'a' + 10);
    if (c >= 'A' && c <= 'F') return static_cast<std::uint8_t>(c - 'A' + 10);
    return std::nullopt;
}
[[nodiscard]] constexpr bool isUuidDash(std::size_t at) noexcept { return at == 8 || at == 13 || at == 18 || at == 23; }
} // namespace detail

// Canonical text form to over-the-air bytes; nullopt unless it is exactly 8-4-4-4-12 hex.
[[nodiscard]] constexpr std::optional<Uuid128Bytes> uuidBytes(std::string_view text) noexcept
{
    if (text.size() != kUuidTextLength) return std::nullopt;
    Uuid128Bytes bigEndian{};
    std::size_t out = 0;
    for (std::size_t at = 0; at < text.size();) {
        if (detail::isUuidDash(at)) {
            if (text[at] != '-') return std::nullopt;
            ++at;
            continue;
        }
        const auto high = detail::hexDigit(text[at]);
        const auto low = detail::isUuidDash(at + 1) ? std::nullopt : detail::hexDigit(text[at + 1]);
        if (!high || !low) return std::nullopt;
        bigEndian[out++] = static_cast<std::uint8_t>((*high << 4) | *low);
        at += 2;
    }
    Uuid128Bytes wire{};
    for (std::size_t i = 0; i < wire.size(); ++i) wire[i] = bigEndian[wire.size() - 1 - i];
    return wire;
}

// ---- Characteristics ------------------------------------------------------------------
enum class Characteristic : std::uint8_t { RobotInfo, Command, EmergencyStop, Telemetry, CommandResult };
inline constexpr std::size_t kCharacteristicCount = 5;

// The ATT maximum attribute value length (Bluetooth Core, Vol 3, Part F, 3.2.9). Both written
// characteristics accept it, so that every write reaches the peripheral controller: a write
// longer than the attribute's maximum would be refused with an ATT error by the stack before
// any callback, which would let an oversized Emergency Stop write skip the latch (ble-gatt.md:
// any write of any length latches) and an oversized Command write skip its MalformedLength
// result (the GATT layer must not return ATT errors for protocol rejections).
inline constexpr std::uint16_t kMaxAttributeValueLength = 512;

struct CharacteristicSpec {
    Characteristic id;
    const char* uuid;
    bool read;
    bool write;
    bool writeWithoutResponse;
    bool notify;  // Notify characteristics carry a CCCD (0x2902).
    std::uint16_t maxValueLength;
};

inline constexpr std::array<CharacteristicSpec, kCharacteristicCount> kCharacteristics{{
    {Characteristic::RobotInfo, "4d520002-1f2a-4032-8415-63a5fef8a0dc", true, false, false, false,
     static_cast<std::uint16_t>(protocol::v1::kRobotInfoLength)},
    {Characteristic::Command, "4d520003-1f2a-4032-8415-63a5fef8a0dc", false, true, true, false,
     kMaxAttributeValueLength},
    {Characteristic::EmergencyStop, "4d520004-1f2a-4032-8415-63a5fef8a0dc", false, true, true, false,
     kMaxAttributeValueLength},
    {Characteristic::Telemetry, "4d520005-1f2a-4032-8415-63a5fef8a0dc", true, false, false, true,
     static_cast<std::uint16_t>(protocol::v1::kTelemetryLength)},
    {Characteristic::CommandResult, "4d520006-1f2a-4032-8415-63a5fef8a0dc", false, false, false, true,
     static_cast<std::uint16_t>(protocol::v1::kCommandResultLength)},
}};

[[nodiscard]] constexpr const CharacteristicSpec& specOf(Characteristic id) noexcept
{
    return kCharacteristics[static_cast<std::size_t>(id)];
}

[[nodiscard]] constexpr const char* toString(Characteristic id) noexcept
{
    switch (id) {
    case Characteristic::RobotInfo: return "robot_info";
    case Characteristic::Command: return "command";
    case Characteristic::EmergencyStop: return "emergency_stop";
    case Characteristic::Telemetry: return "telemetry";
    case Characteristic::CommandResult: return "command_result";
    }
    return "unknown";
}

// Every v1 payload fits the default ATT MTU; a larger negotiated MTU is never used to send
// longer or concatenated frames (ble-gatt.md, link parameters).
inline constexpr std::uint16_t kDefaultAttMtu = 23;
static_assert(protocol::v1::kMaxAttPayload == kDefaultAttMtu - 3, "ATT notification/write header is 3 bytes");

// Robot Info firmware version bytes (#164; diagnostic only, robot-protocol.md). 0.1.0 is the
// first image that publishes the whole v1 surface: Robot Info, CommandResults and Telemetry.
// Pre-1.0: a version names an image, it promises no compatibility beyond protocol v1, and no
// desktop decision may depend on it. Change it when the image's public behaviour changes; the
// exact build is identified by the ELF SHA-256 in the boot line and the build provenance.
inline constexpr protocol::v1::FirmwareVersion kRobotInfoFirmwareVersion{0, 1, 0};

// ---- Advertising ----------------------------------------------------------------------
inline constexpr std::uint8_t kAdTypeFlags = 0x01;
inline constexpr std::uint8_t kAdTypeComplete128BitUuids = 0x07;
inline constexpr std::uint8_t kAdTypeCompleteLocalName = 0x09;
// LE General Discoverable Mode, BR/EDR not supported.
inline constexpr std::uint8_t kAdvertisingFlags = 0x06;
inline constexpr std::size_t kMaxLegacyAdvertisingPayload = 31;

// Flags (3 bytes) and the complete list of 128-bit service UUIDs with the Robot Control
// service (18 bytes): 21 bytes, as specified.
inline constexpr std::size_t kAdvertisingDataLength = 21;
using AdvertisingData = std::array<std::uint8_t, kAdvertisingDataLength>;

[[nodiscard]] constexpr AdvertisingData advertisingData() noexcept
{
    AdvertisingData out{};
    std::size_t at = 0;
    out[at++] = 2;  // Length of type + value.
    out[at++] = kAdTypeFlags;
    out[at++] = kAdvertisingFlags;
    out[at++] = static_cast<std::uint8_t>(1 + kUuid128Length);
    out[at++] = kAdTypeComplete128BitUuids;
    const auto uuid = uuidBytes(kServiceUuid).value_or(Uuid128Bytes{});
    for (const auto byte : uuid) out[at++] = byte;
    return out;
}
static_assert(uuidBytes(kServiceUuid).has_value());
static_assert(kAdvertisingDataLength <= kMaxLegacyAdvertisingPayload);

// Scan response: the Complete Local Name "MR-<robotId in decimal>".
struct ScanResponse {
    std::array<std::uint8_t, kMaxLegacyAdvertisingPayload> bytes{};
    std::size_t length{0};
    [[nodiscard]] std::span<const std::uint8_t> view() const noexcept { return {bytes.data(), length}; }
};

[[nodiscard]] inline ScanResponse scanResponse(provisioning::RobotId robotId) noexcept
{
    const auto name = provisioning::advertisedName(robotId);
    const std::string_view text{name.data()};
    ScanResponse out;
    out.bytes[0] = static_cast<std::uint8_t>(1 + text.size());
    out.bytes[1] = kAdTypeCompleteLocalName;
    for (std::size_t i = 0; i < text.size(); ++i) out.bytes[2 + i] = static_cast<std::uint8_t>(text[i]);
    out.length = 2 + text.size();
    return out;
}
static_assert(2 + provisioning::kAdvertisedNameCapacity - 1 <= kMaxLegacyAdvertisingPayload,
              "the longest name, MR-4294967295, fits a legacy scan response");

// ---- Link parameters ------------------------------------------------------------------
// Requested after connecting (ble-gatt.md, link parameters): interval 7.5-30 ms, peripheral
// latency 0, supervision timeout 400 ms. These are hints, never safety guarantees: the
// central may refuse them, and macOS manages its own. The command validity window and the
// stop on disconnect bound motion whatever is negotiated.
inline constexpr std::chrono::microseconds kConnectionIntervalUnit{1250};
inline constexpr std::chrono::milliseconds kSupervisionTimeoutUnit{10};
inline constexpr std::chrono::microseconds kRequestedMinConnectionInterval{7500};
inline constexpr std::chrono::microseconds kRequestedMaxConnectionInterval{timing::kRequestedMaxConnectionInterval};
inline constexpr std::uint16_t kRequestedPeripheralLatency = 0;
inline constexpr std::chrono::milliseconds kRequestedSupervisionTimeout{400};

// Values in Link Layer units: interval in 1.25 ms, supervision timeout in 10 ms.
struct LinkParameterRequest {
    std::uint16_t minIntervalUnits{0};
    std::uint16_t maxIntervalUnits{0};
    std::uint16_t latency{0};
    std::uint16_t supervisionTimeoutUnits{0};
};

inline constexpr LinkParameterRequest kLinkParameterRequest{
    static_cast<std::uint16_t>(kRequestedMinConnectionInterval / kConnectionIntervalUnit),
    static_cast<std::uint16_t>(kRequestedMaxConnectionInterval / kConnectionIntervalUnit),
    kRequestedPeripheralLatency,
    static_cast<std::uint16_t>(kRequestedSupervisionTimeout / kSupervisionTimeoutUnit),
};
static_assert(kRequestedMinConnectionInterval % kConnectionIntervalUnit == std::chrono::microseconds::zero());
static_assert(kRequestedMaxConnectionInterval % kConnectionIntervalUnit == std::chrono::microseconds::zero());
static_assert(kRequestedSupervisionTimeout % kSupervisionTimeoutUnit == std::chrono::milliseconds::zero());

// Parameters the stack reports for a connection, in the same Link Layer units.
struct LinkParameters {
    std::uint16_t intervalUnits{0};
    std::uint16_t latency{0};
    std::uint16_t supervisionTimeoutUnits{0};
};

[[nodiscard]] constexpr std::chrono::microseconds connectionInterval(const LinkParameters& p) noexcept
{
    return p.intervalUnits * kConnectionIntervalUnit;
}
[[nodiscard]] constexpr std::chrono::milliseconds supervisionTimeout(const LinkParameters& p) noexcept
{
    return p.supervisionTimeoutUnits * kSupervisionTimeoutUnit;
}
// Whether the negotiated values are those requested. Diagnostic only: nothing depends on it.
[[nodiscard]] constexpr bool withinRequest(const LinkParameters& p, const LinkParameterRequest& r) noexcept
{
    return p.intervalUnits >= r.minIntervalUnits && p.intervalUnits <= r.maxIntervalUnits && p.latency == r.latency
        && p.supervisionTimeoutUnits == r.supervisionTimeoutUnits;
}
} // namespace codecell::ble
