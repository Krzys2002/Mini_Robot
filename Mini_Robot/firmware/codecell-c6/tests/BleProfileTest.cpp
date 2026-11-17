// Host tests of the #163 GATT layout, advertising payloads, link-parameter hints and the
// session readiness policy, checked byte for byte against docs/protocol/ble-gatt.md.
#include "ProvisioningFakes.hpp"
#include "ble/GattProfile.hpp"
#include "ble/SessionPolicy.hpp"
#include "protocol/V1Codec.hpp"
#include "provisioning/RobotIdentity.hpp"
#include <gtest/gtest.h>
#include <array>
#include <cstdint>
#include <set>
#include <string>
#include <string_view>
#include <vector>

using namespace codecell;
using namespace codecell::ble;
using namespace std::chrono_literals;

namespace {
// The v1 service UUID 4d520001-1f2a-4032-8415-63a5fef8a0dc, least significant byte first.
constexpr Uuid128Bytes kServiceUuidOverTheAir{0xDC, 0xA0, 0xF8, 0xFE, 0xA5, 0x63, 0x15, 0x84,
                                              0x32, 0x40, 0x2A, 0x1F, 0x01, 0x00, 0x52, 0x4D};
} // namespace

TEST(BleGattProfileTest, UuidsAreTheFixedV1ValuesOnOneBase)
{
    EXPECT_EQ(std::string_view{kServiceUuid}, "4d520001-1f2a-4032-8415-63a5fef8a0dc");
    const std::array<std::string_view, kCharacteristicCount> expected{
        "4d520002-1f2a-4032-8415-63a5fef8a0dc", "4d520003-1f2a-4032-8415-63a5fef8a0dc",
        "4d520004-1f2a-4032-8415-63a5fef8a0dc", "4d520005-1f2a-4032-8415-63a5fef8a0dc",
        "4d520006-1f2a-4032-8415-63a5fef8a0dc"};
    std::set<std::string> distinct{kServiceUuid};
    for (std::size_t i = 0; i < kCharacteristics.size(); ++i) {
        EXPECT_EQ(static_cast<std::size_t>(kCharacteristics[i].id), i) << "table order is the enum order";
        EXPECT_EQ(std::string_view{kCharacteristics[i].uuid}, expected[i]);
        const auto bytes = uuidBytes(kCharacteristics[i].uuid);
        ASSERT_TRUE(bytes);
        // Only the first 32-bit field (the last four bytes over the air) differs from the service.
        for (std::size_t b = 0; b < 12; ++b) EXPECT_EQ((*bytes)[b], kServiceUuidOverTheAir[b]);
        distinct.insert(kCharacteristics[i].uuid);
    }
    EXPECT_EQ(distinct.size(), kCharacteristicCount + 1);
    EXPECT_EQ(uuidBytes(kServiceUuid), kServiceUuidOverTheAir);
}

TEST(BleGattProfileTest, UuidParserRejectsAnythingButCanonicalText)
{
    EXPECT_FALSE(uuidBytes(""));
    EXPECT_FALSE(uuidBytes("4d520001-1f2a-4032-8415-63a5fef8a0d"));    // short
    EXPECT_FALSE(uuidBytes("4d520001-1f2a-4032-8415-63a5fef8a0dc0"));  // long
    EXPECT_FALSE(uuidBytes("4d5200011f2a-4032-8415-63a5fef8a0dc-"));   // dash moved
    EXPECT_FALSE(uuidBytes("4d520001-1f2a-4032-8415-63a5fef8a0dg"));   // not hex
    EXPECT_TRUE(uuidBytes("4D520001-1F2A-4032-8415-63A5FEF8A0DC"));
    EXPECT_EQ(uuidBytes("4D520001-1F2A-4032-8415-63A5FEF8A0DC"), kServiceUuidOverTheAir);
}

TEST(BleGattProfileTest, PropertiesAndCccdsMatchTheContract)
{
    const auto& info = specOf(Characteristic::RobotInfo);
    EXPECT_TRUE(info.read);
    EXPECT_FALSE(info.write || info.writeWithoutResponse || info.notify);
    EXPECT_EQ(static_cast<std::size_t>(info.maxValueLength), protocol::v1::kRobotInfoLength);

    for (const auto id : {Characteristic::Command, Characteristic::EmergencyStop}) {
        const auto& written = specOf(id);
        EXPECT_TRUE(written.write) << toString(id);
        EXPECT_TRUE(written.writeWithoutResponse) << toString(id);
        EXPECT_FALSE(written.read || written.notify) << toString(id);
        // Every write must reach the callback, so the stack never refuses one by length.
        EXPECT_EQ(written.maxValueLength, kMaxAttributeValueLength) << toString(id);
    }

    const auto& telemetry = specOf(Characteristic::Telemetry);
    EXPECT_TRUE(telemetry.read && telemetry.notify);
    EXPECT_FALSE(telemetry.write || telemetry.writeWithoutResponse);
    EXPECT_EQ(static_cast<std::size_t>(telemetry.maxValueLength), protocol::v1::kTelemetryLength);

    const auto& result = specOf(Characteristic::CommandResult);
    EXPECT_TRUE(result.notify);
    EXPECT_FALSE(result.read || result.write || result.writeWithoutResponse);
    EXPECT_EQ(static_cast<std::size_t>(result.maxValueLength), protocol::v1::kCommandResultLength);

    int cccds = 0;
    for (const auto& c : kCharacteristics) cccds += c.notify ? 1 : 0;
    EXPECT_EQ(cccds, 2) << "Telemetry and Command Result each carry one CCCD";
}

TEST(BleGattProfileTest, EveryV1PayloadFitsTheDefaultMtu)
{
    constexpr std::size_t kAttHeader = 3;
    constexpr std::size_t kMtu = kDefaultAttMtu;
    for (const auto& c : kCharacteristics) {
        if (!c.read && !c.notify) continue;
        EXPECT_LE(c.maxValueLength + kAttHeader, kMtu) << toString(c.id);
    }
    EXPECT_LE(protocol::v1::kVelocityLength + kAttHeader, kMtu);
}

TEST(BleGattProfileTest, AdvertisingIsFlagsAndTheCompleteServiceUuidListIn21Bytes)
{
    const auto data = advertisingData();
    std::vector<std::uint8_t> expected{0x02, 0x01, 0x06, 0x11, 0x07};
    expected.insert(expected.end(), kServiceUuidOverTheAir.begin(), kServiceUuidOverTheAir.end());
    EXPECT_EQ(std::vector<std::uint8_t>(data.begin(), data.end()), expected);
    EXPECT_EQ(data.size(), 21U);
}

TEST(BleGattProfileTest, ScanResponseIsTheCompleteLocalNameMrRobotId)
{
    const auto bytesOf = [](provisioning::RobotId id) {
        const auto r = scanResponse(id);
        return std::vector<std::uint8_t>(r.view().begin(), r.view().end());
    };
    EXPECT_EQ(bytesOf(7), (std::vector<std::uint8_t>{0x05, 0x09, 'M', 'R', '-', '7'}));
    EXPECT_EQ(bytesOf(0), (std::vector<std::uint8_t>{0x05, 0x09, 'M', 'R', '-', '0'}));
    const auto longest = bytesOf(4'294'967'295U);
    const std::string_view name{"MR-4294967295"};
    ASSERT_EQ(longest.size(), 2 + name.size());
    EXPECT_EQ(static_cast<std::size_t>(longest[0]), 1 + name.size());
    EXPECT_EQ(longest[1], kAdTypeCompleteLocalName);
    EXPECT_EQ(std::string(longest.begin() + 2, longest.end()), name);
    EXPECT_LE(longest.size(), kMaxLegacyAdvertisingPayload);
}

TEST(BleGattProfileTest, LinkParameterRequestIsTheDocumentedHint)
{
    EXPECT_EQ(kLinkParameterRequest.minIntervalUnits, 6);       // 7.5 ms
    EXPECT_EQ(kLinkParameterRequest.maxIntervalUnits, 24);      // 30 ms
    EXPECT_EQ(kLinkParameterRequest.latency, 0);
    EXPECT_EQ(kLinkParameterRequest.supervisionTimeoutUnits, 40);  // 400 ms
    EXPECT_EQ(connectionInterval({24, 0, 40}), 30ms);
    EXPECT_EQ(supervisionTimeout({24, 0, 40}), 400ms);
    EXPECT_TRUE(withinRequest({6, 0, 40}, kLinkParameterRequest));
    EXPECT_TRUE(withinRequest({24, 0, 40}, kLinkParameterRequest));
    EXPECT_FALSE(withinRequest({36, 0, 40}, kLinkParameterRequest));  // 45 ms, as a central may choose
    EXPECT_FALSE(withinRequest({24, 1, 40}, kLinkParameterRequest));
    EXPECT_FALSE(withinRequest({24, 0, 500}, kLinkParameterRequest));
}

TEST(BleGattProfileTest, RobotInfoIsTheProvisionedRecordWithTheImageFirmwareVersion)
{
    const auto config = provisioning::ValidatedRobotConfig::validated(provisioning::fakes::validConfig(7));
    ASSERT_TRUE(config);
    const auto info = provisioning::robotInfoValues(*config);
    const auto record = protocol::v1::encodeRobotInfo(info, kRobotInfoFirmwareVersion);
    ASSERT_TRUE(record);
    // 01 01 01 | id 7 | firmware 0.1.0 | 150 mm/s | 3000 mrad/s | 200 ms | 100 ms
    const std::vector<std::uint8_t> expected{0x01, 0x01, 0x01, 0x07, 0x00, 0x00, 0x00, 0x00, 0x01,
                                             0x00, 0x96, 0x00, 0xB8, 0x0B, 0xC8, 0x00, 0x64, 0x00};
    EXPECT_EQ(std::vector<std::uint8_t>(record->begin(), record->end()), expected);
}

TEST(BleSessionPolicyTest, RestrictedLabReadinessNeedsTheOwnerAndBothCccdsAndNothingElse)
{
    EXPECT_STREQ(toString(kSecurityProfile), "restricted-lab-v1");
    for (int bits = 0; bits < 16; ++bits) {
        const ReadinessInputs in{(bits & 1) != 0, (bits & 2) != 0, (bits & 4) != 0, (bits & 8) != 0};
        const bool expected = in.ownsSession && !in.closing && in.commandResultNotify && in.telemetryNotify;
        EXPECT_EQ(sessionMayBeReady(SecurityProfile::RestrictedLabV1, in), expected) << bits;
    }
}
