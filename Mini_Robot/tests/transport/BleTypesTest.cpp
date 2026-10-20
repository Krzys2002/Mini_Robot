#include "transport/ble/BleTypes.hpp"

#include <gtest/gtest.h>
#include <array>
#include <limits>

namespace {
using namespace micro_robot;
constexpr std::array<std::uint8_t, 18> infoBytes{
    1, 1, 1, 7, 0, 0, 0, 0, 1, 0, 0x2c, 1, 0x70, 0x17, 0xf4, 1, 0x64, 0};
TEST(BleTypesTest, ParsesAdvertisedIdentityHintsWithoutConflatingAddresses)
{
    EXPECT_EQ(advertisedBleRobotId("MR-0"), RobotId{0});
    EXPECT_EQ(advertisedBleRobotId("MR-4294967295"), RobotId{std::numeric_limits<std::uint32_t>::max()});
    for (const auto* name : {"", "MR-", "MR--1", "MR-+1", "MR-7extra", "MR-4294967296", "MR- 7", "robot-7"})
        EXPECT_FALSE(advertisedBleRobotId(name)) << name;
}
TEST(BleTypesTest, ValidatesDocumentedRobotInfoWithLittleEndianLimits)
{
    const auto result = validateBleRobotInfo(infoBytes, RobotId{7});
    ASSERT_TRUE(result.info) << result.error;
    EXPECT_EQ(result.info->robotId, RobotId{7});
    EXPECT_EQ(result.info->maximumLinearMillimetersPerSecond, 300);
    EXPECT_EQ(result.info->maximumAngularMilliradiansPerSecond, 6000);
    EXPECT_EQ(result.info->maximumValidityMilliseconds, 500);
    EXPECT_EQ(result.info->telemetryPeriodMilliseconds, 100);
    EXPECT_FALSE(result.unsupportedVersion);
    EXPECT_FALSE(validateBleRobotInfo(infoBytes, RobotId{8}).info);
}
TEST(BleTypesTest, RejectsEveryTruncatedInfoAndUnknownVersionsAndFormats)
{
    for (std::size_t size = 0; size < infoBytes.size(); ++size) {
        const auto result = validateBleRobotInfo(std::span{infoBytes}.first(size), RobotId{7});
        EXPECT_FALSE(result.info);
        EXPECT_TRUE(result.unsupportedVersion);
    }
    std::array<std::uint8_t, 19> oversized{};
    EXPECT_FALSE(validateBleRobotInfo(oversized, RobotId{7}).info);
    for (std::size_t i = 0; i < 3; ++i) {
        auto bytes = infoBytes;
        bytes[i] = 2; // Unknown format, range excluding v1 or inverted range.
        if (i == 2) bytes[1] = 2;
        EXPECT_TRUE(validateBleRobotInfo(bytes, RobotId{7}).unsupportedVersion);
    }
}
TEST(BleTypesTest, RejectsInvalidLimitsAndPreservesFullUnsignedIdentityRange)
{
    for (std::size_t offset : {10U, 12U, 14U, 16U}) {
        auto bytes = infoBytes;
        bytes[offset] = bytes[offset + 1] = 0;
        EXPECT_FALSE(validateBleRobotInfo(bytes, RobotId{7}).info);
    }
    for (std::size_t offset : {10U, 12U}) {
        auto bytes = infoBytes;
        bytes[offset + 1] = 0x80;
        EXPECT_FALSE(validateBleRobotInfo(bytes, RobotId{7}).info);
    }
    auto bytes = infoBytes;
    bytes[3] = bytes[4] = bytes[5] = bytes[6] = 0xff;
    ASSERT_TRUE(validateBleRobotInfo(bytes, RobotId{0xffffffff}).info);
}
TEST(BleTypesTest, ValidatesAllRequiredCharacteristicsPropertiesAndNotificationDescriptors)
{
    const std::array<BleCharacteristicCapabilities, 5> valid{{
        {true, true, false, false, false, false},
        {true, false, true, true, false, false},
        {true, false, true, true, false, false},
        {true, true, false, false, true, true},
        {true, false, false, false, true, true}}};
    EXPECT_TRUE(validateBleGattLayout(valid).empty());
    for (std::size_t i = 0; i < valid.size(); ++i) {
        auto layout = valid;
        layout[i].present = false;
        EXPECT_FALSE(validateBleGattLayout(layout).empty());
    }
    const auto reject = [&](std::size_t index, bool BleCharacteristicCapabilities::*property) {
        auto layout = valid;
        layout[index].*property = false;
        EXPECT_FALSE(validateBleGattLayout(layout).empty());
    };
    reject(0, &BleCharacteristicCapabilities::read);
    for (std::size_t i : {1U, 2U}) {
        reject(i, &BleCharacteristicCapabilities::write);
        reject(i, &BleCharacteristicCapabilities::writeWithoutResponse);
    }
    reject(3, &BleCharacteristicCapabilities::read);
    for (std::size_t i : {3U, 4U}) {
        reject(i, &BleCharacteristicCapabilities::notify);
        reject(i, &BleCharacteristicCapabilities::cccd);
    }
}
} // namespace
