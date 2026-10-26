#include "../../firmware/ble-lifecycle/src/FixtureData.hpp"
#include "transport/ble/BleProtocol.hpp"

#include <gtest/gtest.h>

namespace {
using namespace micro_robot;
TEST(BleLifecycleFixtureTest, ProvisioningAcceptsFullIdentityRangeAndRejectsInvalidInput)
{
    std::uint32_t id = 42;
    ASSERT_TRUE(ble_fixture::parseId("0", id));
    EXPECT_EQ(id, 0U);
    ASSERT_TRUE(ble_fixture::parseId("4294967295", id));
    EXPECT_EQ(id, 0xffffffffU);
    for (const auto* text : {"", "-1", "+1", " 1", "1 ", "1x", "4294967296", "99999999999999999999999999"}) {
        id = 42;
        EXPECT_FALSE(ble_fixture::parseId(text, id)) << text;
        EXPECT_EQ(id, 42U);
    }
}
TEST(BleLifecycleFixtureTest, ActualFirmwareIdentityRecordsMatchDesktopValidation)
{
    for (const auto id : {0U, 1U, 7U, 0xffffffffU}) {
        const auto normal = validateBleRobotInfo(ble_fixture::robotInfo(id, false, false), RobotId{id});
        ASSERT_TRUE(normal.info) << normal.error;
        EXPECT_EQ(normal.info->robotId, RobotId{id});
        EXPECT_EQ(normal.info->maximumLinearMillimetersPerSecond, 300);
        EXPECT_EQ(normal.info->maximumAngularMilliradiansPerSecond, 6000);
        EXPECT_EQ(normal.info->maximumValidityMilliseconds, 500);
        EXPECT_EQ(normal.info->telemetryPeriodMilliseconds, 100);
        const auto wrongId = validateBleRobotInfo(ble_fixture::robotInfo(id, true, false), RobotId{id});
        EXPECT_FALSE(wrongId.info);
        EXPECT_FALSE(wrongId.unsupportedVersion);
        const auto wrongVersion = validateBleRobotInfo(ble_fixture::robotInfo(id, false, true), RobotId{id});
        EXPECT_FALSE(wrongVersion.info);
        EXPECT_TRUE(wrongVersion.unsupportedVersion);
    }
}
TEST(BleLifecycleFixtureTest, ActualFirmwareTelemetryDecodesAsIdleWithUnknownMeasurements)
{
    const auto decoded = decodeBleNotification(BleEndpoint::Telemetry,
        ble_fixture::idleTelemetry(0xfedcba98U, 0xffff));
    ASSERT_TRUE(decoded.frame) << decoded.error;
    EXPECT_EQ(decoded.frame->sequence, 0xffff);
    const auto& telemetry = std::get<BleTelemetryFrame>(decoded.frame->payload);
    EXPECT_EQ(telemetry.state, BleRobotState::Idle);
    EXPECT_EQ(telemetry.uptimeMilliseconds, 0xfedcba98U);
    EXPECT_FALSE(telemetry.commandExpired);
    EXPECT_FALSE(telemetry.batteryLow);
    EXPECT_FALSE(telemetry.lastAcceptedSequence);
    EXPECT_FALSE(telemetry.batteryVolts);
    EXPECT_FALSE(telemetry.linearMetersPerSecond);
    EXPECT_FALSE(telemetry.angularRadiansPerSecond);
}
} // namespace
