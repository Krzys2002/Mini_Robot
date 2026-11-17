#include "transport/ble/BleProtocol.hpp"
#include "transport/ble/BleWire.hpp"

#include <gtest/gtest.h>
#include <array>
#include <limits>

namespace {
using namespace micro_robot;
using namespace std::chrono_literals;
const BleRobotInfo limits{RobotId{7}, 300, 6000, 500, 100};
const Timestamp now{10s};
RobotCommand velocity(double linear = 0.150, double angular = -0.500)
{
    return {VelocityCommand{linear, angular}, now, 250ms};
}
const std::vector<std::uint8_t> telemetry{1, 0x80, 7, 0, 1, 4, 2, 1, 0x40, 0xe2, 1, 0, 0x48, 0xf, 0x94, 0, 0, 0x80};
const std::vector<std::uint8_t> result{1, 0x81, 8, 0, 2, 1, 2, 0, 1};
TEST(BleProtocolTest, EncodesDocumentedVelocityStopAndEmergencyStopBytesAndRouting)
{
    auto command = velocity();
    command.metadata.sequence = 99999; // Domain identifiers never become wire counters.
    const auto encoded = encodeBleCommand(command, limits, 258, now);
    ASSERT_TRUE(encoded.frame) << encoded.error;
    EXPECT_EQ(encoded.frame->bytes, (std::vector<std::uint8_t>{1, 2, 2, 1, 0x96, 0, 0xc, 0xfe, 0xfa, 0}));
    EXPECT_EQ(encoded.frame->endpoint, BleEndpoint::Command);
    EXPECT_FALSE(encoded.frame->withResponse);
    command.payload = StopCommand{};
    const auto stop = encodeBleCommand(command, limits, 259, now);
    ASSERT_TRUE(stop.frame);
    EXPECT_EQ(stop.frame->bytes, (std::vector<std::uint8_t>{1, 1, 3, 1}));
    EXPECT_TRUE(stop.frame->withResponse);
    const auto estop = encodeBleEmergencyStop(260);
    EXPECT_EQ(estop.bytes, (std::vector<std::uint8_t>{1, 3, 4, 1}));
    EXPECT_EQ(estop.endpoint, BleEndpoint::EmergencyStop);
    EXPECT_TRUE(estop.withResponse);
}
TEST(BleProtocolTest, RoundsAwayFromZeroAndUsesAdvertisedLimitsWithoutSaturation)
{
    const auto encoded = encodeBleCommand(velocity(0.0005, -0.0005), limits, 0, now);
    ASSERT_TRUE(encoded.frame);
    EXPECT_EQ(encoded.frame->bytes[4], 1);
    EXPECT_EQ(encoded.frame->bytes[6], 0xff);
    EXPECT_EQ(encoded.frame->bytes[7], 0xff);
    EXPECT_TRUE(encodeBleCommand(velocity(-0.3, 6), limits, 0, now).frame);
    EXPECT_FALSE(encodeBleCommand(velocity(0.3001, 0), limits, 0, now).frame);
    EXPECT_FALSE(encodeBleCommand(velocity(0, -6.001), limits, 0, now).frame);
    for (double value : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
                         -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::max()}) {
        EXPECT_FALSE(encodeBleCommand(velocity(value, 0), limits, 0, now).frame);
        EXPECT_FALSE(encodeBleCommand(velocity(0, value), limits, 0, now).frame);
    }
}
TEST(BleProtocolTest, RecomputesRemainingValidityAtDispatchAndRejectsDeadlineAndClockOverflow)
{
    auto command = velocity();
    const auto aged = encodeBleCommand(command, limits, 0, now + 100ms + 500us);
    ASSERT_TRUE(aged.frame);
    EXPECT_EQ(aged.frame->bytes[8], 149); // floor, never round upward.
    EXPECT_FALSE(encodeBleCommand(command, limits, 0, now + 250ms).frame);
    EXPECT_FALSE(encodeBleCommand(command, limits, 0, now + 249ms + 1us).frame);
    EXPECT_FALSE(encodeBleCommand(command, limits, 0, now - 1ns).frame);
    command.validFor = Duration::zero();
    EXPECT_FALSE(encodeBleCommand(command, limits, 0, now).frame);
    command.validFor = -1ms;
    EXPECT_FALSE(encodeBleCommand(command, limits, 0, now).frame);
    command.validFor = 5s;
    const auto capped = encodeBleCommand(command, limits, 0, now);
    ASSERT_TRUE(capped.frame);
    EXPECT_EQ(capped.frame->bytes[8], 0xf4);
    EXPECT_EQ(capped.frame->bytes[9], 1);
    command.issuedAt = Timestamp::min(); command.validFor = Duration::max();
    EXPECT_FALSE(encodeBleCommand(command, limits, 0, Timestamp::max()).frame);
}
TEST(BleProtocolTest, ValidatesDomainVersionAndAdvertisedLimitsButNeverExpiresStop)
{
    auto command = velocity();
    command.metadata.modelVersion = 2;
    EXPECT_EQ(encodeBleCommand(command, limits, 0, now).code, TransportErrorCode::UnsupportedVersion);
    command = velocity(); command.payload = StopCommand{};
    EXPECT_TRUE(encodeBleCommand(command, limits, 0, now + 250ms).frame);
    command.issuedAt = Timestamp::min();
    command.validFor = Duration::zero();
    EXPECT_TRUE(encodeBleCommand(command, limits, 0, Timestamp::max()).frame);
    auto invalidLimits = limits;
    invalidLimits.maximumValidityMilliseconds = 0;
    EXPECT_FALSE(encodeBleCommand(velocity(), invalidLimits, 0, now).frame);
    invalidLimits = limits; invalidLimits.maximumAngularMilliradiansPerSecond = 32768;
    EXPECT_FALSE(encodeBleCommand(velocity(), invalidLimits, 0, now).frame);
}
TEST(BleProtocolTest, DecodesDocumentedTelemetryAndCommandResult)
{
    const auto decoded = decodeBleNotification(BleEndpoint::Telemetry, telemetry);
    ASSERT_TRUE(decoded.frame) << decoded.error;
    EXPECT_EQ(decoded.frame->sequence, 7);
    const auto& t = std::get<BleTelemetryFrame>(decoded.frame->payload);
    EXPECT_EQ(t.state, BleRobotState::Moving);
    EXPECT_EQ(t.lastAcceptedSequence, 258);
    EXPECT_EQ(t.uptimeMilliseconds, 123456U);
    EXPECT_DOUBLE_EQ(*t.batteryVolts, 3.912);
    EXPECT_DOUBLE_EQ(*t.linearMetersPerSecond, 0.148);
    EXPECT_FALSE(t.angularRadiansPerSecond);
    const auto reply = decodeBleNotification(BleEndpoint::CommandResult, result);
    ASSERT_TRUE(reply.frame);
    const auto& r = std::get<BleCommandResultFrame>(reply.frame->payload);
    EXPECT_EQ(r.commandSequence, 258);
    EXPECT_EQ(r.commandType, 2);
    EXPECT_EQ(r.result, 0);
    EXPECT_EQ(r.state, BleRobotState::Moving);
}
TEST(BleProtocolTest, UnknownMeasurementsFlagsAndSignedExtremesRemainExplicit)
{
    auto bytes = telemetry;
    bytes[5] = 0xfb; // Reserved bits ignored, no accepted-command flag.
    bytes[12] = bytes[13] = 0xff;
    bytes[14] = 1; bytes[15] = 0x80; // -32767.
    bytes[16] = 0xff; bytes[17] = 0x7f; // +32767.
    const auto decoded = decodeBleNotification(BleEndpoint::Telemetry, bytes);
    ASSERT_TRUE(decoded.frame);
    const auto& t = std::get<BleTelemetryFrame>(decoded.frame->payload);
    EXPECT_FALSE(t.batteryVolts);
    EXPECT_FALSE(t.lastAcceptedSequence);
    EXPECT_TRUE(t.commandExpired);
    EXPECT_TRUE(t.batteryLow);
    EXPECT_DOUBLE_EQ(*t.linearMetersPerSecond, -32.767);
    EXPECT_DOUBLE_EQ(*t.angularRadiansPerSecond, 32.767);
}
TEST(BleProtocolTest, RejectsMalformedFramesVersionsTypesChannelsAndReservedEnums)
{
    for (const auto endpoint : {BleEndpoint::Telemetry, BleEndpoint::CommandResult}) {
        const auto& valid = endpoint == BleEndpoint::Telemetry ? telemetry : result;
        for (std::size_t length = 0; length < valid.size(); ++length)
            EXPECT_FALSE(decodeBleNotification(endpoint, std::span{valid}.first(length)).frame);
        auto bytes = valid; bytes.push_back(0);
        EXPECT_FALSE(decodeBleNotification(endpoint, bytes).frame);
        bytes = valid; bytes[0] = 2;
        EXPECT_EQ(decodeBleNotification(endpoint, bytes).code, TransportErrorCode::UnsupportedVersion);
        bytes = valid; bytes[1] = 0xff;
        EXPECT_FALSE(decodeBleNotification(endpoint, bytes).frame);
        bytes = valid; bytes[endpoint == BleEndpoint::Telemetry ? 4 : 8] = 4;
        EXPECT_FALSE(decodeBleNotification(endpoint, bytes).frame);
    }
    EXPECT_FALSE(decodeBleNotification(BleEndpoint::CommandResult, telemetry).frame);
    EXPECT_FALSE(decodeBleNotification(BleEndpoint::Command, telemetry).frame);
    auto bytes = result; bytes[7] = 9;
    EXPECT_FALSE(decodeBleNotification(BleEndpoint::CommandResult, bytes).frame);
    constexpr const char* names[]{"Accepted", "UnsupportedVersion", "UnknownMessageType", "MalformedLength",
        "StaleSequence", "InvalidValidity", "OutOfRange", "EmergencyStopLatched", "RobotFault"};
    for (std::uint8_t code = 1; code <= 8; ++code) {
        bytes = result; bytes[7] = code;
        EXPECT_TRUE(decodeBleNotification(BleEndpoint::CommandResult, bytes).frame);
        EXPECT_STREQ(bleResultName(code), names[code]);
        EXPECT_EQ(bleRejectionCode(code), code == 1 ? TransportErrorCode::UnsupportedVersion
            : code == 7 ? TransportErrorCode::EmergencyStopLatched
            : code == 8 ? TransportErrorCode::RobotFault
            : code >= 4 ? TransportErrorCode::InvalidCommand : TransportErrorCode::BackendFailure);
    }
}
TEST(BleProtocolTest, NamedWireDefinitionsKeepTheDocumentedV1Values)
{
    // Independent literals from robot-protocol.md and ble-gatt.md; the names must not drift.
    using namespace ble_wire;
    EXPECT_EQ(version, 1); EXPECT_EQ(versionOffset, 0U); EXPECT_EQ(typeOffset, 1U);
    EXPECT_EQ(sequenceOffset, 2U); EXPECT_EQ(headerLength, 4U); EXPECT_EQ(identifyingPrefixLength, 2U);
    EXPECT_EQ(telemetryLength, 18U); EXPECT_EQ(resultLength, 9U);
    EXPECT_EQ(telemetryStateOffset, 4U); EXPECT_EQ(flagsOffset, 5U); EXPECT_EQ(lastSequenceOffset, 6U);
    EXPECT_EQ(uptimeOffset, 8U); EXPECT_EQ(batteryOffset, 12U); EXPECT_EQ(linearOffset, 14U);
    EXPECT_EQ(angularOffset, 16U); EXPECT_EQ(resultSequenceOffset, 4U); EXPECT_EQ(resultTypeOffset, 6U);
    EXPECT_EQ(resultCodeOffset, 7U); EXPECT_EQ(resultStateOffset, 8U);
    EXPECT_EQ(unknownBattery, 0xffff); EXPECT_EQ(unknownMeasurement, 0x8000);
    EXPECT_EQ(minimumValidityMilliseconds, 1); EXPECT_EQ(sequenceHalfRange, 32768U);
    EXPECT_EQ(robotInfoFormat, 1); EXPECT_EQ(robotInfoLength, 18U); EXPECT_EQ(robotIdOffset, 3U);
    EXPECT_EQ(maximumLinearOffset, 10U); EXPECT_EQ(maximumAngularOffset, 12U);
    EXPECT_EQ(maximumValidityOffset, 14U); EXPECT_EQ(telemetryPeriodOffset, 16U);
    EXPECT_EQ(advertisedNamePrefix, "MR-");
    using namespace ble_gatt;
    EXPECT_EQ(robotControlService, 0x0001); EXPECT_EQ(robotInfoCharacteristic, 0x0002);
    EXPECT_EQ(commandCharacteristic, 0x0003); EXPECT_EQ(emergencyStopCharacteristic, 0x0004);
    EXPECT_EQ(telemetryCharacteristic, 0x0005); EXPECT_EQ(commandResultCharacteristic, 0x0006);
    EXPECT_EQ(characteristicCount, 5U); EXPECT_EQ(notificationCopyLimit, 21U);
}
TEST(BleProtocolTest, RejectsHeaderlessAndEveryOverlengthNotificationUpToTheBackendCopyBound)
{
    for (const auto endpoint : {BleEndpoint::Telemetry, BleEndpoint::CommandResult}) {
        const auto& valid = endpoint == BleEndpoint::Telemetry ? telemetry : result;
        // Without version and type the frame is malformed, not an unsupported version.
        const std::vector<std::uint8_t> versionOnly{2};
        EXPECT_EQ(decodeBleNotification(endpoint, versionOnly).code, TransportErrorCode::BackendFailure);
        EXPECT_EQ(decodeBleNotification(endpoint, std::vector<std::uint8_t>{2, valid[1]}).code,
                  TransportErrorCode::UnsupportedVersion);
        auto bytes = valid;
        while (bytes.size() < ble_gatt::notificationCopyLimit) {
            bytes.push_back(0);
            const auto decoded = decodeBleNotification(endpoint, bytes);
            EXPECT_FALSE(decoded.frame) << bytes.size();
            EXPECT_EQ(decoded.code, TransportErrorCode::BackendFailure) << bytes.size();
        }
    }
}
TEST(BleProtocolTest, AcceptsEveryDefinedRobotStateAndDecodesTheFullUptimeWidth)
{
    for (std::uint8_t state = 0; state <= static_cast<std::uint8_t>(BleRobotState::Fault); ++state) {
        auto t = telemetry; t[4] = state;
        auto r = result; r[8] = state;
        const auto decodedTelemetry = decodeBleNotification(BleEndpoint::Telemetry, t);
        const auto decodedResult = decodeBleNotification(BleEndpoint::CommandResult, r);
        ASSERT_TRUE(decodedTelemetry.frame);
        ASSERT_TRUE(decodedResult.frame);
        EXPECT_EQ(std::get<BleTelemetryFrame>(decodedTelemetry.frame->payload).state, BleRobotState{state});
        EXPECT_EQ(std::get<BleCommandResultFrame>(decodedResult.frame->payload).state, BleRobotState{state});
    }
    auto bytes = telemetry;
    bytes[8] = 0x78; bytes[9] = 0x56; bytes[10] = 0x34; bytes[11] = 0x12;
    EXPECT_EQ(std::get<BleTelemetryFrame>(decodeBleNotification(BleEndpoint::Telemetry, bytes).frame->payload)
                  .uptimeMilliseconds, 0x12345678U);
    bytes[8] = bytes[9] = bytes[10] = bytes[11] = 0xff;
    EXPECT_EQ(std::get<BleTelemetryFrame>(decodeBleNotification(BleEndpoint::Telemetry, bytes).frame->payload)
                  .uptimeMilliseconds, 0xffffffffU);
}
TEST(BleProtocolTest, SharedReceiveCounterHandlesGapsDuplicatesWrapAndHalfRange)
{
    BleReceiveSequence counter;
    ASSERT_TRUE(counter.accept(65534));
    EXPECT_FALSE(counter.accept(65534));
    EXPECT_FALSE(counter.accept(65533));
    const auto wrapped = counter.accept(1);
    ASSERT_TRUE(wrapped);
    EXPECT_EQ(wrapped->unwrapped, 65537U);
    EXPECT_EQ(wrapped->gap, 2);
    EXPECT_FALSE(counter.accept(32769)); // Exactly half-range is ambiguous.
    EXPECT_FALSE(counter.accept(65535));
    EXPECT_EQ(counter.accept(2)->unwrapped, 65538U);
    counter = {};
    EXPECT_EQ(counter.accept(0)->unwrapped, 0U);
}
} // namespace

TEST(BleProtocolSafetyTest, EmergencyClearUsesCommandCharacteristicWithResponse)
{
    const auto frame = micro_robot::encodeBleEmergencyClear(0x1234);
    EXPECT_EQ(frame.bytes, (std::vector<std::uint8_t>{1, 4, 0x34, 0x12}));
    EXPECT_EQ(frame.endpoint, micro_robot::BleEndpoint::Command);
    EXPECT_TRUE(frame.withResponse);
}
