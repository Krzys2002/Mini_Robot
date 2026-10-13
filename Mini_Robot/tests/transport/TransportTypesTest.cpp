#include "transport/TransportTypes.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <limits>

namespace micro_robot {
namespace {
using namespace std::chrono_literals;

TEST(TransportDomainTest, DefaultsUseSupportedVersionAndUnassignedSequences)
{
    const RobotCommand command;
    const RobotTelemetry telemetry{RobotId{7}, Timestamp{}, {}, {}, {}};
    EXPECT_TRUE(std::holds_alternative<StopCommand>(command.payload));
    EXPECT_EQ(command.validFor, Duration::zero());
    EXPECT_EQ(command.metadata.modelVersion, currentTransportModelVersion);
    EXPECT_EQ(telemetry.metadata.modelVersion, currentTransportModelVersion);
    EXPECT_FALSE(command.metadata.sequence);
    EXPECT_FALSE(telemetry.metadata.sequence);
    EXPECT_FALSE(telemetry.batteryVoltageVolts);
    EXPECT_FALSE(telemetry.linearMetersPerSecond);
    EXPECT_FALSE(telemetry.angularRadiansPerSecond);
}

TEST(TransportDomainTest, VersionSupportIsExplicitAndRejectsUnknownValues)
{
    static_assert(isSupportedTransportModelVersion(currentTransportModelVersion));
    EXPECT_TRUE(isSupportedTransportModelVersion(1));
    EXPECT_FALSE(isSupportedTransportModelVersion(0));
    EXPECT_FALSE(isSupportedTransportModelVersion(2));
    EXPECT_FALSE(isSupportedTransportModelVersion(std::numeric_limits<std::uint16_t>::max()));
}

TEST(TransportDomainTest, MetadataIsOwnedAndZeroIsAnAssignedSequence)
{
    TransportMessageMetadata metadata{currentTransportModelVersion, 0};
    RobotCommand command{VelocityCommand{0.2, -0.5}, Timestamp{1s}, 100ms, metadata};
    RobotTelemetry telemetry{RobotId{7}, Timestamp{1s}, 3.7, {}, {}, metadata};
    metadata.modelVersion = 2;
    metadata.sequence = std::numeric_limits<std::uint64_t>::max();
    ASSERT_TRUE(command.metadata.sequence);
    EXPECT_EQ(command.metadata.sequence, 0u);
    EXPECT_EQ(telemetry.metadata.sequence, 0u);
    EXPECT_EQ(command.metadata.modelVersion, currentTransportModelVersion);
    command.metadata.sequence = metadata.sequence;
    const auto copy = command;
    command.metadata.sequence.reset();
    EXPECT_EQ(copy.metadata.sequence, std::numeric_limits<std::uint64_t>::max());
    EXPECT_EQ(copy.issuedAt, Timestamp{1s});
    EXPECT_EQ(copy.validFor, 100ms);
}

} // namespace
} // namespace micro_robot
