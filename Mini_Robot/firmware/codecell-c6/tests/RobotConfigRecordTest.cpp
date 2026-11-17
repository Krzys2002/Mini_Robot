// Host tests of the #159 record schema: ranges, identity round trips, the stored layout,
// corruption/truncation/version handling and the read-only Robot Info/advertising view.
#include "ProvisioningFakes.hpp"
#include "provisioning/RobotConfigRecord.hpp"
#include "provisioning/RobotIdentity.hpp"
#include <gtest/gtest.h>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

using namespace codecell::provisioning;
using codecell::provisioning::fakes::validConfig;
namespace timing = codecell::timing;

namespace {
std::vector<std::uint8_t> bytesOf(const RecordBytes& record) { return {record.begin(), record.end()}; }

void resealChecksum(std::vector<std::uint8_t>& bytes)
{
    const auto at = bytes.size() - kChecksumBytes;
    const auto crc = crc32(std::span<const std::uint8_t>{bytes}.first(at));
    for (std::size_t i = 0; i < kChecksumBytes; ++i) bytes[at + i] = static_cast<std::uint8_t>(crc >> (8 * i));
}
} // namespace

TEST(RobotConfigRecordTest, Crc32MatchesTheStandardCheckValue)
{
    constexpr std::string_view kCheck = "123456789";
    std::vector<std::uint8_t> bytes(kCheck.begin(), kCheck.end());
    EXPECT_EQ(crc32(bytes), 0xCBF43926U);
}

TEST(RobotConfigRecordTest, DefaultConfigurationIsInvalidSoNothingArmsByDefault)
{
    EXPECT_NE(validate(RobotConfig{}), RecordError::None);
    EXPECT_FALSE(ValidatedRobotConfig::validated(RobotConfig{}));
    EXPECT_EQ(validate(validConfig()), RecordError::None);
}

TEST(RobotConfigRecordTest, ZeroAndExtremeRobotIdsRoundTrip)
{
    for (const RobotId id : {RobotId{0}, RobotId{1}, RobotId{0x80000000U}, std::numeric_limits<RobotId>::max()}) {
        const auto decoded = decode(encode(validConfig(id), 3));
        ASSERT_EQ(decoded.error, RecordError::None) << id;
        EXPECT_EQ(decoded.config, validConfig(id));
        EXPECT_EQ(decoded.generation, 3U);
        ASSERT_TRUE(ValidatedRobotConfig::validated(decoded.config));
        EXPECT_EQ(ValidatedRobotConfig::validated(decoded.config)->robotId(), id);
    }
}

TEST(RobotConfigRecordTest, EncodedLayoutIsLittleEndianWithFrozenHeader)
{
    auto c = validConfig(0x04030201U);
    const auto bytes = encode(c, 0xA1B2C3D4U);
    ASSERT_EQ(bytes.size(), 42U);
    EXPECT_EQ(std::string(bytes.begin(), bytes.begin() + 4), "MRCF");
    EXPECT_EQ(bytes[4], 1);  // schemaVersion
    EXPECT_EQ(bytes[5], 0);
    EXPECT_EQ(bytes[6], 42);  // recordLength
    EXPECT_EQ(bytes[8], 0xD4);
    EXPECT_EQ(bytes[11], 0xA1);
    EXPECT_EQ(bytes[12], 0x01);
    EXPECT_EQ(bytes[15], 0x04);
    EXPECT_EQ(bytes[16], 1);   // board profile
    EXPECT_EQ(bytes[36], 100); // telemetry period low byte
}

TEST(RobotConfigRecordTest, EveryRangeRuleIsEnforced)
{
    struct Case {
        const char* name;
        void (*mutate)(RobotConfig&);
        RecordError expected;
    };
    const Case cases[] = {
        {"board 0", [](RobotConfig& c) { c.boardProfile = 0; }, RecordError::UnsupportedBoard},
        {"board 2", [](RobotConfig& c) { c.boardProfile = 2; }, RecordError::UnsupportedBoard},
        {"reserved capability", [](RobotConfig& c) { c.requiredCapabilities |= 0x80; }, RecordError::ReservedCapabilityBits},
        {"no drive", [](RobotConfig& c) { c.requiredCapabilities = 0; }, RecordError::DriveCapabilityMissing},
        {"imu unsupported",
         [](RobotConfig& c) { c.requiredCapabilities = capability::kDifferentialDrive | capability::kInertialSensor; },
         RecordError::CapabilityNotInFirmware},
        {"reserved polarity", [](RobotConfig& c) { c.drivePolarity = 0x04; }, RecordError::ReservedPolarityBits},
        {"output 0", [](RobotConfig& c) { c.maxDriveOutputPercent = 0; }, RecordError::DriveOutputOutOfRange},
        {"output 101", [](RobotConfig& c) { c.maxDriveOutputPercent = 101; }, RecordError::DriveOutputOutOfRange},
        {"wheel 0", [](RobotConfig& c) { c.wheelDiameterMicrometres = 0; }, RecordError::WheelDiameterZero},
        {"track 0", [](RobotConfig& c) { c.trackWidthMicrometres = 0; }, RecordError::TrackWidthZero},
        {"wheel speed 0", [](RobotConfig& c) { c.maxWheelSpeedMillimetresPerSecond = 0; }, RecordError::WheelSpeedZero},
        {"linear 0", [](RobotConfig& c) { c.maxLinearMillimetresPerSecond = 0; }, RecordError::LinearLimitOutOfRange},
        {"linear 32768",
         [](RobotConfig& c) {
             c.maxWheelSpeedMillimetresPerSecond = 0xFFFF;
             c.maxLinearMillimetresPerSecond = 32768;
         },
         RecordError::LinearLimitOutOfRange},
        {"angular 0", [](RobotConfig& c) { c.maxAngularMilliradiansPerSecond = 0; }, RecordError::AngularLimitOutOfRange},
        {"angular 32768", [](RobotConfig& c) { c.maxAngularMilliradiansPerSecond = 32768; },
         RecordError::AngularLimitOutOfRange},
        {"linear above wheel", [](RobotConfig& c) { c.maxLinearMillimetresPerSecond = 201; },
         RecordError::LinearLimitAboveWheelSpeed},
        // 200 mm/s wheels on a 60 mm track allow 2 * 0.2 / 0.06 = 6.666 rad/s on the spot.
        {"angular above wheel", [](RobotConfig& c) { c.maxAngularMilliradiansPerSecond = 6'667; },
         RecordError::AngularLimitAboveWheelSpeed},
        {"validity 0", [](RobotConfig& c) { c.maxCommandValidityMilliseconds = 0; }, RecordError::ValidityOutOfRange},
        {"validity above ceiling",
         [](RobotConfig& c) {
             c.maxCommandValidityMilliseconds = static_cast<std::uint16_t>(timing::kCommandValidityCeiling.count() + 1);
         },
         RecordError::ValidityOutOfRange},
        {"telemetry 0", [](RobotConfig& c) { c.telemetryPeriodMilliseconds = 0; }, RecordError::TelemetryPeriodMismatch},
        {"telemetry 99", [](RobotConfig& c) { c.telemetryPeriodMilliseconds = 99; }, RecordError::TelemetryPeriodMismatch},
    };
    for (const auto& test : cases) {
        auto c = validConfig();
        test.mutate(c);
        EXPECT_EQ(validate(c), test.expected) << test.name;
        // The same rule applies to a stored record with an intact checksum.
        const auto decoded = decode(encode(c, 1));
        EXPECT_EQ(decoded.error, test.expected) << test.name;
        EXPECT_EQ(decoded.generation, 1U) << test.name;
    }
}

TEST(RobotConfigRecordTest, BoundaryValuesAreAccepted)
{
    auto c = validConfig();
    c.maxDriveOutputPercent = kDriveOutputPercentFullScale;
    c.maxLinearMillimetresPerSecond = c.maxWheelSpeedMillimetresPerSecond;
    c.maxAngularMilliradiansPerSecond = 6'666;
    c.maxCommandValidityMilliseconds = static_cast<std::uint16_t>(timing::kCommandValidityCeiling.count());
    EXPECT_EQ(validate(c), RecordError::None);
    c.maxCommandValidityMilliseconds = 1;
    EXPECT_EQ(validate(c), RecordError::None);
    c.maxWheelSpeedMillimetresPerSecond = 0xFFFF;
    c.maxLinearMillimetresPerSecond = kMaxAdvertisedVelocityRaw;
    c.maxAngularMilliradiansPerSecond = kMaxAdvertisedVelocityRaw;
    c.trackWidthMicrometres = 1;
    EXPECT_EQ(validate(c), RecordError::None);
}

TEST(RobotConfigRecordTest, EverySingleBitFlipIsDetected)
{
    const auto good = bytesOf(encode(validConfig(), 9));
    for (std::size_t i = 0; i < good.size(); ++i) {
        for (int bit = 0; bit < 8; ++bit) {
            auto bad = good;
            bad[i] = static_cast<std::uint8_t>(bad[i] ^ (1U << bit));
            EXPECT_NE(decode(bad).error, RecordError::None) << "byte " << i << " bit " << bit;
        }
    }
}

TEST(RobotConfigRecordTest, EveryTruncationIsRejectedWithoutAGeneration)
{
    const auto good = bytesOf(encode(validConfig(), 9));
    for (std::size_t length = 0; length < good.size(); ++length) {
        const std::vector<std::uint8_t> partial(good.begin(), good.begin() + static_cast<std::ptrdiff_t>(length));
        const auto decoded = decode(partial);
        EXPECT_NE(decoded.error, RecordError::None) << length;
        EXPECT_FALSE(decoded.generation) << length;
    }
}

TEST(RobotConfigRecordTest, OtherSchemaVersionsAreIntactButNeverReinterpreted)
{
    for (const std::uint8_t version : {std::uint8_t{0}, std::uint8_t{2}, std::uint8_t{0xFF}}) {
        auto bytes = bytesOf(encode(validConfig(), 11));
        bytes[4] = version;
        resealChecksum(bytes);
        const auto decoded = decode(bytes);
        EXPECT_EQ(decoded.error, RecordError::UnsupportedVersion);
        EXPECT_EQ(decoded.generation, 11U);  // Still orders against the other slot.
    }
    // A longer future record with its own length and CRC is intact but unsupported.
    auto longer = bytesOf(encode(validConfig(), 12));
    longer.insert(longer.end() - kChecksumBytes, {0xAA, 0xBB});
    longer[4] = 2;
    longer[6] = static_cast<std::uint8_t>(longer.size());
    resealChecksum(longer);
    EXPECT_EQ(decode(longer).error, RecordError::UnsupportedVersion);
    EXPECT_EQ(decode(longer).generation, 12U);
}

TEST(RobotConfigRecordTest, HeaderDamageIsClassified)
{
    auto bytes = bytesOf(encode(validConfig(), 1));
    bytes[0] = 'X';
    EXPECT_EQ(decode(bytes).error, RecordError::BadMagic);

    bytes = bytesOf(encode(validConfig(), 1));
    bytes[6] = 40;
    EXPECT_EQ(decode(bytes).error, RecordError::LengthMismatch);

    // Version 1 with a resealed wrong length is still not a version 1 record.
    bytes = bytesOf(encode(validConfig(), 1));
    bytes.insert(bytes.end() - kChecksumBytes, std::uint8_t{0});
    bytes[6] = static_cast<std::uint8_t>(bytes.size());
    resealChecksum(bytes);
    EXPECT_EQ(decode(bytes).error, RecordError::LengthMismatch);
}

TEST(RobotConfigRecordTest, ValidatedViewUsesSiUnits)
{
    const auto view = ValidatedRobotConfig::validated(validConfig());
    ASSERT_TRUE(view);
    EXPECT_DOUBLE_EQ(view->wheelDiameterMetres(), 0.017);
    EXPECT_DOUBLE_EQ(view->trackWidthMetres(), 0.06);
    EXPECT_DOUBLE_EQ(view->maxWheelSpeedMetresPerSecond(), 0.2);
    EXPECT_DOUBLE_EQ(view->maxLinearMetresPerSecond(), 0.15);
    EXPECT_DOUBLE_EQ(view->maxAngularRadiansPerSecond(), 3.0);
    EXPECT_EQ(view->maxCommandValidity(), std::chrono::milliseconds{200});
    EXPECT_EQ(view->telemetryPeriod(), timing::kTelemetryPeriod);
    EXPECT_FALSE(view->leftWheelReversed());
    EXPECT_TRUE(view->rightWheelReversed());
    EXPECT_EQ(view->maxDriveOutputPercent(), 40);
}

TEST(RobotIdentityTest, RobotInfoAndAdvertisedNameComeFromTheSameRecord)
{
    for (const RobotId id : {RobotId{0}, RobotId{7}, std::numeric_limits<RobotId>::max()}) {
        const auto view = ValidatedRobotConfig::validated(validConfig(id));
        ASSERT_TRUE(view);
        const auto info = robotInfoValues(*view);
        EXPECT_EQ(info.robotId, id);
        EXPECT_EQ(info.maxLinearMillimetresPerSecond, 150);
        EXPECT_EQ(info.maxAngularMilliradiansPerSecond, 3'000);
        EXPECT_EQ(info.maxCommandValidityMilliseconds, 200);
        EXPECT_EQ(info.telemetryPeriodMilliseconds, timing::kTelemetryPeriod.count());
        // Nonzero and representable in the v1 Robot Info record.
        EXPECT_GE(info.maxLinearMillimetresPerSecond, 1);
        EXPECT_LE(info.maxLinearMillimetresPerSecond, kMaxAdvertisedVelocityRaw);
        EXPECT_GE(info.maxAngularMilliradiansPerSecond, 1);
        EXPECT_LE(info.maxAngularMilliradiansPerSecond, kMaxAdvertisedVelocityRaw);
        EXPECT_GE(info.maxCommandValidityMilliseconds, 1);
        EXPECT_GE(info.telemetryPeriodMilliseconds, 1);
        EXPECT_EQ(std::string{advertisedName(id).data()}, "MR-" + std::to_string(id));
    }
    EXPECT_EQ(std::string{advertisedName(0).data()}, "MR-0");
    EXPECT_EQ(std::string{advertisedName(std::numeric_limits<RobotId>::max()).data()}, "MR-4294967295");
}
