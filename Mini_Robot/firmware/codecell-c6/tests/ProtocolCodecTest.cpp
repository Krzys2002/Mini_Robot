// Host tests of the #161 v1 codec: the shared #64 byte vectors byte for byte, every
// truncation and trailing extension, every version and type byte, signed extrema,
// sentinels, validity/range limits and the Robot Info record.
#include "ProtocolVectors.hpp"
#include "ProvisioningFakes.hpp"
#include "protocol/V1Codec.hpp"
#include <gtest/gtest.h>
#include <array>
#include <cstdint>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <vector>

using namespace codecell;
using namespace codecell::protocol::v1;
using codecell::protocol::fixtures::loadVectors;
using codecell::protocol::fixtures::Vector;
using codecell::protocol::fixtures::vectorNamed;
using codecell::protocol::fixtures::vectorsOfKind;
using runtime::MotionState;

namespace {
constexpr CommandLimits kLimits{150, 3000, 200};

bool isCommandKind(const std::string& kind)
{
    return kind == "stop" || kind == "velocity" || kind == "estop" || kind == "clear_estop";
}

MessageType typeOfKind(const std::string& kind)
{
    if (kind == "stop") return MessageType::Stop;
    if (kind == "velocity") return MessageType::Velocity;
    if (kind == "estop") return MessageType::EmergencyStop;
    return MessageType::ClearEmergencyStop;
}

MotionState stateNamed(const std::string& name)
{
    if (name == "idle") return MotionState::Idle;
    if (name == "moving") return MotionState::Moving;
    if (name == "estop") return MotionState::EmergencyStop;
    if (name != "fault") ADD_FAILURE() << "unknown state name " << name;
    return MotionState::Fault;
}

template <typename T>
std::optional<T> optionalField(const Vector& vector, const std::string& key)
{
    const auto value = vector.optionalInteger(key);
    if (!value) return std::nullopt;
    return static_cast<T>(*value);
}

std::vector<std::uint8_t> asVector(std::span<const std::uint8_t> bytes) { return {bytes.begin(), bytes.end()}; }

std::vector<Vector> commandVectors()
{
    std::vector<Vector> result;
    for (auto& vector : loadVectors())
        if (isCommandKind(vector.kind)) result.push_back(std::move(vector));
    return result;
}
} // namespace

TEST(ProtocolVectorsTest, SharedFixtureHoldsEveryPublishedExampleFrame)
{
    const auto vectors = loadVectors();
    std::vector<std::string> published;
    for (const auto& vector : vectors)
        if (vector.source == "published") published.push_back(vector.name);
    EXPECT_EQ(published, (std::vector<std::string>{"velocity_seq258", "stop_seq259", "estop_seq260", "telemetry_seq7",
                                                   "result_seq8", "robot_info_id7"}));
    for (const auto& vector : vectors) {
        SCOPED_TRACE(vector.name);
        EXPECT_TRUE(vector.source == "published" || vector.source == "derived");
        EXPECT_LE(vector.bytes.size(), kMaxAttPayload);
    }
}

TEST(ProtocolCodecTest, EveryCommandVectorDecodesToItsFields)
{
    const auto vectors = commandVectors();
    ASSERT_EQ(vectors.size(), 6U);
    for (const auto& vector : vectors) {
        SCOPED_TRACE(vector.name);
        const auto decoded = decodeCommand(vector.bytes);
        ASSERT_TRUE(decoded.frame);
        EXPECT_EQ(decoded.frame->type, typeOfKind(vector.kind));
        EXPECT_EQ(decoded.frame->sequence, vector.integer("seq"));
        if (vector.kind == "velocity") {
            EXPECT_EQ(decoded.frame->velocity.linearMillimetresPerSecond, vector.integer("linear_mm_s"));
            EXPECT_EQ(decoded.frame->velocity.angularMilliradiansPerSecond, vector.integer("angular_mrad_s"));
            EXPECT_EQ(decoded.frame->velocity.validForMilliseconds, vector.integer("valid_for_ms"));
        } else {
            EXPECT_EQ(decoded.frame->velocity, VelocityFields{});
        }
    }
}

TEST(ProtocolCodecTest, PublishedVelocityHasFixedUnitsInSi)
{
    const auto decoded = decodeCommand(vectorNamed("velocity_seq258").bytes);
    ASSERT_TRUE(decoded.frame);
    const auto velocity = toBodyVelocity(decoded.frame->velocity);
    EXPECT_DOUBLE_EQ(velocity.linearMetresPerSecond, 0.150);
    EXPECT_DOUBLE_EQ(velocity.angularRadiansPerSecond, -0.500);
    EXPECT_EQ(decoded.frame->velocity.validForMilliseconds, 250);
}

TEST(ProtocolCodecTest, EveryTelemetryVectorEncodesByteForByte)
{
    const auto vectors = vectorsOfKind("telemetry");
    ASSERT_EQ(vectors.size(), 3U);
    for (const auto& vector : vectors) {
        SCOPED_TRACE(vector.name);
        TelemetryValues values;
        values.state = stateNamed(vector.field("state"));
        values.commandExpired = vector.boolean("command_expired");
        values.batteryLow = vector.boolean("battery_low");
        values.lastAcceptedSequence = optionalField<std::uint16_t>(vector, "last_seq");
        values.uptimeMilliseconds = static_cast<std::uint32_t>(vector.integer("uptime_ms"));
        values.batteryMillivolts = optionalField<std::uint16_t>(vector, "battery_mv");
        values.measuredLinearMillimetresPerSecond = optionalField<std::int16_t>(vector, "linear_mm_s");
        values.measuredAngularMilliradiansPerSecond = optionalField<std::int16_t>(vector, "angular_mrad_s");
        const auto bytes = encodeTelemetry(static_cast<std::uint16_t>(vector.integer("seq")), values);
        ASSERT_TRUE(bytes);
        EXPECT_EQ(asVector(*bytes), vector.bytes);
    }
}

TEST(ProtocolCodecTest, EveryCommandResultVectorEncodesByteForByte)
{
    const auto vectors = vectorsOfKind("result");
    ASSERT_EQ(vectors.size(), 2U);
    for (const auto& vector : vectors) {
        SCOPED_TRACE(vector.name);
        const CommandResultValues values{static_cast<std::uint16_t>(vector.integer("cmd_seq")),
                                         static_cast<std::uint8_t>(vector.integer("cmd_type")),
                                         static_cast<ResultCode>(vector.integer("result")),
                                         stateNamed(vector.field("state"))};
        EXPECT_EQ(asVector(encodeCommandResult(static_cast<std::uint16_t>(vector.integer("seq")), values)),
                  vector.bytes);
    }
}

TEST(ProtocolCodecTest, EveryRobotInfoVectorEncodesByteForByte)
{
    const auto vectors = vectorsOfKind("robot_info");
    ASSERT_EQ(vectors.size(), 2U);
    for (const auto& vector : vectors) {
        SCOPED_TRACE(vector.name);
        EXPECT_EQ(vector.integer("info_format"), kRobotInfoFormat);
        EXPECT_EQ(vector.integer("min_version"), kMinProtocolVersion);
        EXPECT_EQ(vector.integer("max_version"), kMaxProtocolVersion);
        const provisioning::RobotInfoValues info{static_cast<std::uint32_t>(vector.integer("robot_id")),
                                                 static_cast<std::uint16_t>(vector.integer("max_linear_mm_s")),
                                                 static_cast<std::uint16_t>(vector.integer("max_angular_mrad_s")),
                                                 static_cast<std::uint16_t>(vector.integer("max_validity_ms")),
                                                 static_cast<std::uint16_t>(vector.integer("telemetry_period_ms"))};
        const auto& text = vector.field("firmware");
        const auto first = text.find('.');
        const auto second = text.find('.', first + 1);
        const FirmwareVersion firmware{static_cast<std::uint8_t>(std::stoi(text.substr(0, first))),
                                       static_cast<std::uint8_t>(std::stoi(text.substr(first + 1, second - first - 1))),
                                       static_cast<std::uint8_t>(std::stoi(text.substr(second + 1)))};
        const auto bytes = encodeRobotInfo(info, firmware);
        ASSERT_TRUE(bytes);
        EXPECT_EQ(asVector(*bytes), vector.bytes);
    }
}

TEST(ProtocolCodecTest, EveryTruncationOfEveryCommandFrameIsMalformedWithTheSpecifiedEcho)
{
    for (const auto& vector : commandVectors()) {
        for (std::size_t length = 0; length < vector.bytes.size(); ++length) {
            SCOPED_TRACE(vector.name + " length " + std::to_string(length));
            const auto decoded = decodeCommand(std::span{vector.bytes}.first(length));
            EXPECT_FALSE(decoded.frame);
            EXPECT_EQ(decoded.rejection, ResultCode::MalformedLength);
            EXPECT_EQ(decoded.echoType, length < kMinimumReadableLength ? kUnreadableType : vector.bytes[kTypeAt]);
            EXPECT_EQ(decoded.echoSequence,
                      length < kHeaderLength ? kUnreadableSequence : vector.integer("seq"));
        }
    }
}

TEST(ProtocolCodecTest, AnyTrailingByteMakesACommandMalformed)
{
    for (const auto& vector : commandVectors()) {
        for (std::size_t extra = 1; extra <= 2 * kMaxAttPayload; ++extra) {
            SCOPED_TRACE(vector.name + " +" + std::to_string(extra));
            auto bytes = vector.bytes;
            bytes.resize(bytes.size() + extra, 0x00);
            const auto decoded = decodeCommand(bytes);
            EXPECT_FALSE(decoded.frame);
            EXPECT_EQ(decoded.rejection, ResultCode::MalformedLength);
            EXPECT_EQ(decoded.echoType, vector.bytes[kTypeAt]);
            EXPECT_EQ(decoded.echoSequence, vector.integer("seq"));
        }
    }
}

TEST(ProtocolCodecTest, EveryVersionTypeAndLengthFollowsTheSpecifiedPrecedence)
{
    const std::array<std::size_t, 8> lengths{2, 3, 4, 5, 9, 10, 11, 18};
    for (int version = 0; version <= 0xFF; ++version) {
        for (int type = 0; type <= 0xFF; ++type) {
            for (const auto length : lengths) {
                std::vector<std::uint8_t> bytes(length, 0x5A);
                bytes[kVersionAt] = static_cast<std::uint8_t>(version);
                bytes[kTypeAt] = static_cast<std::uint8_t>(type);
                const auto decoded = decodeCommand(bytes);
                const bool command = type >= 0x01 && type <= 0x04;
                const std::size_t exact = type == 0x02 ? kVelocityLength : kHeaderLength;
                ResultCode expected = ResultCode::Accepted;
                if (version != kProtocolVersion) expected = ResultCode::UnsupportedVersion;
                else if (!command) expected = ResultCode::UnknownMessageType;
                else if (length != exact) expected = ResultCode::MalformedLength;
                if (expected == ResultCode::Accepted) {
                    ASSERT_TRUE(decoded.frame) << version << ' ' << type << ' ' << length;
                    EXPECT_EQ(static_cast<int>(decoded.frame->type), type);
                    continue;
                }
                ASSERT_FALSE(decoded.frame) << version << ' ' << type << ' ' << length;
                ASSERT_EQ(decoded.rejection, expected) << version << ' ' << type << ' ' << length;
                ASSERT_EQ(decoded.echoType, type);
                ASSERT_EQ(decoded.echoSequence, length >= kHeaderLength ? 0x5A5A : kUnreadableSequence);
            }
        }
    }
}

TEST(ProtocolCodecTest, RobotToDesktopAndReservedTypesAreNeverCommands)
{
    for (const auto& vector : loadVectors()) {
        if (vector.kind != "telemetry" && vector.kind != "result") continue;
        SCOPED_TRACE(vector.name);
        const auto decoded = decodeCommand(vector.bytes);  // A robot frame written back to Command.
        EXPECT_FALSE(decoded.frame);
        EXPECT_EQ(decoded.rejection, ResultCode::UnknownMessageType);
    }
    for (const std::uint8_t type : {std::uint8_t{0x00}, std::uint8_t{0x05}, std::uint8_t{0x7F}, std::uint8_t{0x82},
                                    std::uint8_t{0xFF}}) {
        const std::array<std::uint8_t, kHeaderLength> frame{kProtocolVersion, type, 0x01, 0x00};
        EXPECT_EQ(decodeCommand(frame).rejection, ResultCode::UnknownMessageType) << int{type};
    }
}

TEST(ProtocolCodecTest, JsonIsNeverParsedFromTheWire)
{
    // The v1 JSON mirror of the published Velocity (robot-protocol.md, "JSON bring-up").
    constexpr std::string_view kJson =
        R"({"v":1,"type":"velocity","seq":258,"linear_mm_s":150,"angular_mrad_s":-500,"valid_for_ms":250})";
    const std::vector<std::uint8_t> bytes(kJson.begin(), kJson.end());
    const auto decoded = decodeCommand(bytes);
    EXPECT_FALSE(decoded.frame);
    EXPECT_EQ(decoded.rejection, ResultCode::UnsupportedVersion);
    // Even a JSON object short enough to fit a v1 frame length is rejected on byte 0.
    const std::vector<std::uint8_t> shortJson{'{', '}', ' ', ' '};
    EXPECT_EQ(decodeCommand(shortJson).rejection, ResultCode::UnsupportedVersion);
}

TEST(ProtocolCodecTest, SignedExtremaAndTheUnknownSentinelDecodeExactly)
{
    const auto limits = vectorNamed("velocity_signed_limits");
    const auto extreme = decodeCommand(limits.bytes);
    ASSERT_TRUE(extreme.frame);
    EXPECT_EQ(extreme.frame->velocity.linearMillimetresPerSecond, std::numeric_limits<std::int16_t>::max());
    EXPECT_EQ(extreme.frame->velocity.angularMilliradiansPerSecond, -32767);
    EXPECT_EQ(extreme.frame->velocity.validForMilliseconds, std::numeric_limits<std::uint16_t>::max());
    EXPECT_DOUBLE_EQ(toBodyVelocity(extreme.frame->velocity).linearMetresPerSecond, 32.767);
    // At the largest advertised limits these are the largest acceptable values.
    constexpr CommandLimits kWidest{kMaxAdvertisedVelocity, kMaxAdvertisedVelocity, 0xFFFF};
    EXPECT_EQ(checkVelocityFields(extreme.frame->velocity, kWidest), ResultCode::Accepted);

    const auto sentinel = decodeCommand(vectorNamed("velocity_unknown_sentinel").bytes);
    ASSERT_TRUE(sentinel.frame);
    EXPECT_EQ(sentinel.frame->velocity.linearMillimetresPerSecond, kUnknownVelocity);
    // -32768 is out of range in commands for every representable maximum.
    EXPECT_EQ(checkVelocityFields(sentinel.frame->velocity, kWidest), ResultCode::OutOfRange);
    VelocityFields angular{0, kUnknownVelocity, 1};
    EXPECT_EQ(checkVelocityFields(angular, kWidest), ResultCode::OutOfRange);
}

TEST(ProtocolCodecTest, ValidityIsCheckedBeforeRangeAtTheAdvertisedBoundaries)
{
    EXPECT_EQ(checkVelocityFields({0, 0, 0}, kLimits), ResultCode::InvalidValidity);
    EXPECT_EQ(checkVelocityFields({0, 0, 1}, kLimits), ResultCode::Accepted);
    EXPECT_EQ(checkVelocityFields({0, 0, 200}, kLimits), ResultCode::Accepted);
    EXPECT_EQ(checkVelocityFields({0, 0, 201}, kLimits), ResultCode::InvalidValidity);
    EXPECT_EQ(checkVelocityFields({0, 0, 0xFFFF}, kLimits), ResultCode::InvalidValidity);
    for (const std::int16_t linear : {std::int16_t{150}, std::int16_t{-150}})
        EXPECT_EQ(checkVelocityFields({linear, 0, 1}, kLimits), ResultCode::Accepted);
    for (const std::int16_t linear : {std::int16_t{151}, std::int16_t{-151}})
        EXPECT_EQ(checkVelocityFields({linear, 0, 1}, kLimits), ResultCode::OutOfRange);
    for (const std::int16_t angular : {std::int16_t{3000}, std::int16_t{-3000}})
        EXPECT_EQ(checkVelocityFields({0, angular, 1}, kLimits), ResultCode::Accepted);
    for (const std::int16_t angular : {std::int16_t{3001}, std::int16_t{-3001}})
        EXPECT_EQ(checkVelocityFields({0, angular, 1}, kLimits), ResultCode::OutOfRange);
    // Both invalid: validity is reported, as the specified order requires.
    EXPECT_EQ(checkVelocityFields({151, 3001, 0}, kLimits), ResultCode::InvalidValidity);
}

// The authority (#160 CommandProcessor) compares SI values against the provisioned limit
// converted the same way. Its verdict must equal the integer rule on the advertised value.
TEST(ProtocolCodecTest, IntegerRangeRuleMatchesTheAuthoritySiComparison)
{
    const auto agree = [](std::int16_t raw, std::uint16_t maximum) {
        const double si = raw / kMilliUnitsPerUnit;
        const double limit = maximum / provisioning::kMillimetresPerMetre;
        return magnitudeWithin(raw, maximum) == (std::abs(si) <= limit);
    };
    for (std::uint32_t maximum = 1; maximum <= kMaxAdvertisedVelocity; ++maximum) {
        const auto m = static_cast<std::int32_t>(maximum);
        for (const std::int32_t raw : {m - 1, m, m + 1, -m - 1, -m, -m + 1}) {
            if (raw < std::numeric_limits<std::int16_t>::min() || raw > std::numeric_limits<std::int16_t>::max())
                continue;
            ASSERT_TRUE(agree(static_cast<std::int16_t>(raw), static_cast<std::uint16_t>(maximum)))
                << raw << " vs " << maximum;
        }
    }
    for (std::int32_t raw = std::numeric_limits<std::int16_t>::min(); raw <= std::numeric_limits<std::int16_t>::max();
         ++raw)
        for (const std::uint16_t maximum : {std::uint16_t{1}, std::uint16_t{150}, std::uint16_t{2007},
                                            std::uint16_t{3000}, kMaxAdvertisedVelocity})
            ASSERT_TRUE(agree(static_cast<std::int16_t>(raw), maximum)) << raw << " vs " << maximum;
}

TEST(ProtocolCodecTest, TelemetryNeverSendsAKnownValueThatLooksUnknown)
{
    TelemetryValues values;
    values.batteryMillivolts = kUnknownBattery;
    EXPECT_FALSE(encodeTelemetry(0, values));
    values = {};
    values.measuredLinearMillimetresPerSecond = kUnknownVelocity;
    EXPECT_FALSE(encodeTelemetry(0, values));
    values = {};
    values.measuredAngularMilliradiansPerSecond = kUnknownVelocity;
    EXPECT_FALSE(encodeTelemetry(0, values));
    values = {};  // Everything unknown, no accepted command: sentinels, flag clear, zero sequence field.
    const auto bytes = encodeTelemetry(0x1234, values);
    ASSERT_TRUE(bytes);
    EXPECT_EQ(asVector(*bytes), (std::vector<std::uint8_t>{0x01, 0x80, 0x34, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                                           0x00, 0x00, 0xFF, 0xFF, 0x00, 0x80, 0x00, 0x80}));
}

TEST(ProtocolCodecTest, TelemetryFlagsUseOnlyTheDefinedBits)
{
    TelemetryValues values;
    values.commandExpired = values.batteryLow = true;
    values.lastAcceptedSequence = 0;  // Sequence 0 is a valid accepted sequence.
    const auto bytes = encodeTelemetry(0, values);
    ASSERT_TRUE(bytes);
    EXPECT_EQ((*bytes)[kTelemetryFlagsAt], kFlagCommandExpired | kFlagBatteryLow | kFlagHasAcceptedCommand);
    EXPECT_EQ(readU16(*bytes, kTelemetryLastAcceptedAt), 0);
}

TEST(ProtocolCodecTest, MeasuredSiValuesRoundAwayFromZeroAndNeverClamp)
{
    EXPECT_EQ(measuredToWire(0.148), 148);
    EXPECT_EQ(measuredToWire(0.0005), 1);
    EXPECT_EQ(measuredToWire(-0.0005), -1);
    EXPECT_EQ(measuredToWire(0.0), 0);
    EXPECT_EQ(measuredToWire(32.767), 32767);
    EXPECT_EQ(measuredToWire(-32.767), -32767);
    EXPECT_EQ(measuredToWire(32.7674), 32767);
    EXPECT_FALSE(measuredToWire(32.7675));
    EXPECT_FALSE(measuredToWire(-32.768));  // Would collide with the unknown sentinel.
    EXPECT_FALSE(measuredToWire(std::numeric_limits<double>::quiet_NaN()));
    EXPECT_FALSE(measuredToWire(std::numeric_limits<double>::infinity()));
    EXPECT_FALSE(measuredToWire(-std::numeric_limits<double>::infinity()));
    EXPECT_FALSE(measuredToWire(std::numeric_limits<double>::max()));
}

TEST(ProtocolCodecTest, CommandResultReportsAnImpossibleStateAsFault)
{
    const auto bytes = encodeCommandResult(0, {0, 0, ResultCode::Accepted, static_cast<MotionState>(0x7F)});
    EXPECT_EQ(bytes[kResultStateAt], kStateFault);
}

TEST(ProtocolCodecTest, RobotInfoRefusesValuesTheRecordCannotRepresent)
{
    const provisioning::RobotInfoValues valid{7, 300, 6000, 500, 100};
    ASSERT_TRUE(encodeRobotInfo(valid, {}));
    auto bad = valid;
    bad.maxLinearMillimetresPerSecond = 0;
    EXPECT_FALSE(encodeRobotInfo(bad, {}));
    bad = valid;
    bad.maxLinearMillimetresPerSecond = kMaxAdvertisedVelocity + 1;
    EXPECT_FALSE(encodeRobotInfo(bad, {}));
    bad = valid;
    bad.maxAngularMilliradiansPerSecond = 0;
    EXPECT_FALSE(encodeRobotInfo(bad, {}));
    bad = valid;
    bad.maxAngularMilliradiansPerSecond = kMaxAdvertisedVelocity + 1;
    EXPECT_FALSE(encodeRobotInfo(bad, {}));
    bad = valid;
    bad.maxCommandValidityMilliseconds = 0;
    EXPECT_FALSE(encodeRobotInfo(bad, {}));
    bad = valid;
    bad.telemetryPeriodMilliseconds = 0;
    EXPECT_FALSE(encodeRobotInfo(bad, {}));
}

TEST(ProtocolCodecTest, RobotInfoAndCommandLimitsComeFromTheProvisionedRecord)
{
    const auto config = provisioning::ValidatedRobotConfig::validated(provisioning::fakes::validConfig(42));
    ASSERT_TRUE(config);
    const auto info = provisioning::robotInfoValues(*config);
    const auto bytes = encodeRobotInfo(info, {0, 1, 0});
    ASSERT_TRUE(bytes);
    EXPECT_EQ(asVector(*bytes), (std::vector<std::uint8_t>{0x01, 0x01, 0x01, 42, 0, 0, 0, 0x00, 0x01, 0x00, 150, 0,
                                                           0xB8, 0x0B, 200, 0, 100, 0}));
    const auto limits = commandLimits(info);
    EXPECT_EQ(limits.maxLinearMillimetresPerSecond, 150);
    EXPECT_EQ(limits.maxAngularMilliradiansPerSecond, 3000);
    EXPECT_EQ(limits.maxCommandValidityMilliseconds, 200);
}

namespace {
constexpr std::array<std::uint8_t, kVelocityLength> kPublishedVelocity{0x01, 0x02, 0x02, 0x01, 0x96,
                                                                       0x00, 0x0C, 0xFE, 0xFA, 0x00};
constexpr auto kDecodedAtCompileTime = decodeCommand(kPublishedVelocity);
// Decoding is a constant expression: no allocation, no I/O, a fixed amount of work.
static_assert(kDecodedAtCompileTime.frame.has_value());
static_assert(kDecodedAtCompileTime.frame->sequence == 258);
static_assert(kDecodedAtCompileTime.frame->velocity.angularMilliradiansPerSecond == -500);
static_assert(encodeCommandResult(8, {258, 2, ResultCode::Accepted, MotionState::Moving})
              == CommandResultBytes{0x01, 0x81, 0x08, 0x00, 0x02, 0x01, 0x02, 0x00, 0x01});
} // namespace
