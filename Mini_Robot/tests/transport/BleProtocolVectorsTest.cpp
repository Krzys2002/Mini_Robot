// Shared v1 byte vectors (#161) against the desktop codec (#66). The firmware codec tests
// read the same file (firmware/codecell-c6/tests/ProtocolCodecTest.cpp),
// so both endpoints are held to one immutable set of bytes.
#include "transport/ble/BleProtocol.hpp"
#include "transport/ble/BleTypes.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#ifndef MICRO_ROBOT_PROTOCOL_V1_VECTORS
#error "MICRO_ROBOT_PROTOCOL_V1_VECTORS must name docs/protocol/fixtures/v1-frames.txt"
#endif

namespace {
using namespace micro_robot;
using namespace std::chrono_literals;

struct Vector {
    std::string name, kind, source;
    std::map<std::string, std::string> fields;
    std::vector<std::uint8_t> bytes;
    long long integer(const std::string& key) const { return std::stoll(fields.at(key)); }
    bool isNull(const std::string& key) const { return fields.at(key) == "null"; }
    bool boolean(const std::string& key) const { return fields.at(key) == "true"; }
};

std::vector<Vector> loadVectors()
{
    std::ifstream file(MICRO_ROBOT_PROTOCOL_V1_VECTORS);
    if (!file) throw std::runtime_error("cannot open " MICRO_ROBOT_PROTOCOL_V1_VECTORS);
    std::vector<Vector> vectors;
    for (std::string line; std::getline(file, line);) {
        if (line.empty() || line.front() == '#') continue;
        const auto bar = line.find('|');
        if (bar == std::string::npos) throw std::runtime_error("vector without bytes: " + line);
        Vector v;
        std::istringstream head(line.substr(0, bar));
        head >> v.name >> v.kind >> v.source;
        for (std::string token; head >> token;) {
            const auto equals = token.find('=');
            v.fields[token.substr(0, equals)] = token.substr(equals + 1);
        }
        std::istringstream hex(line.substr(bar + 1));
        for (std::string byte; hex >> byte;) v.bytes.push_back(static_cast<std::uint8_t>(std::stoul(byte, nullptr, 16)));
        vectors.push_back(std::move(v));
    }
    return vectors;
}

const Vector& named(const std::vector<Vector>& vectors, const std::string& name)
{
    for (const auto& v : vectors)
        if (v.name == name) return v;
    throw std::runtime_error("no vector named " + name);
}

BleRobotState stateNamed(const std::string& name)
{
    if (name == "idle") return BleRobotState::Idle;
    if (name == "moving") return BleRobotState::Moving;
    if (name == "estop") return BleRobotState::EmergencyStop;
    return BleRobotState::Fault;
}

std::optional<double> scaled(const Vector& v, const std::string& key)
{
    if (v.isNull(key)) return std::nullopt;
    return static_cast<double>(v.integer(key)) / 1000.0;
}
} // namespace

TEST(BleProtocolVectorsTest, DesktopEncodesEveryCommandVectorWithinThePublishedLimits)
{
    const auto vectors = loadVectors();
    const auto info = validateBleRobotInfo(named(vectors, "robot_info_id7").bytes, RobotId{7});
    ASSERT_TRUE(info.info) << info.error;
    const Timestamp now{10s};
    int encoded = 0;
    for (const auto& v : vectors) {
        SCOPED_TRACE(v.name);
        if (v.kind != "estop" && v.kind != "clear_estop" && v.kind != "stop" && v.kind != "velocity") continue;
        const auto sequence = static_cast<std::uint16_t>(v.integer("seq"));
        if (v.kind == "estop") {
            EXPECT_EQ(encodeBleEmergencyStop(sequence).bytes, v.bytes);
        } else if (v.kind == "clear_estop") {
            EXPECT_EQ(encodeBleEmergencyClear(sequence).bytes, v.bytes);
        } else if (v.kind == "stop") {
            const auto frame = encodeBleCommand({StopCommand{}, now, 250ms}, *info.info, sequence, now);
            ASSERT_TRUE(frame.frame) << frame.error;
            EXPECT_EQ(frame.frame->bytes, v.bytes);
        } else if (v.kind == "velocity") {
            const auto linear = v.integer("linear_mm_s");
            const auto angular = v.integer("angular_mrad_s");
            const RobotCommand command{VelocityCommand{linear / 1000.0, angular / 1000.0}, now,
                                       std::chrono::milliseconds{v.integer("valid_for_ms")}};
            const auto frame = encodeBleCommand(command, *info.info, sequence, now);
            const bool within = std::llabs(linear) <= info.info->maximumLinearMillimetersPerSecond
                && std::llabs(angular) <= info.info->maximumAngularMilliradiansPerSecond;
            if (!within) {
                EXPECT_FALSE(frame.frame);  // Never clamped into a different frame.
                continue;
            }
            ASSERT_TRUE(frame.frame) << frame.error;
            EXPECT_EQ(frame.frame->bytes, v.bytes);
        }
        ++encoded;
    }
    EXPECT_EQ(encoded, 4);  // Published Velocity, Stop and EmergencyStop, plus the derived Clear.
}

TEST(BleProtocolVectorsTest, DesktopDecodesEveryRobotVectorToItsFields)
{
    int decoded = 0;
    for (const auto& v : loadVectors()) {
        SCOPED_TRACE(v.name);
        if (v.kind == "telemetry") {
            const auto result = decodeBleNotification(BleEndpoint::Telemetry, v.bytes);
            ASSERT_TRUE(result.frame) << result.error;
            EXPECT_EQ(result.frame->sequence, v.integer("seq"));
            const auto& t = std::get<BleTelemetryFrame>(result.frame->payload);
            EXPECT_EQ(t.state, stateNamed(v.fields.at("state")));
            EXPECT_EQ(t.commandExpired, v.boolean("command_expired"));
            EXPECT_EQ(t.batteryLow, v.boolean("battery_low"));
            if (v.isNull("last_seq")) {
                EXPECT_FALSE(t.lastAcceptedSequence);
            } else {
                EXPECT_EQ(t.lastAcceptedSequence, v.integer("last_seq"));
            }
            EXPECT_EQ(t.uptimeMilliseconds, v.integer("uptime_ms"));
            EXPECT_EQ(t.batteryVolts, scaled(v, "battery_mv"));
            EXPECT_EQ(t.linearMetersPerSecond, scaled(v, "linear_mm_s"));
            EXPECT_EQ(t.angularRadiansPerSecond, scaled(v, "angular_mrad_s"));
        } else if (v.kind == "result") {
            const auto result = decodeBleNotification(BleEndpoint::CommandResult, v.bytes);
            ASSERT_TRUE(result.frame) << result.error;
            EXPECT_EQ(result.frame->sequence, v.integer("seq"));
            const auto& r = std::get<BleCommandResultFrame>(result.frame->payload);
            EXPECT_EQ(r.commandSequence, v.integer("cmd_seq"));
            EXPECT_EQ(r.commandType, v.integer("cmd_type"));
            EXPECT_EQ(r.result, v.integer("result"));
            EXPECT_EQ(r.state, stateNamed(v.fields.at("state")));
        } else if (v.kind == "robot_info") {
            const auto id = static_cast<std::uint32_t>(v.integer("robot_id"));
            const auto result = validateBleRobotInfo(v.bytes, RobotId{id});
            ASSERT_TRUE(result.info) << result.error;
            EXPECT_EQ(result.info->robotId, RobotId{id});
            EXPECT_EQ(result.info->maximumLinearMillimetersPerSecond, v.integer("max_linear_mm_s"));
            EXPECT_EQ(result.info->maximumAngularMilliradiansPerSecond, v.integer("max_angular_mrad_s"));
            EXPECT_EQ(result.info->maximumValidityMilliseconds, v.integer("max_validity_ms"));
            EXPECT_EQ(result.info->telemetryPeriodMilliseconds, v.integer("telemetry_period_ms"));
        } else {
            continue;
        }
        ++decoded;
    }
    EXPECT_EQ(decoded, 7);
}
