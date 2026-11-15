// Host tests of the #161 per-session command validation. A recording authority checks
// the specified error precedence, one CommandResult per frame and that rejected frames
// never reach or change anything; the real #158/#160 runtime behind the session checks
// that the advertised limits are the enforced ones and that rejections keep motion state.
#include "ProtocolVectors.hpp"
#include "ProvisioningFakes.hpp"
#include "RuntimeFakes.hpp"
#include "protocol/V1CommandSession.hpp"
#include "provisioning/ProvisionedConfiguration.hpp"
#include "runtime/FirmwareRuntime.hpp"
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cstdint>
#include <optional>
#include <string_view>
#include <vector>

using namespace codecell;
using namespace codecell::protocol::v1;
using namespace std::chrono_literals;
using codecell::protocol::fixtures::vectorNamed;
using runtime::CommandOutcome;
using runtime::Instant;
using runtime::MotionState;

namespace {
constexpr CommandLimits kLimits{150, 3000, 200};
constexpr Instant kReceipt{5s};

struct Call {
    enum class Kind { Velocity, Stop, EmergencyStop, Clear, SessionEnded } kind;
    runtime::BodyVelocity velocity{};
    std::chrono::milliseconds validFor{};
    Instant receipt{};
};

// Records every authority call and answers with configurable outcomes.
class RecordingCommands final : public runtime::IMotionCommandSink {
public:
    std::vector<Call> calls;
    CommandOutcome velocityOutcome{CommandOutcome::Accepted};
    CommandOutcome clearOutcome{CommandOutcome::Accepted};

    CommandOutcome velocity(const runtime::BodyVelocity& v, std::chrono::milliseconds validFor,
                            Instant receipt) noexcept override
    {
        calls.push_back({Call::Kind::Velocity, v, validFor, receipt});
        return velocityOutcome;
    }
    CommandOutcome stop() noexcept override
    {
        calls.push_back({Call::Kind::Stop});
        return CommandOutcome::Accepted;
    }
    CommandOutcome emergencyStop() noexcept override
    {
        calls.push_back({Call::Kind::EmergencyStop});
        return CommandOutcome::Accepted;
    }
    CommandOutcome clearEmergencyStop() noexcept override
    {
        calls.push_back({Call::Kind::Clear});
        return clearOutcome;
    }
    void sessionEnded(runtime::SessionEndReason) noexcept override { calls.push_back({Call::Kind::SessionEnded}); }
};

class FixedStatus final : public IRuntimeStatusView {
public:
    runtime::RuntimeStatus value{MotionState::Idle, runtime::FaultCode::None, true};
    runtime::RuntimeStatus status() const noexcept override { return value; }
};

struct Result {
    std::uint16_t sequence;
    std::uint16_t commandSequence;
    std::uint8_t commandType;
    ResultCode code;
    std::uint8_t state;
};
Result parse(const CommandResultBytes& bytes)
{
    EXPECT_EQ(bytes[kVersionAt], kProtocolVersion);
    EXPECT_EQ(bytes[kTypeAt], static_cast<std::uint8_t>(MessageType::CommandResult));
    return {readU16(bytes, kSequenceAt), readU16(bytes, kResultCommandSequenceAt), bytes[kResultCommandTypeAt],
            static_cast<ResultCode>(bytes[kResultCodeAt]), bytes[kResultStateAt]};
}

std::vector<std::uint8_t> velocityFrame(std::uint16_t sequence, std::int16_t linear, std::int16_t angular,
                                        std::uint16_t validFor)
{
    std::vector<std::uint8_t> b{kProtocolVersion, static_cast<std::uint8_t>(MessageType::Velocity), 0, 0, 0, 0,
                                0, 0, 0, 0};
    const auto put = [&b](std::size_t at, std::uint16_t v) {
        b[at] = static_cast<std::uint8_t>(v & 0xFFU);
        b[at + 1] = static_cast<std::uint8_t>(v >> 8);
    };
    put(kSequenceAt, sequence);
    put(kVelocityLinearAt, static_cast<std::uint16_t>(linear));
    put(kVelocityAngularAt, static_cast<std::uint16_t>(angular));
    put(kVelocityValidForAt, validFor);
    return b;
}
std::vector<std::uint8_t> headerFrame(MessageType type, std::uint16_t sequence)
{
    return {kProtocolVersion, static_cast<std::uint8_t>(type), static_cast<std::uint8_t>(sequence & 0xFFU),
            static_cast<std::uint8_t>(sequence >> 8)};
}

struct FakeRig {
    RecordingCommands commands;
    FixedStatus runtime;
    CommandSession session{commands, runtime, kLimits};
    Result write(const std::vector<std::uint8_t>& bytes) { return parse(session.onCommandWrite(bytes, kReceipt)); }
};
} // namespace

TEST(ProtocolSessionTest, PublishedVelocityReachesTheAuthorityInSiWithReceiptTime)
{
    FakeRig rig;
    const auto velocity = vectorNamed("velocity_seq258").bytes;  // 0.150 m/s, -0.500 rad/s, 250 ms.
    CommandSession wide{rig.commands, rig.runtime, CommandLimits{300, 6000, 500}};
    const auto result = parse(wide.onCommandWrite(velocity, kReceipt));
    EXPECT_EQ(result.code, ResultCode::Accepted);
    EXPECT_EQ(result.commandSequence, 258);
    EXPECT_EQ(result.commandType, 0x02);
    ASSERT_EQ(rig.commands.calls.size(), 1U);
    const auto& call = rig.commands.calls[0];
    EXPECT_EQ(call.kind, Call::Kind::Velocity);
    EXPECT_DOUBLE_EQ(call.velocity.linearMetresPerSecond, 0.150);
    EXPECT_DOUBLE_EQ(call.velocity.angularRadiansPerSecond, -0.500);
    EXPECT_EQ(call.validFor, 250ms);
    EXPECT_EQ(call.receipt, kReceipt);
    EXPECT_EQ(wide.lastAcceptedSequence(), 258);
}

TEST(ProtocolSessionTest, EveryFrameGetsExactlyOneResultOnTheSharedRobotCounter)
{
    FakeRig rig;
    const std::vector<std::vector<std::uint8_t>> frames{
        {}, {0x01}, {0x02, 0x01, 0x00, 0x00}, headerFrame(MessageType::Stop, 1), velocityFrame(2, 0, 0, 0),
        headerFrame(MessageType::EmergencyStop, 3), {0x01, 0x80, 0, 0}};
    std::uint16_t expected = 0;
    for (const auto& frame : frames) EXPECT_EQ(rig.write(frame).sequence, expected++);
    const auto telemetry = rig.session.telemetry({});
    ASSERT_TRUE(telemetry);
    EXPECT_EQ(readU16(*telemetry, kSequenceAt), expected++);  // Telemetry shares the counter.
    EXPECT_EQ(parse(rig.session.onEmergencyStopWrite({})).sequence, expected++);
    TelemetryValues unrepresentable;
    unrepresentable.batteryMillivolts = kUnknownBattery;
    EXPECT_FALSE(rig.session.telemetry(unrepresentable));
    EXPECT_EQ(rig.session.nextTransmitSequence(), expected);  // Nothing sent, nothing used.
}

TEST(ProtocolSessionTest, HeaderRejectionsEchoWhatCouldBeRead)
{
    FakeRig rig;
    auto r = rig.write({});
    EXPECT_EQ(r.code, ResultCode::MalformedLength);
    EXPECT_EQ(r.commandType, kUnreadableType);
    EXPECT_EQ(r.commandSequence, kUnreadableSequence);
    r = rig.write({0x7B, 0x02, 0x34});  // Unsupported version, sequence not present.
    EXPECT_EQ(r.code, ResultCode::UnsupportedVersion);
    EXPECT_EQ(r.commandType, 0x02);
    EXPECT_EQ(r.commandSequence, kUnreadableSequence);
    r = rig.write({0x02, 0x02, 0x34, 0x12});  // A future version: echo type and sequence.
    EXPECT_EQ(r.code, ResultCode::UnsupportedVersion);
    EXPECT_EQ(r.commandSequence, 0x1234);
    r = rig.write({0x01, 0x81, 0x34, 0x12});  // Robot-to-desktop type on Command.
    EXPECT_EQ(r.code, ResultCode::UnknownMessageType);
    EXPECT_EQ(r.commandType, 0x81);
    r = rig.write({0x01, 0x02, 0x34, 0x12});  // Velocity header only.
    EXPECT_EQ(r.code, ResultCode::MalformedLength);
    EXPECT_EQ(r.commandType, 0x02);
    EXPECT_EQ(r.commandSequence, 0x1234);
    EXPECT_TRUE(rig.commands.calls.empty());
    EXPECT_FALSE(rig.session.lastAcceptedSequence());
}

// Step 5 order: sequence, then state, then validity, then range.
TEST(ProtocolSessionTest, VelocityRejectionsFollowSequenceStateValidityRangeOrder)
{
    struct Case {
        const char* name;
        MotionState state;
        bool ready;
        bool stale;
        std::uint16_t validFor;
        std::int16_t linear;
        ResultCode expected;
    };
    const std::array<Case, 12> cases{{
        {"stale beats fault", MotionState::Fault, false, true, 0, 151, ResultCode::StaleSequence},
        {"stale beats latch", MotionState::EmergencyStop, true, true, 0, 151, ResultCode::StaleSequence},
        {"fault beats validity", MotionState::Fault, false, false, 0, 151, ResultCode::RobotFault},
        {"latch beats validity", MotionState::EmergencyStop, true, false, 0, 151, ResultCode::EmergencyStopLatched},
        {"latch beats range", MotionState::EmergencyStop, true, false, 100, 151, ResultCode::EmergencyStopLatched},
        {"not ready beats validity", MotionState::Idle, false, false, 0, 0, ResultCode::RobotFault},
        {"validity beats range", MotionState::Idle, true, false, 0, 151, ResultCode::InvalidValidity},
        {"validity above ceiling", MotionState::Moving, true, false, 201, 0, ResultCode::InvalidValidity},
        {"range", MotionState::Idle, true, false, 200, 151, ResultCode::OutOfRange},
        {"negative range", MotionState::Moving, true, false, 1, -151, ResultCode::OutOfRange},
        {"sentinel is out of range", MotionState::Idle, true, false, 1, -32768, ResultCode::OutOfRange},
        {"valid", MotionState::Idle, true, false, 200, -150, ResultCode::Accepted},
    }};
    for (const auto& c : cases) {
        SCOPED_TRACE(c.name);
        FakeRig rig;
        ASSERT_EQ(rig.write(velocityFrame(10, 0, 0, 100)).code, ResultCode::Accepted);
        rig.commands.calls.clear();
        rig.runtime.value.state = c.state;
        rig.runtime.value.ready = c.ready;
        // The authority applies the same state rules for a frame that reaches it.
        rig.commands.velocityOutcome = c.state == MotionState::Fault ? CommandOutcome::RejectedFault
            : c.state == MotionState::EmergencyStop                  ? CommandOutcome::RejectedEmergencyStopLatched
            : !c.ready                                               ? CommandOutcome::RejectedNotReady
                                                                     : CommandOutcome::Accepted;
        const auto result = rig.write(velocityFrame(c.stale ? 10 : 11, c.linear, 0, c.validFor));
        EXPECT_EQ(result.code, c.expected) << toString(result.code);
        EXPECT_EQ(result.state, static_cast<std::uint8_t>(c.state));
        // Invalid frames never reach the authority; valid ones reach it exactly once.
        const bool frameValid = !c.stale && c.validFor >= 1 && c.validFor <= kLimits.maxCommandValidityMilliseconds
            && c.linear >= -150 && c.linear <= 150;
        EXPECT_EQ(rig.commands.calls.size(), frameValid ? 1U : 0U);
        EXPECT_EQ(rig.session.lastAcceptedSequence(), c.expected == ResultCode::Accepted ? 11 : 10);
    }
}

TEST(ProtocolSessionTest, AuthorityOutcomesMapToV1ResultCodes)
{
    EXPECT_EQ(resultCodeFor(CommandOutcome::Accepted), ResultCode::Accepted);
    EXPECT_EQ(resultCodeFor(CommandOutcome::RejectedEmergencyStopLatched), ResultCode::EmergencyStopLatched);
    EXPECT_EQ(resultCodeFor(CommandOutcome::RejectedFault), ResultCode::RobotFault);
    EXPECT_EQ(resultCodeFor(CommandOutcome::RejectedNotReady), ResultCode::RobotFault);
    EXPECT_EQ(resultCodeFor(CommandOutcome::RejectedInvalid), ResultCode::InvalidValidity);
    EXPECT_EQ(resultCodeFor(CommandOutcome::RejectedOutOfRange), ResultCode::OutOfRange);
    EXPECT_EQ(resultCodeFor(static_cast<CommandOutcome>(0x7F)), ResultCode::RobotFault);
}

TEST(ProtocolSessionTest, StaleStopIsExecutedAndAcceptedWithoutAdvancingLastAccepted)
{
    FakeRig rig;
    ASSERT_EQ(rig.write(velocityFrame(100, 50, 0, 100)).code, ResultCode::Accepted);
    for (const std::uint16_t stale : {std::uint16_t{100}, std::uint16_t{1}, std::uint16_t{100 + 0x8000}}) {
        rig.commands.calls.clear();
        const auto result = rig.write(headerFrame(MessageType::Stop, stale));
        EXPECT_EQ(result.code, ResultCode::Accepted);
        EXPECT_EQ(result.commandSequence, stale);
        ASSERT_EQ(rig.commands.calls.size(), 1U);
        EXPECT_EQ(rig.commands.calls[0].kind, Call::Kind::Stop);
        EXPECT_EQ(rig.session.lastAcceptedSequence(), 100);
    }
    EXPECT_EQ(rig.write(headerFrame(MessageType::Stop, 101)).code, ResultCode::Accepted);
    EXPECT_EQ(rig.session.lastAcceptedSequence(), 101);
    EXPECT_EQ(rig.write(velocityFrame(101, 0, 0, 100)).code, ResultCode::StaleSequence);
}

TEST(ProtocolSessionTest, EmergencyStopOnCommandIgnoresSequenceStateAndLastAccepted)
{
    FakeRig rig;
    ASSERT_EQ(rig.write(velocityFrame(100, 50, 0, 100)).code, ResultCode::Accepted);
    for (const auto state : {MotionState::Moving, MotionState::EmergencyStop, MotionState::Fault}) {
        rig.runtime.value.state = state;
        rig.commands.calls.clear();
        const auto result = rig.write(headerFrame(MessageType::EmergencyStop, 7));  // Older than 100.
        EXPECT_EQ(result.code, ResultCode::Accepted);
        EXPECT_EQ(result.commandSequence, 7);
        ASSERT_EQ(rig.commands.calls.size(), 1U);
        EXPECT_EQ(rig.commands.calls[0].kind, Call::Kind::EmergencyStop);
        EXPECT_EQ(rig.session.lastAcceptedSequence(), 100);
    }
}

TEST(ProtocolSessionTest, ClearIsSequenceCheckedBeforeTheAuthorityDecides)
{
    FakeRig rig;
    ASSERT_EQ(rig.write(headerFrame(MessageType::ClearEmergencyStop, 40)).code, ResultCode::Accepted);
    EXPECT_EQ(rig.session.lastAcceptedSequence(), 40);
    rig.commands.calls.clear();
    EXPECT_EQ(rig.write(headerFrame(MessageType::ClearEmergencyStop, 40)).code, ResultCode::StaleSequence);
    EXPECT_TRUE(rig.commands.calls.empty());
    rig.commands.clearOutcome = CommandOutcome::RejectedFault;
    rig.runtime.value.state = MotionState::Fault;
    EXPECT_EQ(rig.write(headerFrame(MessageType::ClearEmergencyStop, 41)).code, ResultCode::RobotFault);
    EXPECT_EQ(rig.session.lastAcceptedSequence(), 40);  // A rejected clear changes nothing.
}

TEST(ProtocolSessionTest, EmergencyStopCharacteristicLatchesOnAnyWriteBeforeParsing)
{
    const std::string_view json = R"({"v":1,"type":"estop","seq":3})";
    const std::vector<std::vector<std::uint8_t>> payloads{
        {}, {0x00}, {0x01, 0x03, 0x05}, {0x02, 0x03, 0x05, 0x00}, headerFrame(MessageType::Stop, 9),
        velocityFrame(9, 0, 0, 1), std::vector<std::uint8_t>(json.begin(), json.end()),
        std::vector<std::uint8_t>(64, 0xFF)};
    for (const auto& payload : payloads) {
        FakeRig rig;
        ASSERT_EQ(rig.write(velocityFrame(100, 50, 0, 100)).code, ResultCode::Accepted);
        rig.commands.calls.clear();
        const auto result = parse(rig.session.onEmergencyStopWrite(payload));
        ASSERT_EQ(rig.commands.calls.size(), 1U);
        EXPECT_EQ(rig.commands.calls[0].kind, Call::Kind::EmergencyStop);
        EXPECT_EQ(result.code, ResultCode::Accepted);
        EXPECT_EQ(result.commandType, 0x03);
        EXPECT_EQ(result.commandSequence, kUnreadableSequence);
        EXPECT_EQ(rig.session.lastAcceptedSequence(), 100);
    }
    FakeRig rig;
    const auto result = parse(rig.session.onEmergencyStopWrite(vectorNamed("estop_seq260").bytes));
    EXPECT_EQ(result.code, ResultCode::Accepted);
    EXPECT_EQ(result.commandSequence, 260);  // Well-formed: its own sequence is echoed.
    EXPECT_FALSE(rig.session.lastAcceptedSequence());
}

TEST(ProtocolSessionTest, WireSequenceNeverReachesTheAuthority)
{
    FakeRig a;
    FakeRig b;
    ASSERT_EQ(a.write(velocityFrame(1, 120, -2500, 150)).code, ResultCode::Accepted);
    ASSERT_EQ(b.write(velocityFrame(0xBEEF, 120, -2500, 150)).code, ResultCode::Accepted);
    ASSERT_EQ(a.commands.calls.size(), 1U);
    ASSERT_EQ(b.commands.calls.size(), 1U);
    EXPECT_EQ(a.commands.calls[0].velocity.linearMetresPerSecond, b.commands.calls[0].velocity.linearMetresPerSecond);
    EXPECT_EQ(a.commands.calls[0].velocity.angularRadiansPerSecond, b.commands.calls[0].velocity.angularRadiansPerSecond);
    EXPECT_EQ(a.commands.calls[0].validFor, b.commands.calls[0].validFor);
}

TEST(ProtocolSessionTest, TelemetryCarriesLastAcceptedAndResetStartsANewSession)
{
    FakeRig rig;
    auto telemetry = rig.session.telemetry({});
    ASSERT_TRUE(telemetry);
    EXPECT_EQ((*telemetry)[kTelemetryFlagsAt] & kFlagHasAcceptedCommand, 0);
    ASSERT_EQ(rig.write(velocityFrame(0xFFFF, 0, 0, 100)).code, ResultCode::Accepted);
    ASSERT_EQ(rig.write(velocityFrame(0x0000, 0, 0, 100)).code, ResultCode::Accepted);  // Wrapped.
    TelemetryValues values;
    values.lastAcceptedSequence = 0x4242;  // Ignored: the session owns lastAccepted.
    telemetry = rig.session.telemetry(values);
    ASSERT_TRUE(telemetry);
    EXPECT_NE((*telemetry)[kTelemetryFlagsAt] & kFlagHasAcceptedCommand, 0);
    EXPECT_EQ(readU16(*telemetry, kTelemetryLastAcceptedAt), 0x0000);

    rig.session.reset();
    EXPECT_FALSE(rig.session.lastAcceptedSequence());
    EXPECT_EQ(rig.session.nextTransmitSequence(), 0);
    const auto first = rig.write(velocityFrame(0x9000, 0, 0, 100));  // Older than before; new session.
    EXPECT_EQ(first.code, ResultCode::Accepted);
    EXPECT_EQ(first.sequence, 0);
    EXPECT_TRUE(std::none_of(rig.commands.calls.begin(), rig.commands.calls.end(),
                             [](const Call& c) { return c.kind == Call::Kind::SessionEnded; }));
}

// ---- The real runtime behind the session ----------------------------------------------

namespace {
using namespace codecell::runtime::fakes;

class RuntimeView final : public IRuntimeStatusView {
public:
    explicit RuntimeView(const runtime::FirmwareRuntime& runtime) : runtime_(runtime) {}
    runtime::RuntimeStatus status() const noexcept override { return runtime_.status(); }

private:
    const runtime::FirmwareRuntime& runtime_;
};

// The provisioned record drives both the runtime configuration and the advertised
// limits, exactly as ProvisionedConfiguration does in production.
struct RuntimeRig {
    FakeClock clock;
    FakePlatform platform;
    FakeDrive drive;
    FakeConfiguration configuration;
    std::optional<runtime::FirmwareRuntime> runtime;
    std::optional<RuntimeView> view;
    std::optional<CommandSession> session;
    provisioning::RobotInfoValues info{};

    RuntimeRig()
    {
        const auto config = provisioning::ValidatedRobotConfig::validated(provisioning::fakes::validConfig());
        EXPECT_TRUE(config);
        configuration.value = provisioning::runtimeConfigFor(*config);
        info = provisioning::robotInfoValues(*config);
        clock.time = 1s;
        drive.section = &platform.section;
        runtime.emplace(runtime::RuntimeDependencies{clock, platform, drive, configuration, nullptr});
        EXPECT_EQ(runtime->start(), runtime::StartResult::Running);
        view.emplace(*runtime);
        session.emplace(runtime->commands(), *view, commandLimits(info));
    }
    Result write(const std::vector<std::uint8_t>& bytes)
    {
        return parse(session->onCommandWrite(bytes, clock.nowMicroseconds()));
    }
    runtime::RuntimeStatus status() const { return runtime->status(); }
    void run(Instant duration)
    {
        const Instant end = clock.time + duration;
        while (clock.time < end) {
            clock.advance(Instant{timing::kSafetyCheckPeriod});
            runtime->safetyStep().step();
            if ((clock.time / Instant{timing::kSafetyCheckPeriod}) % 2 == 0) runtime->controlStep().step();
        }
    }
};
} // namespace

TEST(ProtocolRuntimeTest, AdvertisedLimitsAreExactlyTheEnforcedOnes)
{
    RuntimeRig rig;
    const auto maxLinear = static_cast<std::int16_t>(rig.info.maxLinearMillimetresPerSecond);
    const auto maxAngular = static_cast<std::int16_t>(rig.info.maxAngularMilliradiansPerSecond);
    const auto ceiling = rig.info.maxCommandValidityMilliseconds;
    std::uint16_t sequence = 0;
    EXPECT_EQ(rig.write(velocityFrame(sequence++, maxLinear, maxAngular, ceiling)).code, ResultCode::Accepted);
    EXPECT_EQ(rig.write(velocityFrame(sequence++, static_cast<std::int16_t>(-maxLinear),
                                      static_cast<std::int16_t>(-maxAngular), 1))
                  .code,
              ResultCode::Accepted);
    EXPECT_EQ(rig.write(velocityFrame(sequence++, static_cast<std::int16_t>(maxLinear + 1), 0, 1)).code,
              ResultCode::OutOfRange);
    EXPECT_EQ(rig.write(velocityFrame(sequence++, 0, static_cast<std::int16_t>(maxAngular + 1), 1)).code,
              ResultCode::OutOfRange);
    EXPECT_EQ(rig.write(velocityFrame(sequence++, 0, 0, static_cast<std::uint16_t>(ceiling + 1))).code,
              ResultCode::InvalidValidity);
    EXPECT_EQ(rig.write(velocityFrame(sequence++, 0, 0, 0)).code, ResultCode::InvalidValidity);
}

TEST(ProtocolRuntimeTest, AcceptedVelocityMovesAndStopEndsItThroughTheAuthority)
{
    RuntimeRig rig;
    const auto moving = rig.write(velocityFrame(1, 100, 0, 150));
    EXPECT_EQ(moving.code, ResultCode::Accepted);
    EXPECT_EQ(moving.state, kStateMoving);  // State after processing.
    rig.run(20ms);
    ASSERT_TRUE(rig.drive.lastCommand());
    EXPECT_DOUBLE_EQ(rig.drive.lastCommand()->linearMetresPerSecond, 0.100);
    const auto stopped = rig.write(headerFrame(MessageType::Stop, 2));
    EXPECT_EQ(stopped.code, ResultCode::Accepted);
    EXPECT_EQ(stopped.state, kStateIdle);
}

TEST(ProtocolRuntimeTest, RejectedFramesKeepMotionStateDeadlineAndLastAccepted)
{
    RuntimeRig rig;
    ASSERT_EQ(rig.write(velocityFrame(10, 100, 0, 100)).code, ResultCode::Accepted);
    rig.run(20ms);
    const auto before = rig.status();
    const auto commandsBefore = rig.drive.commandCount();
    const std::vector<std::vector<std::uint8_t>> rejected{
        {}, {0x01}, {0x7B, 0x22}, velocityFrame(10, 50, 0, 100) /* equal sequence */,
        velocityFrame(9, 50, 0, 100) /* older */, velocityFrame(11, 151, 0, 100), velocityFrame(11, 0, 0, 0),
        velocityFrame(11, 0, 0, 0xFFFF), headerFrame(MessageType::ClearEmergencyStop, 10),
        {0x01, 0x02, 0x0B, 0x00}, {0x01, 0x80, 0x0B, 0x00}};
    for (const auto& frame : rejected) {
        EXPECT_NE(rig.write(frame).code, ResultCode::Accepted);
        EXPECT_EQ(rig.status().state, before.state);
        EXPECT_EQ(rig.status().outputEnabled, before.outputEnabled);
        EXPECT_EQ(rig.session->lastAcceptedSequence(), 10);
    }
    EXPECT_DOUBLE_EQ(rig.drive.lastCommand()->linearMetresPerSecond, 0.100);  // The old setpoint is untouched...
    rig.run(70ms);
    EXPECT_EQ(rig.status().state, MotionState::Moving);
    rig.run(20ms);  // ...and still expires on its own deadline: no rejection extended it.
    EXPECT_EQ(rig.status().state, MotionState::Idle);
    EXPECT_GT(rig.drive.commandCount(), commandsBefore);
}

TEST(ProtocolRuntimeTest, LatchedRobotRejectsVelocityAndClearRequiresANewCommand)
{
    RuntimeRig rig;
    ASSERT_EQ(rig.write(velocityFrame(1, 100, 0, 150)).code, ResultCode::Accepted);
    const std::vector<std::uint8_t> garbage{0xDE, 0xAD};
    const auto latched = parse(rig.session->onEmergencyStopWrite(garbage));
    EXPECT_EQ(latched.state, kStateEmergencyStop);
    EXPECT_FALSE(rig.status().outputEnabled);
    EXPECT_EQ(rig.write(velocityFrame(2, 100, 0, 150)).code, ResultCode::EmergencyStopLatched);
    EXPECT_EQ(rig.write(velocityFrame(3, 151, 0, 0)).code, ResultCode::EmergencyStopLatched);  // State first.
    EXPECT_EQ(rig.write(headerFrame(MessageType::Stop, 4)).code, ResultCode::Accepted);        // No effect.
    EXPECT_EQ(rig.status().state, MotionState::EmergencyStop);
    const auto cleared = rig.write(headerFrame(MessageType::ClearEmergencyStop, 5));
    EXPECT_EQ(cleared.code, ResultCode::Accepted);
    EXPECT_EQ(cleared.state, kStateIdle);
    rig.run(50ms);
    EXPECT_FALSE(rig.status().outputEnabled);  // Motion does not resume by itself.
}

TEST(ProtocolRuntimeTest, FaultedRobotReportsRobotFaultAndNeverMoves)
{
    RuntimeRig rig;
    rig.drive.failCommand = true;
    ASSERT_EQ(rig.write(velocityFrame(1, 100, 0, 150)).code, ResultCode::Accepted);
    rig.run(20ms);
    ASSERT_EQ(rig.status().state, MotionState::Fault);
    EXPECT_EQ(rig.write(velocityFrame(2, 100, 0, 150)).code, ResultCode::RobotFault);
    EXPECT_EQ(rig.write(velocityFrame(3, 0, 0, 0)).code, ResultCode::RobotFault);  // State before validity.
    EXPECT_EQ(rig.write(headerFrame(MessageType::ClearEmergencyStop, 4)).code, ResultCode::RobotFault);
    EXPECT_EQ(rig.write(headerFrame(MessageType::Stop, 5)).code, ResultCode::Accepted);
    EXPECT_EQ(rig.status().state, MotionState::Fault);
}

TEST(ProtocolRuntimeTest, JsonWritesNeverCommandMotion)
{
    RuntimeRig rig;
    constexpr std::string_view kJson =
        R"({"v":1,"type":"velocity","seq":258,"linear_mm_s":150,"angular_mrad_s":-500,"valid_for_ms":250})";
    const auto result = rig.write(std::vector<std::uint8_t>(kJson.begin(), kJson.end()));
    EXPECT_EQ(result.code, ResultCode::UnsupportedVersion);
    rig.run(20ms);
    EXPECT_EQ(rig.status().state, MotionState::Idle);
    EXPECT_EQ(rig.drive.commandCount(), 0);
}
