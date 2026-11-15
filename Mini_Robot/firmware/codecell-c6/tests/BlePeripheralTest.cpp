// Host tests of the #163 BLE session lifecycle. A fake radio link records every
// notification, termination, advertising and link-parameter request; the real #158-#162
// runtime sits behind the controller, so motion, the latch and session tokens are checked at
// the drive fake rather than at a recording authority. Callback orderings that the NimBLE
// host produces (CCCD-off events before the disconnect, events for a stale handle, a second
// central) are injected directly.
#include "AllocationCounter.hpp"
#include "ProtocolVectors.hpp"
#include "ProvisioningFakes.hpp"
#include "RuntimeFakes.hpp"
#include "ble/PeripheralController.hpp"
#include "protocol/V1CommandSession.hpp"
#include "provisioning/ProvisionedConfiguration.hpp"
#include "runtime/FirmwareRuntime.hpp"
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

using namespace codecell;
using namespace codecell::ble;
using namespace codecell::protocol::v1;
using namespace codecell::runtime::fakes;
using namespace std::chrono_literals;
using codecell::protocol::fixtures::vectorNamed;
using runtime::Counter;
using runtime::Instant;
using runtime::MotionState;

namespace {
constexpr ConnectionHandle kOwner = 1;
constexpr ConnectionHandle kSecond = 2;
constexpr int kRemoteUserTerminated = 0x13;

// Fixed-size records, so that recording allocates nothing once the vectors are reserved.
struct Notification {
    ConnectionHandle to{};
    std::array<std::uint8_t, kMaxAttPayload> bytes{};
    std::size_t length{0};
};

class FakeLink final : public IPeripheralLink {
public:
    std::vector<Notification> notifications;
    std::vector<ConnectionHandle> terminated;
    int notifyAttempts{0};
    int advertisingStarts{0};
    int advertisingStops{0};
    int linkRequests{0};
    std::optional<ConnectionHandle> lastLinkRequestTo{};
    LinkParameterRequest lastLinkRequest{};
    bool advertising{false};
    std::uint16_t stackMtu{0};  // What the stack reports for a connection (0: unknown).
    bool failNotify{false};
    bool failAdvertising{false};
    bool failTerminate{false};
    bool refuseLinkRequest{false};
    std::function<void()> whileStartingAdvertising;
    std::function<void()> whileTerminating;

    bool notifyCommandResult(ConnectionHandle to, std::span<const std::uint8_t> frame) noexcept override
    {
        ++notifyAttempts;
        if (failNotify || frame.size() > kMaxAttPayload) return false;
        Notification n{to, {}, frame.size()};
        std::copy(frame.begin(), frame.end(), n.bytes.begin());
        notifications.push_back(n);
        return true;
    }
    bool terminate(ConnectionHandle connection) noexcept override
    {
        if (whileTerminating) whileTerminating();
        terminated.push_back(connection);
        return !failTerminate;
    }
    bool startAdvertising() noexcept override
    {
        if (whileStartingAdvertising) whileStartingAdvertising();
        ++advertisingStarts;
        advertising = !failAdvertising;
        return !failAdvertising;
    }
    bool stopAdvertising() noexcept override
    {
        ++advertisingStops;
        advertising = false;
        return true;
    }
    std::uint16_t attMtu(ConnectionHandle) const noexcept override { return stackMtu; }
    bool requestLinkParameters(ConnectionHandle connection, const LinkParameterRequest& request) noexcept override
    {
        ++linkRequests;
        lastLinkRequestTo = connection;
        lastLinkRequest = request;
        return !refuseLinkRequest;
    }
};

struct Result {
    ConnectionHandle to;
    std::uint16_t sequence;
    std::uint16_t commandSequence;
    std::uint8_t commandType;
    ResultCode code;
    std::uint8_t state;
};
Result parse(const Notification& n)
{
    EXPECT_EQ(n.length, kCommandResultLength);
    EXPECT_EQ(n.bytes[kVersionAt], kProtocolVersion);
    EXPECT_EQ(n.bytes[kTypeAt], static_cast<std::uint8_t>(MessageType::CommandResult));
    const std::span<const std::uint8_t> b{n.bytes.data(), n.length};
    return {n.to, readU16(b, kSequenceAt), readU16(b, kResultCommandSequenceAt), n.bytes[kResultCommandTypeAt],
            static_cast<ResultCode>(n.bytes[kResultCodeAt]), n.bytes[kResultStateAt]};
}

std::vector<std::uint8_t> velocityFrame(std::uint16_t sequence, std::int16_t linear, std::uint16_t validFor)
{
    std::vector<std::uint8_t> b(kVelocityLength, 0);
    b[kVersionAt] = kProtocolVersion;
    b[kTypeAt] = static_cast<std::uint8_t>(MessageType::Velocity);
    const auto put = [&b](std::size_t at, std::uint16_t v) {
        b[at] = static_cast<std::uint8_t>(v & 0xFFU);
        b[at + 1] = static_cast<std::uint8_t>(v >> 8);
    };
    put(kSequenceAt, sequence);
    put(kVelocityLinearAt, static_cast<std::uint16_t>(linear));
    put(kVelocityValidForAt, validFor);
    return b;
}
std::vector<std::uint8_t> headerFrame(MessageType type, std::uint16_t sequence)
{
    return {kProtocolVersion, static_cast<std::uint8_t>(type), static_cast<std::uint8_t>(sequence & 0xFFU),
            static_cast<std::uint8_t>(sequence >> 8)};
}

class RuntimeView final : public IRuntimeStatusView {
public:
    explicit RuntimeView(const runtime::FirmwareRuntime& runtime) : runtime_(runtime) {}
    runtime::RuntimeStatus status() const noexcept override { return runtime_.status(); }

private:
    const runtime::FirmwareRuntime& runtime_;
};

// The provisioned record (fixture values, not CodeCell calibration) gives both the runtime
// configuration and the advertised limits, as in production: 150 mm/s, 3000 mrad/s, 200 ms.
constexpr std::int16_t kForward = 100;
constexpr std::uint16_t kValidity = 200;

struct Rig {
    FakeClock clock;
    FakePlatform platform;
    FakeDrive drive;
    FakeConfiguration configuration;
    FakeLink link;
    std::optional<runtime::FirmwareRuntime> runtime;
    std::optional<RuntimeView> view;
    std::optional<PeripheralController> ble;

    explicit Rig(bool startAdvertising = true)
    {
        const auto config = provisioning::ValidatedRobotConfig::validated(provisioning::fakes::validConfig());
        EXPECT_TRUE(config);
        configuration.value = provisioning::runtimeConfigFor(*config);
        clock.time = 1s;
        drive.section = &platform.section;
        runtime.emplace(runtime::RuntimeDependencies{clock, platform, drive, configuration, nullptr});
        EXPECT_EQ(runtime->start(), runtime::StartResult::Running);
        view.emplace(*runtime);
        ble.emplace(runtime->commands(), *view, commandLimits(provisioning::robotInfoValues(*config)), link, clock);
        if (startAdvertising) ble->start();
    }
    void subscribe(ConnectionHandle h, Characteristic c, bool on = true) { ble->onSubscriptionChanged(h, c, on); }
    void makeReady(ConnectionHandle h = kOwner)
    {
        ble->onConnected(h);
        subscribe(h, Characteristic::CommandResult);
        subscribe(h, Characteristic::Telemetry);
    }
    void command(ConnectionHandle h, const std::vector<std::uint8_t>& bytes) { ble->onCommandWrite(h, bytes); }
    void estop(ConnectionHandle h, const std::vector<std::uint8_t>& bytes) { ble->onEmergencyStopWrite(h, bytes); }
    [[nodiscard]] Result last() const { return parse(link.notifications.back()); }
    [[nodiscard]] runtime::RuntimeStatus status() const { return runtime->status(); }
    [[nodiscard]] std::uint32_t runtimeCount(Counter c) const { return runtime->diagnostics()[c]; }
    [[nodiscard]] std::uint32_t bleCount(BleCounter c) const { return ble->diagnostics()[c]; }
    void run(Instant duration)
    {
        const Instant end = clock.time + duration;
        while (clock.time < end) {
            clock.advance(Instant{timing::kSafetyCheckPeriod});
            runtime->safetyStep().step();
            if ((clock.time / Instant{timing::kSafetyCheckPeriod}) % 2 == 0) runtime->controlStep().step();
        }
    }
    // A ready session with an accepted Velocity that has reached the drive.
    void makeMoving(std::uint16_t sequence = 0)
    {
        makeReady();
        command(kOwner, velocityFrame(sequence, kForward, kValidity));
        EXPECT_EQ(last().code, ResultCode::Accepted);
        run(30ms);
        EXPECT_EQ(status().state, MotionState::Moving);
        EXPECT_TRUE(drive.enabled);
    }
};
} // namespace

// ---- Advertising and connection ownership ---------------------------------------------

TEST(BlePeripheralTest, StartAdvertisesOnceAndOwnerConnectionStopsIt)
{
    Rig rig;
    EXPECT_EQ(rig.ble->state(), LinkState::Advertising);
    rig.ble->start();
    EXPECT_EQ(rig.link.advertisingStarts, 1);
    rig.ble->onConnected(kOwner);
    EXPECT_EQ(rig.ble->state(), LinkState::Connected);
    EXPECT_EQ(rig.link.advertisingStops, 1);
    EXPECT_FALSE(rig.link.advertising);
    rig.ble->onConnected(kOwner);  // Repeated event: nothing changes, nothing is terminated.
    EXPECT_TRUE(rig.link.terminated.empty());
    EXPECT_EQ(rig.bleCount(BleCounter::Connections), 1U);
}

TEST(BlePeripheralTest, AdvertisingFailureIsReportedAndLeavesOutputsOff)
{
    Rig rig;
    rig.link.failAdvertising = true;
    rig.makeReady();
    rig.ble->onDisconnected(kOwner, kRemoteUserTerminated);
    EXPECT_EQ(rig.ble->state(), LinkState::AdvertisingFailed);
    EXPECT_EQ(rig.bleCount(BleCounter::AdvertisingFailures), 1U);
    EXPECT_FALSE(rig.status().sessionReady);
    EXPECT_EQ(rig.drive.commandCount(), 0);
}

TEST(BlePeripheralTest, SecondCentralIsTerminatedAndCannotTouchTheActiveSession)
{
    Rig rig;
    rig.makeMoving();
    const auto resultsBefore = rig.link.notifications.size();
    rig.ble->onConnected(kSecond);
    EXPECT_EQ(rig.link.terminated, std::vector<ConnectionHandle>{kSecond});
    // Nothing the newcomer does reaches the session: no subscription, no Stop, no Clear.
    rig.subscribe(kSecond, Characteristic::CommandResult);
    rig.subscribe(kSecond, Characteristic::Telemetry);
    rig.command(kSecond, headerFrame(MessageType::Stop, 9));
    rig.command(kSecond, velocityFrame(9, -kForward, kValidity));
    rig.ble->onLinkParametersUpdated(kSecond, {80, 4, 600});
    rig.ble->onMtuChanged(kSecond, 247);
    EXPECT_EQ(rig.link.notifications.size(), resultsBefore);
    EXPECT_EQ(rig.status().state, MotionState::Moving);
    EXPECT_EQ(rig.ble->state(), LinkState::Ready);
    EXPECT_FALSE(rig.ble->status().negotiated);
    EXPECT_EQ(rig.ble->status().mtu, kDefaultAttMtu);
    // Its disconnect neither ends the owner's session nor restarts advertising.
    rig.ble->onDisconnected(kSecond, kRemoteUserTerminated);
    EXPECT_TRUE(rig.status().sessionReady);
    EXPECT_EQ(rig.link.advertisingStarts, 1);
    EXPECT_FALSE(rig.ble->status().lastDisconnectReason);
    // The owner keeps its session and its unbroken transmit counter.
    rig.command(kOwner, velocityFrame(1, kForward, kValidity));
    EXPECT_EQ(rig.last().to, kOwner);
    EXPECT_EQ(rig.last().sequence, 1);
    EXPECT_EQ(rig.bleCount(BleCounter::RejectedCentrals), 1U);
    EXPECT_EQ(rig.bleCount(BleCounter::ForeignEvents), 4U);
}

TEST(BlePeripheralTest, EmergencyStopFromANonOwnerStillLatchesWithoutAnAnswer)
{
    Rig rig;
    rig.makeMoving();
    rig.ble->onConnected(kSecond);
    rig.estop(kSecond, {});
    EXPECT_EQ(rig.status().state, MotionState::EmergencyStop);
    EXPECT_FALSE(rig.drive.enabled);
    rig.command(kOwner, headerFrame(MessageType::Stop, 1));
    EXPECT_EQ(rig.last().sequence, 1) << "no result was sent for the foreign write, so no gap";
    for (const auto& n : rig.link.notifications) EXPECT_EQ(n.to, kOwner);
}

// ---- Readiness ------------------------------------------------------------------------

TEST(BlePeripheralTest, ReadinessNeedsBothCccdsInEitherOrder)
{
    for (const bool resultFirst : {true, false}) {
        Rig rig;
        rig.ble->onConnected(kOwner);
        const auto first = resultFirst ? Characteristic::CommandResult : Characteristic::Telemetry;
        const auto second = resultFirst ? Characteristic::Telemetry : Characteristic::CommandResult;
        rig.subscribe(kOwner, first);
        EXPECT_EQ(rig.ble->state(), LinkState::Connected);
        EXPECT_FALSE(rig.status().sessionReady);
        rig.subscribe(kOwner, second);
        EXPECT_EQ(rig.ble->state(), LinkState::Ready);
        EXPECT_TRUE(rig.status().sessionReady);
        EXPECT_TRUE(rig.ble->status().commandResultNotify && rig.ble->status().telemetryNotify);
        // Robot Info and Command are not CCCDs: they never change readiness.
        rig.subscribe(kOwner, Characteristic::RobotInfo, false);
        EXPECT_EQ(rig.ble->state(), LinkState::Ready);
    }
}

TEST(BlePeripheralTest, RepeatedLifecycleRequestsAreIdempotent)
{
    Rig rig;
    rig.makeReady();
    rig.command(kOwner, headerFrame(MessageType::Stop, 0));
    EXPECT_EQ(rig.last().sequence, 0);
    rig.subscribe(kOwner, Characteristic::Telemetry);
    rig.subscribe(kOwner, Characteristic::CommandResult);
    rig.ble->onConnected(kOwner);
    rig.ble->start();
    rig.command(kOwner, headerFrame(MessageType::Stop, 1));
    EXPECT_EQ(rig.last().sequence, 1) << "no new session: the counter kept running";
    EXPECT_EQ(rig.runtimeCount(Counter::SessionOpened), 1U);
    rig.ble->onDisconnected(kOwner, kRemoteUserTerminated);
    rig.ble->onDisconnected(kOwner, kRemoteUserTerminated);
    EXPECT_EQ(rig.runtimeCount(Counter::SessionEnded), 1U);
    EXPECT_EQ(rig.link.advertisingStarts, 2);
    EXPECT_EQ(rig.bleCount(BleCounter::Disconnects), 1U);
}

TEST(BlePeripheralTest, BeforeReadinessVelocityIsIgnoredWhileStopAndEmergencyStopAct)
{
    for (const auto partial : {Characteristic::CommandResult, Characteristic::Telemetry}) {
        Rig rig;
        rig.ble->onConnected(kOwner);
        rig.subscribe(kOwner, partial);
        rig.command(kOwner, velocityFrame(0, kForward, kValidity));
        rig.run(50ms);
        EXPECT_EQ(rig.drive.commandCount(), 0);
        EXPECT_EQ(rig.status().state, MotionState::Idle);
        rig.command(kOwner, headerFrame(MessageType::EmergencyStop, 1));
        EXPECT_EQ(rig.status().state, MotionState::EmergencyStop);
        rig.command(kOwner, headerFrame(MessageType::ClearEmergencyStop, 2));
        EXPECT_EQ(rig.status().state, MotionState::EmergencyStop) << "clear needs a ready session (#162)";
        rig.estop(kOwner, {0x7B});
        rig.command(kOwner, headerFrame(MessageType::Stop, 3));
        EXPECT_EQ(rig.status().state, MotionState::EmergencyStop);
        EXPECT_EQ(rig.link.notifyAttempts, 0) << "no notification channel before readiness";
        EXPECT_EQ(rig.bleCount(BleCounter::UnansweredCommandWrites), 4U);
        EXPECT_EQ(rig.bleCount(BleCounter::EmergencyStopWrites), 1U);
    }
}

// ---- Command results ------------------------------------------------------------------

TEST(BlePeripheralTest, EveryFrameWhileReadyGetsExactlyOneResultOnTheSharedCounter)
{
    Rig rig;
    rig.makeReady();
    const std::vector<std::vector<std::uint8_t>> commandWrites{
        headerFrame(MessageType::Stop, 10),
        velocityFrame(11, kForward, kValidity),
        {},
        {0x01},
        {0x02, 0x01, 0x00, 0x00},                                         // unsupported version
        headerFrame(MessageType::Telemetry, 12),                          // robot-to-desktop type
        {'{', '"', 'v', '"', ':', '1', '}'},                              // JSON
        std::vector<std::uint8_t>(kMaxAttributeValueLength, 0x01),       // longest ATT value
        headerFrame(MessageType::EmergencyStop, 13),                      // redundant E-stop on Command
        velocityFrame(14, kForward, kValidity),                           // latched
    };
    std::size_t writes = 0;
    for (const auto& bytes : commandWrites) {
        rig.command(kOwner, bytes);
        ++writes;
        rig.estop(kOwner, {});
        ++writes;
    }
    ASSERT_EQ(rig.link.notifications.size(), writes);
    for (std::size_t i = 0; i < writes; ++i) {
        const auto r = parse(rig.link.notifications[i]);
        EXPECT_EQ(r.to, kOwner);
        EXPECT_EQ(static_cast<std::size_t>(r.sequence), i) << "one counter for every result, starting at 0";
        EXPECT_LE(rig.link.notifications[i].length + 3, static_cast<std::size_t>(kDefaultAttMtu));
    }
    EXPECT_EQ(rig.bleCount(BleCounter::ResultsSent), writes);
}

TEST(BlePeripheralTest, SharedV1VectorsAreAnsweredOverTheGattPath)
{
    Rig rig;
    rig.makeReady();
    // 250 ms exceeds this robot's advertised 200 ms ceiling: rejected, no motion.
    rig.command(kOwner, vectorNamed("velocity_seq258").bytes);
    auto r = rig.last();
    EXPECT_EQ(r.commandSequence, 258);
    EXPECT_EQ(r.commandType, static_cast<std::uint8_t>(MessageType::Velocity));
    EXPECT_EQ(r.code, ResultCode::InvalidValidity);
    rig.run(30ms);
    EXPECT_EQ(rig.drive.commandCount(), 0);
    rig.command(kOwner, vectorNamed("stop_seq259").bytes);
    r = rig.last();
    EXPECT_EQ(r.commandSequence, 259);
    EXPECT_EQ(r.commandType, static_cast<std::uint8_t>(MessageType::Stop));
    EXPECT_EQ(r.code, ResultCode::Accepted);
    // The same Velocity is now older than the accepted Stop.
    rig.command(kOwner, vectorNamed("velocity_seq258").bytes);
    EXPECT_EQ(rig.last().code, ResultCode::StaleSequence);
    rig.estop(kOwner, vectorNamed("estop_seq260").bytes);
    r = rig.last();
    EXPECT_EQ(r.commandSequence, 260);
    EXPECT_EQ(r.commandType, static_cast<std::uint8_t>(MessageType::EmergencyStop));
    EXPECT_EQ(r.code, ResultCode::Accepted);
    EXPECT_EQ(r.state, static_cast<std::uint8_t>(MotionState::EmergencyStop));
}

TEST(BlePeripheralTest, EmergencyStopLatchesAnyPayloadInsideTheCallBehindACommandBacklog)
{
    const std::vector<std::vector<std::uint8_t>> payloads{
        {},
        {0x01},
        {0x02, 0x03, 0x04, 0x01},
        {'{', '}'},
        std::vector<std::uint8_t>(kMaxAttributeValueLength, 0xFF),
        headerFrame(MessageType::EmergencyStop, 77),
    };
    for (std::size_t p = 0; p < payloads.size(); ++p) {
        Rig rig;
        rig.makeMoving();
        // A backlog of the 24 writes the host can hold ahead of the E-stop (#156 B8).
        for (std::uint16_t s = 1; s <= timing::kInboundAclBuffers; ++s)
            rig.command(kOwner, velocityFrame(s, kForward, kValidity));
        rig.estop(kOwner, payloads[p]);
        // Latched and disabled on return, with no supervisor or control step in between.
        EXPECT_EQ(rig.status().state, MotionState::EmergencyStop) << p;
        EXPECT_FALSE(rig.drive.enabled) << p;
        const auto r = rig.last();
        EXPECT_EQ(r.commandType, static_cast<std::uint8_t>(MessageType::EmergencyStop)) << p;
        EXPECT_EQ(r.code, ResultCode::Accepted) << p;
        EXPECT_EQ(r.commandSequence, p + 1 == payloads.size() ? 77 : 0) << p;
        const auto commands = rig.drive.commandCount();
        rig.run(100ms);
        EXPECT_EQ(rig.drive.commandCount(), commands) << "nothing queued resumes after the latch";
    }
}

TEST(BlePeripheralTest, ResultQueueExhaustionStopsTheSessionThenTerminatesTheLink)
{
    Rig rig;
    rig.makeMoving();
    std::optional<runtime::RuntimeStatus> atTermination;
    rig.link.whileTerminating = [&rig, &atTermination] { atTermination = rig.status(); };
    rig.link.failNotify = true;
    rig.command(kOwner, velocityFrame(1, kForward, kValidity));
    ASSERT_TRUE(atTermination);
    EXPECT_FALSE(atTermination->sessionReady) << "stop as for Stop before the link is terminated";
    EXPECT_NE(atTermination->state, MotionState::Moving);
    EXPECT_EQ(rig.link.terminated, std::vector<ConnectionHandle>{kOwner});
    EXPECT_EQ(rig.ble->state(), LinkState::Closing);
    EXPECT_EQ(rig.runtimeCount(Counter::SessionOverload), 1U);
    // Nothing is buffered or retried, a re-subscription cannot revive it, E-stop still latches.
    const int attempts = rig.link.notifyAttempts;
    rig.link.failNotify = false;
    rig.command(kOwner, velocityFrame(2, kForward, kValidity));
    rig.subscribe(kOwner, Characteristic::Telemetry, false);
    rig.subscribe(kOwner, Characteristic::Telemetry, true);
    EXPECT_EQ(rig.ble->state(), LinkState::Closing);
    rig.estop(kOwner, {});
    EXPECT_EQ(rig.status().state, MotionState::EmergencyStop);
    EXPECT_EQ(rig.link.notifyAttempts, attempts);
    rig.run(200ms);
    EXPECT_FALSE(rig.drive.enabled);
    rig.ble->onDisconnected(kOwner, kRemoteUserTerminated);
    EXPECT_EQ(rig.ble->state(), LinkState::Advertising);
}

TEST(BlePeripheralTest, FailedTerminationIsCountedAndTheSessionStaysEnded)
{
    Rig rig;
    rig.makeMoving();
    rig.link.failNotify = true;
    rig.link.failTerminate = true;
    rig.command(kOwner, headerFrame(MessageType::Stop, 1));
    EXPECT_EQ(rig.bleCount(BleCounter::TerminateFailures), 1U);
    EXPECT_EQ(rig.ble->state(), LinkState::Closing);
    rig.link.failNotify = false;
    rig.command(kOwner, velocityFrame(2, kForward, kValidity));
    rig.run(200ms);
    EXPECT_FALSE(rig.status().sessionReady);
    EXPECT_FALSE(rig.drive.enabled);
}

// ---- Session end ----------------------------------------------------------------------

TEST(BlePeripheralTest, DisconnectStopsKeepsNothingAndAdvertisesOnlyAfterTheSessionEnded)
{
    Rig rig;
    rig.makeMoving(1000);
    std::optional<runtime::RuntimeStatus> atAdvertising;
    rig.link.whileStartingAdvertising = [&rig, &atAdvertising] { atAdvertising = rig.status(); };
    rig.ble->onDisconnected(kOwner, kRemoteUserTerminated);
    ASSERT_TRUE(atAdvertising);
    EXPECT_FALSE(atAdvertising->sessionReady);
    EXPECT_NE(atAdvertising->state, MotionState::Moving);
    EXPECT_EQ(rig.ble->state(), LinkState::Advertising);
    const auto link = rig.ble->status();
    EXPECT_EQ(link.lastDisconnectReason, kRemoteUserTerminated);
    EXPECT_EQ(link.lastWriteBeforeDisconnect, 30ms) << "time from the last write (#156 I10)";
    EXPECT_FALSE(link.commandResultNotify || link.telemetryNotify);
    rig.run(200ms);
    EXPECT_FALSE(rig.drive.enabled);
    const auto commands = rig.drive.commandCount();

    // Reconnect: a new session with fresh sequences and no retained intent.
    rig.makeReady();
    rig.run(100ms);
    EXPECT_EQ(rig.drive.commandCount(), commands) << "no auto-resume";
    rig.command(kOwner, velocityFrame(5, kForward, kValidity));  // Older than 1000: first of a new session.
    EXPECT_EQ(rig.last().sequence, 0);
    EXPECT_EQ(rig.last().code, ResultCode::Accepted);
}

TEST(BlePeripheralTest, NotificationDisableEndsTheSessionAsForStopAndNeverResumes)
{
    for (const auto disabled : {Characteristic::CommandResult, Characteristic::Telemetry}) {
        Rig rig;
        rig.makeMoving(40);
        rig.subscribe(kOwner, disabled, false);
        EXPECT_EQ(rig.ble->state(), LinkState::Connected);
        EXPECT_FALSE(rig.status().sessionReady);
        EXPECT_NE(rig.status().state, MotionState::Moving);
        const auto results = rig.link.notifications.size();
        rig.command(kOwner, velocityFrame(41, kForward, kValidity));
        EXPECT_EQ(rig.link.notifications.size(), results);
        rig.run(200ms);
        EXPECT_FALSE(rig.drive.enabled);
        const auto commands = rig.drive.commandCount();
        rig.subscribe(kOwner, disabled, true);
        EXPECT_EQ(rig.ble->state(), LinkState::Ready);
        rig.run(100ms);
        EXPECT_EQ(rig.drive.commandCount(), commands) << "re-enabling opens a new session, without intent";
        rig.command(kOwner, velocityFrame(3, kForward, kValidity));
        EXPECT_EQ(rig.last().sequence, 0);
        EXPECT_EQ(rig.last().code, ResultCode::Accepted);
        EXPECT_EQ(rig.runtimeCount(Counter::SessionOpened), 2U);
    }
}

TEST(BlePeripheralTest, HostCallbackOrderAroundADisconnectEndsTheSessionOnce)
{
    Rig rig;
    rig.makeMoving();
    // NimBLE reports each subscription as ended before the disconnect itself.
    rig.subscribe(kOwner, Characteristic::CommandResult, false);
    rig.subscribe(kOwner, Characteristic::Telemetry, false);
    rig.ble->onDisconnected(kOwner, kRemoteUserTerminated);
    EXPECT_EQ(rig.runtimeCount(Counter::SessionEnded), 1U);
    EXPECT_EQ(rig.bleCount(BleCounter::SessionsEnded), 1U);
    // Late events for the closed handle are foreign: no session, no motion, no result.
    rig.subscribe(kOwner, Characteristic::CommandResult);
    rig.subscribe(kOwner, Characteristic::Telemetry);
    rig.command(kOwner, velocityFrame(9, kForward, kValidity));
    rig.run(100ms);
    EXPECT_FALSE(rig.status().sessionReady);
    EXPECT_FALSE(rig.drive.enabled);
    EXPECT_EQ(rig.ble->state(), LinkState::Advertising);
}

TEST(BlePeripheralTest, LatchSurvivesDisconnectAndReconnectAndOnlyAnExplicitClearReleasesIt)
{
    Rig rig;
    rig.makeMoving();
    rig.estop(kOwner, headerFrame(MessageType::EmergencyStop, 1));
    rig.ble->onDisconnected(kOwner, kRemoteUserTerminated);
    rig.makeReady();
    EXPECT_EQ(rig.status().state, MotionState::EmergencyStop);
    rig.command(kOwner, velocityFrame(0, kForward, kValidity));
    EXPECT_EQ(rig.last().code, ResultCode::EmergencyStopLatched);
    rig.command(kOwner, headerFrame(MessageType::ClearEmergencyStop, 1));
    EXPECT_EQ(rig.last().code, ResultCode::Accepted);
    EXPECT_EQ(rig.last().state, static_cast<std::uint8_t>(MotionState::Idle));
    const auto commands = rig.drive.commandCount();
    rig.run(100ms);
    EXPECT_EQ(rig.drive.commandCount(), commands) << "clearing does not resume motion";
    rig.command(kOwner, velocityFrame(2, kForward, kValidity));
    EXPECT_EQ(rig.last().code, ResultCode::Accepted);
}

// ---- Link parameters ------------------------------------------------------------------

TEST(BlePeripheralTest, LinkParametersAreARequestedHintAndOnlyReported)
{
    Rig rig;
    rig.ble->onConnected(kOwner);
    EXPECT_EQ(rig.link.linkRequests, 1);
    EXPECT_EQ(rig.link.lastLinkRequestTo, kOwner);
    EXPECT_EQ(rig.link.lastLinkRequest.maxIntervalUnits, kLinkParameterRequest.maxIntervalUnits);
    EXPECT_EQ(rig.ble->status().linkRequest, LinkRequestOutcome::Requested);
    EXPECT_EQ(rig.ble->status().mtu, kDefaultAttMtu);
    // The central chose other values: reported, and the session still works.
    rig.ble->onLinkParametersUpdated(kOwner, {36, 0, 42});
    rig.ble->onMtuChanged(kOwner, 247);
    const auto link = rig.ble->status();
    ASSERT_TRUE(link.negotiated);
    EXPECT_EQ(connectionInterval(*link.negotiated), 45ms);
    EXPECT_EQ(supervisionTimeout(*link.negotiated), 420ms);
    EXPECT_FALSE(withinRequest(*link.negotiated, kLinkParameterRequest));
    EXPECT_EQ(link.mtu, 247);
    rig.subscribe(kOwner, Characteristic::CommandResult);
    rig.subscribe(kOwner, Characteristic::Telemetry);
    rig.command(kOwner, headerFrame(MessageType::Stop, 0));
    EXPECT_EQ(rig.link.notifications.back().length, kCommandResultLength) << "never longer frames on a larger MTU";
}

TEST(BlePeripheralTest, MtuExchangedBeforeTheConnectCallbackIsStillReported)
{
    // NimBLE can deliver the central's MTU exchange before the connect callback: the event is
    // then for a connection that is not the owner yet. The value comes from the stack instead.
    Rig rig;
    rig.ble->onMtuChanged(kOwner, 256);
    rig.link.stackMtu = 256;
    rig.ble->onConnected(kOwner);
    EXPECT_EQ(rig.ble->status().mtu, 256);
    rig.ble->onDisconnected(kOwner, kRemoteUserTerminated);
    rig.link.stackMtu = 0;  // Unknown to the stack: the default until an exchange is reported.
    rig.ble->onConnected(kOwner);
    EXPECT_EQ(rig.ble->status().mtu, kDefaultAttMtu);
}

// ---- Callback orderings around the connect callback (review findings) -------------------

TEST(BlePeripheralTest, CccdsEnabledBeforeTheConnectCallbackMakeTheSessionReady)
{
    // NimBLE installs the connection before delivering the connect callback, so the central's
    // CCCD writes can be reported first (as the MTU exchange is on the bench).
    Rig rig;
    rig.subscribe(kOwner, Characteristic::CommandResult);
    rig.subscribe(kOwner, Characteristic::Telemetry);
    EXPECT_EQ(rig.ble->state(), LinkState::Advertising);
    EXPECT_FALSE(rig.status().sessionReady);
    rig.ble->onConnected(kOwner);
    EXPECT_EQ(rig.ble->state(), LinkState::Ready);
    EXPECT_TRUE(rig.status().sessionReady);
    EXPECT_TRUE(rig.ble->status().commandResultNotify && rig.ble->status().telemetryNotify);
    rig.command(kOwner, headerFrame(MessageType::Stop, 0));
    ASSERT_EQ(rig.link.notifications.size(), 1U);
    EXPECT_EQ(rig.last().to, kOwner);
    EXPECT_EQ(rig.last().code, ResultCode::Accepted);
    EXPECT_EQ(rig.bleCount(BleCounter::EarlySubscriptions), 2U);
    EXPECT_EQ(rig.bleCount(BleCounter::SessionsReady), 1U);
}

TEST(BlePeripheralTest, EarlyCccdStateIsTheLatestAndOnlyForItsOwnConnection)
{
    {
        Rig rig;  // Enabled, then disabled again before the connect callback: not ready.
        rig.subscribe(kOwner, Characteristic::CommandResult);
        rig.subscribe(kOwner, Characteristic::Telemetry);
        rig.subscribe(kOwner, Characteristic::CommandResult, false);
        rig.ble->onConnected(kOwner);
        EXPECT_EQ(rig.ble->state(), LinkState::Connected);
        EXPECT_FALSE(rig.status().sessionReady);
        rig.subscribe(kOwner, Characteristic::CommandResult);
        EXPECT_EQ(rig.ble->state(), LinkState::Ready);
    }
    {
        Rig rig;  // Another handle's early CCCDs never make the owner ready.
        rig.subscribe(kSecond, Characteristic::CommandResult);
        rig.subscribe(kSecond, Characteristic::Telemetry);
        rig.ble->onConnected(kOwner);
        EXPECT_EQ(rig.ble->state(), LinkState::Connected);
        EXPECT_FALSE(rig.status().sessionReady);
        rig.ble->onConnected(kSecond);  // Refused: its held CCCDs are dropped with it.
        EXPECT_EQ(rig.link.terminated, std::vector<ConnectionHandle>{kSecond});
        rig.ble->onDisconnected(kOwner, kRemoteUserTerminated);
        rig.ble->onConnected(kSecond);
        EXPECT_EQ(rig.ble->state(), LinkState::Connected);
        EXPECT_FALSE(rig.status().sessionReady);
    }
}

TEST(BlePeripheralTest, EarlyCccdsOfAConnectionThatClosedAreForgotten)
{
    Rig rig;
    rig.subscribe(kOwner, Characteristic::CommandResult);
    rig.subscribe(kOwner, Characteristic::Telemetry);
    rig.ble->onDisconnected(kOwner, kRemoteUserTerminated);  // Closed before its connect callback.
    rig.ble->onConnected(kOwner);  // A new connection on the reused handle.
    EXPECT_EQ(rig.ble->state(), LinkState::Connected);
    EXPECT_FALSE(rig.status().sessionReady);
    rig.command(kOwner, headerFrame(MessageType::Stop, 0));
    EXPECT_TRUE(rig.link.notifications.empty());
}

TEST(BlePeripheralTest, ConnectionDuringTheInitialAdvertisingStartIsNotOverwritten)
{
    // start() runs in setup() (loopTask); the higher-priority host task can process a
    // connection and both CCCDs before startAdvertising() returns.
    for (const bool failAdvertising : {false, true}) {
        Rig rig{false};
        rig.link.failAdvertising = failAdvertising;
        rig.link.whileStartingAdvertising = [&rig] { rig.makeReady(); };
        rig.ble->start();
        rig.link.whileStartingAdvertising = nullptr;
        EXPECT_EQ(rig.ble->state(), LinkState::Ready) << failAdvertising;
        EXPECT_TRUE(rig.status().sessionReady) << failAdvertising;
        rig.command(kOwner, headerFrame(MessageType::Stop, 0));
        EXPECT_EQ(rig.link.notifications.size(), 1U) << failAdvertising;
        rig.ble->start();  // Still a no-op once started.
        EXPECT_EQ(rig.link.advertisingStarts, 1) << failAdvertising;
    }
}

TEST(BlePeripheralTest, RefusedLinkParameterRequestIsReportedAndChangesNothingElse)
{
    Rig rig;
    rig.link.refuseLinkRequest = true;
    rig.makeReady();
    EXPECT_EQ(rig.ble->status().linkRequest, LinkRequestOutcome::Refused);
    EXPECT_EQ(rig.bleCount(BleCounter::LinkRequestsRefused), 1U);
    EXPECT_EQ(rig.ble->state(), LinkState::Ready);
}

// ---- Resources ------------------------------------------------------------------------

TEST(BlePeripheralTest, RepeatedLifecycleCyclesAllocateNothingAndLeaveNoState)
{
    constexpr int kCycles = 1000;
    Rig rig;
    rig.link.notifications.reserve(16 * kCycles);
    rig.link.terminated.reserve(4 * kCycles);
    rig.drive.events.reserve(16 * kCycles);  // The fakes record; the code under test must not allocate.
    const auto velocity = velocityFrame(7, kForward, kValidity);
    const auto stale = velocityFrame(6, kForward, kValidity);
    const auto clear = headerFrame(MessageType::ClearEmergencyStop, 8);
    const auto stop = headerFrame(MessageType::Stop, 9);
    const std::vector<std::uint8_t> oversized(kMaxAttributeValueLength, 0xFF);
    const std::vector<std::uint8_t> empty;
    std::size_t allocations = 0;
    {
        codecell::testing::AllocationCount counter;
        for (int cycle = 0; cycle < kCycles; ++cycle) {
            const auto handle = static_cast<ConnectionHandle>(1 + cycle % 3);
            rig.ble->onConnected(handle);
            rig.ble->onConnected(static_cast<ConnectionHandle>(handle + 10));  // A second central each time.
            rig.ble->onMtuChanged(handle, 185);
            rig.ble->onLinkParametersUpdated(handle, {24, 0, 40});
            rig.ble->onSubscriptionChanged(handle, Characteristic::CommandResult, true);
            rig.ble->onSubscriptionChanged(handle, Characteristic::Telemetry, true);
            rig.ble->onCommandWrite(handle, velocity);
            rig.ble->onCommandWrite(handle, stale);
            rig.ble->onCommandWrite(handle, oversized);
            rig.ble->onEmergencyStopWrite(handle, empty);
            rig.ble->onCommandWrite(handle, clear);
            rig.ble->onCommandWrite(handle, stop);
            switch (cycle % 3) {
            case 0: rig.ble->onSubscriptionChanged(handle, Characteristic::Telemetry, false); break;
            case 1:
                rig.link.failNotify = true;
                rig.ble->onCommandWrite(handle, stop);
                rig.link.failNotify = false;
                break;
            default: break;
            }
            rig.ble->onDisconnected(static_cast<ConnectionHandle>(handle + 10), kRemoteUserTerminated);
            rig.ble->onDisconnected(handle, kRemoteUserTerminated);
        }
        allocations = counter.count();
    }
    EXPECT_EQ(allocations, 0U);
    EXPECT_EQ(rig.ble->state(), LinkState::Advertising);
    EXPECT_FALSE(rig.status().sessionReady);
    EXPECT_EQ(rig.status().state, MotionState::Idle);
    EXPECT_EQ(rig.bleCount(BleCounter::Connections), static_cast<std::uint32_t>(kCycles));
    EXPECT_EQ(rig.bleCount(BleCounter::Disconnects), static_cast<std::uint32_t>(kCycles));
    EXPECT_EQ(rig.bleCount(BleCounter::RejectedCentrals), static_cast<std::uint32_t>(kCycles));
    EXPECT_EQ(rig.bleCount(BleCounter::SessionsReady), rig.bleCount(BleCounter::SessionsEnded));
    EXPECT_EQ(rig.runtimeCount(Counter::SessionOpened), rig.runtimeCount(Counter::SessionEnded));
    EXPECT_EQ(rig.link.advertisingStarts, kCycles + 1);
    rig.run(200ms);
    EXPECT_FALSE(rig.drive.enabled);
}
