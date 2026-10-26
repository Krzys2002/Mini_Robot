#include "transport/ble/BleTransport.hpp"
#include "app/RobotManager.hpp"
#include "transport/ble/QtBleBackend.hpp"
#if defined(MICRO_ROBOT_BLE_WITH_DESKTOP)
#include "app/DesktopController.hpp"
#endif

#include <QCoreApplication>
#include <QElapsedTimer>
#include <QThread>
#include <gtest/gtest.h>

#include <functional>
#include <algorithm>

namespace {
using namespace micro_robot;
using namespace std::chrono_literals;
constexpr RobotId robot{7};
void drain()
{
    for (int i = 0; i < 10; ++i) QCoreApplication::processEvents();
}
bool until(const std::function<bool()>& predicate)
{
    QElapsedTimer timer;
    timer.start();
    while (!predicate() && timer.elapsed() < 2000) {
        QCoreApplication::processEvents();
        QThread::yieldCurrentThread();
    }
    return predicate();
}
class FakeBleBackend final : public IBleBackend {
public:
    void scan(std::uint64_t token, std::size_t limit) override { scanToken = token; maximumDevices = limit; ++scans; }
    void stopScan() override { ++stops; }
    void open(std::uint64_t token, const std::string& key, RobotId expected) override
    { links[expected] = token; lastKey = key; ++opens; }
    void close(std::uint64_t token) override { lastClosed = token; ++closes; }
    void release(std::uint64_t token) override { released.push_back(token); }
    void ready(std::uint64_t token)
    {
        BleRobotInfo info{robot, 300, 6000, 500, 100};
        for (const auto& [id, value] : links) if (value == token) info.robotId = id;
        emit IBleBackend::ready(token, info);
    }
    BleWriteOutcome write(std::uint64_t token, const BleWriteRequest& request) override
    {
        if (writeError) return {false, {}, *writeError, "Injected write failure"};
        const auto now = MonotonicClock::now() + dispatchDelay;
        const auto encoded = request.emergencyStop
            ? BleEncodeResult{encodeBleEmergencyStop(request.sequence), TransportErrorCode::InvalidCommand, {}}
            : encodeBleCommand(request.command, BleRobotInfo{robot, 300, 6000, 500, 100}, request.sequence, now);
        if (!encoded.frame) return {false, {}, encoded.code, encoded.error};
        writes.push_back({token, request, *encoded.frame});
        if (completeWrites) emit written(token, request.sequence);
        return {true, now, TransportErrorCode::BackendFailure, {}};
    }
    void found(const std::string& key = "opaque-a", const std::string& name = "MR-7")
    { emit deviceFound(scanToken, {key, name, {}, {}}); }
    void finish() { emit scanFinished(scanToken); }
    std::uint64_t scanToken{}, lastClosed{};
    std::size_t maximumDevices{};
    std::map<RobotId, std::uint64_t> links;
    std::vector<std::uint64_t> released;
    std::string lastKey;
    int scans{}, stops{}, opens{}, closes{};
    struct Write { std::uint64_t token; BleWriteRequest request; EncodedBleCommand frame; };
    std::vector<Write> writes;
    bool completeWrites{true};
    Duration dispatchDelay{};
    std::optional<TransportErrorCode> writeError;
};
class Events final : public IRobotTransportEvents {
public:
    BleTransport* transport{};
    std::vector<ConnectionState> states;
    std::vector<TransportError> errors;
    std::function<void(ConnectionState)> onState;
    int telemetry{};
    std::vector<RobotTelemetry> samples;
    void onConnectionStateChanged(RobotId id, ConnectionState state, Timestamp) override
    {
        EXPECT_EQ(transport->connectionState(id), state);
        states.push_back(state);
        if (onState) onState(state);
    }
    void onTelemetry(const RobotTelemetry& sample) override { ++telemetry; samples.push_back(sample); }
    void onTransportError(const TransportError& error) override { errors.push_back(error); }
};
class MemoryLog final : public ILogSink {
public:
    void write(const LogRecord& record) override { records.push_back(record); }
    std::vector<LogRecord> records;
    bool contains(const std::string& text) const
    {
        return std::any_of(records.begin(), records.end(), [&](const auto& r) { return r.message.find(text) != std::string::npos; });
    }
};
struct Fixture {
    Logger logger{LogLevel::Debug};
    std::shared_ptr<MemoryLog> logs = std::make_shared<MemoryLog>();
    FakeBleBackend* backend{};
    std::unique_ptr<BleTransport> transport;
    std::shared_ptr<Events> events = std::make_shared<Events>();
    explicit Fixture(BleTransportConfig config = {})
    {
        auto implementation = std::make_unique<FakeBleBackend>();
        backend = implementation.get();
        logger.addSink(logs);
        transport = std::make_unique<BleTransport>(std::move(implementation), config, logger.channel("ble"));
        events->transport = transport.get();
        transport->setEventSink(events);
    }
    void connecting()
    {
        transport->connect(robot);
        drain();
        backend->found(); backend->finish(); drain();
        ASSERT_EQ(transport->phase(robot), BlePhase::Connecting);
    }
    void ready()
    {
        connecting();
        emit backend->ready(backend->links.at(robot)); drain();
        ASSERT_EQ(transport->connectionState(robot), ConnectionState::Connected);
    }
};
RobotCommand freshVelocity(double value = 0.15, Duration validity = 250ms)
{
    return {VelocityCommand{value, -0.5}, MonotonicClock::now(), validity};
}
QByteArray telemetryBytes(std::uint16_t sequence, unsigned state = 1)
{
    auto bytes = QByteArray::fromHex("018007000104020140e20100480f94000080");
    bytes[2] = static_cast<char>(sequence); bytes[3] = static_cast<char>(sequence >> 8);
    bytes[4] = static_cast<char>(state);
    return bytes;
}
QByteArray resultBytes(std::uint16_t sequence, std::uint16_t commandSequence = 0,
                       unsigned code = 0, unsigned type = 2, unsigned state = 1)
{
    auto bytes = QByteArray::fromHex("018108000000020001");
    bytes[2] = static_cast<char>(sequence); bytes[3] = static_cast<char>(sequence >> 8);
    bytes[4] = static_cast<char>(commandSequence); bytes[5] = static_cast<char>(commandSequence >> 8);
    bytes[6] = static_cast<char>(type); bytes[7] = static_cast<char>(code); bytes[8] = static_cast<char>(state);
    return bytes;
}
TEST(BleTransportTest, DefersRequestsAndPublishesReadyOnlyAfterBackendSetup)
{
    Fixture f;
    EXPECT_EQ(f.transport->connectionState(robot), ConnectionState::Disconnected);
    f.transport->connect(robot);
    f.transport->connect(robot);
    EXPECT_TRUE(f.events->states.empty());
    drain();
    EXPECT_EQ(f.backend->scans, 1);
    EXPECT_EQ(f.transport->phase(robot), BlePhase::Scanning);
    f.backend->found(); f.backend->finish(); drain();
    EXPECT_EQ(f.backend->opens, 1);
    EXPECT_EQ(f.backend->lastKey, "opaque-a");
    for (const auto phase : {BlePhase::DiscoveringServices, BlePhase::DiscoveringCharacteristics,
                            BlePhase::ReadingIdentity, BlePhase::EnablingNotifications}) {
        emit f.backend->phaseChanged(f.backend->links.at(robot), phase); drain();
        EXPECT_EQ(f.transport->phase(robot), phase);
        EXPECT_EQ(f.transport->connectionState(robot), ConnectionState::Connecting);
    }
    emit f.backend->ready(f.backend->links.at(robot)); drain();
    EXPECT_EQ(f.transport->connectionState(robot), ConnectionState::Connected);
    EXPECT_EQ(f.transport->phase(robot), BlePhase::Ready);
    EXPECT_EQ(f.events->telemetry, 0);
}
TEST(BleTransportTest, DiscoveryDeduplicatesUpdatesBoundsInventoryAndKeepsUnknownNames)
{
    BleTransportConfig config;
    config.maximumDevices = 2;
    Fixture f{config};
    f.transport->startDiscovery(); drain();
    f.backend->found("a", "MR-7");
    f.backend->found("a", "MR-8");
    f.backend->found("b", "anonymous");
    f.backend->found("c", "MR-9"); drain();
    ASSERT_EQ(f.transport->devices().size(), 2U);
    EXPECT_EQ(f.transport->devices()[0].advertisedId, RobotId{8});
    EXPECT_FALSE(f.transport->devices()[1].advertisedId);
    EXPECT_FALSE(f.transport->devices()[0].address); // No fabricated macOS address.
    EXPECT_EQ(f.backend->maximumDevices, 2U);
    f.transport->cancelDiscovery(); drain();
    EXPECT_FALSE(f.transport->discovering());
}
TEST(BleTransportTest, RejectsMissingAndAmbiguousIdentitiesWithoutOpeningALink)
{
    for (bool ambiguous : {false, true}) {
        Fixture f;
        f.transport->connect(robot); drain();
        if (ambiguous) { f.backend->found("a"); f.backend->found("b"); }
        f.backend->finish(); drain();
        EXPECT_EQ(f.backend->opens, 0);
        EXPECT_EQ(f.transport->connectionState(robot), ConnectionState::Fault);
        ASSERT_EQ(f.events->errors.size(), 1U);
        EXPECT_EQ(f.events->errors[0].code, TransportErrorCode::Unavailable);
        EXPECT_NE(f.events->errors[0].message.find(ambiguous ? "Ambiguous" : "not found"), std::string::npos);
    }
}
TEST(BleTransportTest, CancelPendingScanAndConnectionIgnoresLateResults)
{
    Fixture f;
    f.transport->connect(robot); drain();
    f.transport->disconnect(robot); drain();
    f.backend->found(); f.backend->finish(); drain();
    EXPECT_EQ(f.backend->opens, 0);
    EXPECT_EQ(f.transport->connectionState(robot), ConnectionState::Disconnected);
    f.connecting();
    const auto token = f.backend->links.at(robot);
    f.transport->disconnect(robot); drain();
    emit f.backend->ready(token); drain();
    EXPECT_EQ(f.transport->connectionState(robot), ConnectionState::Disconnecting);
    emit f.backend->closed(token); drain();
    EXPECT_EQ(f.transport->connectionState(robot), ConnectionState::Disconnected);
    emit f.backend->ready(token); drain();
    EXPECT_EQ(f.transport->connectionState(robot), ConnectionState::Disconnected);
}
TEST(BleTransportTest, RepeatedConnectDisconnectResetsSessionAndRejectsOldTokens)
{
    Fixture f;
    std::uint64_t old{};
    for (int cycle = 0; cycle < 3; ++cycle) {
        f.ready();
        const auto token = f.backend->links.at(robot);
        EXPECT_NE(token, old);
        emit f.backend->failed(old, TransportErrorCode::BackendFailure, "Late failure"); drain();
        EXPECT_EQ(f.transport->connectionState(robot), ConnectionState::Connected);
        f.transport->disconnect(robot); f.transport->disconnect(robot); drain();
        emit f.backend->closed(token); drain();
        EXPECT_EQ(f.transport->connectionState(robot), ConnectionState::Disconnected);
        old = token;
    }
    EXPECT_EQ(f.backend->closes, 3);
}
TEST(BleTransportTest, BackendErrorsAreTerminalAndRecoveryStartsANewScan)
{
    Fixture f;
    f.connecting();
    const auto token = f.backend->links.at(robot);
    emit f.backend->failed(token, TransportErrorCode::UnsupportedVersion, "Incompatible Robot Info"); drain();
    ASSERT_EQ(f.events->errors.size(), 1U);
    EXPECT_EQ(f.events->errors[0].code, TransportErrorCode::UnsupportedVersion);
    EXPECT_EQ(f.transport->connectionState(robot), ConnectionState::Fault);
    emit f.backend->ready(token); drain();
    EXPECT_EQ(f.transport->connectionState(robot), ConnectionState::Fault);
    f.ready();
    EXPECT_NE(f.backend->links.at(robot), token);
}
TEST(BleTransportTest, ScanPermissionAdapterAndPowerFailuresAreDeferred)
{
    Fixture f;
    f.transport->connect(robot); drain();
    emit f.backend->failed(f.backend->scanToken, TransportErrorCode::Unavailable, "Bluetooth permission denied");
    EXPECT_TRUE(f.events->errors.empty());
    drain();
    ASSERT_EQ(f.events->errors.size(), 1U);
    EXPECT_EQ(f.transport->connectionState(robot), ConnectionState::Fault);
    EXPECT_NE(f.events->errors[0].message.find("permission"), std::string::npos);
}
TEST(BleTransportTest, TimesOutSilentScanAndEachSetupPhase)
{
    for (const auto phase : {BlePhase::Scanning, BlePhase::Connecting, BlePhase::DiscoveringServices,
                            BlePhase::DiscoveringCharacteristics, BlePhase::ReadingIdentity,
                            BlePhase::EnablingNotifications}) {
        BleTransportConfig config;
        config.scanTimeout = config.phaseTimeout = 15ms;
        Fixture f{config};
        if (phase == BlePhase::Scanning) { f.transport->connect(robot); drain(); }
        else {
            f.connecting();
            emit f.backend->phaseChanged(f.backend->links.at(robot), phase); drain();
        }
        ASSERT_TRUE(until([&] { return f.transport->connectionState(robot) == ConnectionState::Fault; }));
        ASSERT_EQ(f.events->errors.size(), 1U);
        EXPECT_EQ(f.events->errors[0].operation, TransportOperation::Connect);
    }
}
TEST(BleTransportTest, DisconnectTimeoutAndUnexpectedLinkLossAreTerminal)
{
    Fixture f{BleTransportConfig{10000ms, 10000ms, 15ms}};
    f.ready();
    f.transport->disconnect(robot); drain();
    ASSERT_TRUE(until([&] { return f.transport->connectionState(robot) == ConnectionState::Fault; }));
    ASSERT_EQ(f.events->errors.size(), 1U);
    EXPECT_EQ(f.events->errors[0].operation, TransportOperation::Disconnect);
    f.ready();
    emit f.backend->closed(f.backend->links.at(robot)); drain();
    EXPECT_EQ(f.transport->connectionState(robot), ConnectionState::Fault);
}
TEST(BleTransportTest, CallbackDisconnectIsDeferredAndSinkReplacementIsSafe)
{
    Fixture f;
    f.events->onState = [&](ConnectionState state) {
        if (state == ConnectionState::Connected) {
            f.transport->disconnect(robot);
            EXPECT_EQ(f.transport->connectionState(robot), ConnectionState::Connected);
        }
    };
    f.connecting();
    emit f.backend->ready(f.backend->links.at(robot)); drain();
    EXPECT_EQ(f.transport->connectionState(robot), ConnectionState::Disconnecting);
    f.transport->setEventSink({});
    const auto count = f.events->states.size();
    emit f.backend->closed(f.backend->links.at(robot)); drain();
    EXPECT_EQ(f.events->states.size(), count);
}
TEST(BleTransportTest, DifferentRobotsShareScanAndHaveIndependentLinkStates)
{
    Fixture f;
    f.transport->connect(robot); f.transport->connect(RobotId{8}); drain();
    EXPECT_EQ(f.backend->scans, 1);
    f.backend->found("a", "MR-7"); f.backend->found("b", "MR-8"); f.backend->finish(); drain();
    emit f.backend->ready(f.backend->links.at(robot));
    emit f.backend->failed(f.backend->links.at(RobotId{8}), TransportErrorCode::BackendFailure, "Missing service"); drain();
    EXPECT_EQ(f.transport->connectionState(robot), ConnectionState::Connected);
    EXPECT_EQ(f.transport->connectionState(RobotId{8}), ConnectionState::Fault);
}
TEST(BleTransportTest, CommandsRequireConnectionValidFreshIntentAndSupportedVersion)
{
    Fixture f;
    RobotCommand command;
    f.transport->sendCommand(robot, command);
    EXPECT_TRUE(f.events->errors.empty()); drain();
    ASSERT_EQ(f.events->errors.size(), 1U);
    EXPECT_EQ(f.events->errors.back().code, TransportErrorCode::NotConnected);
    f.ready();
    f.transport->sendCommand(robot, command); drain();
    EXPECT_EQ(f.events->errors.back().code, TransportErrorCode::InvalidCommand);
    command = {VelocityCommand{0.15, -0.5}, MonotonicClock::now(), 250ms};
    f.transport->sendCommand(robot, command); drain();
    ASSERT_EQ(f.backend->writes.size(), 1U);
    EXPECT_EQ(f.backend->writes[0].frame.endpoint, BleEndpoint::Command);
    EXPECT_FALSE(f.backend->writes[0].frame.withResponse);
    command.metadata.modelVersion = 99;
    f.transport->sendCommand(robot, command); drain();
    EXPECT_EQ(f.events->errors.back().code, TransportErrorCode::UnsupportedVersion);
    EXPECT_EQ(f.events->telemetry, 0);
}
TEST(BleTransportTest, ConfigurationAndConstructionDoNotTouchBluetoothHardware)
{
    auto backend = makeQtBleBackend(); // Native scanner/controller allocated only on request.
    BleTransport transport{std::move(backend)};
    EXPECT_FALSE(transport.discovering());
    EXPECT_EQ(transport.connectionState(robot), ConnectionState::Disconnected);
    EXPECT_THROW((BleTransport{std::make_unique<FakeBleBackend>(), BleTransportConfig{0ms}}), std::invalid_argument);
}
TEST(BleTransportTest, CancelsQueuedCallbacksOnDestructionAndUsesWeakSinks)
{
    Fixture f;
    f.connecting();
    const auto count = f.events->states.size();
    emit f.backend->ready(f.backend->links.at(robot)); // Queued but not dispatched.
    f.transport.reset();
    drain();
    EXPECT_EQ(f.events->states.size(), count);
    Fixture detached;
    detached.events.reset();
    detached.connecting();
    emit detached.backend->ready(detached.backend->links.at(robot)); drain();
    EXPECT_EQ(detached.transport->connectionState(robot), ConnectionState::Connected);
}
TEST(BleTransportTest, CoalescesLatestVelocityAndKeepsStopAheadOfUnsentMotion)
{
    Fixture f;
    f.ready();
    for (int i = 1; i <= 100; ++i) f.transport->sendCommand(robot, freshVelocity(i / 1000.0));
    EXPECT_TRUE(f.backend->writes.empty());
    drain();
    ASSERT_EQ(f.backend->writes.size(), 1U);
    EXPECT_EQ(f.backend->writes[0].frame.bytes[4], 100);
    f.backend->completeWrites = false;
    RobotCommand stop{StopCommand{}, MonotonicClock::now(), 500ms};
    f.transport->sendCommand(robot, stop); drain();
    ASSERT_EQ(f.backend->writes.size(), 2U);
    EXPECT_TRUE(f.backend->writes.back().frame.withResponse);
    f.transport->sendCommand(robot, freshVelocity(0.1));
    f.transport->sendCommand(robot, stop);
    f.transport->sendCommand(robot, freshVelocity(0.2)); drain();
    EXPECT_EQ(f.backend->writes.size(), 2U); // One pending slot, no native backlog.
    emit f.backend->written(f.backend->links.at(robot), 1); drain();
    ASSERT_EQ(f.backend->writes.size(), 3U);
    EXPECT_EQ(f.backend->writes.back().frame.bytes[1], 1); // Stop retained.
    EXPECT_EQ(f.backend->writes.back().request.sequence, 2);
}
TEST(BleTransportTest, RejectsExpiredPendingIntentAndNativeFreshnessFailureWithoutAllocatingSequence)
{
    Fixture f;
    f.ready();
    f.backend->completeWrites = false;
    f.transport->sendCommand(robot, {StopCommand{}, MonotonicClock::now(), 1s}); drain();
    f.transport->sendCommand(robot, freshVelocity(0.1, 10ms));
    QElapsedTimer elapsed; elapsed.start();
    ASSERT_TRUE(until([&] { return elapsed.elapsed() >= 15; }));
    emit f.backend->written(f.backend->links.at(robot), 0); drain();
    EXPECT_EQ(f.backend->writes.size(), 1U);
    ASSERT_FALSE(f.events->errors.empty());
    EXPECT_EQ(f.events->errors.back().code, TransportErrorCode::InvalidCommand);
    f.backend->dispatchDelay = 1s;
    f.transport->sendCommand(robot, freshVelocity()); drain();
    EXPECT_EQ(f.backend->writes.size(), 1U);
    f.backend->dispatchDelay = {};
    f.transport->sendCommand(robot, freshVelocity()); drain();
    ASSERT_EQ(f.backend->writes.size(), 2U);
    EXPECT_EQ(f.backend->writes.back().request.sequence, 1);
}
TEST(BleTransportTest, PublishesOnlyValidatedTelemetryWithReceiptClockAndSharedUnwrappedSequence)
{
    Fixture f;
    f.ready();
    const auto token = f.backend->links.at(robot);
    const auto receipt = MonotonicClock::now();
    emit f.backend->notification(token, BleEndpoint::Telemetry, telemetryBytes(65534), receipt);
    EXPECT_EQ(f.events->telemetry, 0); drain();
    ASSERT_EQ(f.events->samples.size(), 1U);
    EXPECT_EQ(f.events->samples[0].robotId, robot);
    EXPECT_EQ(f.events->samples[0].receivedAt, receipt);
    EXPECT_EQ(f.events->samples[0].metadata.modelVersion, currentTransportModelVersion);
    EXPECT_EQ(f.events->samples[0].metadata.sequence, 65534U);
    EXPECT_DOUBLE_EQ(*f.events->samples[0].batteryVoltageVolts, 3.912);
    EXPECT_FALSE(f.events->samples[0].angularRadiansPerSecond);
    emit f.backend->notification(token, BleEndpoint::CommandResult, resultBytes(65535), MonotonicClock::now());
    emit f.backend->notification(token, BleEndpoint::Telemetry, telemetryBytes(1), MonotonicClock::now());
    emit f.backend->notification(token, BleEndpoint::Telemetry, telemetryBytes(0), MonotonicClock::now());
    emit f.backend->notification(token, BleEndpoint::Telemetry, telemetryBytes(1), MonotonicClock::now()); drain();
    ASSERT_EQ(f.events->samples.size(), 2U);
    EXPECT_EQ(f.events->samples.back().metadata.sequence, 65537U);
    EXPECT_TRUE(f.logs->contains("gap=1"));
    EXPECT_TRUE(f.logs->contains("robot_uptime_ms=123456"));
    EXPECT_TRUE(f.logs->contains("last_accepted_sequence=258"));
}
TEST(BleTransportTest, InvalidFramesDoNotAdvanceReceiveCounterAndRepeatedFailuresFaultTheLink)
{
    Fixture f;
    f.ready();
    const auto token = f.backend->links.at(robot);
    auto incompatible = telemetryBytes(20); incompatible[0] = 2;
    emit f.backend->notification(token, BleEndpoint::Telemetry, incompatible, MonotonicClock::now()); drain();
    ASSERT_EQ(f.events->errors.size(), 1U);
    EXPECT_EQ(f.events->errors[0].operation, TransportOperation::ReceiveTelemetry);
    EXPECT_EQ(f.events->errors[0].code, TransportErrorCode::UnsupportedVersion);
    f.transport->sendCommand(robot, freshVelocity()); drain();
    EXPECT_TRUE(f.backend->writes.empty()); // Invalid input is conservative for motion.
    emit f.backend->notification(token, BleEndpoint::Telemetry, telemetryBytes(0), MonotonicClock::now()); drain();
    ASSERT_EQ(f.events->telemetry, 1);
    EXPECT_EQ(f.events->samples.back().metadata.sequence, 0U);
    for (int i = 0; i < 3; ++i)
        emit f.backend->notification(token, BleEndpoint::CommandResult, telemetryBytes(1), MonotonicClock::now());
    drain();
    EXPECT_EQ(f.transport->connectionState(robot), ConnectionState::Fault);
    EXPECT_EQ(f.events->telemetry, 1);
    EXPECT_EQ(f.events->errors.back().operation, TransportOperation::ReceiveTelemetry);
}
TEST(BleTransportTest, CorrelatesCommandResultsAndMapsRejectionsWithoutTreatingAttAsAcceptance)
{
    Fixture f;
    f.ready();
    auto command = freshVelocity(); command.metadata.sequence = 123456;
    f.transport->sendCommand(robot, command); drain();
    ASSERT_EQ(f.backend->writes.size(), 1U);
    EXPECT_TRUE(f.events->errors.empty());
    EXPECT_EQ(f.events->telemetry, 0);
    EXPECT_TRUE(f.logs->contains("domain_sequence=123456"));
    emit f.backend->notification(f.backend->links.at(robot), BleEndpoint::CommandResult,
        resultBytes(0, 0, 6), MonotonicClock::now()); drain();
    ASSERT_EQ(f.events->errors.size(), 1U);
    EXPECT_EQ(f.events->errors[0].operation, TransportOperation::SendCommand);
    EXPECT_EQ(f.events->errors[0].code, TransportErrorCode::InvalidCommand);
    EXPECT_NE(f.events->errors[0].message.find("OutOfRange"), std::string::npos);
    EXPECT_TRUE(f.logs->contains("round_trip_us="));
    EXPECT_TRUE(f.logs->contains("steady_us="));
}
TEST(BleTransportTest, DropsPendingWorkAndLateFramesAcrossDisconnectAndResetsBothCounters)
{
    Fixture f;
    f.ready();
    const auto token = f.backend->links.at(robot);
    f.transport->sendCommand(robot, freshVelocity());
    f.transport->disconnect(robot);
    drain(); // The earlier command may hand off, but cannot outlive this generation.
    emit f.backend->closed(token); drain();
    const auto writes = f.backend->writes.size();
    emit f.backend->notification(token, BleEndpoint::Telemetry, telemetryBytes(500), MonotonicClock::now()); drain();
    EXPECT_EQ(f.events->telemetry, 0);
    f.ready();
    emit f.backend->notification(token, BleEndpoint::Telemetry, telemetryBytes(501), MonotonicClock::now());
    emit f.backend->notification(f.backend->links.at(robot), BleEndpoint::Telemetry, telemetryBytes(0), MonotonicClock::now());
    f.transport->sendCommand(robot, freshVelocity()); drain();
    EXPECT_EQ(f.events->telemetry, 1);
    EXPECT_EQ(f.events->samples.back().metadata.sequence, 0U);
    ASSERT_EQ(f.backend->writes.size(), writes + 1);
    EXPECT_EQ(f.backend->writes.back().request.sequence, 0);
}
TEST(BleTransportTest, EmergencyStopBypassesOrdinaryWriteSlotCancelsPendingMotionAndStopsRetriesOnTelemetry)
{
    BleTransportConfig config;
    config.emergencyStopRetry = 10ms;
    Fixture f{config};
    f.ready();
    f.backend->completeWrites = false;
    f.transport->sendCommand(robot, {StopCommand{}, MonotonicClock::now(), 1s}); drain();
    f.transport->sendCommand(robot, freshVelocity());
    f.transport->requestEmergencyStop(robot); drain();
    ASSERT_EQ(f.backend->writes.size(), 2U);
    EXPECT_EQ(f.backend->writes[1].frame.endpoint, BleEndpoint::EmergencyStop);
    EXPECT_TRUE(f.backend->writes[1].frame.withResponse);
    EXPECT_EQ(f.backend->writes[1].frame.bytes[1], 3);
    emit f.backend->written(f.backend->links.at(robot), 1);
    f.backend->completeWrites = true;
    ASSERT_TRUE(until([&] { return f.backend->writes.size() >= 3; }));
    EXPECT_EQ(f.backend->writes.back().request.sequence, 2);
    emit f.backend->notification(f.backend->links.at(robot), BleEndpoint::Telemetry,
        telemetryBytes(0, 2), MonotonicClock::now()); drain();
    const auto count = f.backend->writes.size();
    QElapsedTimer elapsed; elapsed.start();
    ASSERT_TRUE(until([&] { return elapsed.elapsed() >= 30; }));
    EXPECT_EQ(f.backend->writes.size(), count);
    f.transport->sendCommand(robot, freshVelocity()); drain();
    EXPECT_EQ(f.events->errors.back().code, TransportErrorCode::InvalidCommand);
}
TEST(BleTransportTest, ReassertsEmergencyIntentBeforePublishingConnectedAfterReconnect)
{
    Fixture f;
    f.transport->requestEmergencyStop(robot);
    f.connecting();
    const auto token = f.backend->links.at(robot);
    emit f.backend->ready(token); drain();
    EXPECT_EQ(f.transport->connectionState(robot), ConnectionState::Connecting);
    ASSERT_EQ(f.backend->writes.size(), 1U);
    EXPECT_EQ(f.backend->writes[0].frame.endpoint, BleEndpoint::EmergencyStop);
    emit f.backend->notification(token, BleEndpoint::CommandResult, resultBytes(0, 0, 0, 3, 2), MonotonicClock::now()); drain();
    EXPECT_EQ(f.transport->connectionState(robot), ConnectionState::Connected);
    f.transport->disconnect(robot); drain(); emit f.backend->closed(token); drain();
    f.connecting(); emit f.backend->ready(f.backend->links.at(robot)); drain();
    EXPECT_EQ(f.transport->connectionState(robot), ConnectionState::Connecting);
    ASSERT_EQ(f.backend->writes.size(), 2U);
    EXPECT_EQ(f.backend->writes.back().request.sequence, 0);
}
TEST(BleTransportTest, WriteErrorsAndMissingAttConfirmationFaultTheSession)
{
    Fixture f;
    f.ready();
    emit f.backend->writeFailed(f.backend->links.at(robot), "Injected GATT write error"); drain();
    EXPECT_EQ(f.transport->connectionState(robot), ConnectionState::Fault);
    EXPECT_EQ(f.events->errors.back().operation, TransportOperation::SendCommand);
    BleTransportConfig config; config.phaseTimeout = 15ms;
    Fixture silent{config};
    silent.ready(); silent.backend->completeWrites = false;
    silent.transport->sendCommand(robot, {StopCommand{}, MonotonicClock::now(), 1s}); drain();
    ASSERT_TRUE(until([&] { return silent.transport->connectionState(robot) == ConnectionState::Fault; }));
    EXPECT_EQ(silent.events->errors.back().operation, TransportOperation::SendCommand);
}
TEST(BleTransportTest, RobotFaultAndEmergencyStatusBlockVelocityUntilValidReadyStatusReturns)
{
    Fixture f;
    f.ready();
    const auto token = f.backend->links.at(robot);
    for (const auto state : {2U, 3U}) {
        emit f.backend->notification(token, BleEndpoint::Telemetry, telemetryBytes(static_cast<std::uint16_t>(state), state), MonotonicClock::now()); drain();
        f.transport->sendCommand(robot, freshVelocity()); drain();
        EXPECT_TRUE(f.backend->writes.empty());
    }
    emit f.backend->notification(token, BleEndpoint::Telemetry, telemetryBytes(4, 0), MonotonicClock::now()); drain();
    f.transport->sendCommand(robot, freshVelocity()); drain();
    EXPECT_EQ(f.backend->writes.size(), 1U);
}
TEST(BleTransportTest, BoundsCorrelationHistoryAndRateLimitsToNewestPendingVelocity)
{
    BleTransportConfig config;
    config.maximumTrackedCommands = 1;
    config.minimumVelocityInterval = 30ms;
    Fixture f{config};
    f.ready();
    f.transport->sendCommand(robot, freshVelocity(0.1)); drain();
    f.transport->sendCommand(robot, freshVelocity(0.2)); drain();
    EXPECT_EQ(f.backend->writes.size(), 1U);
    f.transport->sendCommand(robot, freshVelocity(0.25)); drain();
    ASSERT_TRUE(until([&] { return f.backend->writes.size() == 2; }));
    EXPECT_EQ(f.backend->writes.back().frame.bytes[4], 250);
    emit f.backend->notification(f.backend->links.at(robot), BleEndpoint::CommandResult,
        resultBytes(0, 0), MonotonicClock::now()); drain();
    EXPECT_TRUE(f.logs->contains("round_trip_us=unknown")); // Evicted entry is not an invented RTT.
}
TEST(BleTransportTest, PhaseObserversCannotSubmitCommandsAfterIoHasBeenInvalidated)
{
    Fixture f;
    f.ready();
    QObject::connect(f.transport.get(), &BleTransport::phaseChanged, f.transport.get(),
        [&](RobotId id, BlePhase phase) {
            if (phase == BlePhase::Disconnecting || phase == BlePhase::Fault)
                f.transport->sendCommand(id, freshVelocity());
        });
    f.transport->disconnect(robot); drain();
    EXPECT_TRUE(f.backend->writes.empty());
    ASSERT_FALSE(f.events->errors.empty());
    EXPECT_EQ(f.events->errors.back().code, TransportErrorCode::NotConnected);
    emit f.backend->closed(f.backend->links.at(robot)); drain();
    f.ready();
    emit f.backend->writeFailed(f.backend->links.at(robot), "Injected write failure"); drain();
    EXPECT_TRUE(f.backend->writes.empty());
    EXPECT_EQ(f.events->errors.back().code, TransportErrorCode::NotConnected);
}
TEST(BleTransportTest, RobotManagerRoutesTwoValidatedEndpointsAndDrainsRemoval)
{
    Fixture f;
    WorldModel world;
    RobotManager robots{*f.transport, world};
    const RobotId second{8};
    ASSERT_TRUE(robots.addRobot(robot, MonotonicClock::now()));
    ASSERT_TRUE(robots.addRobot(second, MonotonicClock::now()));
    ASSERT_TRUE(robots.connect(robot));
    ASSERT_TRUE(robots.connect(second)); drain();
    f.backend->found("opaque-a", "MR-7");
    f.backend->found("opaque-b", "MR-8"); f.backend->finish(); drain();
    const auto firstToken = f.backend->links.at(robot);
    const auto secondToken = f.backend->links.at(second);
    emit f.backend->ready(firstToken); emit f.backend->ready(secondToken); drain();
    ASSERT_EQ(robots.connectionState(robot), ConnectionState::Connected);
    ASSERT_EQ(robots.connectionState(second), ConnectionState::Connected);
    EXPECT_EQ(robots.sendCommand(second, freshVelocity()), CommandSubmission::Submitted);
    EXPECT_EQ(robots.stop(robot, MonotonicClock::now(), 1s), CommandSubmission::Submitted); drain();
    ASSERT_EQ(f.backend->writes.size(), 2U);
    EXPECT_TRUE(std::any_of(f.backend->writes.begin(), f.backend->writes.end(), [&](const auto& write) {
        return write.token == secondToken && std::holds_alternative<VelocityCommand>(write.request.command.payload);
    }));
    EXPECT_TRUE(std::any_of(f.backend->writes.begin(), f.backend->writes.end(), [&](const auto& write) {
        return write.token == firstToken && std::holds_alternative<StopCommand>(write.request.command.payload);
    }));
    emit f.backend->notification(secondToken, BleEndpoint::Telemetry, telemetryBytes(0), MonotonicClock::now()); drain();
    ASSERT_TRUE(robots.session(second)->telemetry);
    EXPECT_EQ(robots.session(second)->telemetry->robotId, second);
    EXPECT_FALSE(robots.session(robot)->telemetry);
    EXPECT_FALSE(world.robot(second)->pose);
    ASSERT_TRUE(robots.removeRobot(robot)); drain();
    ASSERT_TRUE(robots.session(robot));
    emit f.backend->closed(firstToken); drain();
    EXPECT_FALSE(robots.session(robot));
    EXPECT_TRUE(robots.session(second)->telemetry);
    robots.shutdown(); drain(); emit f.backend->closed(secondToken); drain();
    EXPECT_TRUE(robots.isStopped());
}
#if defined(MICRO_ROBOT_BLE_WITH_DESKTOP)
TEST(BleTransportTest, ValidatedWireTelemetryAndErrorsReachDesktopServicesWithoutWidgets)
{
    Fixture f;
    WorldModel world;
    RobotManager robots{*f.transport, world};
    DesktopController desktop{robots, world, robot, f.logger.channel("desktop")};
    QString telemetry, error;
    QObject::connect(&desktop, &DesktopController::telemetryDescriptionChanged, &desktop,
        [&](const QString& text) { telemetry = text; });
    QObject::connect(&desktop, &DesktopController::errorOccurred, &desktop,
        [&](const QString& text) { error = text; });
    desktop.initialize(MonotonicClock::now());
    desktop.connectRobot(); drain();
    f.backend->found(); f.backend->finish(); drain();
    const auto token = f.backend->links.at(robot);
    emit f.backend->ready(token); drain();
    emit f.backend->notification(token, BleEndpoint::Telemetry, telemetryBytes(0), MonotonicClock::now()); drain();
    EXPECT_TRUE(telemetry.contains(QStringLiteral("3.91 V")));
    ASSERT_TRUE(world.robot(robot));
    EXPECT_FALSE(world.robot(robot)->pose); // Telemetry cannot invent an authoritative camera pose.
    auto malformed = telemetryBytes(1); malformed[0] = 2;
    emit f.backend->notification(token, BleEndpoint::Telemetry, malformed, MonotonicClock::now()); drain();
    EXPECT_TRUE(error.contains(QStringLiteral("Unsupported BLE wire version")));
    desktop.shutdown(); drain(); emit f.backend->closed(token); drain();
    EXPECT_EQ(f.transport->connectionState(robot), ConnectionState::Disconnected);
}
#endif
} // namespace
