#include "transport/ble/BleTransport.hpp"
#include "transport/ble/QtBleBackend.hpp"

#include <QCoreApplication>
#include <QElapsedTimer>
#include <QThread>
#include <gtest/gtest.h>

#include <functional>

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
    void found(const std::string& key = "opaque-a", const std::string& name = "MR-7")
    { emit deviceFound(scanToken, {key, name, {}, {}}); }
    void finish() { emit scanFinished(scanToken); }
    std::uint64_t scanToken{}, lastClosed{};
    std::size_t maximumDevices{};
    std::map<RobotId, std::uint64_t> links;
    std::vector<std::uint64_t> released;
    std::string lastKey;
    int scans{}, stops{}, opens{}, closes{};
};
class Events final : public IRobotTransportEvents {
public:
    BleTransport* transport{};
    std::vector<ConnectionState> states;
    std::vector<TransportError> errors;
    std::function<void(ConnectionState)> onState;
    int telemetry{};
    void onConnectionStateChanged(RobotId id, ConnectionState state, Timestamp) override
    {
        EXPECT_EQ(transport->connectionState(id), state);
        states.push_back(state);
        if (onState) onState(state);
    }
    void onTelemetry(const RobotTelemetry&) override { ++telemetry; }
    void onTransportError(const TransportError& error) override { errors.push_back(error); }
};
struct Fixture {
    FakeBleBackend* backend{};
    std::unique_ptr<BleTransport> transport;
    std::shared_ptr<Events> events = std::make_shared<Events>();
    explicit Fixture(BleTransportConfig config = {})
    {
        auto implementation = std::make_unique<FakeBleBackend>();
        backend = implementation.get();
        transport = std::make_unique<BleTransport>(std::move(implementation), config);
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
TEST(BleTransportTest, CommandsExplicitlyFailWithoutWritesOrFakeTelemetry)
{
    Fixture f;
    RobotCommand command;
    f.transport->sendCommand(robot, command);
    EXPECT_TRUE(f.events->errors.empty()); drain();
    ASSERT_EQ(f.events->errors.size(), 1U);
    EXPECT_EQ(f.events->errors.back().code, TransportErrorCode::NotConnected);
    f.ready();
    f.transport->sendCommand(robot, command); drain();
    EXPECT_EQ(f.events->errors.back().code, TransportErrorCode::Unavailable);
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
} // namespace
