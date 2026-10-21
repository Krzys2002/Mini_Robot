#include "MockRadioLink.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <type_traits>
#include <utility>
#include <vector>

namespace micro_robot {
namespace {

using namespace std::chrono_literals;
using testing::MockRadioLink;

static_assert(std::is_abstract_v<IRobotDiscovery>);
static_assert(std::has_virtual_destructor_v<IRobotDiscovery>);
static_assert(std::has_virtual_destructor_v<IRobotDiscoveryEvents>);

class DiscoveryEvents final : public IRobotDiscoveryEvents {
public:
    explicit DiscoveryEvents(const IRobotDiscovery& discovery) : discovery_(discovery) {}

    void onScanStateChanged(RobotScanState state, Timestamp at) override
    {
        states.emplace_back(state, at);
        stateMatchedQuery = stateMatchedQuery && discovery_.scanState() == state;
        if (onState) {
            onState(state);
        }
    }
    void onRobotsChanged(std::vector<DiscoveredRobot> robots) override
    {
        snapshots.push_back(std::move(robots));
    }
    void onDiscoveryError(const RobotDiscoveryError& error) override { errors.push_back(error); }

    std::vector<std::pair<RobotScanState, Timestamp>> states;
    std::vector<std::vector<DiscoveredRobot>> snapshots;
    std::vector<RobotDiscoveryError> errors;
    bool stateMatchedQuery{true};
    std::function<void(RobotScanState)> onState;

private:
    const IRobotDiscovery& discovery_;
};

class TransportEvents final : public IRobotTransportEvents {
public:
    void onConnectionStateChanged(RobotId id, ConnectionState state, Timestamp) override
    {
        states.emplace_back(id, state);
    }
    void onTelemetry(const RobotTelemetry& value) override { telemetry.push_back(value); }
    void onTransportError(const TransportError& value) override { errors.push_back(value); }

    std::vector<std::pair<RobotId, ConnectionState>> states;
    std::vector<RobotTelemetry> telemetry;
    std::vector<TransportError> errors;
};

class RobotDiscoveryTest : public ::testing::Test {
protected:
    RobotDiscoveryTest()
    {
        discovery.setEventSink(events);
        transport.setEventSink(transportEvents);
    }

    Timestamp deliver()
    {
        now += 10ms;
        link.deliver(now);
        return now;
    }

    MockRadioLink link;
    IRobotDiscovery& discovery{link};
    IRobotTransport& transport{link};
    std::shared_ptr<DiscoveryEvents> events{std::make_shared<DiscoveryEvents>(link)};
    std::shared_ptr<TransportEvents> transportEvents{std::make_shared<TransportEvents>()};
    Timestamp now{1s};
};

TEST_F(RobotDiscoveryTest, ScanIsDeferredAndPublishesStateBeforeAnOrderedSnapshot)
{
    link.advertise("peer-b", RobotId{9}, "MR-9");
    link.advertise("peer-a", RobotId{7}, "MR-7");
    discovery.startScan();
    EXPECT_EQ(discovery.scanState(), RobotScanState::Idle);
    EXPECT_TRUE(events->states.empty());
    EXPECT_TRUE(events->snapshots.empty());

    const auto at = deliver();
    ASSERT_EQ(events->states.size(), 1U);
    EXPECT_EQ(events->states.front(), std::make_pair(RobotScanState::Scanning, at));
    EXPECT_TRUE(events->stateMatchedQuery);
    ASSERT_EQ(events->snapshots.size(), 1U);
    const auto& robots = events->snapshots.front();
    ASSERT_EQ(robots.size(), 2U);
    EXPECT_EQ(robots[0].robotId, RobotId{7});
    EXPECT_EQ(robots[0].displayName, "MR-7");
    EXPECT_EQ(robots[0].lastSeenAt, at);
    EXPECT_FALSE(robots[0].ambiguous);
    EXPECT_EQ(robots[1].robotId, RobotId{9});
    EXPECT_EQ(discovery.robots().size(), 2U);
    EXPECT_TRUE(events->errors.empty());
}

TEST_F(RobotDiscoveryTest, RequestsAreIdempotentAndStopKeepsTheInventory)
{
    discovery.startScan();
    discovery.stopScan();
    deliver();
    EXPECT_TRUE(events->states.empty());

    link.advertise("peer-a", RobotId{7}, "MR-7");
    discovery.startScan();
    discovery.startScan();
    deliver();
    discovery.startScan();
    deliver();
    ASSERT_EQ(events->states.size(), 1U);
    EXPECT_EQ(events->snapshots.size(), 1U); // Unchanged inventory is not republished.

    discovery.stopScan();
    discovery.stopScan();
    deliver();
    deliver();
    ASSERT_EQ(events->states.size(), 2U);
    EXPECT_EQ(events->states.back().first, RobotScanState::Idle);
    EXPECT_EQ(discovery.scanState(), RobotScanState::Idle);
    ASSERT_EQ(discovery.robots().size(), 1U);
    EXPECT_EQ(discovery.robots().front().robotId, RobotId{7});
}

TEST_F(RobotDiscoveryTest, SnapshotsAreCompleteAndFollowTheRadioEnvironment)
{
    link.advertise("peer-a", RobotId{7}, "MR-7");
    discovery.startScan();
    const auto first = deliver();

    link.advertise("peer-b", RobotId{3}, "MR-3");
    deliver();
    ASSERT_EQ(events->snapshots.size(), 2U);
    ASSERT_EQ(events->snapshots.back().size(), 2U);
    EXPECT_EQ(events->snapshots.back()[0].robotId, RobotId{3});
    EXPECT_EQ(events->snapshots.back()[1].robotId, RobotId{7});
    EXPECT_EQ(events->snapshots.back()[1].lastSeenAt, first);

    link.advertise("peer-a", RobotId{7}, "MR-7");
    const auto heardAgain = deliver();
    ASSERT_EQ(events->snapshots.size(), 3U);
    EXPECT_EQ(events->snapshots.back()[1].lastSeenAt, heardAgain);

    link.withdraw("peer-b");
    link.withdraw("peer-a");
    deliver();
    ASSERT_EQ(events->snapshots.size(), 4U);
    EXPECT_TRUE(events->snapshots.back().empty()); // An empty inventory is a result.
    EXPECT_TRUE(discovery.robots().empty());
}

TEST_F(RobotDiscoveryTest, NewScanReplacesThePreviousInventory)
{
    link.advertise("peer-a", RobotId{7}, "MR-7");
    link.advertise("peer-b", RobotId{8}, "MR-8");
    discovery.startScan();
    deliver();
    discovery.stopScan();
    deliver();
    link.withdraw("peer-a");
    EXPECT_EQ(discovery.robots().size(), 2U);

    discovery.startScan();
    deliver();
    ASSERT_EQ(events->snapshots.size(), 2U);
    ASSERT_EQ(events->snapshots.back().size(), 1U);
    EXPECT_EQ(events->snapshots.back().front().robotId, RobotId{8});
}

TEST_F(RobotDiscoveryTest, DuplicateClaimsAreReportedAndNeverResolvedByChoice)
{
    link.advertise("peer-a", RobotId{7}, "MR-7");
    link.advertise("peer-b", RobotId{7}, "MR-7");
    discovery.startScan();
    deliver();
    ASSERT_EQ(discovery.robots().size(), 1U);
    EXPECT_TRUE(discovery.robots().front().ambiguous);

    transport.connect(RobotId{7});
    deliver();
    EXPECT_EQ(transport.connectionState(RobotId{7}), ConnectionState::Fault);
    ASSERT_EQ(transportEvents->errors.size(), 1U);
    EXPECT_EQ(transportEvents->errors.front().operation, TransportOperation::Connect);
    EXPECT_EQ(transportEvents->errors.front().code, TransportErrorCode::Unavailable);
    EXPECT_FALSE(link.lastTransmitted("peer-a"));

    link.withdraw("peer-b");
    deliver();
    EXPECT_FALSE(discovery.robots().front().ambiguous);
    transport.connect(RobotId{7}); // Fault is quiescent; connect attempts recovery.
    deliver();
    EXPECT_EQ(transport.connectionState(RobotId{7}), ConnectionState::Connected);
}

TEST_F(RobotDiscoveryTest, UnavailableAdapterReportsErrorsWithoutCrashingOrScanning)
{
    link.setAdapterAvailable(false);
    discovery.startScan();
    const auto refused = deliver();
    EXPECT_TRUE(events->states.empty());
    ASSERT_EQ(events->errors.size(), 1U);
    EXPECT_EQ(events->errors.front().code, RobotDiscoveryErrorCode::Unavailable);
    EXPECT_EQ(events->errors.front().occurredAt, refused);
    deliver();
    EXPECT_EQ(events->errors.size(), 1U); // A refused start is not retried.

    link.setAdapterAvailable(true);
    discovery.startScan();
    deliver();
    link.setAdapterAvailable(false);
    deliver();
    ASSERT_EQ(events->states.size(), 2U);
    EXPECT_EQ(events->states.back().first, RobotScanState::Idle);
    EXPECT_EQ(discovery.scanState(), RobotScanState::Idle);
    EXPECT_EQ(events->errors.size(), 2U);
}

TEST_F(RobotDiscoveryTest, CallbacksMayStopAndSubscriptionsAreWeak)
{
    link.advertise("peer-a", RobotId{7}, "MR-7");
    events->onState = [&](RobotScanState state) {
        if (state == RobotScanState::Scanning) {
            discovery.stopScan();
        }
    };
    discovery.startScan();
    deliver();
    ASSERT_EQ(events->states.size(), 2U);
    EXPECT_EQ(events->states.back().first, RobotScanState::Idle);
    EXPECT_TRUE(events->snapshots.empty());

    EXPECT_EQ(events.use_count(), 1);
    events.reset();
    discovery.startScan();
    deliver(); // An expired sink suppresses delivery without changing the backend.
    EXPECT_EQ(discovery.scanState(), RobotScanState::Scanning);
    EXPECT_EQ(discovery.robots().size(), 1U);
}

TEST_F(RobotDiscoveryTest, TransportResolvesDiscoveredRobotsPrivately)
{
    link.advertise("peer-a", RobotId{7}, "MR-7");
    transport.connect(RobotId{7});
    deliver();
    // Connecting before any scan fails explicitly instead of guessing a peer.
    EXPECT_EQ(transport.connectionState(RobotId{7}), ConnectionState::Fault);
    ASSERT_EQ(transportEvents->errors.size(), 1U);
    EXPECT_EQ(transportEvents->errors.front().code, TransportErrorCode::Unavailable);

    discovery.startScan();
    transport.connect(RobotId{7});
    const auto at = deliver();
    ASSERT_GE(transportEvents->states.size(), 3U);
    EXPECT_EQ(transportEvents->states.back(), std::make_pair(RobotId{7}, ConnectionState::Connected));

    transport.sendCommand(RobotId{7}, {VelocityCommand{0.1, 0.2}, at, 1s});
    link.receiveTelemetry("peer-a", 3.6);
    link.receiveTelemetry("peer-unknown", 3.1);
    deliver();
    const auto sent = link.lastTransmitted("peer-a");
    ASSERT_TRUE(sent);
    EXPECT_DOUBLE_EQ(std::get<VelocityCommand>(sent->payload).angularRadiansPerSecond, 0.2);
    ASSERT_EQ(transportEvents->telemetry.size(), 1U);
    EXPECT_EQ(transportEvents->telemetry.front().robotId, RobotId{7});
    EXPECT_DOUBLE_EQ(*transportEvents->telemetry.front().batteryVoltageVolts, 3.6);
}

TEST_F(RobotDiscoveryTest, LinkLossPublishesFaultAndConnectRecovers)
{
    link.advertise("peer-a", RobotId{7}, "MR-7");
    discovery.startScan();
    transport.connect(RobotId{7});
    deliver();
    ASSERT_EQ(transport.connectionState(RobotId{7}), ConnectionState::Connected);

    link.loseLink("peer-a");
    const auto at = deliver();
    EXPECT_EQ(transport.connectionState(RobotId{7}), ConnectionState::Fault);
    ASSERT_EQ(transportEvents->errors.size(), 1U);
    EXPECT_EQ(transportEvents->errors.front().code, TransportErrorCode::BackendFailure);
    EXPECT_EQ(transportEvents->errors.front().occurredAt, at);
    transport.sendCommand(RobotId{7}, {StopCommand{}, at, 1s});
    deliver();
    ASSERT_EQ(transportEvents->errors.size(), 2U);
    EXPECT_EQ(transportEvents->errors.back().code, TransportErrorCode::NotConnected);

    transport.disconnect(RobotId{7}); // A quiescent Fault may be preserved.
    deliver();
    EXPECT_EQ(transport.connectionState(RobotId{7}), ConnectionState::Fault);
    transport.connect(RobotId{7});
    deliver();
    EXPECT_EQ(transport.connectionState(RobotId{7}), ConnectionState::Connected);
}

} // namespace
} // namespace micro_robot
