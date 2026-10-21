#include "MockRadioLink.hpp"
#include "simulation/SimulationTransport.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <type_traits>
#include <vector>

namespace micro_robot {
namespace {

using namespace std::chrono_literals;

// The same IRobotTransport contract checks run against every backend that can be
// exercised without hardware. A future radio adapter must satisfy them as well.
class SimulationHarness {
public:
    IRobotTransport& transport() { return transport_; }
    void makeReachable(RobotId) {}
    void produceTelemetry(RobotId) {} // Simulation emits telemetry on each advance.
    Timestamp settle()
    {
        now_ += 10ms;
        transport_.advanceTo(now_);
        return now_;
    }
    [[nodiscard]] Timestamp now() const { return now_; }

private:
    SimulationTransport transport_;
    Timestamp now_{1s};
};

class MockRadioHarness {
public:
    IRobotTransport& transport() { return link_; }
    void makeReachable(RobotId id)
    {
        link_.advertise(address(id), id, "MR-" + std::to_string(id.value()));
        link_.startScan();
        settle();
    }
    void produceTelemetry(RobotId id) { link_.receiveTelemetry(address(id), 3.7); }
    Timestamp settle()
    {
        now_ += 10ms;
        link_.deliver(now_);
        return now_;
    }
    [[nodiscard]] Timestamp now() const { return now_; }

private:
    static std::string address(RobotId id) { return "peer-" + std::to_string(id.value()); }

    testing::MockRadioLink link_;
    Timestamp now_{1s};
};

class ContractEvents final : public IRobotTransportEvents {
public:
    explicit ContractEvents(const IRobotTransport& transport) : transport_(transport) {}

    void onConnectionStateChanged(RobotId id, ConnectionState state, Timestamp at) override
    {
        states.emplace_back(id, state, at);
        stateMatchedQuery = stateMatchedQuery && transport_.connectionState(id) == state;
    }
    void onTelemetry(const RobotTelemetry& value) override { telemetry.push_back(value); }
    void onTransportError(const TransportError& value) override { errors.push_back(value); }

    [[nodiscard]] std::size_t count(ConnectionState state) const
    {
        std::size_t total = 0;
        for (const auto& [id, value, at] : states) {
            (void)id;
            (void)at;
            total += value == state ? 1 : 0;
        }
        return total;
    }

    std::vector<std::tuple<RobotId, ConnectionState, Timestamp>> states;
    std::vector<RobotTelemetry> telemetry;
    std::vector<TransportError> errors;
    bool stateMatchedQuery{true};

private:
    const IRobotTransport& transport_;
};

template <typename Harness>
class TransportContractTest : public ::testing::Test {
protected:
    TransportContractTest() { transport.setEventSink(events); }

    void connectRobot()
    {
        harness.makeReachable(id);
        transport.connect(id);
        harness.settle();
        ASSERT_EQ(transport.connectionState(id), ConnectionState::Connected);
    }
    [[nodiscard]] RobotCommand command(VelocityCommand velocity) const
    {
        return {velocity, harness.now(), 1s};
    }

    Harness harness;
    IRobotTransport& transport{harness.transport()};
    std::shared_ptr<ContractEvents> events{std::make_shared<ContractEvents>(transport)};
    const RobotId id{7};
};

using Backends = ::testing::Types<SimulationHarness, MockRadioHarness>;
class BackendNames {
public:
    template <typename T>
    static std::string GetName(int)
    {
        return std::is_same_v<T, SimulationHarness> ? "Simulation" : "MockRadio";
    }
};
TYPED_TEST_SUITE(TransportContractTest, Backends, BackendNames);

TYPED_TEST(TransportContractTest, UnknownRobotsAreDisconnectedAndConnectIsDeferred)
{
    EXPECT_EQ(this->transport.connectionState(RobotId{99}), ConnectionState::Disconnected);
    this->harness.makeReachable(this->id);
    this->transport.connect(this->id);
    this->transport.connect(this->id);
    EXPECT_TRUE(this->events->states.empty());

    const auto at = this->harness.settle();
    EXPECT_EQ(this->transport.connectionState(this->id), ConnectionState::Connected);
    EXPECT_EQ(this->events->count(ConnectionState::Connected), 1U);
    EXPECT_EQ(std::get<2>(this->events->states.back()), at);
    EXPECT_TRUE(this->events->stateMatchedQuery);
    this->harness.settle();
    EXPECT_EQ(this->events->count(ConnectionState::Connected), 1U);
    EXPECT_EQ(this->transport.connectionState(RobotId{99}), ConnectionState::Disconnected);
}

TYPED_TEST(TransportContractTest, DisconnectCancelsAPendingConnect)
{
    this->harness.makeReachable(this->id);
    this->transport.connect(this->id);
    this->transport.disconnect(this->id);
    this->harness.settle();
    EXPECT_EQ(this->events->count(ConnectionState::Connected), 0U);
    EXPECT_EQ(this->transport.connectionState(this->id), ConnectionState::Disconnected);
}

TYPED_TEST(TransportContractTest, DisconnectEndsInADisconnectedState)
{
    this->connectRobot();
    this->transport.disconnect(this->id);
    EXPECT_EQ(this->transport.connectionState(this->id), ConnectionState::Connected);
    this->harness.settle();
    EXPECT_EQ(this->transport.connectionState(this->id), ConnectionState::Disconnected);
    EXPECT_EQ(std::get<1>(this->events->states.back()), ConnectionState::Disconnected);
    EXPECT_TRUE(this->events->stateMatchedQuery);
}

TYPED_TEST(TransportContractTest, CommandsToDisconnectedRobotsReportNotConnected)
{
    this->transport.sendCommand(this->id, this->command({0.1, 0.0}));
    EXPECT_TRUE(this->events->errors.empty());
    const auto at = this->harness.settle();
    ASSERT_EQ(this->events->errors.size(), 1U);
    const auto& error = this->events->errors.front();
    EXPECT_EQ(error.robotId, this->id);
    EXPECT_EQ(error.operation, TransportOperation::SendCommand);
    EXPECT_EQ(error.code, TransportErrorCode::NotConnected);
    EXPECT_EQ(error.occurredAt, at);
}

TYPED_TEST(TransportContractTest, InvalidAndUnsupportedCommandsAreRejectedAsynchronously)
{
    this->connectRobot();
    this->transport.sendCommand(this->id,
        this->command({std::numeric_limits<double>::quiet_NaN(), 0.0}));
    EXPECT_TRUE(this->events->errors.empty());
    this->harness.settle();

    auto expired = this->command({0.1, 0.0});
    expired.validFor = Duration::zero();
    this->transport.sendCommand(this->id, expired);
    this->harness.settle();

    auto unsupported = this->command({0.1, 0.0});
    unsupported.metadata.modelVersion = currentTransportModelVersion + 1;
    this->transport.sendCommand(this->id, unsupported);
    this->harness.settle();

    ASSERT_EQ(this->events->errors.size(), 3U);
    EXPECT_EQ(this->events->errors[0].code, TransportErrorCode::InvalidCommand);
    EXPECT_EQ(this->events->errors[1].code, TransportErrorCode::InvalidCommand);
    EXPECT_EQ(this->events->errors[2].code, TransportErrorCode::UnsupportedVersion);
    for (const auto& error : this->events->errors) {
        EXPECT_EQ(error.operation, TransportOperation::SendCommand);
    }
    EXPECT_EQ(this->transport.connectionState(this->id), ConnectionState::Connected);
}

TYPED_TEST(TransportContractTest, ValidCommandsAreAcceptedAndTelemetryNamesTheRobot)
{
    this->connectRobot();
    this->transport.sendCommand(this->id, this->command({0.1, 0.2}));
    this->harness.produceTelemetry(this->id);
    this->harness.settle();
    EXPECT_TRUE(this->events->errors.empty());
    ASSERT_FALSE(this->events->telemetry.empty());
    const auto& telemetry = this->events->telemetry.back();
    EXPECT_EQ(telemetry.robotId, this->id);
    EXPECT_EQ(telemetry.receivedAt, this->harness.now());
    EXPECT_TRUE(isSupportedTransportModelVersion(telemetry.metadata.modelVersion));
}

TYPED_TEST(TransportContractTest, ExpiredSinksSuppressDeliveryWithoutChangingState)
{
    this->harness.makeReachable(this->id);
    EXPECT_EQ(this->events.use_count(), 1);
    this->events.reset();
    this->transport.connect(this->id);
    this->harness.settle();
    EXPECT_EQ(this->transport.connectionState(this->id), ConnectionState::Connected);

    auto replacement = std::make_shared<ContractEvents>(this->transport);
    this->transport.setEventSink(replacement);
    EXPECT_TRUE(replacement->states.empty()); // No inline replay.
    this->transport.disconnect(this->id);
    this->harness.settle();
    ASSERT_FALSE(replacement->states.empty());
    EXPECT_EQ(std::get<1>(replacement->states.back()), ConnectionState::Disconnected);
}

} // namespace
} // namespace micro_robot
