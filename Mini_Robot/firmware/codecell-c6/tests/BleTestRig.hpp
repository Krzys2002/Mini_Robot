#pragma once
// Shared host rig of the BLE peripheral tests (#163, #164): a fake radio link that records
// every notification, Telemetry Read value, termination, advertising and link-parameter
// request, with the real #158-#162 runtime behind the controller, so motion, the latch and
// session tokens are checked at the drive fake rather than at a recording authority.
#include "ProvisioningFakes.hpp"
#include "RuntimeFakes.hpp"
#include "ble/PeripheralController.hpp"
#include "protocol/V1CommandSession.hpp"
#include "provisioning/ProvisionedConfiguration.hpp"
#include "runtime/FirmwareRuntime.hpp"
#include "telemetry/BatteryMonitor.hpp"
#include "telemetry/TelemetryInputs.hpp"
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <optional>
#include <span>
#include <vector>

namespace codecell::ble::testrig {
using namespace codecell::protocol::v1;
using namespace codecell::runtime::fakes;
using namespace std::chrono_literals;
using runtime::Counter;
using runtime::Instant;
using runtime::MotionState;

inline constexpr ConnectionHandle kOwner = 1;
inline constexpr ConnectionHandle kSecond = 2;
inline constexpr int kRemoteUserTerminated = 0x13;

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
    bool failTelemetry{false};
    int telemetryAttempts{0};
    int telemetryClears{0};
    std::optional<Notification> telemetryValue{};  // The Telemetry characteristic's Read value.
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
    bool publishTelemetry(ConnectionHandle to, std::span<const std::uint8_t> frame) noexcept override
    {
        ++telemetryAttempts;
        if (failTelemetry || frame.size() > kMaxAttPayload) return false;
        Notification n{to, {}, frame.size()};
        std::copy(frame.begin(), frame.end(), n.bytes.begin());
        notifications.push_back(n);
        telemetryValue = n;
        return true;
    }
    void clearTelemetry() noexcept override
    {
        ++telemetryClears;
        telemetryValue.reset();
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
inline Result parse(const Notification& n)
{
    EXPECT_EQ(n.length, kCommandResultLength);
    EXPECT_EQ(n.bytes[kVersionAt], kProtocolVersion);
    EXPECT_EQ(n.bytes[kTypeAt], static_cast<std::uint8_t>(MessageType::CommandResult));
    const std::span<const std::uint8_t> b{n.bytes.data(), n.length};
    return {n.to, readU16(b, kSequenceAt), readU16(b, kResultCommandSequenceAt), n.bytes[kResultCommandTypeAt],
            static_cast<ResultCode>(n.bytes[kResultCodeAt]), n.bytes[kResultStateAt]};
}

inline std::vector<std::uint8_t> velocityFrame(std::uint16_t sequence, std::int16_t linear, std::uint16_t validFor)
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
inline std::vector<std::uint8_t> headerFrame(MessageType type, std::uint16_t sequence)
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
inline constexpr std::int16_t kForward = 100;
inline constexpr std::uint16_t kValidity = 200;

// The production inputs (no battery evidence, no velocity source) unless a test overrides them.
class OverridableInputs final : public telemetry::ITelemetryInputs {
public:
    explicit OverridableInputs(const telemetry::ITelemetryInputs& production) : production_(production) {}
    std::optional<telemetry::TelemetryMeasurements> override{};
    telemetry::TelemetryMeasurements measurements(Instant now) const noexcept override
    {
        return override ? *override : production_.measurements(now);
    }

private:
    const telemetry::ITelemetryInputs& production_;
};

struct Rig {
    FakeClock clock;
    FakePlatform platform;
    FakeDrive drive;
    FakeConfiguration configuration;
    FakeLink link;
    telemetry::BatteryMonitor battery{std::nullopt, nullptr};
    telemetry::RobotTelemetryInputs productionInputs{battery};
    OverridableInputs inputs{productionInputs};
    bool telemetryTicks{false};  // run() also delivers a telemetry tick every 5 ms.
    std::optional<runtime::FirmwareRuntime> runtime;
    std::optional<RuntimeView> view;
    RestrictedLabPassThroughSecurity passThrough;
    ISessionSecurity* security{&passThrough};
    SecurityMode securityMode{SecurityMode::RestrictedLabPassThrough};
    std::optional<PeripheralController> ble;

    // Default: the explicitly selected restricted-lab pass-through, as in the image.
    explicit Rig(bool startAdvertising = true, ISessionSecurity* custom = nullptr,
                 SecurityMode mode = SecurityMode::RestrictedLabPassThrough)
    {
        if (custom) security = custom;
        securityMode = mode;
        const auto config = provisioning::ValidatedRobotConfig::validated(provisioning::fakes::validConfig());
        EXPECT_TRUE(config);
        configuration.value = provisioning::runtimeConfigFor(*config);
        clock.time = 1s;
        drive.section = &platform.section;
        runtime.emplace(runtime::RuntimeDependencies{clock, platform, drive, configuration, nullptr});
        EXPECT_EQ(runtime->start(), runtime::StartResult::Running);
        view.emplace(*runtime);
        const auto info = provisioning::robotInfoValues(*config);
        ble.emplace(runtime->commands(), *view, commandLimits(info), link, clock, inputs,
                    std::chrono::milliseconds{info.telemetryPeriodMilliseconds}, *security, securityMode);
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
            if (telemetryTicks) ble->onTelemetryTick();
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
} // namespace codecell::ble::testrig
