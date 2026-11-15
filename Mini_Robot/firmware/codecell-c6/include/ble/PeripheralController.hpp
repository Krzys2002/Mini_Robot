#pragma once
// Session lifecycle of the v1 BLE peripheral (#163; contract: docs/protocol/ble-gatt.md).
// Portable C++20: the radio stack is behind IPeripheralLink, and the NimBLE adapter
// (src/NimBlePeripheral.hpp) only forwards its callbacks here. Host tests drive every
// callback ordering with fakes.
//
// Rules (restricted-lab v1 profile of #157, see ble/SessionPolicy.hpp):
// - One central per robot (F1). The first connection owns the link and advertising is
//   stopped explicitly; while the owner exists any other connection is terminated at once, and
//   its Command writes and subscriptions are never applied. Advertising restarts only after the
//   owner's disconnect has been processed. This holds whatever connection limit the stack was
//   built with (the pinned Arduino-ESP32 sdkconfig fixes NimBLE's at 3; see the firmware guide).
// - Readiness (F2): the session is ready only while the owner has notifications enabled on
//   both CCCDs. Becoming ready opens a session at the motion authority (#162 token, fresh
//   sequence state and transmit counter); losing either CCCD or the link ends it as for Stop,
//   keeps the RAM E-stop latch and never resumes motion. Repeated events are idempotent.
// - Before readiness, Command writes still go through the v1 session: Stop and EmergencyStop
//   take effect, Velocity and ClearEmergencyStop are refused by the authority (no ready
//   token), and no CommandResult is sent because there is no notification channel (v1).
// - Every Emergency Stop write latches first (F4): before ownership of an answer, readiness
//   or parsing is considered, whatever its length or content, from any connection.
// - While ready, exactly one CommandResult per Command or Emergency Stop write is queued from
//   inside the callback, on the session's shared robot transmit counter (#156 B11). If the
//   stack cannot queue one, the session ends as for Stop (ResultQueueExhausted) and the link
//   is terminated (#156 overload policy): no result is buffered here and none goes missing
//   silently while the session continues.
// - Link parameters are requested once per connection as hints (ble-gatt.md); the request
//   outcome and the negotiated values are reported, and nothing depends on them.
//
// Threading: every on*() and start() runs in the single BLE host callback context
// (nimble_host, priority 21) and never blocks, allocates or logs. status() and diagnostics()
// read atomics and may be called from any task; fields of one status() may come from
// different moments, so it is diagnostic only.
#include "FirmwareClock.hpp"
#include "ble/GattProfile.hpp"
#include "ble/SessionPolicy.hpp"
#include "protocol/V1CommandSession.hpp"
#include "runtime/RuntimePorts.hpp"
#include <array>
#include <atomic>
#include <chrono>
#include <limits>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace codecell::ble {

using ConnectionHandle = std::uint16_t;

// Radio stack boundary. Every call is non-blocking and made from the host callback context.
class IPeripheralLink {
public:
    virtual ~IPeripheralLink() = default;
    // Queue one CommandResult notification to that connection. False when the stack could not
    // queue it (no buffer, link gone); the controller then applies the overload policy.
    [[nodiscard]] virtual bool notifyCommandResult(ConnectionHandle connection,
                                                   std::span<const std::uint8_t> frame) noexcept = 0;
    // Request termination of that connection; completion arrives as onDisconnected().
    [[nodiscard]] virtual bool terminate(ConnectionHandle connection) noexcept = 0;
    // Start connectable advertising with the v1 advertising data and scan response.
    [[nodiscard]] virtual bool startAdvertising() noexcept = 0;
    // Stop advertising. True when it is not running afterwards (including when it was not).
    [[nodiscard]] virtual bool stopAdvertising() noexcept = 0;
    // Current ATT MTU of that connection; 0 when the stack does not know the connection.
    // Read at connect: the stack may report an MTU exchange before the connection itself.
    [[nodiscard]] virtual std::uint16_t attMtu(ConnectionHandle connection) const noexcept = 0;
    // Ask for the v1 link parameters. False when the stack refused to send the request.
    [[nodiscard]] virtual bool requestLinkParameters(ConnectionHandle connection,
                                                     const LinkParameterRequest& request) noexcept = 0;
};

enum class LinkState : std::uint8_t {
    Stopped,            // Not started (no validated identity, or before start()).
    Advertising,
    AdvertisingFailed,  // The stack refused to advertise: unreachable, outputs unaffected.
    Connected,          // One central owns the link; the session is not ready.
    Ready,              // Both CCCDs enabled: results are sent and motion may be admitted.
    Closing,            // Session ended for overload; termination requested, waiting for it.
};

enum class LinkRequestOutcome : std::uint8_t { None, Requested, Refused };

enum class BleCounter : std::uint8_t {
    Connections,              // Connections that became the owner.
    RejectedCentrals,         // Connections terminated because another central owned the link.
    Disconnects,              // Owner disconnects processed.
    SessionsReady,
    SessionsEnded,            // Ready sessions ended (CCCD disable, disconnect, overload).
    ResultsSent,
    ResultNotifyFailures,     // A CommandResult could not be queued (overload policy applied).
    UnansweredCommandWrites,  // Command writes while not ready: processed, no result (v1).
    EmergencyStopWrites,      // Every write to the Emergency Stop characteristic (each latched).
    ForeignEvents,            // Writes or subscriptions from a connection that does not own the link.
    AdvertisingStarts,
    AdvertisingFailures,      // Advertising could not be started or stopped when required.
    TerminateFailures,
    LinkRequestsRefused,
    Count_,
};

struct BleDiagnosticsSnapshot {
    std::array<std::uint32_t, static_cast<std::size_t>(BleCounter::Count_)> counts{};
    [[nodiscard]] std::uint32_t operator[](BleCounter counter) const noexcept
    {
        return counts[static_cast<std::size_t>(counter)];
    }
};

struct LinkStatus {
    LinkState state{LinkState::Stopped};
    bool commandResultNotify{false};
    bool telemetryNotify{false};
    std::uint16_t mtu{0};  // ATT MTU of the owner's link; 0 without one.
    LinkRequestOutcome linkRequest{LinkRequestOutcome::None};
    std::optional<LinkParameters> negotiated{};  // Latest values the stack reported for the owner.
    std::optional<int> lastDisconnectReason{};   // Stack reason code of the last owner disconnect.
    // Time from the owner's last write to its disconnect (#156 I10); empty without a write.
    std::optional<std::chrono::milliseconds> lastWriteBeforeDisconnect{};
};

class PeripheralController {
public:
    PeripheralController(runtime::IMotionCommandSink& commands, const protocol::v1::IRuntimeStatusView& runtime,
                         const protocol::v1::CommandLimits& limits, IPeripheralLink& link,
                         const IFirmwareClock& clock) noexcept
        : commands_(commands), link_(link), clock_(clock), session_(commands, runtime, limits)
    {
    }
    PeripheralController(const PeripheralController&) = delete;
    PeripheralController& operator=(const PeripheralController&) = delete;

    // Once the GATT server is registered: start advertising. Later calls do nothing.
    void start() noexcept
    {
        if (state() != LinkState::Stopped) return;
        advertise();
    }

    void onConnected(ConnectionHandle connection) noexcept
    {
        if (owner_) {
            if (*owner_ == connection) return;  // Repeated event for the owner.
            count(BleCounter::RejectedCentrals);
            if (!link_.terminate(connection)) count(BleCounter::TerminateFailures);
            return;
        }
        owner_ = connection;
        // Legacy connectable advertising ends when a connection is created; stopping it
        // explicitly keeps the robot invisible to a second central whatever the stack does.
        if (!link_.stopAdvertising()) count(BleCounter::AdvertisingFailures);
        resultNotify_ = false;
        telemetryNotify_ = false;
        lastWrite_.reset();
        publishSubscriptions();
        // An MTU exchange can be reported before this callback (it is then not the owner's
        // yet and is dropped), so start from what the stack already knows.
        const auto mtu = link_.attMtu(connection);
        mtu_.store(mtu != 0 ? mtu : kDefaultAttMtu, std::memory_order_relaxed);
        negotiatedIntervalUnits_.store(0, std::memory_order_relaxed);
        count(BleCounter::Connections);
        setState(LinkState::Connected);
        const bool requested = link_.requestLinkParameters(connection, kLinkParameterRequest);
        linkRequest_.store(requested ? LinkRequestOutcome::Requested : LinkRequestOutcome::Refused,
                           std::memory_order_relaxed);
        if (!requested) count(BleCounter::LinkRequestsRefused);
    }

    void onDisconnected(ConnectionHandle connection, int reason) noexcept
    {
        if (!owns(connection)) return;  // A refused newcomer, or already processed.
        endSession(runtime::SessionEndReason::Disconnected);
        disconnectReason_.store(reason, std::memory_order_relaxed);
        hasDisconnected_.store(true, std::memory_order_relaxed);
        writeBeforeDisconnectMs_.store(millisecondsSinceLastWrite(), std::memory_order_relaxed);
        owner_.reset();
        resultNotify_ = false;
        telemetryNotify_ = false;
        publishSubscriptions();
        mtu_.store(0, std::memory_order_relaxed);
        count(BleCounter::Disconnects);
        advertise();  // Only now: the session end above has already been applied.
    }

    // `notifications` is the CCCD notification bit; indications are not part of v1.
    void onSubscriptionChanged(ConnectionHandle connection, Characteristic characteristic,
                               bool notifications) noexcept
    {
        if (!owns(connection)) {
            count(BleCounter::ForeignEvents);
            return;
        }
        if (characteristic == Characteristic::CommandResult) resultNotify_ = notifications;
        else if (characteristic == Characteristic::Telemetry) telemetryNotify_ = notifications;
        else return;
        publishSubscriptions();
        updateReadiness();
    }

    void onCommandWrite(ConnectionHandle connection, std::span<const std::uint8_t> bytes) noexcept
    {
        const runtime::Instant receipt = clock_.nowMicroseconds();  // Receipt at callback entry (#156 I1).
        if (!owns(connection)) {
            count(BleCounter::ForeignEvents);  // F1: never applied.
            return;
        }
        lastWrite_ = receipt;
        const auto result = session_.onCommandWrite(bytes, receipt);
        if (state() != LinkState::Ready) {
            count(BleCounter::UnansweredCommandWrites);
            return;
        }
        send(result);
    }

    void onEmergencyStopWrite(ConnectionHandle connection, std::span<const std::uint8_t> bytes) noexcept
    {
        // F4: the latch is the first effect of every write. All that is decided before it is
        // whether an answer channel exists (two comparisons); the payload is not looked at.
        if (!owns(connection) || state() != LinkState::Ready) {
            (void)commands_.emergencyStop();  // Never rejected; the drive is off on return.
            count(BleCounter::EmergencyStopWrites);
            if (owns(connection)) lastWrite_ = clock_.nowMicroseconds();
            else count(BleCounter::ForeignEvents);
            return;
        }
        const auto result = session_.onEmergencyStopWrite(bytes);  // Latches before it parses.
        count(BleCounter::EmergencyStopWrites);
        lastWrite_ = clock_.nowMicroseconds();
        send(result);
    }

    void onLinkParametersUpdated(ConnectionHandle connection, const LinkParameters& parameters) noexcept
    {
        if (!owns(connection)) return;
        negotiatedLatency_.store(parameters.latency, std::memory_order_relaxed);
        negotiatedTimeoutUnits_.store(parameters.supervisionTimeoutUnits, std::memory_order_relaxed);
        negotiatedIntervalUnits_.store(parameters.intervalUnits, std::memory_order_relaxed);
    }

    void onMtuChanged(ConnectionHandle connection, std::uint16_t mtu) noexcept
    {
        if (owns(connection)) mtu_.store(mtu, std::memory_order_relaxed);
    }

    [[nodiscard]] LinkState state() const noexcept { return state_.load(std::memory_order_acquire); }

    [[nodiscard]] LinkStatus status() const noexcept
    {
        LinkStatus out;
        out.state = state();
        out.commandResultNotify = resultNotifyShown_.load(std::memory_order_relaxed);
        out.telemetryNotify = telemetryNotifyShown_.load(std::memory_order_relaxed);
        out.mtu = mtu_.load(std::memory_order_relaxed);
        out.linkRequest = linkRequest_.load(std::memory_order_relaxed);
        if (const auto interval = negotiatedIntervalUnits_.load(std::memory_order_relaxed); interval != 0)
            out.negotiated = LinkParameters{interval, negotiatedLatency_.load(std::memory_order_relaxed),
                                            negotiatedTimeoutUnits_.load(std::memory_order_relaxed)};
        if (hasDisconnected_.load(std::memory_order_relaxed)) {
            out.lastDisconnectReason = disconnectReason_.load(std::memory_order_relaxed);
            if (const auto ms = writeBeforeDisconnectMs_.load(std::memory_order_relaxed); ms != kNoWrite)
                out.lastWriteBeforeDisconnect = std::chrono::milliseconds{ms};
        }
        return out;
    }

    [[nodiscard]] BleDiagnosticsSnapshot diagnostics() const noexcept
    {
        BleDiagnosticsSnapshot out;
        for (std::size_t i = 0; i < out.counts.size(); ++i) out.counts[i] = counters_[i].load(std::memory_order_relaxed);
        return out;
    }

private:
    // 32-bit, so the published value stays a lock-free atomic on the 32-bit C6.
    static constexpr std::int32_t kNoWrite = -1;
    static constexpr std::int32_t kMaxReportedMilliseconds = std::numeric_limits<std::int32_t>::max();

    [[nodiscard]] std::int32_t millisecondsSinceLastWrite() const noexcept
    {
        if (!lastWrite_) return kNoWrite;
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(clock_.nowMicroseconds() - *lastWrite_);
        if (elapsed.count() < 0) return 0;
        return elapsed.count() > kMaxReportedMilliseconds ? kMaxReportedMilliseconds
                                                          : static_cast<std::int32_t>(elapsed.count());
    }

    [[nodiscard]] bool owns(ConnectionHandle connection) const noexcept { return owner_ && *owner_ == connection; }

    void updateReadiness() noexcept
    {
        const auto current = state();
        if (current != LinkState::Connected && current != LinkState::Ready) return;
        const bool ready = sessionMayBeReady(kSecurityProfile, {owner_.has_value(), current == LinkState::Closing,
                                                                resultNotify_, telemetryNotify_});
        if (ready && current == LinkState::Connected) {
            session_.open();
            setState(LinkState::Ready);
            count(BleCounter::SessionsReady);
        } else if (!ready && current == LinkState::Ready) {
            endSession(runtime::SessionEndReason::NotificationsDisabled);
            setState(LinkState::Connected);
        }
    }

    void send(const protocol::v1::CommandResultBytes& result) noexcept
    {
        if (link_.notifyCommandResult(*owner_, result)) {
            count(BleCounter::ResultsSent);
            return;
        }
        count(BleCounter::ResultNotifyFailures);
        // #156 overload policy: stop as for Stop, then terminate. The latch persists.
        endSession(runtime::SessionEndReason::ResultQueueExhausted);
        setState(LinkState::Closing);
        if (!link_.terminate(*owner_)) count(BleCounter::TerminateFailures);
    }

    // Ends the authority's session (stop as for Stop, latch kept) and restarts both sequence
    // counters. Safe when no session is open.
    void endSession(runtime::SessionEndReason reason) noexcept
    {
        const bool wasReady = state() == LinkState::Ready;
        session_.end(reason);
        if (wasReady) count(BleCounter::SessionsEnded);
    }

    void advertise() noexcept
    {
        if (link_.startAdvertising()) {
            count(BleCounter::AdvertisingStarts);
            setState(LinkState::Advertising);
            return;
        }
        count(BleCounter::AdvertisingFailures);
        setState(LinkState::AdvertisingFailed);
    }

    void publishSubscriptions() noexcept
    {
        resultNotifyShown_.store(resultNotify_, std::memory_order_relaxed);
        telemetryNotifyShown_.store(telemetryNotify_, std::memory_order_relaxed);
    }

    void setState(LinkState next) noexcept { state_.store(next, std::memory_order_release); }
    void count(BleCounter counter) noexcept
    {
        counters_[static_cast<std::size_t>(counter)].fetch_add(1, std::memory_order_relaxed);
    }

    runtime::IMotionCommandSink& commands_;
    IPeripheralLink& link_;
    const IFirmwareClock& clock_;
    protocol::v1::CommandSession session_;

    // Callback context only.
    std::optional<ConnectionHandle> owner_{};
    bool resultNotify_{false};
    bool telemetryNotify_{false};
    std::optional<runtime::Instant> lastWrite_{};

    // Published for status() and diagnostics().
    std::atomic<LinkState> state_{LinkState::Stopped};
    std::atomic<bool> resultNotifyShown_{false};
    std::atomic<bool> telemetryNotifyShown_{false};
    std::atomic<std::uint16_t> mtu_{0};
    std::atomic<LinkRequestOutcome> linkRequest_{LinkRequestOutcome::None};
    std::atomic<std::uint16_t> negotiatedIntervalUnits_{0};  // 0: none reported (never a valid interval).
    std::atomic<std::uint16_t> negotiatedLatency_{0};
    std::atomic<std::uint16_t> negotiatedTimeoutUnits_{0};
    std::atomic<bool> hasDisconnected_{false};
    std::atomic<int> disconnectReason_{0};
    std::atomic<std::int32_t> writeBeforeDisconnectMs_{kNoWrite};
    std::array<std::atomic<std::uint32_t>, static_cast<std::size_t>(BleCounter::Count_)> counters_{};
};

[[nodiscard]] constexpr const char* toString(LinkState state) noexcept
{
    switch (state) {
    case LinkState::Stopped: return "stopped";
    case LinkState::Advertising: return "advertising";
    case LinkState::AdvertisingFailed: return "advertising_failed";
    case LinkState::Connected: return "connected";
    case LinkState::Ready: return "ready";
    case LinkState::Closing: return "closing";
    }
    return "unknown";
}

[[nodiscard]] constexpr const char* toString(LinkRequestOutcome outcome) noexcept
{
    switch (outcome) {
    case LinkRequestOutcome::None: return "none";
    case LinkRequestOutcome::Requested: return "requested";
    case LinkRequestOutcome::Refused: return "refused";
    }
    return "unknown";
}

[[nodiscard]] constexpr const char* toString(BleCounter counter) noexcept
{
    switch (counter) {
    case BleCounter::Connections: return "connections";
    case BleCounter::RejectedCentrals: return "rejected_centrals";
    case BleCounter::Disconnects: return "disconnects";
    case BleCounter::SessionsReady: return "sessions_ready";
    case BleCounter::SessionsEnded: return "sessions_ended";
    case BleCounter::ResultsSent: return "results_sent";
    case BleCounter::ResultNotifyFailures: return "result_notify_failures";
    case BleCounter::UnansweredCommandWrites: return "unanswered_writes";
    case BleCounter::EmergencyStopWrites: return "estop_writes";
    case BleCounter::ForeignEvents: return "foreign_events";
    case BleCounter::AdvertisingStarts: return "advertising_starts";
    case BleCounter::AdvertisingFailures: return "advertising_failures";
    case BleCounter::TerminateFailures: return "terminate_failures";
    case BleCounter::LinkRequestsRefused: return "link_requests_refused";
    case BleCounter::Count_: break;
    }
    return "unknown";
}
} // namespace codecell::ble
