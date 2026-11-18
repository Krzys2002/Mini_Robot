#pragma once
// Local diagnostic report lines (#164): fixed-size `KEY key=value ...` text for the USB serial
// log, formatted by the lowest-priority reporter from values that other contexts publish
// atomically. Portable C++20 (snprintf only), so the host tests bound every line and check its
// keys.
//
// What the lines contain: reset and fault reasons, RobotId, firmware version and build, the
// session number, command/telemetry/BLE/log counters, queue and drop counts, task wake
// lateness and step times, stack and heap. What they never contain: a key, a token, a
// password or a network credential. The restricted-lab v1 profile has none (#157), BLE stores
// no bond keys (#163), and the image has no Wi-Fi (#158). The tests reject any key outside
// each line's allowlist, so a new field has to be added there deliberately.
#include "ble/PeripheralController.hpp"
#include "diagnostics/LogRing.hpp"
#include "protocol/V1Codec.hpp"
#include "runtime/ActuatorGate.hpp"
#include "runtime/RuntimeDiagnostics.hpp"
#include "runtime/TaskTiming.hpp"
#include "telemetry/BatteryMonitor.hpp"
#include "telemetry/TelemetryDiagnostics.hpp"
#include <array>
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <span>
#include <string_view>

namespace codecell::diagnostics {

// Every report line fits this, terminator excluded; the tests format worst-case values.
inline constexpr std::size_t kReportLineMax = 640;

struct ReportLine {
    std::array<char, kReportLineMax + 1> text{};
    std::size_t length{0};
    bool truncated{false};
    [[nodiscard]] std::string_view view() const noexcept { return {text.data(), length}; }
};

#if defined(__GNUC__)
__attribute__((format(printf, 2, 3)))
#endif
inline void appendf(ReportLine& line, const char* format, ...) noexcept
{
    if (line.truncated) return;
    const std::size_t room = line.text.size() - line.length;
    std::va_list args;
    va_start(args, format);
    const int written = std::vsnprintf(line.text.data() + line.length, room, format, args);
    va_end(args);
    if (written < 0) {
        line.truncated = true;
        return;
    }
    if (static_cast<std::size_t>(written) >= room) {
        line.truncated = true;
        line.length = line.text.size() - 1;
        return;
    }
    line.length += static_cast<std::size_t>(written);
}

using ULong = unsigned long;
[[nodiscard]] constexpr ULong ul(std::uint32_t value) noexcept { return static_cast<ULong>(value); }

// ---- RUNTIME: the local safety authority ----------------------------------------------
struct RuntimeReport {
    std::uint32_t uptimeMilliseconds{0};
    bool running{false};
    runtime::RuntimeStatus status{};
    int dutyM1{0};
    int dutyM2{0};
    const char* resetReason{"unknown"};
    const char* provisioning{"unknown"};
    std::optional<std::uint32_t> robotId{};
    runtime::DiagnosticsSnapshot counters{};
};

[[nodiscard]] inline ReportLine runtimeLine(const RuntimeReport& r) noexcept
{
    using runtime::Counter;
    const auto& s = r.status;
    const char* output = s.stopRamp ? "stop_ramp" : (s.outputEnabled ? "enabled" : "disabled");
    ReportLine line;
    appendf(line, "RUNTIME uptime_ms=%lu start=%s ready=%d state=%s output=%s duty_m1=%d duty_m2=%d fault=%s ",
            ul(r.uptimeMilliseconds), r.running ? "running" : "faulted", s.ready ? 1 : 0, runtime::toString(s.state),
            output, r.dutyM1, r.dutyM2, runtime::toString(s.fault));
    appendf(line, "maintenance=%d session=%d expired_flag=%d reset=%s provisioning=%s ", s.maintenance ? 1 : 0,
            s.sessionReady ? 1 : 0, s.commandExpired ? 1 : 0, r.resetReason, r.provisioning);
    if (r.robotId) appendf(line, "robot_id=%lu ", ul(*r.robotId));
    else appendf(line, "robot_id=none ");
    appendf(line, "accepted=%lu rejected=%lu expired=%lu overwrites=%lu estops=%lu sessions_opened=%lu "
                  "sessions_ended=%lu overloads=%lu stale_blocked=%lu",
            ul(r.counters[Counter::VelocityAccepted]), ul(r.counters[Counter::CommandRejected]),
            ul(r.counters[Counter::CommandExpired]), ul(r.counters[Counter::MailboxOverwrite]),
            ul(r.counters[Counter::EmergencyStop]), ul(r.counters[Counter::SessionOpened]),
            ul(r.counters[Counter::SessionEnded]), ul(r.counters[Counter::SessionOverload]),
            ul(r.counters[Counter::StaleSetpointBlocked]));
    return line;
}

// ---- BLE: link state and transport counters -------------------------------------------
[[nodiscard]] inline ReportLine bleLine(const ble::LinkStatus& link, const ble::BleDiagnosticsSnapshot& c) noexcept
{
    using C = ble::BleCounter;
    ReportLine line;
    appendf(line, "BLE state=%s cccd_result=%d cccd_telemetry=%d mtu=%u link_request=%s negotiated=",
            ble::toString(link.state), link.commandResultNotify ? 1 : 0, link.telemetryNotify ? 1 : 0,
            static_cast<unsigned>(link.mtu), ble::toString(link.linkRequest));
    if (link.negotiated)
        appendf(line, "interval_us=%ld latency=%u timeout_ms=%ld in_request=%d",
                static_cast<long>(ble::connectionInterval(*link.negotiated).count()),
                static_cast<unsigned>(link.negotiated->latency),
                static_cast<long>(ble::supervisionTimeout(*link.negotiated).count()),
                ble::withinRequest(*link.negotiated, ble::kLinkParameterRequest) ? 1 : 0);
    else appendf(line, "none");
    appendf(line, " last_disconnect=");
    if (link.lastDisconnectReason)
        appendf(line, "%d since_write_ms=%ld", *link.lastDisconnectReason,
                link.lastWriteBeforeDisconnect ? static_cast<long>(link.lastWriteBeforeDisconnect->count()) : -1L);
    else appendf(line, "none");
    appendf(line, " connections=%lu rejected_centrals=%lu sessions=%lu results=%lu result_failures=%lu unanswered=%lu "
                  "estop_writes=%lu foreign=%lu early_cccd=%lu adv_failures=%lu terminate_failures=%lu",
            ul(c[C::Connections]), ul(c[C::RejectedCentrals]), ul(c[C::SessionsReady]), ul(c[C::ResultsSent]),
            ul(c[C::ResultNotifyFailures]), ul(c[C::UnansweredCommandWrites]), ul(c[C::EmergencyStopWrites]),
            ul(c[C::ForeignEvents]), ul(c[C::EarlySubscriptions]), ul(c[C::AdvertisingFailures]),
            ul(c[C::TerminateFailures]));
    // Session security (#157 review): the pass-through reports "bypassed".
    appendf(line, " security=%s security_sessions=%lu security_refusals=%lu",
            !link.securitySession ? "none" : link.securityBypassed ? "bypassed" : "established",
            ul(c[C::SecuritySessions]), ul(c[C::SecurityRefusals]));
    return line;
}

// ---- TELEMETRY: publication, spacing and measurement availability ---------------------
struct TelemetryReport {
    std::uint32_t session{0};  // Sessions that became ready since boot; 0: none yet.
    std::chrono::milliseconds period{0};
    bool taskRunning{false};
    std::uint32_t ticksCoalesced{0};  // Ticks not posted because one was still pending in the host.
    telemetry::TelemetryCounters counters{};
    telemetry::TelemetryWindow window{};
    telemetry::BatteryAssessment battery{};
    bool batterySensor{false};
    std::optional<std::uint16_t> batteryRawMillivolts{};
    std::uint32_t batteryReadFailures{0};
};

inline void appendOptional(ReportLine& line, const char* key, const std::optional<std::uint32_t>& value) noexcept
{
    if (value) appendf(line, " %s=%lu", key, ul(*value));
    else appendf(line, " %s=none", key);
}

[[nodiscard]] inline ReportLine telemetryLine(const TelemetryReport& r) noexcept
{
    using T = telemetry::TelemetryCounter;
    const auto& c = r.counters;
    ReportLine line;
    appendf(line, "TELEMETRY session=%lu period_ms=%ld task=%s ticks=%lu not_ready=%lu coalesced=%lu sent=%lu "
                  "session_start=%lu state_change=%lu periodic=%lu notify_failures=%lu encode_failures=%lu",
            ul(r.session), static_cast<long>(r.period.count()), r.taskRunning ? "running" : "stopped", ul(c[T::Ticks]),
            ul(c[T::TicksNotReady]), ul(r.ticksCoalesced), ul(c.framesSent()), ul(c[T::SessionStartFrames]),
            ul(c[T::StateChangeFrames]), ul(c[T::PeriodicFrames]), ul(c[T::NotifyFailures]), ul(c[T::EncodeFailures]));
    appendOptional(line, "gap_min_us", r.window.minIntervalUs);
    appendOptional(line, "gap_max_us", r.window.maxIntervalUs);
    appendOptional(line, "periodic_dev_max_us", r.window.periodicDeviationMaxUs);
    appendf(line, " battery_mv=");
    if (r.battery.telemetryMillivolts) appendf(line, "%u", static_cast<unsigned>(*r.battery.telemetryMillivolts));
    else appendf(line, "unknown");
    appendf(line, " battery_quality=%s battery_low=%d battery_sensor=%s", telemetry::toString(r.battery.quality),
            r.battery.low ? 1 : 0, r.batterySensor ? "wired" : "none");
    const std::optional<std::uint32_t> raw =
        r.batteryRawMillivolts ? std::optional<std::uint32_t>{*r.batteryRawMillivolts} : std::nullopt;
    appendOptional(line, "battery_raw_mv", raw);
    appendf(line, " battery_read_failures=%lu velocity=unknown", ul(r.batteryReadFailures));
    return line;
}

// ---- TASK: one line per task ----------------------------------------------------------
struct TaskReport {
    const char* name{"unknown"};
    int priority{0};
    std::chrono::milliseconds period{0};  // 0: not a periodic task of the runtime.
    std::size_t stackBytes{0};            // Configured stack size; 0 when unknown.
    std::optional<std::size_t> stackFreeBytes{};  // High-water mark: least free stack since start.
    std::optional<runtime::TaskTimingSnapshot> timing{};
};

[[nodiscard]] inline ReportLine taskLine(const TaskReport& r) noexcept
{
    ReportLine line;
    appendf(line, "TASK name=%s prio=%d", r.name, r.priority);
    if (r.timing) {
        const auto& t = *r.timing;
        appendf(line, " period_ms=%ld steps=%lu overruns=%lu late_max_us=%lu late_worst_us=%lu step_max_us=%lu "
                      "step_worst_us=%lu",
                static_cast<long>(r.period.count()), ul(t.steps), ul(t.overruns), ul(t.latenessMaxUs),
                ul(t.latenessWorstUs), ul(t.stepMaxUs), ul(t.stepWorstUs));
    }
    if (r.stackFreeBytes) {
        appendf(line, " stack_free=%lu", static_cast<ULong>(*r.stackFreeBytes));
        if (r.stackBytes > 0)
            appendf(line, " stack_size=%lu stack_free_pct=%lu", static_cast<ULong>(r.stackBytes),
                    static_cast<ULong>(*r.stackFreeBytes * 100 / r.stackBytes));
    } else {
        appendf(line, " stack_free=unknown");
    }
    return line;
}

// ---- RESOURCES: heap and the log ring -------------------------------------------------
struct HeapReport {
    std::size_t freeBytes{0};
    std::size_t minimumFreeBytes{0};
    std::size_t largestFreeBlock{0};
};

[[nodiscard]] inline ReportLine resourcesLine(const HeapReport& heap, const LogStats& log) noexcept
{
    ReportLine line;
    appendf(line, "RESOURCES heap_free=%lu heap_min_free=%lu heap_largest=%lu log_lines=%lu log_dropped_lines=%lu "
                  "log_dropped_bytes=%lu log_written=%lu log_high_water=%lu log_capacity=%lu",
            static_cast<ULong>(heap.freeBytes), static_cast<ULong>(heap.minimumFreeBytes),
            static_cast<ULong>(heap.largestFreeBlock), ul(log.lines), ul(log.droppedLines), ul(log.droppedBytes),
            ul(log.writtenBytes), static_cast<ULong>(log.highWater), static_cast<ULong>(LogRing::kCapacity));
    return line;
}

// ---- IDENTITY: which robot, image and session the other lines belong to ----------------
struct IdentityReport {
    std::optional<std::uint32_t> robotId{};
    protocol::v1::FirmwareVersion firmware{};
    std::string_view build{"unknown"};  // ELF SHA-256 prefix of the running image.
    const char* resetReason{"unknown"};
    std::uint32_t session{0};
    std::uint32_t uptimeMilliseconds{0};
    std::string_view profile{ble::kSecurityProfileId};  // F10: the security profile of this image.
};

[[nodiscard]] inline ReportLine identityLine(const IdentityReport& r) noexcept
{
    ReportLine line;
    appendf(line, "IDENTITY");
    appendOptional(line, "robot_id", r.robotId);
    appendf(line, " firmware=%u.%u.%u build=%.*s profile=%.*s reset=%s session=%lu uptime_ms=%lu", r.firmware.major,
            r.firmware.minor, r.firmware.patch, static_cast<int>(r.build.size()), r.build.data(),
            static_cast<int>(r.profile.size()), r.profile.data(), r.resetReason, ul(r.session),
            ul(r.uptimeMilliseconds));
    return line;
}
} // namespace codecell::diagnostics
