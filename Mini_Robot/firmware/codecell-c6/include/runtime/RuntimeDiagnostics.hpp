#pragma once
// Diagnostic counters of the runtime (#158). Lock-free 32-bit atomics: callable from any
// task or callback without blocking, allocation or serial output. Only the lowest-
// priority reporter (Arduino loopTask, the B13 log drain) formats and prints them.
// v1 telemetry has no counter fields (#164 does not extend the frozen frames): the counters are
// reported in the local RUNTIME line (diagnostics/StatusReport.hpp).
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace codecell::runtime {

enum class Counter : std::uint8_t {
    VelocityAccepted,
    CommandRejected,     // Any rejected Velocity or ClearEmergencyStop.
    MailboxOverwrite,    // A newer setpoint replaced one the control task had not read (B12).
    CommandExpired,
    EmergencyStop,
    SessionOpened,       // A session became ready (#162).
    SessionEnded,        // The current session ended (a stale end is not counted).
    SessionOverload,     // SessionEndReason::ResultQueueExhausted.
    SensorUnavailable,   // An optional inertial read returned nothing.
    StaleSetpointBlocked,// The gate refused a snapshot from an older generation or past its deadline.
    Count_,
};

struct DiagnosticsSnapshot {
    std::array<std::uint32_t, static_cast<std::size_t>(Counter::Count_)> counts{};
    [[nodiscard]] std::uint32_t operator[](Counter counter) const noexcept
    {
        return counts[static_cast<std::size_t>(counter)];
    }
};

class RuntimeDiagnostics {
public:
    void count(Counter counter) noexcept
    {
        counters_[static_cast<std::size_t>(counter)].fetch_add(1, std::memory_order_relaxed);
    }
    [[nodiscard]] DiagnosticsSnapshot snapshot() const noexcept
    {
        DiagnosticsSnapshot result;
        for (std::size_t i = 0; i < result.counts.size(); ++i)
            result.counts[i] = counters_[i].load(std::memory_order_relaxed);
        return result;
    }

private:
    std::array<std::atomic<std::uint32_t>, static_cast<std::size_t>(Counter::Count_)> counters_{};
};

[[nodiscard]] constexpr const char* toString(Counter counter) noexcept
{
    switch (counter) {
    case Counter::VelocityAccepted: return "velocity_accepted";
    case Counter::CommandRejected: return "command_rejected";
    case Counter::MailboxOverwrite: return "mailbox_overwrite";
    case Counter::CommandExpired: return "command_expired";
    case Counter::EmergencyStop: return "emergency_stop";
    case Counter::SessionOpened: return "session_opened";
    case Counter::SessionEnded: return "session_ended";
    case Counter::SessionOverload: return "session_overload";
    case Counter::SensorUnavailable: return "sensor_unavailable";
    case Counter::StaleSetpointBlocked: return "stale_setpoint_blocked";
    case Counter::Count_: break;
    }
    return "unknown";
}
} // namespace codecell::runtime
