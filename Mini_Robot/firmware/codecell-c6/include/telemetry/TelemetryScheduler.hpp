#pragma once
// When the robot sends a v1 Telemetry frame (#164; robot-protocol.md "Telemetry", #156 B10).
// Portable and pure: no clock, radio or task. The BLE peripheral controller asks it once per
// telemetry tick, in the BLE host context, while the session is ready.
//
// Rules:
// - The first frame of a session goes out at the first tick after readiness.
// - A state change (the v1 state byte or the commandExpired / batteryLow flags) is sent at
//   the first tick that sees it, but never closer than the state spacing to the previous
//   frame. Changes in between coalesce: the frame carries the latest state (latest wins), so a
//   state that lasts less than a tick can be missed. Every command's resulting state is still
//   in its CommandResult.
// - Otherwise a frame is sent once the period has passed since the previous one. Measuring
//   from the previous frame keeps every gap at most one period (plus lateness), so the
//   desktop's silence threshold of three periods is never approached by the schedule itself.
// - Each check allows half a tick (timing::kTelemetryTickTolerance), because ticks arrive on
//   a fixed grid but late by scheduling lateness. So frames stay on that grid at the nominal
//   spacing, and no two frames are ever closer than timing::kTelemetryMinFrameSpacing, which
//   bounds the rate (B10: at most 60 per second) across sessions too.
// - A frame that could not be queued still counts as an attempt for the spacing, so a
//   congested link sees no retry burst. A state change that failed is tried again after the
//   spacing; the periodic frame after the next period. Nothing is buffered.
#include "TimingBudget.hpp"
#include "runtime/RuntimeTypes.hpp"
#include <cstdint>
#include <optional>

namespace codecell::telemetry {

using runtime::Instant;

// The telemetry fields whose change triggers a frame. lastAcceptedSequence is not one of
// them: it changes with every accepted command, and the CommandResult already reports it.
struct TelemetryKey {
    runtime::MotionState state{runtime::MotionState::Idle};
    bool commandExpired{false};
    bool batteryLow{false};
    friend constexpr bool operator==(const TelemetryKey&, const TelemetryKey&) = default;
};

enum class FrameReason : std::uint8_t { None, SessionStart, StateChange, Periodic };

class TelemetryScheduler {
public:
    // `period` is the advertised telemetryPeriodMs (validated equal to timing::kTelemetryPeriod).
    explicit constexpr TelemetryScheduler(std::chrono::milliseconds period) noexcept : period_(period) {}

    // A session became ready: its first frame is due. The time of the previous frame is kept,
    // so the minimum spacing holds across sessions as well.
    constexpr void sessionStarted() noexcept { lastSentKey_.reset(); }

    [[nodiscard]] constexpr FrameReason due(Instant now, const TelemetryKey& key) const noexcept
    {
        if (lastAttempt_) {
            const auto elapsed = now - *lastAttempt_;
            if (elapsed < timing::kTelemetryMinFrameSpacing) return FrameReason::None;
            if (!lastSentKey_) return FrameReason::SessionStart;
            if (key != *lastSentKey_) return FrameReason::StateChange;
            if (elapsed >= Instant{period_} - timing::kTelemetryTickTolerance) return FrameReason::Periodic;
            return FrameReason::None;
        }
        return lastSentKey_ ? FrameReason::None : FrameReason::SessionStart;
    }

    // A frame was attempted at `now` with `key`. `queued`: the stack accepted it.
    constexpr void attempted(Instant now, const TelemetryKey& key, bool queued) noexcept
    {
        lastAttempt_ = now;
        if (queued) lastSentKey_ = key;
    }

    [[nodiscard]] constexpr std::optional<Instant> lastAttempt() const noexcept { return lastAttempt_; }
    [[nodiscard]] constexpr std::chrono::milliseconds period() const noexcept { return period_; }

private:
    std::chrono::milliseconds period_;
    std::optional<Instant> lastAttempt_{};
    std::optional<TelemetryKey> lastSentKey_{};
};

// v1 uptimeMs: robot monotonic milliseconds since boot, modulo 2^32 (wraps after about 49.7
// days). Diagnostic only: neither side uses it for freshness or safety.
[[nodiscard]] constexpr std::uint32_t uptimeMilliseconds(Instant now) noexcept
{
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
    return static_cast<std::uint32_t>(static_cast<std::uint64_t>(ms));
}

[[nodiscard]] constexpr const char* toString(FrameReason reason) noexcept
{
    switch (reason) {
    case FrameReason::None: return "none";
    case FrameReason::SessionStart: return "session_start";
    case FrameReason::StateChange: return "state_change";
    case FrameReason::Periodic: return "periodic";
    }
    return "unknown";
}
} // namespace codecell::telemetry
