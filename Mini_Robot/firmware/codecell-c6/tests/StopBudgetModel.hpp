#pragma once
// Test-only reference model of the #156 stopping contract. It is a specification
// oracle for the budget table, NOT firmware: #162 implements the real supervisor
// and can replay these timelines against it. Time is a fake monotonic clock in
// 64-bit microseconds; nothing here sleeps or reads a real clock.
#include "TimingBudget.hpp"
#include <algorithm>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace codecell::timing::model {
using Us = std::chrono::microseconds;

enum class Input { Velocity, Stop, EmergencyStop, Disconnect };
enum class State { Idle, Moving, EmergencyStop, Fault };
struct Event {
    Us at;                 // Callback entry (receipt) on the robot clock.
    Input type;
    milliseconds validFor{}; // Velocity only; already validated: 1 ms..ceiling.
};
struct Scenario {
    Us clockOffset{};                                    // Shifts every time (wrap tests).
    std::function<Us(std::int64_t)> supervisorLateness;  // Per check index, 0..allowance.
    std::function<Us(std::int64_t)> controlLateness;     // Per cycle index.
    std::vector<std::pair<Us, Us>> controlStalls;        // [from, to) relative to the offset.
    bool controlReplaysStaleOutput{false};               // Fault: keeps writing the old nonzero output.
};
struct Trace {
    std::optional<Us> expiryDecision; // Supervisor saw now >= deadline.
    std::optional<Us> outputZero;     // Control ramp reached zero command.
    std::optional<Us> driveDisabled;  // Electrical disable completed.
    State state{State::Idle};
    bool everNonzero{false};          // Output was commanded nonzero at least once.
    bool rejectedMotionWhileLatched{false};
    // Earliest time the output was zero or disabled after it last was nonzero.
    [[nodiscard]] std::optional<Us> outputOff() const
    {
        if (outputZero && driveDisabled) return std::min(*outputZero, *driveDisabled);
        return outputZero ? outputZero : driveDisabled;
    }
};

// Event-driven simulation until `until` (relative to the offset). All reported times
// are relative to the offset as well, so results must not depend on it.
inline Trace simulate(std::vector<Event> events, const Scenario& scenario, Us until)
{
    const Us base = scenario.clockOffset;
    std::sort(events.begin(), events.end(), [](const Event& a, const Event& b) { return a.at < b.at; });
    auto lateness = [](const std::function<Us(std::int64_t)>& f, std::int64_t index) {
        return f ? f(index) : Us::zero();
    };
    auto stalled = [&](Us t) {
        return std::any_of(scenario.controlStalls.begin(), scenario.controlStalls.end(),
                           [&](const auto& w) { return t >= base + w.first && t < base + w.second; });
    };
    Trace trace;
    std::optional<Us> deadline, stopDecision, rampStart;
    double output = 0.0;      // Commanded magnitude, 1 = moving.
    bool moveRequested = false;
    bool driveEnabled = true;
    Us lastControlUpdate = base;
    std::int64_t check = 1, cycle = 1;
    std::size_t next = 0;
    const Us end = base + until;
    auto disable = [&](Us at) {
        if (!driveEnabled) return;
        driveEnabled = false;
        output = 0.0;
        if (!trace.driveDisabled) trace.driveDisabled = at - base;
    };
    auto requestStop = [&](Us at) {
        moveRequested = false;
        deadline.reset();
        if (output > 0.0 && !stopDecision) stopDecision = at;
    };
    for (;;) {
        const Us checkAt = base + check * Us{kSafetyCheckPeriod} + lateness(scenario.supervisorLateness, check);
        const Us cycleAt = base + cycle * Us{kControlPeriod} + lateness(scenario.controlLateness, cycle);
        const Us eventAt = next < events.size() ? base + events[next].at : Us::max();
        const Us now = std::min({checkAt, cycleAt, eventAt});
        if (now > end) break;
        if (now == eventAt) {
            const auto& e = events[next++];
            switch (e.type) {
            case Input::Velocity:
                if (trace.state == State::EmergencyStop || trace.state == State::Fault) {
                    trace.rejectedMotionWhileLatched = true;
                    break;
                }
                deadline = now + Us{std::min(e.validFor, kCommandValidityCeiling)};
                moveRequested = true;
                stopDecision.reset();
                rampStart.reset();
                trace.state = State::Moving;
                break;
            case Input::Stop:
            case Input::Disconnect:
                requestStop(now);
                break;
            case Input::EmergencyStop: // Disabled inside the callback, ahead of any queue.
                disable(now + kEstopCallbackToDisableBound);
                moveRequested = false;
                deadline.reset();
                trace.state = State::EmergencyStop;
                break;
            }
        } else if (now == checkAt) {
            ++check;
            if (deadline && now >= *deadline) { // Expiry exactly at the deadline.
                if (!trace.expiryDecision) trace.expiryDecision = now - base;
                requestStop(now);
            }
            const bool nonzero = driveEnabled && output > 0.0;
            if (nonzero && stopDecision && now - *stopDecision >= kStopEnforcementLimit) {
                disable(now + kOutputDisableAllowance);
                trace.state = State::Fault;
            }
            if (nonzero && now - lastControlUpdate > kControlStallLimit) {
                disable(now + kOutputDisableAllowance);
                trace.state = State::Fault;
            }
        } else {
            ++cycle;
            if (stalled(now)) continue;
            lastControlUpdate = now;
            if (!driveEnabled) continue;
            if (moveRequested && !stopDecision) {
                output = 1.0;
                trace.everNonzero = true;
                trace.outputZero.reset();
            } else if (output > 0.0 && scenario.controlReplaysStaleOutput) {
                continue; // Alive (heartbeat updated) but never ramps: only enforcement stops it.
            } else if (output > 0.0) {
                if (!rampStart) rampStart = now;
                const double fraction = static_cast<double>((now - *rampStart).count())
                    / static_cast<double>(Us{kStopRampMax}.count());
                output = std::max(0.0, 1.0 - fraction);
                if (output == 0.0) {
                    trace.outputZero = now - base;
                    stopDecision.reset();
                    if (trace.state == State::Moving) trace.state = State::Idle;
                }
            }
        }
    }
    return trace;
}
} // namespace codecell::timing::model
