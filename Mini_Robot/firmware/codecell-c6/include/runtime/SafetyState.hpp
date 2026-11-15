#pragma once
// Local safety state (#158, #162). A plain value with pure transitions; it is NOT thread-safe.
// ActuatorGate owns the only instance and serializes every access in its critical section.
//
// Public states are the v1 Idle/Moving/EmergencyStop/Fault. Two internal readiness
// conditions gate motion on top of them:
// - start-up readiness (`ready`): set once start() completed with every task running; lost
//   for good on a fault or maintenance;
// - session readiness: one current SessionToken, issued when the BLE session becomes ready
//   and cleared when it ends. A new session replaces (and thereby ends) the previous one.
// Velocity is admitted only when both hold and the token is the current one.
//
// Every transition that ends or replaces motion advances `generation`. A setpoint carries
// the generation it was admitted with, and the gate only drives for the current one, so a
// snapshot copied before Stop, expiry, E-stop, a fault, a session end or a session
// replacement can never turn the outputs back on. The normal stop ramp and its enforcement
// live in ActuatorGate (#160); the task watchdog and fault power-down follow in #162.
#include "runtime/RuntimeTypes.hpp"
#include <algorithm>
#include <cstdint>
#include <optional>

namespace codecell::runtime {

// `start + window` on the 64-bit robot clock without signed overflow (#162). nullopt when the
// sum is not representable; the caller then refuses the command instead of wrapping it into
// the past or saturating it into a deadline that never comes. The esp_timer clock cannot get
// near this limit (about 292,000 years of uptime); the check exists so that no input can make
// the deadline arithmetic undefined.
[[nodiscard]] constexpr std::optional<Instant> deadlineAfter(Instant start, Instant window) noexcept
{
    if (window < Instant::zero()) return std::nullopt;
    if (start > Instant::max() - window) return std::nullopt;
    return start + window;
}

class SafetyState {
public:
    [[nodiscard]] MotionState state() const noexcept { return state_; }
    [[nodiscard]] FaultCode fault() const noexcept { return fault_; }
    [[nodiscard]] bool ready() const noexcept { return ready_; }
    [[nodiscard]] bool maintenance() const noexcept { return maintenance_; }
    [[nodiscard]] bool sessionReady() const noexcept { return session_ != 0; }
    [[nodiscard]] SessionToken session() const noexcept { return {session_}; }
    // v1 telemetry flag: set when the watchdog ended motion; cleared by the next accepted
    // Velocity or Stop, and at the start of a new session (the flag reports on that session).
    [[nodiscard]] bool commandExpired() const noexcept { return commandExpired_; }
    [[nodiscard]] std::uint32_t generation() const noexcept { return generation_; }
    [[nodiscard]] std::optional<Instant> deadline() const noexcept { return deadline_; }

    // Startup completed with every task running. A faulted runtime, or one that entered
    // maintenance, never becomes ready.
    void markReady() noexcept
    {
        if (state_ != MotionState::Fault && !maintenance_) ready_ = true;
    }

    // Local maintenance (#159): ends motion as for Stop and refuses new motion until a
    // reset. The E-stop latch and faults are kept; one-way by design, so provisioning
    // or a factory reset can never resume an earlier intent.
    void enterMaintenance() noexcept
    {
        maintenance_ = true;
        ready_ = false;
        endMotion();
    }

    // The session became ready. Any earlier session ends as for Stop (its token is no longer
    // current), the latch and faults are kept, and the new session starts without intent.
    [[nodiscard]] SessionToken openSession() noexcept
    {
        endMotion();
        commandExpired_ = false;
        if (++lastIssued_ == 0) ++lastIssued_;  // 0 is never issued; unsigned wrap is defined.
        session_ = lastIssued_;
        return {session_};
    }

    // Disconnect, notification disable or overload of the current session: stop as for Stop,
    // keep the latch. A token that is not current (already ended or replaced) changes nothing
    // and returns false: its intent was invalidated when it stopped being current.
    [[nodiscard]] bool endSession(SessionToken token) noexcept
    {
        if (!isCurrent(token)) return false;
        session_ = 0;
        endMotion();
        return true;
    }

    // Receipt-relative deadline, capped by the ceiling; replaces any previous deadline.
    // Checks follow the v1 order (state incl. readiness, validity, range); a rejection
    // changes nothing. `withinLimits` is the caller's body-limit check against the
    // provisioned profile. `now` is the gate's clock reading: a receipt stamped later than
    // `now` is not trusted beyond it, so no stamp can extend validity past `now + ceiling`.
    // An admitted command whose deadline has already passed is admitted (it supersedes the
    // previous one) and expires on the caller's next expireIfDue().
    [[nodiscard]] CommandOutcome admitVelocity(SessionToken token, Instant receipt, Instant now,
                                               milliseconds validFor, milliseconds ceiling,
                                               bool withinLimits) noexcept
    {
        if (state_ == MotionState::Fault) return CommandOutcome::RejectedFault;
        if (state_ == MotionState::EmergencyStop) return CommandOutcome::RejectedEmergencyStopLatched;
        if (!ready_) return CommandOutcome::RejectedNotReady;
        if (!isCurrent(token)) return CommandOutcome::RejectedSessionNotReady;
        if (validFor <= milliseconds::zero()) return CommandOutcome::RejectedInvalid;
        if (!withinLimits) return CommandOutcome::RejectedOutOfRange;
        const auto window = Instant{std::min(validFor, ceiling)};
        const auto deadline = deadlineAfter(std::min(receipt, now), window);
        if (!deadline) return CommandOutcome::RejectedInvalid;
        deadline_ = *deadline;
        state_ = MotionState::Moving;
        commandExpired_ = false;
        advance();
        return CommandOutcome::Accepted;
    }

    // The Stop command: Moving becomes Idle; the latch and faults are kept.
    void stop() noexcept
    {
        endMotion();
        commandExpired_ = false;
    }

    // Expiry exactly at the deadline (now >= deadline), as in the v1 freshness rules.
    [[nodiscard]] bool expireIfDue(Instant now) noexcept
    {
        if (state_ != MotionState::Moving || !deadline_ || now < *deadline_) return false;
        endMotion();
        commandExpired_ = true;
        return true;
    }

    // Any E-stop latches. A fault is more severe and stays (the drive is already off).
    void latchEmergencyStop() noexcept
    {
        if (state_ != MotionState::Fault) state_ = MotionState::EmergencyStop;
        deadline_.reset();
        advance();
    }

    // Releases the latch to Idle with no setpoint: a new Velocity is required. A no-op
    // when not latched. A fault is not clearable by the desktop in v1. Only the current
    // ready session may clear (v1: before ready only Stop and EmergencyStop take effect).
    [[nodiscard]] CommandOutcome clearEmergencyStop(SessionToken token) noexcept
    {
        if (state_ == MotionState::Fault) return CommandOutcome::RejectedFault;
        if (!isCurrent(token)) return CommandOutcome::RejectedSessionNotReady;
        if (state_ == MotionState::EmergencyStop) {
            state_ = MotionState::Idle;
            advance();
        }
        return CommandOutcome::Accepted;
    }

    // The first fault is kept for diagnosis; later faults only re-assert the state.
    void enterFault(FaultCode code) noexcept
    {
        if (fault_ == FaultCode::None) fault_ = code;
        state_ = MotionState::Fault;
        ready_ = false;
        deadline_.reset();
        advance();
    }

    // The single permission rule for a nonzero actuator write.
    [[nodiscard]] bool permits(std::uint32_t admittedGeneration, Instant now) const noexcept
    {
        return ready_ && session_ != 0 && state_ == MotionState::Moving && admittedGeneration == generation_
            && deadline_ && now < *deadline_;
    }

private:
    [[nodiscard]] bool isCurrent(SessionToken token) const noexcept
    {
        return token.value != 0 && token.value == session_;
    }
    // Stop, expiry, session end and replacement, maintenance: Moving becomes Idle.
    void endMotion() noexcept
    {
        if (state_ == MotionState::Moving) state_ = MotionState::Idle;
        deadline_.reset();
        advance();
    }
    void advance() noexcept { ++generation_; } // Unsigned wrap is well defined; only equality is used.

    MotionState state_{MotionState::Idle};
    FaultCode fault_{FaultCode::None};
    bool ready_{false};
    bool maintenance_{false};
    bool commandExpired_{false};
    std::uint32_t session_{0};
    std::uint32_t lastIssued_{0};
    std::uint32_t generation_{0};
    std::optional<Instant> deadline_{};
};
} // namespace codecell::runtime
