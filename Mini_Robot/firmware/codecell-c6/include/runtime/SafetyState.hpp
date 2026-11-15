#pragma once
// Local safety state (#158). A plain value with pure transitions; it is NOT thread-safe.
// ActuatorGate owns the only instance and serializes every access in its critical section.
//
// Every transition that ends or replaces motion advances `generation`. A setpoint carries
// the generation it was admitted with, and the gate only drives for the current one, so a
// snapshot copied before Stop, expiry, E-stop, a fault or a session end can never turn the
// outputs back on. The fuller watchdog, stop-ramp enforcement and fault recovery are #162.
#include "runtime/RuntimeTypes.hpp"
#include <algorithm>
#include <cstdint>
#include <optional>

namespace codecell::runtime {

class SafetyState {
public:
    [[nodiscard]] MotionState state() const noexcept { return state_; }
    [[nodiscard]] FaultCode fault() const noexcept { return fault_; }
    [[nodiscard]] bool ready() const noexcept { return ready_; }
    [[nodiscard]] std::uint32_t generation() const noexcept { return generation_; }
    [[nodiscard]] std::optional<Instant> deadline() const noexcept { return deadline_; }

    // Startup completed with every task running. A faulted runtime never becomes ready.
    void markReady() noexcept
    {
        if (state_ != MotionState::Fault) ready_ = true;
    }

    // Receipt-relative deadline, capped by the ceiling; replaces any previous deadline.
    [[nodiscard]] CommandOutcome admitVelocity(Instant receipt, milliseconds validFor, milliseconds ceiling) noexcept
    {
        if (state_ == MotionState::Fault) return CommandOutcome::RejectedFault;
        if (state_ == MotionState::EmergencyStop) return CommandOutcome::RejectedEmergencyStopLatched;
        if (!ready_) return CommandOutcome::RejectedNotReady;
        if (validFor <= milliseconds::zero()) return CommandOutcome::RejectedInvalid;
        deadline_ = receipt + Instant{std::min(validFor, ceiling)};
        state_ = MotionState::Moving;
        advance();
        return CommandOutcome::Accepted;
    }

    // Stop, session end and expiry: Moving becomes Idle; the latch and faults are kept.
    void stop() noexcept
    {
        if (state_ == MotionState::Moving) state_ = MotionState::Idle;
        deadline_.reset();
        advance();
    }

    // Expiry exactly at the deadline (now >= deadline), as in the v1 freshness rules.
    [[nodiscard]] bool expireIfDue(Instant now) noexcept
    {
        if (state_ != MotionState::Moving || !deadline_ || now < *deadline_) return false;
        stop();
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
    // when not latched. A fault is not clearable by the desktop in v1.
    [[nodiscard]] CommandOutcome clearEmergencyStop() noexcept
    {
        if (state_ == MotionState::Fault) return CommandOutcome::RejectedFault;
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
        return ready_ && state_ == MotionState::Moving && admittedGeneration == generation_ && deadline_
            && now < *deadline_;
    }

private:
    void advance() noexcept { ++generation_; } // Unsigned wrap is well defined; only equality is used.

    MotionState state_{MotionState::Idle};
    FaultCode fault_{FaultCode::None};
    bool ready_{false};
    std::uint32_t generation_{0};
    std::optional<Instant> deadline_{};
};
} // namespace codecell::runtime
