#pragma once
// The single local safety/actuator authority (#158). It owns the SafetyState and is the
// only holder of the IDriveOutput. Every drive write and every safety transition happens
// inside one critical section, so a transition that ends motion (Stop, expiry, E-stop,
// fault, session end) and the drive write it implies are atomic with respect to the
// control task: once such a call returns, no write can re-enable the outputs for an
// older setpoint. Local safety therefore overrides every actuator write.
//
// Two ways to end motion (#160, #156 B6), kept separate:
// - immediate disable (E-stop, faults, maintenance, boot, latch clear): the drive is
//   switched off inside the call, without a ramp, before the call returns;
// - normal stop (Stop, expiry, session end, a refused snapshot): the gate records the
//   stop decision and each later control step can only move the commanded output toward
//   zero (IDriveOutput::rampToStop); at zero it disables. The supervisor faults and
//   disables if the output is not at zero within kStopEnforcementLimit of the decision,
//   and a stalled control task faults as before (B7). No ramp step ever uses an old
//   setpoint, and an immediate disable during a ramp ends it at once.
#include "FirmwareClock.hpp"
#include "runtime/RuntimeDiagnostics.hpp"
#include "runtime/RuntimePorts.hpp"
#include "runtime/SafetyState.hpp"
#include "runtime/SetpointMailbox.hpp"
#include <cstdint>
#include <optional>

namespace codecell::runtime {

struct RuntimeStatus {
    MotionState state{MotionState::Idle};
    FaultCode fault{FaultCode::None};
    bool ready{false};
    bool outputEnabled{false};  // The drive is enabled: a permitted command or a stop ramp, not a disable.
    bool maintenance{false};    // Motion refused until reset (#159).
    bool stopRamp{false};       // A normal stop ramp is in progress (#160).
};

class ActuatorGate {
public:
    ActuatorGate(IDriveOutput& drive, RuntimeDiagnostics& diagnostics, const IFirmwareClock& clock) noexcept
        : drive_(drive), diagnostics_(diagnostics), clock_(clock)
    {
    }
    ActuatorGate(const ActuatorGate&) = delete;
    ActuatorGate& operator=(const ActuatorGate&) = delete;

    // Single-threaded until bind(); the runtime binds before starting any task.
    void bind(ICriticalSection& section) noexcept { section_ = &section; }

    // First action at boot: outputs off before configuration, tasks or commands.
    [[nodiscard]] bool disableAtBoot() noexcept
    {
        CriticalSectionGuard guard(section_);
        return disableLocked();
    }

    // Start-up only (single-threaded, before bind() and any task): hands the validated
    // profile to the drive adapter. False means the adapter refused it.
    [[nodiscard]] bool configureDrive(const drive::DriveProfile& profile, milliseconds controlPeriod) noexcept
    {
        return drive_.configure(profile, controlPeriod);
    }

    void markReady(Instant now) noexcept
    {
        CriticalSectionGuard guard(section_);
        lastControlUpdate_ = now;
        safety_.markReady();
    }

    // Returns the admission generation for the mailbox when accepted.
    [[nodiscard]] CommandOutcome admitVelocity(Instant receipt, milliseconds validFor, milliseconds ceiling,
                                               bool withinLimits, std::uint32_t& generation) noexcept
    {
        CriticalSectionGuard guard(section_);
        const auto outcome = safety_.admitVelocity(receipt, validFor, ceiling, withinLimits);
        generation = safety_.generation();
        return outcome;
    }

    // Normal stop (Stop, session end): bounded ramp to zero, see the header.
    void stop() noexcept
    {
        CriticalSectionGuard guard(section_);
        safety_.stop();
        beginStopLocked(clock_.nowMicroseconds());
    }

    // Immediate: the drive is disabled before the latch is visible to anyone else,
    // whether or not a stop ramp is running.
    void emergencyStop() noexcept
    {
        CriticalSectionGuard guard(section_);
        disableLocked();
        safety_.latchEmergencyStop();
    }

    [[nodiscard]] CommandOutcome clearEmergencyStop() noexcept
    {
        CriticalSectionGuard guard(section_);
        const bool latched = safety_.state() == MotionState::EmergencyStop;
        const auto outcome = safety_.clearEmergencyStop();
        if (latched) disableLocked();  // Idle with zero output; a new Velocity is required.
        return outcome;
    }

    // Maintenance (#159): clears intent and disables the drive in one critical section,
    // before any configuration write. False when the disable could not be confirmed
    // (the runtime is then in Fault and the caller must not write configuration).
    [[nodiscard]] bool enterMaintenance() noexcept
    {
        CriticalSectionGuard guard(section_);
        safety_.enterMaintenance();
        return disableLocked();
    }

    void fault(FaultCode code) noexcept
    {
        CriticalSectionGuard guard(section_);
        faultLocked(code);
    }

    // Control task: the only path that can turn the outputs on. A permitted snapshot gets
    // one bounded step toward its velocity. Without one (none, older generation, past its
    // deadline, not Moving) an enabled drive ramps toward zero and anything else disables.
    void apply(const std::optional<SetpointSnapshot>& snapshot, Instant now) noexcept
    {
        CriticalSectionGuard guard(section_);
        lastControlUpdate_ = now;
        if (snapshot && safety_.permits(snapshot->generation, now)) {
            stopping_ = false;
            if (drive_.command(snapshot->velocity, now)) {
                outputEnabled_ = true;
                return;
            }
            faultLocked(FaultCode::DriveWriteFailed);
            return;
        }
        if (snapshot && outputEnabled_ && !stopping_) diagnostics_.count(Counter::StaleSetpointBlocked);
        beginStopLocked(now);
        if (stopping_) rampStepLocked(now);
    }

    // Safety supervisor: deadline expiry, stop-ramp enforcement (B6) and control-stall
    // detection (B7), independent of control progress. Returns true when it ended motion
    // (an expiry started the stop ramp, or a fault disabled the drive).
    bool supervise(Instant now, milliseconds controlStallLimit) noexcept
    {
        CriticalSectionGuard guard(section_);
        if (safety_.expireIfDue(now)) {
            diagnostics_.count(Counter::CommandExpired);
            beginStopLocked(now);
            return true;
        }
        if (stopping_ && now - stopDecision_ >= Instant{timing::kStopEnforcementLimit}) {
            faultLocked(FaultCode::StopRampOverrun);
            return true;
        }
        if (outputEnabled_ && now - lastControlUpdate_ > Instant{controlStallLimit}) {
            faultLocked(FaultCode::ControlStalled);
            return true;
        }
        return false;
    }

    [[nodiscard]] RuntimeStatus status() const noexcept
    {
        CriticalSectionGuard guard(section_);
        return {safety_.state(), safety_.fault(), safety_.ready(), outputEnabled_, safety_.maintenance(), stopping_};
    }

private:
    // Normal stop decision. A drive that is off simply stays off (immediate disable); an
    // enabled one starts the ramp, keeping the earliest decision time when one runs.
    void beginStopLocked(Instant now) noexcept
    {
        if (!outputEnabled_) {
            disableLocked();
            return;
        }
        if (stopping_) return;
        stopping_ = true;
        stopDecision_ = now;
    }

    void rampStepLocked(Instant now) noexcept
    {
        switch (drive_.rampToStop(now)) {
        case StopRampStep::Ramping: return;
        case StopRampStep::AtRest: disableLocked(); return;
        case StopRampStep::Refused: faultLocked(FaultCode::DriveWriteFailed); return;
        }
        faultLocked(FaultCode::DriveWriteFailed);  // Unknown step result: fail safe.
    }

    // Immediate disable: no ramp, ends any running stop ramp.
    bool disableLocked() noexcept
    {
        outputEnabled_ = false;
        stopping_ = false;
        if (drive_.disable()) return true;
        safety_.enterFault(FaultCode::DriveDisableFailed);
        return false;
    }
    void faultLocked(FaultCode code) noexcept
    {
        safety_.enterFault(code);
        disableLocked();
    }

    IDriveOutput& drive_;
    RuntimeDiagnostics& diagnostics_;
    const IFirmwareClock& clock_;
    ICriticalSection* section_{nullptr};
    SafetyState safety_{};
    Instant lastControlUpdate_{};
    Instant stopDecision_{};
    bool outputEnabled_{false};
    bool stopping_{false};
};
} // namespace codecell::runtime
