#pragma once
// The single local safety/actuator authority (#158). It owns the SafetyState and is the
// only holder of the IDriveOutput. Every drive write and every safety transition happens
// inside one critical section, so a transition that ends motion (Stop, expiry, E-stop,
// fault, session end) and the drive write it implies are atomic with respect to the
// control task: once such a call returns, no write can re-enable the outputs for an
// older setpoint. Local safety therefore overrides every actuator write.
//
// Two ways to end motion (#160, #156 B6), kept separate:
// - immediate disable (E-stop, faults, maintenance, shutdown, boot, latch clear): the drive
//   is switched off inside the call, without a ramp, before the call returns. Faults and
//   shutdown also power the drive stage down (IDriveOutput::powerDown, #162);
// - normal stop (Stop, expiry, session end or replacement, a refused snapshot): the gate records the
//   stop decision and each later control step can only move the commanded output toward
//   zero (IDriveOutput::rampToStop); at zero it disables. The supervisor faults and
//   disables if the output is not at zero within kStopEnforcementLimit of the decision,
//   and a stalled control task faults as before (B7). No ramp step ever uses an old
//   setpoint, and an immediate disable during a ramp ends it at once.
//
// Independent enforcement (#162): the supervisor task, not the control task, decides expiry,
// the stop-ramp limit and the control heartbeat. The supervisor itself is watched by the
// task watchdog (B17), so a stalled supervisor resets the chip into its hardware-safe boot.
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
    bool ready{false};           // Start-up readiness; false again after a fault, maintenance or shutdown.
    bool outputEnabled{false};   // The drive is enabled: a permitted command or a stop ramp, not a disable.
    bool maintenance{false};     // Motion refused until reset (#159).
    bool stopRamp{false};        // A normal stop ramp is in progress (#160).
    bool sessionReady{false};    // A session is open (#162); Velocity needs its token.
    bool commandExpired{false};  // v1 telemetry flag (#162): the watchdog ended motion.
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

    // Start-up only (single-threaded, before any task): hands the validated
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

    // Returns the admission generation for the mailbox when accepted. The clock is read
    // before the critical section; a later reading could only shorten the window. A command
    // whose receipt-relative deadline has already passed supersedes the previous one and
    // expires at once (now >= deadline): Idle, commandExpired, normal stop.
    [[nodiscard]] CommandOutcome admitVelocity(SessionToken session, Instant receipt, milliseconds validFor,
                                               milliseconds ceiling, bool withinLimits,
                                               std::uint32_t& generation) noexcept
    {
        const Instant now = clock_.nowMicroseconds();
        CriticalSectionGuard guard(section_);
        const auto outcome = safety_.admitVelocity(session, receipt, now, validFor, ceiling, withinLimits);
        if (outcome == CommandOutcome::Accepted && safety_.expireIfDue(now)) {
            diagnostics_.count(Counter::CommandExpired);
            beginStopLocked(now);
        }
        generation = safety_.generation();
        return outcome;
    }

    // Normal stop (the Stop command): bounded ramp to zero, see the header.
    void stop() noexcept
    {
        CriticalSectionGuard guard(section_);
        safety_.stop();
        beginStopLocked(clock_.nowMicroseconds());
    }

    // A ready session; the previous one ends as for Stop.
    [[nodiscard]] SessionToken openSession() noexcept
    {
        CriticalSectionGuard guard(section_);
        const auto token = safety_.openSession();
        beginStopLocked(clock_.nowMicroseconds());
        return token;
    }

    // The current session ended: normal stop. False (nothing changed) for any other token.
    [[nodiscard]] bool endSession(SessionToken session) noexcept
    {
        CriticalSectionGuard guard(section_);
        if (!safety_.endSession(session)) return false;
        beginStopLocked(clock_.nowMicroseconds());
        return true;
    }

    // Immediate: the drive is disabled before the latch is visible to anyone else,
    // whether or not a stop ramp is running.
    void emergencyStop() noexcept
    {
        CriticalSectionGuard guard(section_);
        disableLocked();
        safety_.latchEmergencyStop();
    }

    [[nodiscard]] CommandOutcome clearEmergencyStop(SessionToken session) noexcept
    {
        CriticalSectionGuard guard(section_);
        const bool latched = safety_.state() == MotionState::EmergencyStop;
        const auto outcome = safety_.clearEmergencyStop(session);
        // Idle with zero output; a new Velocity is required.
        if (latched && outcome == CommandOutcome::Accepted) disableLocked();
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

    // Before a deliberate restart (#162): ends motion and the session, refuses motion and
    // disables and powers down the drive. One-way; the next boot starts from scratch.
    void shutdown() noexcept
    {
        CriticalSectionGuard guard(section_);
        safety_.shutDown();
        disableLocked();
        drive_.powerDown();
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

    // Safety supervisor: deadline expiry, stop-ramp enforcement (B6) and the control
    // heartbeat (B7), independent of control progress. The heartbeat is checked while the
    // drive is on (B7) and also whenever the runtime is ready (#162): a dead control task is
    // a task failure even at rest, because commands would otherwise be accepted that can
    // never be executed. Returns true when it ended motion or faulted.
    bool supervise(Instant now, milliseconds controlStallLimit) noexcept
    {
        CriticalSectionGuard guard(section_);
        // Expiry does not end the check: a short command accepted before every period would
        // otherwise expire at each check and hide the ramp limit and the heartbeat below.
        const bool expired = safety_.expireIfDue(now);
        if (expired) {
            diagnostics_.count(Counter::CommandExpired);
            beginStopLocked(now);
        }
        if (stopping_ && now - stopDecision_ >= Instant{timing::kStopEnforcementLimit}) {
            faultLocked(FaultCode::StopRampOverrun);
            return true;
        }
        if ((outputEnabled_ || safety_.ready()) && now - lastControlUpdate_ > Instant{controlStallLimit}) {
            faultLocked(FaultCode::ControlStalled);
            return true;
        }
        return expired;
    }

    [[nodiscard]] RuntimeStatus status() const noexcept
    {
        CriticalSectionGuard guard(section_);
        return {safety_.state(),        safety_.fault(),        safety_.ready(),
                outputEnabled_,         safety_.maintenance(),  stopping_,
                safety_.sessionReady(), safety_.commandExpired()};
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

    // Immediate disable: no ramp, ends any running stop ramp. An unconfirmed disable is a
    // fault, and the power stage goes down as the last resort.
    bool disableLocked() noexcept
    {
        outputEnabled_ = false;
        stopping_ = false;
        if (drive_.disable()) return true;
        safety_.enterFault(FaultCode::DriveDisableFailed);
        drive_.powerDown();
        return false;
    }
    // Every fault: disable, then power down. Faults are cleared only by a reset (v1), so the
    // power stage stays down until the next boot.
    void faultLocked(FaultCode code) noexcept
    {
        safety_.enterFault(code);
        disableLocked();
        drive_.powerDown();
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
