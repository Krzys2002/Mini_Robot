#pragma once
// The single local safety/actuator authority (#158). It owns the SafetyState and is the
// only holder of the IDriveOutput. Every drive write and every safety transition happens
// inside one critical section, so a transition that ends motion (Stop, expiry, E-stop,
// fault, session end) and the drive write it implies are atomic with respect to the
// control task: once such a call returns, no write can re-enable the outputs for an
// older setpoint. Local safety therefore overrides every actuator write.
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
    bool outputEnabled{false};  // The last drive call was a permitted command, not a disable.
};

class ActuatorGate {
public:
    ActuatorGate(IDriveOutput& drive, RuntimeDiagnostics& diagnostics) noexcept
        : drive_(drive), diagnostics_(diagnostics)
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

    void markReady(Instant now) noexcept
    {
        CriticalSectionGuard guard(section_);
        lastControlUpdate_ = now;
        safety_.markReady();
    }

    // Returns the admission generation for the mailbox when accepted.
    [[nodiscard]] CommandOutcome admitVelocity(Instant receipt, milliseconds validFor, milliseconds ceiling,
                                               std::uint32_t& generation) noexcept
    {
        CriticalSectionGuard guard(section_);
        const auto outcome = safety_.admitVelocity(receipt, validFor, ceiling);
        generation = safety_.generation();
        return outcome;
    }

    void stop() noexcept
    {
        CriticalSectionGuard guard(section_);
        safety_.stop();
        disableLocked();
    }

    // The drive is disabled before the latch is visible to anyone else.
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

    void fault(FaultCode code) noexcept
    {
        CriticalSectionGuard guard(section_);
        faultLocked(code);
    }

    // Control task: the only path that can turn the outputs on. Without a permitted
    // snapshot (none, older generation, past its deadline, not Moving) it disables.
    void apply(const std::optional<SetpointSnapshot>& snapshot, Instant now) noexcept
    {
        CriticalSectionGuard guard(section_);
        lastControlUpdate_ = now;
        if (snapshot && safety_.permits(snapshot->generation, now)) {
            if (drive_.command(snapshot->velocity)) {
                outputEnabled_ = true;
                return;
            }
            faultLocked(FaultCode::DriveWriteFailed);
            return;
        }
        if (snapshot && outputEnabled_) diagnostics_.count(Counter::StaleSetpointBlocked);
        disableLocked();
    }

    // Safety supervisor: deadline expiry and control-stall detection, independent of
    // control progress. Returns true when it switched the outputs off.
    bool supervise(Instant now, milliseconds controlStallLimit) noexcept
    {
        CriticalSectionGuard guard(section_);
        if (safety_.expireIfDue(now)) {
            diagnostics_.count(Counter::CommandExpired);
            disableLocked();
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
        return {safety_.state(), safety_.fault(), safety_.ready(), outputEnabled_};
    }

private:
    bool disableLocked() noexcept
    {
        outputEnabled_ = false;
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
    ICriticalSection* section_{nullptr};
    SafetyState safety_{};
    Instant lastControlUpdate_{};
    bool outputEnabled_{false};
};
} // namespace codecell::runtime
