#pragma once
// Command processing (#158): the only implementation of IMotionCommandSink. It turns
// decoded commands into safety transitions on the gate and, for accepted Velocity, a
// generation-tagged setpoint in the depth-1 mailbox. It runs in the caller's context
// (the BLE callbacks in production, one context), never blocks and never allocates.
// Every transition that ends motion also empties the mailbox (#162): no old command
// stays buffered across Stop, E-stop, a session end or a session replacement.
#include "runtime/ActuatorGate.hpp"
#include "runtime/RuntimeDiagnostics.hpp"
#include "runtime/RuntimePorts.hpp"
#include "runtime/SetpointMailbox.hpp"
#include <cmath>

namespace codecell::runtime {

class CommandProcessor final : public IMotionCommandSink {
public:
    CommandProcessor(ActuatorGate& gate, SetpointMailbox& mailbox, RuntimeDiagnostics& diagnostics) noexcept
        : gate_(gate), mailbox_(mailbox), diagnostics_(diagnostics)
    {
    }

    // Set from the validated configuration before the runtime becomes ready.
    void setValidityCeiling(milliseconds ceiling) noexcept { ceiling_ = ceiling; }
    // Body limits from the validated drive profile (#160), the advertised maxLinear and
    // maxAngular. Until set, every nonzero velocity is out of range (and the runtime is
    // not ready anyway).
    void setVelocityLimits(double maxLinearMetresPerSecond, double maxAngularRadiansPerSecond) noexcept
    {
        maxLinear_ = maxLinearMetresPerSecond;
        maxAngular_ = maxAngularRadiansPerSecond;
    }

    [[nodiscard]] SessionToken openSession() noexcept override
    {
        const auto token = gate_.openSession();
        mailbox_.clear();
        diagnostics_.count(Counter::SessionOpened);
        return token;
    }

    [[nodiscard]] CommandOutcome velocity(SessionToken session, const BodyVelocity& velocity, milliseconds validFor,
                                          Instant receipt) noexcept override
    {
        if (!velocity.isFinite()) return reject(CommandOutcome::RejectedInvalid);
        // Invalid body commands are rejected here, before they can reach the actuator path.
        const bool withinLimits = std::abs(velocity.linearMetresPerSecond) <= maxLinear_
            && std::abs(velocity.angularRadiansPerSecond) <= maxAngular_;
        std::uint32_t generation = 0;
        const auto outcome = gate_.admitVelocity(session, receipt, validFor, ceiling_, withinLimits, generation);
        if (outcome != CommandOutcome::Accepted) return reject(outcome);
        // Production serializes command callbacks on the BLE host task (priority 21,
        // above control priority 20). Admission and publication must not be invoked
        // concurrently by multiple command producers.
        // A Stop/E-stop/fault between admission and publication advances the generation,
        // so this snapshot is then refused by the gate instead of driving.
        if (mailbox_.publish({velocity, generation})) diagnostics_.count(Counter::MailboxOverwrite);
        diagnostics_.count(Counter::VelocityAccepted);
        return outcome;
    }

    [[nodiscard]] CommandOutcome stop() noexcept override
    {
        gate_.stop();
        mailbox_.clear();
        return CommandOutcome::Accepted;
    }

    // The gate disables first; the mailbox is emptied only afterwards, off the critical path.
    [[nodiscard]] CommandOutcome emergencyStop() noexcept override
    {
        gate_.emergencyStop();
        mailbox_.clear();
        diagnostics_.count(Counter::EmergencyStop);
        return CommandOutcome::Accepted;
    }

    // The mailbox is already empty while latched: E-stop emptied it and every Velocity is
    // refused until this clear. A no-op clear while Moving must not touch the setpoint.
    [[nodiscard]] CommandOutcome clearEmergencyStop(SessionToken session) noexcept override
    {
        const auto outcome = gate_.clearEmergencyStop(session);
        if (outcome != CommandOutcome::Accepted) diagnostics_.count(Counter::CommandRejected);
        return outcome;
    }

    void sessionEnded(SessionToken session, SessionEndReason reason) noexcept override
    {
        if (!gate_.endSession(session)) return;  // Already ended or replaced: nothing left to stop.
        mailbox_.clear();
        diagnostics_.count(Counter::SessionEnded);
        if (reason == SessionEndReason::ResultQueueExhausted) diagnostics_.count(Counter::SessionOverload);
    }

private:
    CommandOutcome reject(CommandOutcome outcome) noexcept
    {
        diagnostics_.count(Counter::CommandRejected);
        return outcome;
    }

    ActuatorGate& gate_;
    SetpointMailbox& mailbox_;
    RuntimeDiagnostics& diagnostics_;
    milliseconds ceiling_{timing::kCommandValidityCeiling};
    double maxLinear_{0.0};
    double maxAngular_{0.0};
};
} // namespace codecell::runtime
