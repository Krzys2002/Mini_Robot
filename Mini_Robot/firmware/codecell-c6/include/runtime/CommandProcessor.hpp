#pragma once
// Command processing (#158): the only implementation of IMotionCommandSink. It turns
// decoded commands into safety transitions on the gate and, for accepted Velocity, a
// generation-tagged setpoint in the depth-1 mailbox. It runs in the caller's context
// (the BLE write callback in production), never blocks and never allocates.
#include "runtime/ActuatorGate.hpp"
#include "runtime/RuntimeDiagnostics.hpp"
#include "runtime/RuntimePorts.hpp"
#include "runtime/SetpointMailbox.hpp"

namespace codecell::runtime {

class CommandProcessor final : public IMotionCommandSink {
public:
    CommandProcessor(ActuatorGate& gate, SetpointMailbox& mailbox, RuntimeDiagnostics& diagnostics) noexcept
        : gate_(gate), mailbox_(mailbox), diagnostics_(diagnostics)
    {
    }

    // Set from the validated configuration before the runtime becomes ready.
    void setValidityCeiling(milliseconds ceiling) noexcept { ceiling_ = ceiling; }

    [[nodiscard]] CommandOutcome velocity(const BodyVelocity& velocity, milliseconds validFor,
                                          Instant receipt) noexcept override
    {
        if (!velocity.isFinite()) return reject(CommandOutcome::RejectedInvalid);
        std::uint32_t generation = 0;
        const auto outcome = gate_.admitVelocity(receipt, validFor, ceiling_, generation);
        if (outcome != CommandOutcome::Accepted) return reject(outcome);
        // A Stop/E-stop/fault between admission and publication advances the generation,
        // so this snapshot is then refused by the gate instead of driving.
        if (mailbox_.publish({velocity, generation})) diagnostics_.count(Counter::MailboxOverwrite);
        diagnostics_.count(Counter::VelocityAccepted);
        return outcome;
    }

    [[nodiscard]] CommandOutcome stop() noexcept override
    {
        gate_.stop();
        return CommandOutcome::Accepted;
    }

    [[nodiscard]] CommandOutcome emergencyStop() noexcept override
    {
        gate_.emergencyStop();
        diagnostics_.count(Counter::EmergencyStop);
        return CommandOutcome::Accepted;
    }

    [[nodiscard]] CommandOutcome clearEmergencyStop() noexcept override
    {
        const auto outcome = gate_.clearEmergencyStop();
        if (outcome != CommandOutcome::Accepted) diagnostics_.count(Counter::CommandRejected);
        return outcome;
    }

    void sessionEnded(SessionEndReason reason) noexcept override
    {
        gate_.stop();
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
};
} // namespace codecell::runtime
