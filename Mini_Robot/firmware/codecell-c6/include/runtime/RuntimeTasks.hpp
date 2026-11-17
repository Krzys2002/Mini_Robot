#pragma once
// The two periodic activities of the runtime (#158), scheduled by IPlatform:
//
// - SafetySupervisor, priority 22 every 5 ms (B3/B18): deadline expiry, stop-ramp
//   enforcement and the control heartbeat, independent of control progress. It is the only
//   task subscribed to the task watchdog (B17, #162).
// - ControlLoop (actuator update), priority 20 every 10 ms: optional local sensor read,
//   newest setpoint, one gated drive write.
//
// Both are plain steps so host tests can interleave them deterministically on a fake
// clock. Neither logs, allocates or waits; the platform owns the period.
#include "runtime/ActuatorGate.hpp"
#include "runtime/RuntimeDiagnostics.hpp"
#include "runtime/RuntimePorts.hpp"
#include "runtime/SetpointMailbox.hpp"
#include <optional>

namespace codecell::runtime {

class SafetySupervisor final : public IPeriodicStep {
public:
    SafetySupervisor(ActuatorGate& gate, const IFirmwareClock& clock) noexcept : gate_(gate), clock_(clock) {}
    void setStallLimit(milliseconds limit) noexcept { stallLimit_ = limit; }
    void step() noexcept override { gate_.supervise(clock_.nowMicroseconds(), stallLimit_); }

private:
    ActuatorGate& gate_;
    const IFirmwareClock& clock_;
    milliseconds stallLimit_{timing::kControlStallLimit};
};

class ControlLoop final : public IPeriodicStep {
public:
    ControlLoop(ActuatorGate& gate, SetpointMailbox& mailbox, const IFirmwareClock& clock,
                IInertialSensor* sensor, RuntimeDiagnostics& diagnostics) noexcept
        : gate_(gate), mailbox_(mailbox), clock_(clock), sensor_(sensor), diagnostics_(diagnostics)
    {
    }

    void step() noexcept override
    {
        if (sensor_) {
            if (auto sample = sensor_->read()) latestInertial_ = *sample;
            else diagnostics_.count(Counter::SensorUnavailable);
        }
        // Copy first, then read the time: a blocked sensor read or a preemption after the
        // copy shows up as control stall or as an older generation, never as fresh intent.
        const auto snapshot = mailbox_.latest();
        const auto now = clock_.nowMicroseconds();
        gate_.apply(snapshot, now);
    }

    // Local use on the robot only (control task context); never published as a pose.
    [[nodiscard]] const std::optional<InertialSample>& latestInertial() const noexcept { return latestInertial_; }

private:
    ActuatorGate& gate_;
    SetpointMailbox& mailbox_;
    const IFirmwareClock& clock_;
    IInertialSensor* sensor_;
    RuntimeDiagnostics& diagnostics_;
    std::optional<InertialSample> latestInertial_{};
};
} // namespace codecell::runtime
