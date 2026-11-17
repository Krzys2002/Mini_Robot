#pragma once
// Motor HAL boundary of the bounded actuator contract (#160). The production adapter
// (src/CodeCellMotorDriver.hpp) is the only code that touches the H-bridge pins; host tests
// substitute a fake. BoundedDriveOutput calls these from the actuator gate's critical
// section, so implementations must be short, non-blocking writes: no allocation, lock,
// wait, logging or radio call on the success path.
#include "drive/DriveKinematics.hpp"

namespace codecell::drive {

class IMotorDriver {
public:
    virtual ~IMotorDriver() = default;
    // Wakes the bridges with every input low (coasting). Called once, after a valid profile
    // was accepted and before any task runs. False means the bridges could not be enabled.
    [[nodiscard]] virtual bool enable() noexcept = 0;
    // Immediate drive-disable without a ramp: every bridge input low, so both channels
    // coast (high-impedance outputs). Idempotent. False means it could not be confirmed.
    [[nodiscard]] virtual bool coast() noexcept = 0;
    // Writes both channels. |duty| <= kDutyFullScale; the caller has already enforced the
    // profile's tighter bound. Never brakes (both inputs high). False means refused.
    [[nodiscard]] virtual bool write(const MotorDuty& duty) noexcept = 0;
    // Puts both bridges to sleep (#162): outputs high-impedance whatever the inputs are,
    // independent of the PWM peripheral. Last resort for faults and shutdown; only a reset
    // (and a new enable()) wakes them again. Idempotent, a plain register write.
    virtual void sleep() noexcept = 0;
};
} // namespace codecell::drive
