#pragma once
// CodeCell C6 Drive motor HAL (#160). With main.cpp and EspRuntimeAdapters.hpp, this is
// the only production code that touches the H-bridge pins; the portable bounded drive
// (include/drive/) is host-tested against a fake of this interface.
//
// Board facts, from the vendor schematic shipped in the pinned CodeCell 2.0.4 library
// (hardware/CodeCell C6 Drive/CodeCell C6 Drive Schematics.pdf, commit 795616b):
// - two TI DRV8837 H-bridges: U7 drives M1 (IN1 = GPIO21, IN2 = GPIO22) and U8 drives
//   M2 (IN1 = GPIO2, IN2 = GPIO3). CodeCell's Drive1(22, 21) names U7's pins the other
//   way round; this adapter uses the chip pin names;
// - both nSLEEP inputs are on the +3.3V_SENS rail, an LDO whose enable (SENS_PWR) is
//   GPIO18 with a 10 kOhm pull-down: through reset and boot the bridges are asleep and
//   their outputs high-impedance until firmware raises GPIO18. The same rail powers the
//   BNO085 and VCNL4040 sensors.
// DRV8837 IN/IN control: IN1 = IN2 = 0 coasts (outputs high-impedance), IN1 PWM with IN2 = 0
// drives OUT1 (M+) high, IN2 PWM with IN1 = 0 drives OUT2 (M-) high, and IN1 = IN2 = 1
// brakes. This adapter never brakes: zero and disable both coast.
//
// These are source facts, not measurements: pin levels during reset/boot, coasting,
// output timing and the polarity of the installed motors are physical checks (#238).
#include "drive/MotorPorts.hpp"
#include <Arduino.h>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <driver/gpio.h>

namespace codecell::esp {

inline constexpr std::uint8_t kM1In1Pin = 21;
inline constexpr std::uint8_t kM1In2Pin = 22;
inline constexpr std::uint8_t kM2In1Pin = 2;
inline constexpr std::uint8_t kM2In2Pin = 3;
// SENS_PWR: enables the +3.3V_SENS LDO, i.e. wakes both bridges (nSLEEP).
inline constexpr std::uint8_t kBridgeWakePin = 18;
// PWM as in the reviewed DriveCell 1.1.3 driver: 20 kHz (DRIVE_BASE_FREQ), 8 bit (DRIVE_TIMER).
// The DRV8837 accepts PWM up to 250 kHz. A new duty applies from the next PWM period (50 us).
inline constexpr std::uint32_t kPwmFrequencyHz = 20000;
inline constexpr std::uint8_t kPwmResolutionBits = 8;
static_assert((1 << kPwmResolutionBits) - 1 == drive::kDutyFullScale, "duty counts must match the LEDC resolution");

class CodeCellMotorDriver final : public drive::IMotorDriver {
public:
    // Call once from setup(), before the runtime starts. Attaches the four bridge inputs to
    // LEDC at zero duty (inputs low). The bridges stay asleep until enable(). False when a
    // pin could not be attached; coast() then reports failure and the runtime faults.
    [[nodiscard]] bool begin() noexcept
    {
        for (const auto pin : kInputPins) {
            if (!ledcAttach(pin, kPwmFrequencyHz, kPwmResolutionBits) || !ledcWrite(pin, 0)) return false;
        }
        attached_ = true;
        return true;
    }

    // Wakes both bridges with every input low (coasting). Runs once, after a valid profile.
    [[nodiscard]] bool enable() noexcept override
    {
        if (!attached_) return false;
        gpio_hold_dis(static_cast<gpio_num_t>(kBridgeWakePin));  // As CodeCell::Init does.
        pinMode(kBridgeWakePin, OUTPUT);
        digitalWrite(kBridgeWakePin, HIGH);
        return true;
    }

    // Immediate disable: all four inputs low, both bridges coast. Non-blocking LEDC writes.
    [[nodiscard]] bool coast() noexcept override
    {
        if (!attached_) return false;
        bool ok = true;
        for (const auto pin : kInputPins) ok = ledcWrite(pin, 0) && ok;
        return ok;
    }

    [[nodiscard]] bool write(const drive::MotorDuty& duty) noexcept override
    {
        if (!attached_) return false;
        const bool m1 = writeChannel(kM1In1Pin, kM1In2Pin, duty.m1);
        const bool m2 = writeChannel(kM2In1Pin, kM2In2Pin, duty.m2);
        return m1 && m2;
    }

private:
    // The input that must be low is written first, so both inputs are never high together.
    [[nodiscard]] static bool writeChannel(std::uint8_t in1, std::uint8_t in2, std::int16_t duty) noexcept
    {
        const int magnitude = std::abs(static_cast<int>(duty));
        if (magnitude > drive::kDutyFullScale) return false;
        const auto counts = static_cast<std::uint32_t>(magnitude);
        if (duty >= 0) return ledcWrite(in2, 0) && ledcWrite(in1, counts);
        return ledcWrite(in1, 0) && ledcWrite(in2, counts);
    }

    static constexpr std::array<std::uint8_t, 4> kInputPins{kM1In1Pin, kM1In2Pin, kM2In1Pin, kM2In2Pin};
    bool attached_{false};
};
} // namespace codecell::esp
