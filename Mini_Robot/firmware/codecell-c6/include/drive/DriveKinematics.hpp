#pragma once
// Pure conversions of the bounded actuator contract (#160): body velocity -> wheel speed
// intent -> output fraction -> duty counts. Portable C++20, no state, no hardware.
//
// Signs (robot-protocol.md, differential-drive.md): positive linear velocity is forward,
// positive angular velocity is counter-clockwise seen from above, and a positive wheel
// speed rolls the robot forward on that side, whatever the motor wiring. Wiring is
// corrected only at the last step (polarity), and channel assignment after that.
#include "drive/DriveProfile.hpp"
#include "runtime/RuntimeTypes.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace codecell::drive {

// Commanded wheel surface speed intent in metres/second (open loop). Not a measurement.
struct WheelSpeeds {
    double leftMetresPerSecond{0.0};
    double rightMetresPerSecond{0.0};
};

// Signed PWM duty per H-bridge channel in counts of kDutyFullScale. Positive drives the
// channel's OUT1 (M+) high, negative drives OUT2 (M-) high, zero coasts the channel.
struct MotorDuty {
    std::int16_t m1{0};
    std::int16_t m2{0};

    friend constexpr bool operator==(const MotorDuty&, const MotorDuty&) = default;
};

// Differential-drive inverse kinematics on the wheel-centre track b:
//   left = v - omega b / 2,   right = v + omega b / 2.
[[nodiscard]] inline WheelSpeeds wheelSpeedsFor(const runtime::BodyVelocity& body, double trackWidthMetres) noexcept
{
    const double rotation = body.angularRadiansPerSecond * trackWidthMetres / 2.0;
    return {body.linearMetresPerSecond - rotation, body.linearMetresPerSecond + rotation};
}

// Wheel saturation: if either wheel exceeds the limit, both are scaled by the same factor,
// which keeps the signs and the path curvature (as the desktop toBoundedWheel* helpers do)
// and only slows the motion. Values within the limit are returned unchanged.
[[nodiscard]] inline WheelSpeeds saturate(const WheelSpeeds& wheels, double maxWheelSpeedMetresPerSecond) noexcept
{
    const double peak = std::max(std::abs(wheels.leftMetresPerSecond), std::abs(wheels.rightMetresPerSecond));
    if (peak <= maxWheelSpeedMetresPerSecond) return wheels;
    const double scale = maxWheelSpeedMetresPerSecond / peak;
    return {wheels.leftMetresPerSecond * scale, wheels.rightMetresPerSecond * scale};
}

// Initial bounded actuator map (PROVISIONAL, #167 calibrates it): linear and open loop,
// the provisioned wheel speed maps to the approved output bound and zero maps to zero.
// The result never exceeds the bound in magnitude, whatever the input.
[[nodiscard]] inline double outputFractionFor(double wheelSpeedMetresPerSecond, const DriveProfile& profile) noexcept
{
    const double share = std::clamp(wheelSpeedMetresPerSecond / profile.maxWheelSpeedMetresPerSecond, -1.0, 1.0);
    return share * profile.maxOutputFraction;
}

// Quantization with exact final enforcement: the result is within the profile's duty
// bound for every finite input (rounding cannot add a count). Polarity is applied here.
[[nodiscard]] inline std::int16_t dutyCountsFor(double outputFraction, bool reversed, const DriveProfile& profile) noexcept
{
    const std::int32_t bound = dutyBoundCounts(profile);
    const double signedFraction = reversed ? -outputFraction : outputFraction;
    const auto counts = static_cast<std::int32_t>(std::lround(signedFraction * kDutyFullScale));
    return static_cast<std::int16_t>(std::clamp(counts, -bound, bound));
}

// Final write for both wheels, channel-assigned. Callers check that the wheel values are
// finite first: a non-finite value must never reach this point.
[[nodiscard]] inline MotorDuty motorDutyFor(const WheelSpeeds& wheels, const DriveProfile& profile) noexcept
{
    const auto left = dutyCountsFor(outputFractionFor(wheels.leftMetresPerSecond, profile), profile.leftReversed, profile);
    const auto right =
        dutyCountsFor(outputFractionFor(wheels.rightMetresPerSecond, profile), profile.rightReversed, profile);
    MotorDuty duty;
    (kLeftWheelChannel == MotorChannel::M1 ? duty.m1 : duty.m2) = left;
    (kRightWheelChannel == MotorChannel::M1 ? duty.m1 : duty.m2) = right;
    return duty;
}
} // namespace codecell::drive
