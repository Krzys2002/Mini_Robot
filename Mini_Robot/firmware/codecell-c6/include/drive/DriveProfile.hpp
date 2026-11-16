#pragma once
// Validated drive profile of the bounded actuator contract (#160). Portable C++20: no
// Arduino, ESP-IDF, CodeCell or DriveCell types may appear here.
//
// Units are kept apart on purpose:
// - body velocity: SI metres/second and radians/second (runtime::BodyVelocity);
// - wheel speed: commanded wheel *surface* speed intent in metres/second, open loop. It
//   is never a measured wheel speed or body speed; the board has no wheel feedback (#154);
// - output fraction: signed share of the H-bridge supply at the actuator write, [-1, 1];
// - duty counts: the quantized PWM duty actually written, [-kDutyFullScale, kDutyFullScale].
//
// Every physical value comes from the provisioned record (#159), never from a default
// here: a default DriveProfile is invalid, so an unprovisioned robot cannot arm outputs.
#include "TimingBudget.hpp"
#include <chrono>
#include <cstdint>

namespace codecell::drive {
using std::chrono::milliseconds;

// H-bridge channels of the CodeCell C6 Drive. The vendor schematic (CodeCell library
// 2.0.4, hardware/CodeCell C6 Drive) names them by their motor outputs M1 (U7) and M2 (U8).
enum class MotorChannel : std::uint8_t { M1 = 0, M2 = 1 };

// Wheel to channel assignment. UNVERIFIED on the assembled robot (#238): it follows the
// retired prototype, whose joystick mixing drove M1 faster for a clockwise (right) turn,
// which makes M1 the left wheel. The powered rotation-sign check owns its confirmation.
inline constexpr MotorChannel kLeftWheelChannel = MotorChannel::M1;
inline constexpr MotorChannel kRightWheelChannel = MotorChannel::M2;
static_assert(kLeftWheelChannel != kRightWheelChannel, "each wheel needs its own bridge");

// PWM duty full scale at the actuator write: 8-bit LEDC resolution, as in DriveCell 1.1.3
// (DRIVE_TIMER), the reviewed vendor driver (#154).
inline constexpr std::int32_t kDutyFullScale = 255;

// Command-rate limits on the wheel speed intent. PROVISIONAL: there is no motor, wheel or
// chassis acceleration evidence (#154/#238), so the normal ramps reuse the #156 stop-ramp
// budget instead of inventing a value: a change of the full provisioned wheel speed takes
// at least this long. #167 refines it from calibration; it bounds the command only.
inline constexpr milliseconds kNormalFullScaleRampTime = timing::kStopRampMax;
// Stop, expiry and session end (#156 B6): from any output to zero within the budget.
inline constexpr milliseconds kStopFullScaleRampTime = timing::kStopRampMax;
static_assert(kStopFullScaleRampTime <= timing::kStopRampMax, "B6: the stop ramp must fit its budget");
static_assert(kNormalFullScaleRampTime >= kStopFullScaleRampTime,
              "normal deceleration must never be faster than the stop ramp");

// Relative tolerance for the reachability checks. The record (#159) checks the same
// rules exactly in integers; its SI conversion may differ from them by rounding only.
inline constexpr double kLimitRelativeTolerance = 1e-9;

struct DriveProfile {
    double trackWidthMetres{0.0};              // Wheel-centre distance.
    double maxWheelSpeedMetresPerSecond{0.0};  // Wheel surface speed intent mapped to maxOutputFraction.
    double maxOutputFraction{0.0};             // Approved per-motor |output| bound at the write, (0, 1].
    double maxLinearMetresPerSecond{0.0};      // Body command acceptance bound (advertised maxLinear).
    double maxAngularRadiansPerSecond{0.0};    // Body command acceptance bound (advertised maxAngular).
    bool leftReversed{false};                  // Wiring correction: positive wheel intent drives M- high.
    bool rightReversed{false};
};

enum class DriveProfileError : std::uint8_t {
    None = 0,
    NonFinite,
    TrackWidthNotPositive,
    WheelSpeedNotPositive,
    OutputBoundOutOfRange,       // Must be within (0, 1] and at least one duty count.
    LinearLimitOutOfRange,       // Must be within (0, maxWheelSpeed]: straight motion.
    AngularLimitOutOfRange,      // Must be within (0, 2 maxWheelSpeed / track]: rotation on the spot.
};

[[nodiscard]] constexpr const char* toString(DriveProfileError error) noexcept
{
    switch (error) {
    case DriveProfileError::None: return "None";
    case DriveProfileError::NonFinite: return "NonFinite";
    case DriveProfileError::TrackWidthNotPositive: return "TrackWidthNotPositive";
    case DriveProfileError::WheelSpeedNotPositive: return "WheelSpeedNotPositive";
    case DriveProfileError::OutputBoundOutOfRange: return "OutputBoundOutOfRange";
    case DriveProfileError::LinearLimitOutOfRange: return "LinearLimitOutOfRange";
    case DriveProfileError::AngularLimitOutOfRange: return "AngularLimitOutOfRange";
    }
    return "Unknown";
}

// constexpr finiteness test (std::isfinite is not constexpr in C++20): NaN fails the
// self-comparison and an infinity minus itself is NaN.
[[nodiscard]] constexpr bool isFiniteValue(double value) noexcept
{
    return value == value && value - value == 0.0;
}

// Largest duty count the profile may ever write on either channel.
[[nodiscard]] constexpr std::int32_t dutyBoundCounts(const DriveProfile& profile) noexcept
{
    return static_cast<std::int32_t>(profile.maxOutputFraction * kDutyFullScale);
}

[[nodiscard]] constexpr DriveProfileError validate(const DriveProfile& profile) noexcept
{
    for (const double value : {profile.trackWidthMetres, profile.maxWheelSpeedMetresPerSecond,
                               profile.maxOutputFraction, profile.maxLinearMetresPerSecond,
                               profile.maxAngularRadiansPerSecond}) {
        if (!isFiniteValue(value)) return DriveProfileError::NonFinite;
    }
    if (profile.trackWidthMetres <= 0.0) return DriveProfileError::TrackWidthNotPositive;
    if (profile.maxWheelSpeedMetresPerSecond <= 0.0) return DriveProfileError::WheelSpeedNotPositive;
    if (profile.maxOutputFraction <= 0.0 || profile.maxOutputFraction > 1.0 || dutyBoundCounts(profile) < 1)
        return DriveProfileError::OutputBoundOutOfRange;
    const double wheelLimit = profile.maxWheelSpeedMetresPerSecond * (1.0 + kLimitRelativeTolerance);
    if (profile.maxLinearMetresPerSecond <= 0.0 || profile.maxLinearMetresPerSecond > wheelLimit)
        return DriveProfileError::LinearLimitOutOfRange;
    if (profile.maxAngularRadiansPerSecond <= 0.0
        || profile.maxAngularRadiansPerSecond * profile.trackWidthMetres / 2.0 > wheelLimit)
        return DriveProfileError::AngularLimitOutOfRange;
    return DriveProfileError::None;
}

static_assert(validate(DriveProfile{}) != DriveProfileError::None,
              "no physical value is invented: a default profile must not arm the outputs");
} // namespace codecell::drive
