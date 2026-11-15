#pragma once
// Bounded drive output (#160): the runtime's IDriveOutput for the CodeCell C6 Drive.
// Portable C++20; the only hardware access goes through IMotorDriver.
//
// Each normal step converts the body velocity to wheel speed intent, saturates both wheels
// together, applies the per-wheel rate and reversal limits, maps the result through the
// provisional linear actuator map and quantizes it with exact enforcement of the approved
// per-motor bound. There is no startup boost: no step can raise the output above what the
// limited intent maps to. disable() is the immediate path (coast, no ramp) and resets the
// ramps, so re-enabling always starts from zero.
//
// Thread contract: ActuatorGate calls command(), rampToStop() and disable() inside its
// critical section; configure() runs once in start() before any task. lastDuty() may be
// read from any task (lock-free atomics) for status reporting.
#include "drive/DriveKinematics.hpp"
#include "drive/DriveProfile.hpp"
#include "drive/MotorPorts.hpp"
#include "drive/WheelRateLimiter.hpp"
#include "runtime/RuntimePorts.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <optional>

namespace codecell::drive {

class BoundedDriveOutput final : public runtime::IDriveOutput {
public:
    explicit BoundedDriveOutput(IMotorDriver& motors) noexcept : motors_(motors) {}
    BoundedDriveOutput(const BoundedDriveOutput&) = delete;
    BoundedDriveOutput& operator=(const BoundedDriveOutput&) = delete;

    [[nodiscard]] bool configure(const DriveProfile& profile, milliseconds controlPeriod) noexcept override
    {
        if (validate(profile) != DriveProfileError::None || controlPeriod <= milliseconds::zero()) return false;
        profile_ = profile;
        maxStep_ = runtime::Instant{controlPeriod};
        const double wheelSpeed = profile.maxWheelSpeedMetresPerSecond;
        normal_ = {wheelSpeed / seconds(kNormalFullScaleRampTime), wheelSpeed / seconds(kNormalFullScaleRampTime)};
        // The stop ramp only ever shrinks the output, so it has no acceleration.
        stop_ = {0.0, wheelSpeed / seconds(kStopFullScaleRampTime)};
        configured_ = true;
        return motors_.enable();
    }

    [[nodiscard]] bool disable() noexcept override
    {
        left_.reset();
        right_.reset();
        lastStep_.reset();
        record({});
        return motors_.coast();
    }

    [[nodiscard]] bool command(const runtime::BodyVelocity& velocity, runtime::Instant now) noexcept override
    {
        if (!configured_ || !velocity.isFinite()) return false;
        const auto wheels = saturate(wheelSpeedsFor(velocity, profile_.trackWidthMetres),
                                     profile_.maxWheelSpeedMetresPerSecond);
        return stepToward(wheels, now, normal_);
    }

    [[nodiscard]] runtime::StopRampStep rampToStop(runtime::Instant now) noexcept override
    {
        if (!configured_ || !stepToward({}, now, stop_)) return runtime::StopRampStep::Refused;
        return left_.current() == 0.0 && right_.current() == 0.0 ? runtime::StopRampStep::AtRest
                                                                  : runtime::StopRampStep::Ramping;
    }

    // Last duty handed to the motor driver (zero after a disable), for status output.
    [[nodiscard]] MotorDuty lastDuty() const noexcept
    {
        return {static_cast<std::int16_t>(lastM1_.load(std::memory_order_relaxed)),
                static_cast<std::int16_t>(lastM2_.load(std::memory_order_relaxed))};
    }

    // Current wheel speed intent after the rate limits (commanded, not measured).
    [[nodiscard]] WheelSpeeds wheelIntent() const noexcept { return {left_.current(), right_.current()}; }

private:
    [[nodiscard]] static constexpr double seconds(runtime::Instant duration) noexcept
    {
        return std::chrono::duration<double>(duration).count();
    }

    // Elapsed time since the previous write, clamped to [0, one control period]: a late or
    // first write never earns a larger step, and two writes at one instant change nothing.
    [[nodiscard]] double elapsedSeconds(runtime::Instant now) const noexcept
    {
        if (!lastStep_) return seconds(maxStep_);
        return seconds(std::clamp(now - *lastStep_, runtime::Instant::zero(), maxStep_));
    }

    [[nodiscard]] bool stepToward(const WheelSpeeds& target, runtime::Instant now, const RampRates& rates) noexcept
    {
        if (!isFiniteValue(target.leftMetresPerSecond) || !isFiniteValue(target.rightMetresPerSecond)) return false;
        const double elapsed = elapsedSeconds(now);
        lastStep_ = now;
        const WheelSpeeds limited{left_.step(target.leftMetresPerSecond, elapsed, rates),
                                  right_.step(target.rightMetresPerSecond, elapsed, rates)};
        const auto duty = motorDutyFor(limited, profile_);
        if (!motors_.write(duty)) return false;
        record(duty);
        return true;
    }

    void record(const MotorDuty& duty) noexcept
    {
        lastM1_.store(duty.m1, std::memory_order_relaxed);
        lastM2_.store(duty.m2, std::memory_order_relaxed);
    }

    IMotorDriver& motors_;
    DriveProfile profile_{};
    bool configured_{false};
    runtime::Instant maxStep_{};
    RampRates normal_{};
    RampRates stop_{};
    WheelRateLimiter left_{};
    WheelRateLimiter right_{};
    std::optional<runtime::Instant> lastStep_{};
    // Word-sized so that plain loads and stores are lock-free on the RV32 C6.
    std::atomic<std::int32_t> lastM1_{0};
    std::atomic<std::int32_t> lastM2_{0};
};
} // namespace codecell::drive
