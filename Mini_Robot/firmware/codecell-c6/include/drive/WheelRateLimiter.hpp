#pragma once
// Per-wheel command-rate limiter of the bounded actuator contract (#160). Portable,
// deterministic and allocation-free. It limits the commanded wheel speed intent, not the
// physical wheel: without wheel feedback the real acceleration, slip and coasting are
// unknown (#154/#238).
//
// Rules for one step from the current intent toward a target over `elapsed`:
// - a growing magnitude changes by at most acceleration * elapsed;
// - a shrinking magnitude changes by at most deceleration * elapsed;
// - reversal: a target of the opposite sign is first treated as zero, so the step that
//   reaches zero stops exactly there and the opposite sign starts only on a later step.
#include <cmath>

namespace codecell::drive {

struct RampRates {
    double accelerationMetresPerSecondSquared{0.0};
    double decelerationMetresPerSecondSquared{0.0};
};

// Arrival tolerance: absorbs floating-point residue so that a ramp of N equal steps ends
// on step N instead of N + 1. Far below any physically meaningful wheel speed.
inline constexpr double kArrivalToleranceMetresPerSecond = 1e-12;

class WheelRateLimiter {
public:
    [[nodiscard]] double current() const noexcept { return current_; }
    void reset() noexcept { current_ = 0.0; }

    // `target` and `elapsedSeconds` must be finite; `elapsedSeconds` must be >= 0.
    double step(double target, double elapsedSeconds, const RampRates& rates) noexcept
    {
        if (current_ != 0.0 && target != 0.0 && std::signbit(current_) != std::signbit(target)) target = 0.0;
        const bool shrinking = std::abs(target) < std::abs(current_);
        const double rate = shrinking ? rates.decelerationMetresPerSecondSquared : rates.accelerationMetresPerSecondSquared;
        const double allowed = rate * elapsedSeconds;
        const double difference = target - current_;
        if (std::abs(difference) <= allowed + kArrivalToleranceMetresPerSecond) {
            current_ = target;
        } else {
            current_ += difference > 0.0 ? allowed : -allowed;
        }
        return current_;
    }

private:
    double current_{0.0};
};
} // namespace codecell::drive
