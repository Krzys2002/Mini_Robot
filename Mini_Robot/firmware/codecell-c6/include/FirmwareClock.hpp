#pragma once
#if __cplusplus < 202002L
#error "CodeCell firmware requires C++20; use the pinned PlatformIO configuration"
#endif
#include <chrono>
#include <cstdint>
#include <optional>

namespace codecell {
using Milliseconds = std::chrono::milliseconds;
using Microseconds = std::chrono::microseconds;
// Clock readings are monotonic time since boot, never wall-clock or desktop time.
class IFirmwareClock {
public:
    virtual ~IFirmwareClock() = default;
    [[nodiscard]] virtual Milliseconds now() const = 0;
    // Full-resolution reading for deadlines (the #156 budgets use microseconds).
    // Clocks that only have milliseconds may keep this default.
    [[nodiscard]] virtual Microseconds nowMicroseconds() const { return now(); }
};
// Converts a signed 64-bit microsecond count since boot (the ESP timer contract)
// to whole milliseconds, truncating toward zero. Negative counts are not valid
// time-since-boot readings and are reported as unavailable.
[[nodiscard]] constexpr std::optional<Milliseconds> monotonicMilliseconds(
    std::int64_t microsecondsSinceBoot)
{
    if (microsecondsSinceBoot < 0) return {};
    return std::chrono::duration_cast<Milliseconds>(Microseconds{microsecondsSinceBoot});
}
// Reject future/negative readings instead of unsigned underflow. This helper
// establishes time units for host fixtures, not a command watchdog/safety gate.
[[nodiscard]] inline std::optional<Milliseconds> elapsedSince(
    const IFirmwareClock& clock, Milliseconds start)
{
    const auto current = clock.now();
    if (start < Milliseconds::zero() || current < start) return {};
    return current - start;
}
} // namespace codecell
