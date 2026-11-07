#pragma once
#if __cplusplus < 202002L
#error "CodeCell firmware requires C++20; use the pinned PlatformIO configuration"
#endif
#include <chrono>
#include <optional>

namespace codecell {
using Milliseconds = std::chrono::milliseconds;
// Clock readings are monotonic time since boot, never wall-clock or desktop time.
class IFirmwareClock {
public:
    virtual ~IFirmwareClock() = default;
    [[nodiscard]] virtual Milliseconds now() const = 0;
};
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
