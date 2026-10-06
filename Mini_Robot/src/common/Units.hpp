#pragma once

namespace micro_robot {

// Explicit SI value wrappers. Convert to/from display or protocol units at the
// boundary; no dimensional arithmetic or implicit conversion is provided.
struct Meters final {
    double value;

    explicit constexpr Meters(double valueInMeters = 0.0) noexcept
        : value(valueInMeters)
    {
    }
};

struct Radians final {
    double value;

    explicit constexpr Radians(double valueInRadians = 0.0) noexcept
        : value(valueInRadians)
    {
    }
};

} // namespace micro_robot
