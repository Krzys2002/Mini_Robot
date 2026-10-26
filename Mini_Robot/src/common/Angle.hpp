#pragma once

#include "common/Units.hpp"

#include <cmath>
#include <numbers>

namespace micro_robot {

// Wraps an angle into the half-open interval (-pi, pi]; both -pi and pi map to
// +pi. Values already inside the interval are returned unchanged. Non-finite
// input is returned unchanged so that callers can reject it explicitly.
[[nodiscard]] inline Radians normalizeAngle(Radians angle) noexcept
{
    if (!std::isfinite(angle.value)) return angle;
    const double wrapped = std::remainder(angle.value, 2.0 * std::numbers::pi);
    return Radians{wrapped <= -std::numbers::pi ? std::numbers::pi : wrapped};
}

} // namespace micro_robot
