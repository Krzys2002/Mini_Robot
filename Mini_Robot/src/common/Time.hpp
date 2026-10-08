#pragma once

#include <chrono>

namespace micro_robot {

using MonotonicClock = std::chrono::steady_clock;
using Timestamp = MonotonicClock::time_point;
using Duration = MonotonicClock::duration;

// Timestamp's epoch is unspecified: use it for local ordering and elapsed time,
// not wall-clock dates, persistent timestamps, or cross-device synchronization.
// A missing observation should use std::optional<Timestamp>, not an epoch sentinel.

} // namespace micro_robot
