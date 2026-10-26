#pragma once

#include "common/Units.hpp"

namespace micro_robot {

// Pose in the tabletop world frame: x and y in metres, heading in radians.
// Origin is the measured lower-left workspace corner in a top-down world view.
// +x points right and +y points up; the tabletop is z=0 (+z out of the table).
// Heading is zero along +x and increases counterclockwise toward +y.
// These are world coordinates, not image pixels. Heading is not normalized.
struct Pose2D final {
    Meters x{};
    Meters y{};
    Radians heading{};
};

} // namespace micro_robot
