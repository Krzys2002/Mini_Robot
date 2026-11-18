#include "planning/MetricPath.hpp"

namespace micro_robot {

bool MetricPath::append(WorldPoint2D point, const WorkspaceBounds& bounds)
{
    if (!bounds.contains(point)) return false;
    points_.push_back(point);
    return true;
}

bool MetricPath::undo() noexcept
{
    if (points_.empty()) return false;
    points_.pop_back();
    return true;
}

} // namespace micro_robot
