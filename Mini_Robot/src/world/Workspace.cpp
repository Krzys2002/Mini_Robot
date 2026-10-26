#include "world/Workspace.hpp"

#include <cmath>
#include <stdexcept>

namespace micro_robot {
WorkspaceBounds::WorkspaceBounds(Meters width, Meters height) : width_(width), height_(height)
{
    if (!std::isfinite(width.value) || !std::isfinite(height.value)
        || width.value <= 0 || height.value <= 0) {
        throw std::invalid_argument("Workspace width and height must be finite positive metres");
    }
}
bool WorkspaceBounds::contains(WorldPoint2D point) const noexcept
{
    return std::isfinite(point.x.value) && std::isfinite(point.y.value)
        && point.x.value >= 0 && point.x.value <= width_.value
        && point.y.value >= 0 && point.y.value <= height_.value;
}
std::optional<WorkspaceViewPoint> workspaceToView(const WorkspaceBounds& bounds,
                                                WorldPoint2D point) noexcept
{
    if (!bounds.contains(point)) return std::nullopt;
    return WorkspaceViewPoint{point.x.value / bounds.width().value,
                              1.0 - point.y.value / bounds.height().value};
}
std::optional<WorldPoint2D> viewToWorkspace(const WorkspaceBounds& bounds,
                                         WorkspaceViewPoint point) noexcept
{
    if (!std::isfinite(point.u) || !std::isfinite(point.v)
        || point.u < 0 || point.u > 1 || point.v < 0 || point.v > 1) return std::nullopt;
    return WorldPoint2D{Meters{point.u * bounds.width().value},
                        Meters{(1.0 - point.v) * bounds.height().value}};
}
} // namespace micro_robot
