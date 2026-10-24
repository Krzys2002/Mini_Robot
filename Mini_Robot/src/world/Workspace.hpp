#pragma once

#include "common/Units.hpp"

#include <optional>

namespace micro_robot {
// Tabletop plane: origin at the lower-left corner in the world view, +X right,
// +Y up, +Z out of the table. Heading is zero along +X and positive CCW.
struct WorldPoint2D {
    Meters x{}, y{};
};
// Presentation-only fractions, not camera pixels: top-left (0,0), bottom-right
// (1,1). Apply viewport scaling/offset only at the UI boundary.
struct WorkspaceViewPoint {
    double u{}, v{};
};
class WorkspaceBounds final {
public:
    // Throws invalid_argument unless both extents are finite and strictly positive.
    WorkspaceBounds(Meters width, Meters height);
    [[nodiscard]] Meters width() const noexcept { return width_; }
    [[nodiscard]] Meters height() const noexcept { return height_; }
    // Closed bounds [0,width] x [0,height]; non-finite points are outside.
    [[nodiscard]] bool contains(WorldPoint2D point) const noexcept;
private:
    Meters width_, height_;
};
// These are world-view layout conversions, never camera calibration. Out-of-bounds
// or non-finite input returns nullopt rather than clamping or extrapolating.
[[nodiscard]] std::optional<WorkspaceViewPoint> workspaceToView(
    const WorkspaceBounds& bounds, WorldPoint2D point) noexcept;
[[nodiscard]] std::optional<WorldPoint2D> viewToWorkspace(
    const WorkspaceBounds& bounds, WorkspaceViewPoint point) noexcept;
} // namespace micro_robot
