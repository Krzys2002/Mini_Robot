#pragma once

#include "common/Units.hpp"

#include <optional>

namespace micro_robot {
// Tabletop plane: explicit origin, +X right in the world view,
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
    WorkspaceBounds(Meters width, Meters height, WorldPoint2D minimum);
    [[nodiscard]] WorldPoint2D minimum() const noexcept { return minimum_; }
    [[nodiscard]] WorldPoint2D maximum() const noexcept
    { return {Meters{minimum_.x.value + width_.value}, Meters{minimum_.y.value + height_.value}}; }
    [[nodiscard]] bool operator==(const WorkspaceBounds& other) const noexcept
    { return width_.value == other.width_.value && height_.value == other.height_.value
        && minimum_.x.value == other.minimum_.x.value && minimum_.y.value == other.minimum_.y.value; }
    [[nodiscard]] Meters width() const noexcept { return width_; }
    [[nodiscard]] Meters height() const noexcept { return height_; }
    // Closed explicit extents; two-argument construction preserves the legacy zero minimum.
    [[nodiscard]] bool contains(WorldPoint2D point) const noexcept;
private:
    Meters width_, height_;
    WorldPoint2D minimum_;
};
// These are world-view layout conversions, never camera calibration. Out-of-bounds
// or non-finite input returns nullopt rather than clamping or extrapolating.
[[nodiscard]] std::optional<WorkspaceViewPoint> workspaceToView(
    const WorkspaceBounds& bounds, WorldPoint2D point) noexcept;
[[nodiscard]] std::optional<WorldPoint2D> viewToWorkspace(
    const WorkspaceBounds& bounds, WorkspaceViewPoint point) noexcept;
} // namespace micro_robot
