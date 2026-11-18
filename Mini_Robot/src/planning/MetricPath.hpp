#pragma once

#include "world/Workspace.hpp"

#include <cstddef>
#include <vector>

namespace micro_robot {

// Editable tabletop path geometry. Points are always world-frame metres;
// this model has no Qt, camera, calibration or motion-control dependency.
class MetricPath final {
public:
    [[nodiscard]] const std::vector<WorldPoint2D>& points() const noexcept { return points_; }
    [[nodiscard]] bool empty() const noexcept { return points_.empty(); }
    [[nodiscard]] std::size_t size() const noexcept { return points_.size(); }

    // Invalid/out-of-workspace points leave the path unchanged.
    [[nodiscard]] bool append(WorldPoint2D point, const WorkspaceBounds& bounds);
    [[nodiscard]] bool undo() noexcept;
    void clear() noexcept { points_.clear(); }

private:
    std::vector<WorldPoint2D> points_;
};

} // namespace micro_robot
