#pragma once

#include "world/Workspace.hpp"

#include <QColor>
#include <QPointF>
#include <QRectF>

class QPainter;

namespace micro_robot {

inline const QColor WorkspaceMapBackground{25, 28, 33};

[[nodiscard]] QRectF workspaceMapRect(QSizeF canvas, const WorkspaceBounds& bounds);
[[nodiscard]] std::optional<QPointF> workspaceMapPoint(
    const QRectF& viewport, const WorkspaceBounds& bounds, WorldPoint2D point);
void paintWorkspaceMap(QPainter& painter, const QRectF& viewport, const WorkspaceBounds& bounds);

} // namespace micro_robot
