#include "ui/WorkspaceMapPainting.hpp"

#include <QPainter>

namespace micro_robot {

QRectF workspaceMapRect(QSizeF canvas, const WorkspaceBounds& bounds)
{
    const auto fitted = QSizeF(bounds.width().value, bounds.height().value)
        .scaled(canvas - QSizeF(70, 55), Qt::KeepAspectRatio);
    return {QPointF((canvas.width() - fitted.width()) / 2, (canvas.height() - fitted.height()) / 2), fitted};
}

std::optional<QPointF> workspaceMapPoint(const QRectF& viewport, const WorkspaceBounds& bounds, WorldPoint2D point)
{
    const auto position = workspaceToView(bounds, point);
    if (!position) return {};
    return QPointF(viewport.left() + position->u * viewport.width(), viewport.top() + position->v * viewport.height());
}

void paintWorkspaceMap(QPainter& painter, const QRectF& viewport, const WorkspaceBounds& bounds)
{
    constexpr double MillimetersPerMeter = 1000;
    painter.save();
    painter.setPen(Qt::white);
    painter.fillRect(viewport, QColor{45, 50, 55});
    painter.drawRect(viewport);
    painter.drawText(QPointF(viewport.left(), viewport.bottom() + 18),
                     QStringLiteral("%1 mm").arg(bounds.minimum().x.value * MillimetersPerMeter));
    painter.drawText(QRectF(viewport.left(), viewport.bottom() + 3, viewport.width(), 22), Qt::AlignRight,
                     QStringLiteral("%1 mm +X").arg(bounds.maximum().x.value * MillimetersPerMeter));
    painter.drawText(QPointF(viewport.left() - 34, viewport.top() - 5),
                     QStringLiteral("+Y %1 mm").arg(bounds.maximum().y.value * MillimetersPerMeter));
    painter.drawText(QPointF(viewport.left() - 34, viewport.bottom()),
                     QStringLiteral("%1 mm").arg(bounds.minimum().y.value * MillimetersPerMeter));
    if (const auto origin = workspaceMapPoint(viewport, bounds, {})) {
        painter.setPen(Qt::gray);
        painter.drawLine(*origin + QPointF(-5, 0), *origin + QPointF(5, 0));
        painter.drawLine(*origin + QPointF(0, -5), *origin + QPointF(0, 5));
    }
    painter.restore();
}

} // namespace micro_robot
