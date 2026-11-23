#include "ui/WorldModelCanvas.hpp"

#include "app/DesktopController.hpp"
#include "ui/WorkspaceMapPainting.hpp"

#include <QPainter>
#include <cmath>

namespace micro_robot {

WorldModelCanvas::WorldModelCanvas(DesktopController& controller, QWidget* parent)
    : QWidget(parent), controller_(controller)
{
    setObjectName(QStringLiteral("worldModelCanvas"));
    setMinimumSize(360, 260);
    connect(&controller_, &DesktopController::worldDescriptionChanged, this, [this] { update(); });
}

void WorldModelCanvas::paintEvent(QPaintEvent*)
{
    QPainter painter(this);
    painter.fillRect(rect(), WorkspaceMapBackground);

    const auto snapshot = controller_.worldSnapshot();
    if (!snapshot.workspace) {
        painter.setPen(Qt::white);
        painter.drawText(rect(), Qt::AlignCenter, QStringLiteral("Configure a workspace to display the world model."));
        return;
    }

    const auto& bounds = *snapshot.workspace;
    const auto viewport = workspaceMapRect(size(), bounds);
    paintWorkspaceMap(painter, viewport, bounds);
    painter.setRenderHint(QPainter::Antialiasing);
    const auto now = MonotonicClock::now();
    for (const auto& [id, robot] : snapshot.robots) {
        if (!robot.pose) continue;
        const auto status = controller_.worldTracking(id, now).status;
        if (status == TrackingStatus::Lost) continue;
        const auto& pose = *robot.pose;
        const auto point = workspaceMapPoint(viewport, bounds, {pose.x, pose.y});
        if (!point) continue;

        const auto color = status == TrackingStatus::Stale ? QColor(255, 191, 86) : QColor(Qt::cyan);
        constexpr double headingLength = 17;
        const auto direction = QPointF(std::cos(pose.heading.value), -std::sin(pose.heading.value));
        painter.setPen(QPen(color, 2));
        painter.setBrush(color);
        painter.drawLine(*point, *point + direction * headingLength);
        painter.drawEllipse(*point, 4, 4);
        painter.drawText(*point + QPointF(9, 17), QStringLiteral("Robot %1").arg(id.value()));
    }
}

} // namespace micro_robot
