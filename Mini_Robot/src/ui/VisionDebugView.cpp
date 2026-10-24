#include "ui/VisionDebugView.hpp"
#include "app/VisionController.hpp"

#include <QDialogButtonBox>
#include <QFontMetricsF>
#include <QImage>
#include <QLabel>
#include <QPainter>
#include <QPlainTextEdit>
#include <QPolygonF>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <limits>

namespace micro_robot {
namespace {
QString rejectionName(MarkerRejection value)
{
    switch (value) {
    case MarkerRejection::UnsupportedFamily: return QStringLiteral("unsupported family");
    case MarkerRejection::UnassignedId: return QStringLiteral("unassigned");
    case MarkerRejection::DuplicateRobot: return QStringLiteral("duplicate ID");
    case MarkerRejection::InvalidGeometry: return QStringLiteral("invalid geometry");
    }
    return QStringLiteral("rejected");
}
QString markerLabel(const MarkerDebugOverlay& marker)
{
    auto text = QStringLiteral("Tag %1").arg(marker.marker.id);
    if (marker.robotId) text += QStringLiteral(" · Robot %1").arg(marker.robotId->value());
    else if (marker.rejection) text += QStringLiteral(" · %1").arg(rejectionName(*marker.rejection));
    return text;
}
bool drawableFrame(const VisionDebugFramePtr& preview)
{
    return preview && preview->frame && preview->detections && isValidFrame(*preview->frame)
        && preview->frame->sequence == preview->detections->frameSequence
        && preview->frame->capturedAt == preview->detections->capturedAt
        && preview->frame->width == preview->detections->width
        && preview->frame->height == preview->detections->height
        && preview->frame->width <= static_cast<std::uint32_t>(std::numeric_limits<int>::max())
        && preview->frame->height <= static_cast<std::uint32_t>(std::numeric_limits<int>::max())
        && preview->frame->strideBytes <= static_cast<std::size_t>(std::numeric_limits<qsizetype>::max());
}
}
VisionOverlay::VisionOverlay(QWidget* parent) : QWidget(parent)
{
    setObjectName(QStringLiteral("visionOverlay"));
    setMinimumSize(320, 200);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}
void VisionOverlay::present(VisionDebugFramePtr preview)
{
    preview_ = drawableFrame(preview) ? std::move(preview) : VisionDebugFramePtr{};
    update();
}
QRectF VisionOverlay::imageRect() const
{
    if (!preview_) return {};
    const double scale = std::min(static_cast<double>(width()) / preview_->frame->width,
                                  static_cast<double>(height()) / preview_->frame->height);
    const double w = preview_->frame->width * scale, h = preview_->frame->height * scale;
    return {(width() - w) / 2, (height() - h) / 2, w, h};
}
QPointF VisionOverlay::imageToView(ImagePoint point) const
{
    if (!preview_) return {};
    const auto bounds = imageRect();
    const double scale = bounds.width() / preview_->frame->width;
    return {bounds.x() + (point.u + 0.5) * scale, bounds.y() + (point.v + 0.5) * scale};
}
void VisionOverlay::paintEvent(QPaintEvent*)
{
    QPainter painter(this);
    painter.fillRect(rect(), Qt::black);
    if (!preview_) {
        painter.setPen(Qt::white);
        painter.drawText(rect(), Qt::AlignCenter, QStringLiteral("No current processed frame"));
        return;
    }
    const auto& frame = *preview_->frame;
    const auto format = frame.pixelFormat == PixelFormat::Gray8 ? QImage::Format_Grayscale8
        : frame.pixelFormat == PixelFormat::Rgb8 ? QImage::Format_RGB888 : QImage::Format_RGBA8888;
    const QImage image(reinterpret_cast<const uchar*>(frame.pixels->data()), static_cast<int>(frame.width),
        static_cast<int>(frame.height), static_cast<qsizetype>(frame.strideBytes), format);
    painter.drawImage(imageRect(), image);
    painter.setRenderHint(QPainter::Antialiasing);
    const QFontMetricsF metrics(painter.font());
    std::vector<QRectF> labels;
    for (const auto& marker : preview_->markers) {
        QPolygonF polygon;
        bool finite = true;
        for (const auto& corner : marker.marker.corners) {
            if (!std::isfinite(corner.u) || !std::isfinite(corner.v)) { finite = false; break; }
            const auto mapped = imageToView({corner.u, corner.v});
            if (!std::isfinite(mapped.x()) || !std::isfinite(mapped.y())) { finite = false; break; }
            polygon << mapped;
        }
        if (!finite) continue; // Never pass malformed coordinates to QPainter.
        QRectF visualBounds = polygon.boundingRect();
        painter.save();
        painter.setClipRect(imageRect());
        painter.setPen(QPen(marker.robotId ? QColor{0, 255, 100} : QColor{255, 170, 0}, 2,
                            marker.robotId ? Qt::SolidLine : Qt::DashLine));
        painter.setBrush(Qt::NoBrush);
        painter.drawPolygon(polygon);
        if (marker.geometry) {
            const auto& geometry = *marker.geometry;
            const auto center = imageToView(geometry.center);
            painter.setPen(QPen(Qt::cyan, 2));
            painter.drawEllipse(center, 4, 4);
            const double length = geometry.minimumEdgePixels * 0.7;
            const double du = std::cos(geometry.imageHeading.value), dv = -std::sin(geometry.imageHeading.value);
            const auto tip = imageToView({geometry.center.u + du * length, geometry.center.v + dv * length});
            visualBounds = visualBounds.united(QRectF(center, tip).normalized().adjusted(-10, -10, 10, 10));
            const QPointF direction = tip - center;
            const double size = std::hypot(direction.x(), direction.y());
            if (size > 0) {
                const QPointF unit = direction / size, side{-unit.y(), unit.x()};
                painter.setPen(QPen(Qt::yellow, 2));
                painter.drawLine(center, tip);
                painter.drawLine(tip, tip - unit * 9 + side * 4);
                painter.drawLine(tip, tip - unit * 9 - side * 4);
            }
        }
        painter.restore();
        const QString text = markerLabel(marker);
        const double w = std::min(metrics.horizontalAdvance(text) + 10, static_cast<double>(width()));
        const double h = metrics.height() + 6;
        auto anchor = visualBounds.topLeft();
        QRectF label{std::clamp(anchor.x(), 0.0, std::max(0.0, width() - w)),
                     std::clamp(anchor.y() - h - 3, 0.0, std::max(0.0, height() - h)), w, h};
        // Presentation placement only: avoid covering a preceding ID when tags
        // share a row. The full identity/quality list is separately scrollable.
        for (std::size_t attempt = 0; attempt < labels.size() + 1; ++attempt) {
            if (std::none_of(labels.begin(), labels.end(), [&](const auto& other) { return other.intersects(label); })) break;
            label.moveTop(label.bottom() + h <= height() ? label.bottom() + 2 : 0);
        }
        labels.push_back(label);
        painter.fillRect(label, QColor{0, 0, 0, 220});
        painter.setPen(Qt::white);
        painter.drawText(label.adjusted(5, 0, -5, 0), Qt::AlignVCenter,
                         metrics.elidedText(text, Qt::ElideRight, label.width() - 10));
    }
}
VisionDebugDialog::VisionDebugDialog(VisionController& vision, QWidget* parent)
    : QDialog(parent), vision_(vision)
{
    setObjectName(QStringLiteral("visionDebugDialog"));
    setWindowTitle(QStringLiteral("Vision debug — raw image coordinates"));
    setAttribute(Qt::WA_QuitOnClose, false);
    resize(900, 680);
    auto* layout = new QVBoxLayout(this);
    auto* legend = new QLabel(QStringLiteral("Green: assigned robot · orange dashed: rejected tag · cyan: center · yellow: front. "
        "Raw pixels (+u right, +v down); headings are provisional, not world poses."), this);
    legend->setWordWrap(true);
    layout->addWidget(legend);
    overlay_ = new VisionOverlay(this);
    layout->addWidget(overlay_, 1);
    diagnostics_ = new QPlainTextEdit(this);
    diagnostics_->setObjectName(QStringLiteral("visionDiagnostics"));
    diagnostics_->setReadOnly(true);
    diagnostics_->setMaximumHeight(170);
    layout->addWidget(diagnostics_);
    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Close, this);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::close);
    layout->addWidget(buttons);
    connect(&vision, &VisionController::previewAvailable, this,
        [this](const QString& source, VisionDebugFramePtr preview) { present(source, std::move(preview)); });
    ageTimer_.setInterval(200);
    connect(&ageTimer_, &QTimer::timeout, this, [this] { refreshDiagnostics(); });
    present({}, {});
}
void VisionDebugDialog::present(const QString& sourceId, VisionDebugFramePtr preview)
{
    // A previous direct signal observer can invalidate the service and publish a
    // clear before this observer receives the outer (old) signal invocation.
    if (preview && preview->inputGeneration != vision_.inputGeneration()) return;
    sourceId_ = sourceId;
    preview_ = drawableFrame(preview) ? std::move(preview) : VisionDebugFramePtr{};
    overlay_->present(preview_);
    if (preview_) ageTimer_.start(); else ageTimer_.stop();
    refreshDiagnostics();
}
void VisionDebugDialog::refreshDiagnostics()
{
    if (!preview_) { diagnostics_->setPlainText(QStringLiteral("No current processed frame")); return; }
    const auto now = MonotonicClock::now();
    const auto capturedAt = preview_->frame->capturedAt;
    const auto age = now >= capturedAt ? now - capturedAt : Duration::zero();
    if (age >= CameraFrameTimeout) {
        preview_.reset(); overlay_->present({}); ageTimer_.stop();
        diagnostics_->setPlainText(QStringLiteral("Preview expired: no current processed frame"));
        return;
    }
    const auto& detections = *preview_->detections;
    QString text = QStringLiteral("Source %1 · frame %2 · %3 × %4 raw pixels\nAge %5 ms · detector %6 ms · output FPS %7")
        .arg(sourceId_).arg(detections.frameSequence).arg(detections.width).arg(detections.height)
        .arg(std::chrono::duration<double, std::milli>(age).count(), 0, 'f', 1)
        .arg(std::chrono::duration<double, std::milli>(detections.processingTime).count(), 0, 'f', 2)
        .arg(preview_->outputFramesPerSecond ? QString::number(*preview_->outputFramesPerSecond, 'f', 1) : QStringLiteral("n/a"));
    if (!detections.error.empty()) text += QStringLiteral("\nDetector error: ") + QString::fromStdString(detections.error);
    for (const auto& marker : preview_->markers) {
        text += '\n' + markerLabel(marker);
        if (marker.geometry) text += QStringLiteral(" · min edge %1 px").arg(marker.geometry->minimumEdgePixels, 0, 'f', 1);
        text += QStringLiteral(" · margin %1").arg(marker.marker.decisionMargin
            ? QString::number(*marker.marker.decisionMargin, 'f', 1) : QStringLiteral("unavailable"));
    }
    diagnostics_->setPlainText(text);
}
} // namespace micro_robot
