#include "ui/VisionDebugView.hpp"
#include "app/VisionController.hpp"

#include <QDialogButtonBox>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QFontMetricsF>
#include <QImage>
#include <QHideEvent>
#include <QShowEvent>
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
    const auto paintStart=MonotonicClock::now();
    QPainter painter(this);
    painter.fillRect(rect(), Qt::black);
    if (!preview_) {
        painter.setPen(Qt::white);
        painter.drawText(rect(), Qt::AlignCenter, QStringLiteral("No current processed frame"));
        return;
    }
    const auto& frame = *preview_->frame;
    QImage::Format format = QImage::Format_Invalid;
    switch (frame.pixelFormat) {
    case PixelFormat::Gray8: format = QImage::Format_Grayscale8; break;
    case PixelFormat::Rgb8: format = QImage::Format_RGB888; break;
    case PixelFormat::Rgba8: format = QImage::Format_RGBA8888; break;
    }
    if (format == QImage::Format_Invalid) return;
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
            label.moveTop(label.bottom() + 2 + h <= height() ? label.bottom() + 2 : 0);
        }
        labels.push_back(label);
        painter.fillRect(label, QColor{0, 0, 0, 220});
        painter.setPen(Qt::white);
        painter.drawText(label.adjusted(5, 0, -5, 0), Qt::AlignVCenter,
                         metrics.elidedText(text, Qt::ElideRight, label.width() - 10));
    }
    emit paintCompleted(MonotonicClock::now()-paintStart);
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
        "Raw pixels (+u right, +v down); headings are provisional, not world poses. Compare feature toggles under identical commissioned conditions; rates are measurements, not acceptance."), this);
    legend->setWordWrap(true);
    layout->addWidget(legend);
    auto* controls=new QFormLayout;
    auto* debug=new QCheckBox(QStringLiteral("Prepare debug overlays"),this); debug->setChecked(vision.debugEnabled());
    auto* logging=new QCheckBox(QStringLiteral("Per-frame debug logging calls"),this); logging->setChecked(vision.frameLoggingEnabled());
    controls->addRow(debug); controls->addRow(logging);
    connect(debug,&QCheckBox::toggled,&vision,&VisionController::setDebugEnabled);
    connect(logging,&QCheckBox::toggled,&vision,&VisionController::setFrameLoggingEnabled);
    auto* edge=new QDoubleSpinBox(this); edge->setObjectName("minimumEdgeGate"); edge->setRange(0,10000);
    edge->setValue(vision.qualityGates().minimumEdgePixels); edge->setSpecialValueText(QStringLiteral("Disabled"));
    auto* age=new QDoubleSpinBox(this); age->setObjectName("maximumAgeGate"); age->setRange(0,10000);
    age->setSpecialValueText(QStringLiteral("Disabled"));
    if(vision.qualityGates().maximumObservationAge) age->setValue(std::chrono::duration<double,std::milli>(*vision.qualityGates().maximumObservationAge).count());
    controls->addRow(QStringLiteral("Minimum shortest edge (px)"),edge);
    controls->addRow(QStringLiteral("Maximum observation age (ms)"),age);
    const auto gates=[&vision,edge,age] {
        auto g=vision.qualityGates(); g.minimumEdgePixels=edge->value();
        g.maximumObservationAge=age->value()>0?std::optional<Duration>{std::chrono::duration_cast<Duration>(std::chrono::duration<double,std::milli>{age->value()})}:std::nullopt;
        vision.setQualityGates(g);
    };
    connect(edge,&QDoubleSpinBox::valueChanged,this,gates); connect(age,&QDoubleSpinBox::valueChanged,this,gates);
    layout->addLayout(controls);
    overlay_ = new VisionOverlay(this);
    layout->addWidget(overlay_, 1);
    connect(overlay_,&VisionOverlay::paintCompleted,&vision,&VisionController::recordOverlayPaint);
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
    connect(&vision,&VisionController::metricObservationsAvailable,this,[this](const QString&,MetricObservationBatchPtr batch) {
        if(batch && batch->provenance.inputGeneration!=vision_.inputGeneration()) return;
        metric_=std::move(batch);
    });
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
    if (isVisible()) ageTimer_.start();
    refreshDiagnostics();
}
void VisionDebugDialog::showEvent(QShowEvent* event)
{
    QDialog::showEvent(event);
    refreshDiagnostics(); // Recheck age before displaying a retained hidden preview.
    ageTimer_.start();
}
void VisionDebugDialog::hideEvent(QHideEvent* event)
{
    ageTimer_.stop();
    QDialog::hideEvent(event);
}
void VisionDebugDialog::refreshDiagnostics()
{
    if (!isVisible()) return;
    const auto diagnosticsStart=MonotonicClock::now();
    const auto report=vision_.localizationReport(MonotonicClock::now());
    const auto number=[](std::optional<double> v) {return v?QString::number(*v,'f',2):QStringLiteral("unavailable");};
    const auto latency=[&](const QString& name,const LatencyDistribution& d) {
        return QStringLiteral("\n%1 ms p50/p95/p99: %2 / %3 / %4 (n=%5)").arg(name,number(d.p50Milliseconds),number(d.p95Milliseconds),number(d.p99Milliseconds)).arg(d.samples);
    };
    QString performance=QStringLiteral("\nInput receipt FPS %1 · frames %2 · pending replaced %3 · source sequence gaps %4 · invalid %5 · old/duplicate %6 · detector errors %7 · metric unavailable %8")
        .arg(number(report.inputFramesPerSecond)).arg(report.inputFrames).arg(report.pendingReplaced).arg(report.sequenceGaps)
        .arg(report.invalidFrames).arg(report.outOfOrder).arg(report.detectorErrors).arg(report.metricUnavailableFrames);
    performance+=latency("Queue",report.queue)+latency("Detection",report.detection)+latency("Metric conversion (includes enabled reference monitoring)",report.conversion)
        +latency("Debug preparation",report.debugPreparation)+latency("Logging calls",report.logging)+latency("Delivery/observers",report.delivery)
        +latency("World publication",report.publication)+latency("Receipt to output",report.receiptToOutput)+latency("Mapped exposure to output",report.exposureToOutput)
        +latency("Observation age at output",report.observationAge)+latency("Overlay paint CPU",report.overlayPaint)+latency("Diagnostics CPU",report.diagnostics)+latency("Instrumentation update CPU",report.instrumentation);
    performance+=QStringLiteral("\nExposure is unavailable without verified mapping/uncertainty. Upstream buffering/drops may be unknown. Rolling latency: last 256 samples; rates: last 5 s.\nMeasured/filtered at observation time; predicted-current pose/horizon/uncertainty unavailable. Sampling/age quality is a heuristic, not probability or mm covariance.");
    for(const auto& robot:report.robots) performance+=QStringLiteral("\nRobot %1 useful Hz %2 · accepted %3 · rejected %4 · missing frames %5 · observation age %6 ms")
        .arg(robot.id.value()).arg(number(robot.usefulUpdatesPerSecond)).arg(robot.accepted).arg(robot.rejected).arg(robot.missingFrames)
        .arg(number(robot.outputAge?std::optional<double>{std::chrono::duration<double,std::milli>(*robot.outputAge).count()}:std::nullopt))
        +QStringLiteral(" · last rejection: ")+QString::fromStdString(robot.lastRejection);
    if(metric_) {
        if(!metric_->error.empty()) performance+=QStringLiteral("\nMetric error: ")+QString::fromStdString(metric_->error);
        for(const auto& d:metric_->diagnostics) performance+=QStringLiteral("\nTag %1 · sampling %2 px · %3").arg(d.markerId).arg(number(d.minimumEdgePixels)).arg(QString::fromUtf8(measurementRejectionName(d.reason)));
    }
    if (!preview_) { diagnostics_->setPlainText(QStringLiteral("No current processed frame")+performance); vision_.recordDiagnostics(MonotonicClock::now()-diagnosticsStart); return; }
    const auto now = MonotonicClock::now();
    const auto capturedAt = preview_->frame->capturedAt;
    const auto age = now >= capturedAt ? now - capturedAt : Duration::zero();
    if (age >= CameraFrameTimeout) {
        preview_.reset(); overlay_->present({}); ageTimer_.stop();
        diagnostics_->setPlainText(QStringLiteral("Preview expired: no current processed frame"));
        return;
    }
    const auto& detections = *preview_->detections;
    QString text = QStringLiteral("Source %1 · frame %2 · %3 × %4 raw pixels\nObservation age %5 ms · detector %6 ms · output FPS %7")
        .arg(sourceId_).arg(detections.frameSequence).arg(detections.width).arg(detections.height)
        .arg(std::chrono::duration<double, std::milli>(age).count(), 0, 'f', 1)
        .arg(std::chrono::duration<double, std::milli>(detections.processingTime).count(), 0, 'f', 2)
        .arg(preview_->outputFramesPerSecond ? QString::number(*preview_->outputFramesPerSecond, 'f', 1) : QStringLiteral("n/a"));
    text+=QStringLiteral("\nTime source: %1 · upstream buffering %2 · exposure uncertainty %3 ms")
        .arg(QString::fromUtf8(observationTimeSourceName(preview_->frame->timing.observationSource)))
        .arg(preview_->frame->timing.upstreamBufferingKnown?QStringLiteral("documented by source"):QStringLiteral("unknown"))
        .arg(number(hasMappedExposure(preview_->frame->timing)?std::optional<double>{std::chrono::duration<double,std::milli>(*preview_->frame->timing.exposureUncertainty).count()}:std::nullopt));
    if (!detections.error.empty()) text += QStringLiteral("\nDetector error: ") + QString::fromStdString(detections.error);
    for (const auto& marker : preview_->markers) {
        text += '\n' + markerLabel(marker);
        if (marker.geometry) text += QStringLiteral(" · min edge %1 px").arg(marker.geometry->minimumEdgePixels, 0, 'f', 1);
        text += QStringLiteral(" · margin %1").arg(marker.marker.decisionMargin
            ? QString::number(*marker.marker.decisionMargin, 'f', 1) : QStringLiteral("unavailable"));
    }
    diagnostics_->setPlainText(text+performance);
    vision_.recordDiagnostics(MonotonicClock::now()-diagnosticsStart);
}
} // namespace micro_robot
