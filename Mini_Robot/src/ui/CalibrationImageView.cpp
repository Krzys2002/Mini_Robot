#include "ui/CalibrationImageView.hpp"

#include <QGridLayout>
#include <QImage>
#include <QMouseEvent>
#include <QPainter>
#include <QPushButton>
#include <QResizeEvent>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <limits>

namespace micro_robot {

WorkspaceImageView::WorkspaceImageView(QWidget* parent) : QWidget(parent)
{
    setObjectName("workspaceImage");
    setMinimumSize(320, 180);
    setAccessibleName("Calibration image");
    updateCursor();
}

QWidget* WorkspaceImageView::createControls(QWidget* parent)
{
    auto* controls = new QWidget(parent);
    auto* layout = new QGridLayout(controls);
    layout->setContentsMargins(0, 0, 0, 0);
    const auto button = [controls, layout](int row, int column, const char* name, const char* text,
                                         const char* tooltip, bool checkable = false) {
        auto* control = new QPushButton(text, controls);
        control->setObjectName(name);
        control->setToolTip(tooltip);
        control->setCheckable(checkable);
        control->setAutoDefault(false);
        layout->addWidget(control, row, column);
        return control;
    };
    auto* zoomIn = button(0, 0, "workspaceImageZoomIn", "Zoom in", "Zoom about the centre. Use the wheel to zoom about the cursor.");
    auto* zoomOut = button(0, 1, "workspaceImageZoomOut", "Zoom out", "Zoom about the centre. Use the wheel to zoom about the cursor.");
    auto* fit = button(0, 2, "workspaceImageFit", "Fit image", "Fit the full image and centre it.");
    auto* pan = button(1, 0, "workspaceImagePan", "Pan image", "Drag to move the image. Turn off to select points. Middle drag always pans.", true);
    auto* horizontal = button(1, 1, "workspaceImageFlipHorizontal", "Flip horizontal", "Mirror the displayed image left to right.", true);
    auto* vertical = button(1, 2, "workspaceImageFlipVertical", "Flip vertical", "Mirror the displayed image top to bottom.", true);
    connect(zoomIn, &QPushButton::clicked, this, [this] { setZoom(zoom_ * ZoomStep, QRectF(rect()).center()); });
    connect(zoomOut, &QPushButton::clicked, this, [this] { setZoom(zoom_ / ZoomStep, QRectF(rect()).center()); });
    connect(fit, &QPushButton::clicked, this, &WorkspaceImageView::fitImage);
    connect(pan, &QPushButton::toggled, this, [this](bool active) {
        panMode_ = active;
        updateCursor();
    });
    connect(horizontal, &QPushButton::toggled, this, [this](bool active) {
        horizontalFlip_ = active;
        pan_.setX(-pan_.x());
        update();
    });
    connect(vertical, &QPushButton::toggled, this, [this](bool active) {
        verticalFlip_ = active;
        pan_.setY(-pan_.y());
        update();
    });
    controls->setEnabled(bool(frame_));
    connect(this, &WorkspaceImageView::frameAvailabilityChanged, controls, &QWidget::setEnabled);
    return controls;
}

void WorkspaceImageView::setFrame(FramePtr frame)
{
    if (frame && (!isValidFrame(*frame)
        || frame->width > static_cast<unsigned>(std::numeric_limits<int>::max())
        || frame->height > static_cast<unsigned>(std::numeric_limits<int>::max())
        || frame->strideBytes > static_cast<std::size_t>(std::numeric_limits<qsizetype>::max())))
        frame.reset();
    if (frame == frame_) return;
    const bool wasAvailable = bool(frame_);
    const bool sameSize = frame && frame_ && frame->width == frame_->width && frame->height == frame_->height;
    frame_ = std::move(frame);
    dragButton_ = Qt::NoButton;
    if (!sameSize) fitImage();
    updateCursor();
    if (wasAvailable != bool(frame_)) emit frameAvailabilityChanged(bool(frame_));
    update();
}

QSizeF WorkspaceImageView::scaledImageSize() const
{
    if (!frame_) return {0, 0};
    return QSizeF(frame_->width, frame_->height).scaled(size(), Qt::KeepAspectRatio) * zoom_;
}

QTransform WorkspaceImageView::imageToView() const
{
    const QSizeF scaled = scaledImageSize();
    const QPointF offset = QRectF(rect()).center() + pan_;
    return QTransform((horizontalFlip_ ? -1 : 1) * scaled.width() / frame_->width, 0,
                      0, (verticalFlip_ ? -1 : 1) * scaled.height() / frame_->height,
                      offset.x() + (horizontalFlip_ ? 1 : -1) * scaled.width() / 2,
                      offset.y() + (verticalFlip_ ? 1 : -1) * scaled.height() / 2);
}

void WorkspaceImageView::setZoom(double zoom, QPointF anchor)
{
    if (!frame_) return;
    const double next = std::clamp(zoom, MinimumZoom, MaximumZoom);
    if (next == zoom_) return;
    const QPointF centre = QRectF(rect()).center();
    pan_ = (pan_ + centre - anchor) * (next / zoom_) - centre + anchor;
    zoom_ = next;
    constrainPan();
    update();
}

void WorkspaceImageView::fitImage()
{
    zoom_ = MinimumZoom;
    pan_ = {};
    update();
}

void WorkspaceImageView::constrainPan()
{
    const QSizeF scaled = scaledImageSize();
    const double horizontal = std::max(0.0, (scaled.width() - width()) / 2);
    const double vertical = std::max(0.0, (scaled.height() - height()) / 2);
    pan_.setX(std::clamp(pan_.x(), -horizontal, horizontal));
    pan_.setY(std::clamp(pan_.y(), -vertical, vertical));
}

void WorkspaceImageView::updateCursor()
{
    if (!frame_) unsetCursor();
    else if (dragButton_ != Qt::NoButton) setCursor(Qt::ClosedHandCursor);
    else setCursor(panMode_ ? Qt::OpenHandCursor : Qt::CrossCursor);
}

void WorkspaceImageView::mousePressEvent(QMouseEvent* event)
{
    if (!frame_ || dragButton_ != Qt::NoButton) return;
    if (event->button() == Qt::MiddleButton || (panMode_ && event->button() == Qt::LeftButton)) {
        dragButton_ = event->button();
        lastDragPosition_ = event->position();
        updateCursor();
        return;
    }
    if (event->button() != Qt::LeftButton || !clicked) return;
    bool invertible = false;
    const QPointF native = imageToView().inverted(&invertible).map(event->position()) - QPointF{0.5, 0.5};
    if (invertible && native.x() >= 0 && native.y() >= 0
        && native.x() <= frame_->width - 1 && native.y() <= frame_->height - 1)
        clicked({native.x(), native.y()});
}

void WorkspaceImageView::mouseMoveEvent(QMouseEvent* event)
{
    if (dragButton_ == Qt::NoButton) return;
    if (!event->buttons().testFlag(dragButton_)) {
        dragButton_ = Qt::NoButton;
        updateCursor();
        return;
    }
    pan_ += event->position() - lastDragPosition_;
    lastDragPosition_ = event->position();
    constrainPan();
    update();
}

void WorkspaceImageView::mouseReleaseEvent(QMouseEvent* event)
{
    if (dragButton_ == Qt::NoButton || event->button() != dragButton_) return;
    dragButton_ = Qt::NoButton;
    updateCursor();
}

void WorkspaceImageView::wheelEvent(QWheelEvent* event)
{
    const double steps = event->pixelDelta().isNull()
        ? event->angleDelta().y() / WheelUnitsPerStep
        : event->pixelDelta().y() / TrackpadPixelsPerStep;
    if (!frame_ || steps == 0) {
        event->ignore();
        return;
    }
    setZoom(zoom_ * std::pow(ZoomStep, steps), event->position());
    event->accept();
}

void WorkspaceImageView::resizeEvent(QResizeEvent* event)
{
    constrainPan();
    QWidget::resizeEvent(event);
}

void WorkspaceImageView::paintEvent(QPaintEvent*)
{
    QPainter painter(this);
    painter.fillRect(rect(), Qt::black);
    if (!frame_) {
        painter.setPen(Qt::white);
        painter.drawText(rect(), Qt::AlignCenter, "Freeze a compatible undistorted frame to select points");
        return;
    }
    const auto format = frame_->pixelFormat == PixelFormat::Rgb8 ? QImage::Format_RGB888
        : frame_->pixelFormat == PixelFormat::Rgba8 ? QImage::Format_RGBA8888 : QImage::Format_Grayscale8;
    const QImage image(reinterpret_cast<const uchar*>(frame_->pixels->data()),
        static_cast<int>(frame_->width), static_cast<int>(frame_->height),
        static_cast<qsizetype>(frame_->strideBytes), format);
    const auto transform = imageToView();
    painter.save();
    painter.setWorldTransform(transform);
    painter.drawImage(QRectF(0, 0, frame_->width, frame_->height), image);
    painter.restore();
    const auto at = [&transform](ImageCorner p) { return transform.map(QPointF{p.u + 0.5, p.v + 0.5}); };
    painter.setPen(QPen(Qt::cyan, 2));
    for (std::size_t i = 0; i < corners.size(); ++i) {
        painter.drawEllipse(at(corners[i]), 4, 4);
        painter.drawText(at(corners[i]) + QPointF(6,-6), QStringLiteral("C%1").arg(i+1));
        if (i) painter.drawLine(at(corners[i-1]), at(corners[i]));
    }
    if (corners.size() == WorkspaceCornerCount) {
        painter.drawLine(at(corners.back()), at(corners.front()));
        painter.drawText((at(corners[0])+at(corners[1]))/2, "+X");
        painter.drawText((at(corners[0])+at(corners[3]))/2, "+Y");
        // Arrows point along the selected physical edges, independent of screen orientation.
        for (std::size_t i : {std::size_t{1}, std::size_t{3}}) {
            const auto start = at(corners[0]), end = at(corners[i]);
            const auto delta = end-start;
            const auto tip = start + delta*0.4;
            const double length = std::hypot(delta.x(), delta.y());
            if (length > 0) {
                const QPointF direction = delta / length, normal{-direction.y(), direction.x()};
                painter.drawLine(start, tip);
                painter.drawLine(tip, tip-direction*10+normal*5);
                painter.drawLine(tip, tip-direction*10-normal*5);
            }
        }
    }
    int index = 0;
    for (const auto& [p, check] : markers) {
        ++index;
        if (!std::isfinite(p.u) || !std::isfinite(p.v)) continue;
        painter.setPen(QPen(check ? Qt::yellow : Qt::green, 2));
        painter.drawEllipse(at(p), 4, 4);
        if (index<=MaximumWorkspaceOverlayReferenceLabels) painter.drawText(at(p)+QPointF(6,12), QString::number(index));
    }
    if (board.size() == BoardColumns * BoardRows) {
        painter.setPen(QPen(Qt::cyan, 2));
        for (std::size_t i = 0; i < board.size(); ++i) {
            painter.drawEllipse(at(board[i]), 3, 3);
            if (i == 0 || i == boardAnchor || i + 1 == board.size() || i + 1 == BoardColumns
                || i == board.size() - BoardColumns)
                painter.drawText(at(board[i]) + QPointF(4, -4), QStringLiteral("B%1").arg(i));
        }
        if (boardAnchor < board.size()) {
            const auto anchor = at(board[boardAnchor]);
            painter.setPen(QPen(Qt::magenta, 2));
            painter.drawEllipse(anchor, 7, 7);
            painter.drawText(anchor + QPointF(8, 8), "Anchor");
            const bool lastColumn = boardAnchor % BoardColumns == BoardColumns - 1;
            const bool lastRow = boardAnchor / BoardColumns == BoardRows - 1;
            const auto axis = [&](std::size_t other, bool reverse, const char* label) {
                const auto delta = at(board[other]) - anchor;
                const auto tip = anchor + delta * (reverse ? -2 : 2);
                painter.drawLine(anchor, tip);
                painter.drawText(tip, label);
            };
            axis(lastColumn ? boardAnchor - 1 : boardAnchor + 1, lastColumn, "+cols");
            axis(lastRow ? boardAnchor - BoardColumns : boardAnchor + BoardColumns, lastRow, "+rows");
        }
    }
    if (origin) {
        painter.setPen(QPen(Qt::magenta, 2));
        painter.drawEllipse(at(*origin), 6, 6);
        painter.drawText(at(*origin)+QPointF(8,0), "(0,0)");
    }

}

} // namespace micro_robot
