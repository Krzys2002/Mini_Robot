#include "ui/TopViewDialog.hpp"

#include "calibration/TabletopHomography.hpp"

#include <QDialogButtonBox>
#include <QHideEvent>
#include <QLabel>
#include <QPainter>
#include <QPainterPath>
#include <QShowEvent>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace micro_robot {
namespace {
constexpr int MaximumPreviewSidePixels = 960;

bool current(const Frame& frame)
{
    const auto age = MonotonicClock::now() - frame.capturedAt;
    return age >= Duration::zero() && age < CameraFrameTimeout;
}
}

QImage renderTopView(const Frame& frame, const WorkspaceBounds& bounds, const WorkspaceHomography& homography)
{
    if (!isValidFrame(frame) || !supportedCalibrationResolution(frame.width, frame.height)
        || frame.strideBytes > static_cast<std::size_t>(std::numeric_limits<qsizetype>::max()))
        return {};

    const double width = bounds.width().value, height = bounds.height().value;
    const double extent = std::max(width, height);
    QImage result{std::max(1, qRound(MaximumPreviewSidePixels * (width / extent))),
                  std::max(1, qRound(MaximumPreviewSidePixels * (height / extent))),
                  QImage::Format_RGB32};
    if (result.isNull()) return {};

    const auto& h = homography.imageToWorld;
    const double denominatorScale = std::max({std::abs(h[6]), std::abs(h[7]), std::abs(h[8])});
    if (!std::isfinite(denominatorScale) || denominatorScale == 0) return {};
    QPolygonF native;
    std::optional<bool> negative;
    for (const auto corner : workspaceCorners(bounds)) {
        const auto point = unprojectWorkspacePoint(homography, corner);
        if (!point) return {};
        // The forward denominator is proportional to the reciprocal of the
        // inverse denominator at this corner. A sign change crosses a horizon.
        const auto w = (h[6] / denominatorScale) * point->u
                     + (h[7] / denominatorScale) * point->v + h[8] / denominatorScale;
        if (!std::isfinite(w) || w == 0 || (negative && *negative != std::signbit(w))) return {};
        negative = std::signbit(w);
        native.append({point->u + 0.5, point->v + 0.5});
    }
    const auto w = static_cast<double>(result.width()), hgt = static_cast<double>(result.height());
    const QPolygonF overhead{{0, hgt}, {w, hgt}, {w, 0}, {0, 0}};
    QTransform transform;
    if (!QTransform::quadToQuad(native, overhead, transform)) return {};
    // Qt clips homogeneous points with negative w. Select the equivalent
    // representation with positive w over the calibrated table.
    const auto base = native.front();
    if (transform.m13() * base.x() + transform.m23() * base.y() + transform.m33() < 0)
        transform *= -1;

    const auto format = frame.pixelFormat == PixelFormat::Rgb8 ? QImage::Format_RGB888
        : frame.pixelFormat == PixelFormat::Rgba8 ? QImage::Format_RGBA8888 : QImage::Format_Grayscale8;
    const QImage source{reinterpret_cast<const uchar*>(frame.pixels->data()),
                        static_cast<int>(frame.width), static_cast<int>(frame.height),
                        static_cast<qsizetype>(frame.strideBytes), format};
    result.fill(Qt::black);
    QPainter painter(&result);
    painter.setRenderHint(QPainter::SmoothPixmapTransform);
    painter.setTransform(transform);
    // Clip before mapping the image so the source region cannot include a
    // projective horizon outside the table.
    QPainterPath footprint;
    footprint.addPolygon(native);
    painter.setClipPath(footprint);
    painter.drawImage(QPointF{}, source);
    return result;
}

class TopViewImage final : public QWidget {
public:
    explicit TopViewImage(QWidget* parent) : QWidget(parent)
    {
        setObjectName(QStringLiteral("topViewImage"));
        setMinimumSize(300, 220);
    }

    void present(QImage next)
    {
        image_ = std::move(next);
        update();
    }

protected:
    void paintEvent(QPaintEvent*) override
    {
        QPainter painter(this);
        painter.fillRect(rect(), QColor{25, 28, 33});
        if (image_.isNull()) return;
        const QSize fitted = image_.size().scaled(size(), Qt::KeepAspectRatio);
        painter.setRenderHint(QPainter::SmoothPixmapTransform);
        painter.drawImage(QRect{{(width() - fitted.width()) / 2, (height() - fitted.height()) / 2}, fitted}, image_);
    }

private:
    QImage image_;
};

TopViewDialog::TopViewDialog(std::unique_ptr<ICalibrationUndistorter> undistorter, QWidget* parent)
    : QDialog(parent), undistorter_(std::move(undistorter))
{
    if (!undistorter_) throw std::invalid_argument("Top View requires an undistorter.");
    setWindowTitle(QStringLiteral("Top View"));
    setObjectName(QStringLiteral("topViewDialog"));
    setAttribute(Qt::WA_QuitOnClose, false);
    resize(800, 620);
    auto* layout = new QVBoxLayout(this);
    status_ = new QLabel(this);
    status_->setObjectName(QStringLiteral("topViewStatus"));
    status_->setWordWrap(true);
    layout->addWidget(status_);
    image_ = new TopViewImage(this);
    layout->addWidget(image_, 1);
    auto* caption = new QLabel(QStringLiteral("Tabletop preview · +X right · +Y up. Objects above the table can appear distorted."), this);
    caption->setWordWrap(true);
    layout->addWidget(caption);
    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Close, this);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    layout->addWidget(buttons);
    workerContext_ = new QObject;
    workerContext_->moveToThread(&worker_);
    connect(&worker_, &QThread::finished, workerContext_, &QObject::deleteLater);
    worker_.start();
    timer_.setInterval(PreviewInterval);
    connect(&timer_, &QTimer::timeout, this, &TopViewDialog::tick);
    resetPreview();
}

TopViewDialog::~TopViewDialog()
{
    worker_.quit();
    worker_.wait();
}

void TopViewDialog::setCalibration(const std::optional<WorkspaceCalibrationProfile>& p)
{
    calibration_.reset();
    if (p && p->homography && validateWorkspaceCalibrationProfile(*p).empty())
        calibration_.emplace(p->bounds, p->intrinsics, *p->homography);
    resetPreview();
}

void TopViewDialog::setInput(const QString& identity, bool streaming)
{
    if (input_ == identity && streaming_ == streaming) return;
    input_ = identity;
    streaming_ = streaming;
    resetPreview();
}

bool TopViewDialog::compatible() const
{
    return streaming_ && calibration_ && QString::fromStdString(calibration_->intrinsics.sourceId) == input_;
}

void TopViewDialog::resetPreview()
{
    ++generation_;
    pending_.reset();
    latestAt_.reset();
    displayedAt_.reset();
    image_->present({});
    status_->setText(!streaming_ ? QStringLiteral("Start the camera to view the table.")
        : !compatible() ? QStringLiteral("Accept tabletop calibration for this camera to enable Top View.")
        : QStringLiteral("Waiting for a compatible camera frame…"));
}

void TopViewDialog::receiveFrame(FramePtr frame)
{
    if (!isVisible() || !compatible() || !frame || !isValidFrame(*frame))
        return;
    if (frame->width != calibration_->intrinsics.width || frame->height != calibration_->intrinsics.height) {
        resetPreview();
        status_->setText(QStringLiteral("Camera resolution changed. Top View requires compatible calibration."));
        return;
    }
    if (!current(*frame) || (latestAt_ && frame->capturedAt <= *latestAt_))
        return;
    latestAt_ = frame->capturedAt;
    pending_ = std::move(frame);
}

void TopViewDialog::showEvent(QShowEvent* event)
{
    QDialog::showEvent(event);
    timer_.start();
}

void TopViewDialog::hideEvent(QHideEvent* event)
{
    timer_.stop();
    resetPreview();
    QDialog::hideEvent(event);
}

void TopViewDialog::tick()
{
    if (displayedAt_ && MonotonicClock::now() - *displayedAt_ >= CameraFrameTimeout) {
        displayedAt_.reset();
        image_->present({});
        status_->setText(QStringLiteral("Waiting for a fresh camera frame…"));
    }
    if (running_ || !pending_ || !compatible()) return;
    auto frame = std::exchange(pending_, {});
    if (!current(*frame)) return;
    running_ = true;
    const auto calibration = *calibration_;
    const auto epoch = generation_;
    QMetaObject::invokeMethod(workerContext_, [this, frame = std::move(frame), calibration, epoch] {
        QImage output;
        try {
            const auto corrected = undistorter_->undistort(*frame, calibration.intrinsics);
            if (corrected.error.empty() && corrected.frame
                && corrected.frame->width == frame->width && corrected.frame->height == frame->height
                && corrected.frame->sequence == frame->sequence && corrected.frame->capturedAt == frame->capturedAt
                && corrected.frame->pixelFormat == frame->pixelFormat)
                output = renderTopView(*corrected.frame, calibration.bounds, calibration.homography);
        } catch (...) {
            // A preview failure must never escape the worker event loop.
        }
        QMetaObject::invokeMethod(this, [this, frame, epoch, output = std::move(output)]() mutable {
            running_ = false;
            if (epoch != generation_ || !isVisible() || !current(*frame)) return;
            if (output.isNull()) {
                displayedAt_.reset();
                image_->present({});
                status_->setText(QStringLiteral("Could not rectify the camera frame. Check the tabletop calibration."));
                return;
            }
            displayedAt_ = frame->capturedAt;
            image_->present(std::move(output));
            status_->setText(QStringLiteral("Live rectified camera"));
        }, Qt::QueuedConnection);
    }, Qt::QueuedConnection);
}
} // namespace micro_robot
