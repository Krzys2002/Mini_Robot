#include "ui/CalibrationWizard.hpp"
#include "ui/WorkspaceCalibrationDialog.hpp"
#include "app/CalibrationController.hpp"

#include <QComboBox>
#include <QCheckBox>
#include <QHBoxLayout>
#include <QImage>
#include <QLabel>
#include <QPainter>
#include <QPushButton>
#include <QSignalBlocker>
#include <QStandardItemModel>
#include <QTimer>
#include <QVBoxLayout>

#include <numbers>

namespace micro_robot {
class CalibrationPreview final : public QWidget {
public:
    using QWidget::QWidget;
    void present(FramePtr frame, std::vector<ImageCorner> corners, bool accepted, bool approximate)
    {
        frame_ = std::move(frame);
        corners_ = std::move(corners);
        accepted_ = accepted;
        approximate_ = approximate;
        // QWidget coalesces updates. It holds one newest preview, never a frame queue.
        update();
    }
protected:
    void paintEvent(QPaintEvent*) override
    {
        QPainter painter(this);
        painter.fillRect(rect(), Qt::black);
        if (!frame_) {
            painter.setPen(Qt::white);
            painter.drawText(rect(), Qt::AlignCenter, QStringLiteral("Waiting for camera frames"));
            return;
        }
        QImage::Format format = QImage::Format_Grayscale8;
        if (frame_->pixelFormat == PixelFormat::Rgb8) format = QImage::Format_RGB888;
        if (frame_->pixelFormat == PixelFormat::Rgba8) format = QImage::Format_RGBA8888;
        const QImage image(reinterpret_cast<const uchar*>(frame_->pixels->data()),
            static_cast<int>(frame_->width), static_cast<int>(frame_->height),
            static_cast<qsizetype>(frame_->strideBytes), format);
        const QSize fitted = image.size().scaled(size(), Qt::KeepAspectRatio);
        const QRect bounds{QPoint{(width() - fitted.width()) / 2, (height() - fitted.height()) / 2}, fitted};
        painter.drawImage(bounds, image);
        painter.setPen(QPen(approximate_ ? Qt::cyan : accepted_ ? Qt::green : Qt::red, 2));
        const auto map = [&](const ImageCorner& p) {
            return QPointF{bounds.x() + (p.u + 0.5) * bounds.width() / frame_->width,
                           bounds.y() + (p.v + 0.5) * bounds.height() / frame_->height};
        };
        for (const auto& p : corners_) painter.drawEllipse(map(p), 3, 3);
        if (!corners_.empty()) {
            painter.setPen(QPen(Qt::yellow, 2));
            painter.drawEllipse(map(corners_.front()), 6, 6);
            painter.drawText(map(corners_.front()) + QPointF{8, -8}, QStringLiteral("first corner"));
        }
    }
private:
    FramePtr frame_;
    std::vector<ImageCorner> corners_;
    bool accepted_{};
    bool approximate_{};
};

CalibrationWizard::CalibrationWizard(CalibrationController& controller, QWidget* parent)
    : QDialog(parent), controller_(controller)
{
    setWindowTitle(QStringLiteral("Intrinsic camera calibration"));
    resize(900, 740);
    auto* layout = new QVBoxLayout(this);
    const auto& target = controller.session().config().target;
    auto* instructions = new QLabel(QStringLiteral(
        "Chessboard: %1 × %2 inner corners (%3 × %4 squares), %5 mm per square.\n"
        "Use a flat board with a white border. Capture different positions, distances and rotations.\n"
        "Detection up to 5 Hz; choose a live preview mode below. Cyan = held or tracked corners.\n"
        "Green = usable, red = rejected, yellow ring = first corner. Inspect every accepted view.")
        .arg(target.columns).arg(target.rows).arg(target.columns + 1).arg(target.rows + 1)
        .arg(target.squareSizeMeters * 1000), this);
    instructions->setWordWrap(true);
    layout->addWidget(instructions);
    preview_ = new CalibrationPreview(this);
    preview_->setObjectName(QStringLiteral("calibrationPreview"));
    preview_->setMinimumSize(400, 260);
    layout->addWidget(preview_, 1);
    message_ = new QLabel(this);
    message_->setObjectName(QStringLiteral("calibrationMessage"));
    message_->setWordWrap(true);
    message_->setTextFormat(Qt::PlainText);
    layout->addWidget(message_);
    auto* feedback = new QLabel(this);
    feedback->setObjectName(QStringLiteral("calibrationCaptureFeedback"));
    feedback->setWordWrap(true);
    feedback->setTextFormat(Qt::PlainText);
    feedback->setText(controller_.captureFeedback());
    layout->addWidget(feedback);
    connect(&controller_, &CalibrationController::changed, feedback,
        [this, feedback] { feedback->setText(controller_.captureFeedback()); });
    progress_ = new QLabel(this);
    progress_->setObjectName(QStringLiteral("calibrationProgress"));
    progress_->setWordWrap(true);
    layout->addWidget(progress_);
    quality_ = new QLabel(this);
    quality_->setObjectName(QStringLiteral("calibrationQuality"));
    quality_->setWordWrap(true);
    quality_->setTextFormat(Qt::PlainText);
    layout->addWidget(quality_);
    profileStatus_ = new QLabel(this);
    profileStatus_->setObjectName(QStringLiteral("calibrationProfileStatus"));
    profileStatus_->setWordWrap(true);
    profileStatus_->setTextFormat(Qt::PlainText);
    layout->addWidget(profileStatus_);
    auto* profiles = new QHBoxLayout;
    saveProfile_ = new QPushButton(QStringLiteral("Save profile"), this);
    saveProfile_->setObjectName(QStringLiteral("calibrationSaveProfile"));
    profiles->addWidget(saveProfile_);
    reloadProfile_ = new QPushButton(QStringLiteral("Reload saved profile"), this);
    reloadProfile_->setObjectName(QStringLiteral("calibrationReloadProfile"));
    profiles->addWidget(reloadProfile_);
    undistorted_ = new QCheckBox(QStringLiteral("Undistorted preview"), this);
    undistorted_->setObjectName(QStringLiteral("calibrationUndistorted"));
    undistorted_->setToolTip(QStringLiteral("Verification only; captures use original pixels. Black borders are expected. Corner overlays follow the selected pixel mode."));
    profiles->addWidget(undistorted_);
    layout->addLayout(profiles);
    connect(saveProfile_, &QPushButton::clicked, &controller_, &CalibrationController::saveProfile);
    connect(reloadProfile_, &QPushButton::clicked, &controller_, &CalibrationController::reloadProfile);
    connect(undistorted_, &QCheckBox::toggled, &controller_, &CalibrationController::setUndistortedPreview);
    auto* overlayOptions = new QHBoxLayout;
    overlayOptions->addWidget(new QLabel(QStringLiteral("Live preview:"), this));
    overlayMode_ = new QComboBox(this);
    overlayMode_->setObjectName(QStringLiteral("calibrationOverlayMode"));
    overlayMode_->addItem(QStringLiteral("Hold until next detection"));
    overlayMode_->addItem(QStringLiteral("Track between detections"));
    overlayMode_->addItem(QStringLiteral("Matched image and corners (up to 5 Hz)"));
    overlayMode_->setToolTip(QStringLiteral("Hold keeps corners fixed and can lag a moving board. Track follows motion and hides uncertain corners. Matched advances image and corners together. Detection runs at up to 5 Hz in all modes. Capture validates its selected frame."));
    overlayOptions->addWidget(overlayMode_, 1);
    layout->addLayout(overlayOptions);
    connect(overlayMode_, &QComboBox::currentIndexChanged, this,
        [this](int index) { controller_.setLivePreviewMode(static_cast<CalibrationLivePreviewMode>(index)); });
    auto* review = new QHBoxLayout;
    views_ = new QComboBox(this);
    views_->setObjectName(QStringLiteral("calibrationViews"));
    review->addWidget(views_, 1);
    auto* live = new QPushButton(QStringLiteral("Live view"), this);
    live->setObjectName(QStringLiteral("calibrationLive"));
    review->addWidget(live);
    remove_ = new QPushButton(QStringLiteral("Remove / recapture selected"), this);
    remove_->setObjectName(QStringLiteral("calibrationRemove"));
    review->addWidget(remove_);
    layout->addLayout(review);
    auto* actions = new QHBoxLayout;
    auto* restart = new QPushButton(QStringLiteral("Start / restart capture"), this);
    restart->setObjectName(QStringLiteral("calibrationRestart"));
    actions->addWidget(restart);
    capture_ = new QPushButton(QStringLiteral("Capture view"), this);
    capture_->setObjectName(QStringLiteral("calibrationCapture"));
    actions->addWidget(capture_);
    finish_ = new QPushButton(QStringLiteral("Calculate intrinsics"), this);
    finish_->setObjectName(QStringLiteral("calibrationFinish"));
    actions->addWidget(finish_);
    auto* close = new QPushButton(QStringLiteral("Cancel / close"), this);
    actions->addWidget(close);
    layout->addLayout(actions);
    connect(restart, &QPushButton::clicked, &controller_, &CalibrationController::restart);
    connect(capture_, &QPushButton::clicked, &controller_, &CalibrationController::capture);
    connect(finish_, &QPushButton::clicked, &controller_, &CalibrationController::finish);
    connect(live, &QPushButton::clicked, &controller_, &CalibrationController::showLive);
    connect(remove_, &QPushButton::clicked, this, [this] { controller_.removeView(views_->currentIndex()); });
    connect(views_, &QComboBox::activated, &controller_, &CalibrationController::reviewView);
    connect(close, &QPushButton::clicked, this, &CalibrationWizard::reject);
    connect(&controller_, &CalibrationController::changed, this, &CalibrationWizard::refresh);
    connect(&controller_, &CalibrationController::previewChanged, this,
        [this](FramePtr frame, const std::vector<ImageCorner>& corners, bool accepted, bool approximate) {
            preview_->present(std::move(frame), corners, accepted, approximate);
        });
    refresh();
}
void CalibrationWizard::refresh()
{
    const auto& session = controller_.session();
    const auto progress = session.progress();
    const auto& config = session.config();
    message_->setText(controller_.message());
    const QSignalBlocker overlayBlocker(overlayMode_);
    overlayMode_->setCurrentIndex(static_cast<int>(controller_.livePreviewMode()));
    overlayMode_->setEnabled(!controller_.busy());
    if (auto* model = qobject_cast<QStandardItemModel*>(overlayMode_->model()))
        model->item(1)->setEnabled(controller_.canTrackLiveOverlay());
    QString profileText = controller_.profileMessage();
    if (const auto& profile = controller_.profile()) {
        profileText += QStringLiteral("\nCamera %1 · %2 × %3 px · RMS %4 px")
            .arg(QString::fromStdString(profile->sourceId)).arg(profile->width).arg(profile->height)
            .arg(profile->rmsPixels, 0, 'f', 4);
    }
    profileStatus_->setText(profileText);
    saveProfile_->setEnabled(controller_.canSaveProfile() && !controller_.busy());
    reloadProfile_->setEnabled(controller_.canReloadProfile() && !controller_.busy());
    undistorted_->setEnabled(controller_.canUndistort());
    const QSignalBlocker previewBlocker(undistorted_);
    undistorted_->setChecked(controller_.undistortedPreview());
    progress_->setText(QStringLiteral(
        "Views %1 / %2 minimum (%3 maximum) · Image regions %4 / %5\n"
        "Scale spread %6 / %7 · Rotation spread %8° / %9°%10")
        .arg(progress.count).arg(config.minimumViews).arg(config.maximumViews)
        .arg(progress.locationCells).arg(config.minimumLocationCells)
        .arg(progress.scaleSpread, 0, 'f', 2).arg(config.minimumScaleSpread, 0, 'f', 2)
        .arg(progress.angleSpreadRadians * 180 / std::numbers::pi, 0, 'f', 0)
        .arg(config.minimumAngleSpreadRadians * 180 / std::numbers::pi, 0, 'f', 0)
        .arg(session.state() == CalibrationCaptureState::Complete ? QStringLiteral(" · Complete") : QString{}));
    capture_->setEnabled(controller_.canCapture());
    finish_->setEnabled(!controller_.busy() && !controller_.profileBusy() && !controller_.result()
        && ((progress.ready && session.state() == CalibrationCaptureState::Collecting)
            || session.state() == CalibrationCaptureState::Complete));
    const auto& intrinsic = controller_.result();
    const auto threshold = controller_.intrinsicConfig().warningRmsPixels;
    QString quality = QStringLiteral("Reprojection RMS warning threshold: %1 px (aggregate or any view).")
        .arg(threshold, 0, 'f', 2);
    if (intrinsic) {
        const auto& k = intrinsic->cameraMatrix;
        quality += QStringLiteral("\nAggregate RMS: %1 px · %2\nfx %3 · fy %4 · cx %5 · cy %6 px")
            .arg(intrinsic->rmsPixels, 0, 'f', 4)
            .arg(hasReprojectionWarning(*intrinsic, controller_.intrinsicConfig())
                ? QStringLiteral("WARNING — inspect / recapture") : QStringLiteral("Within threshold"))
            .arg(k[0], 0, 'f', 2).arg(k[4], 0, 'f', 2).arg(k[2], 0, 'f', 2).arg(k[5], 0, 'f', 2);
    }
    quality_->setText(quality);
    remove_->setEnabled(!controller_.busy() && !controller_.profileBusy() && progress.count > 0
        && (session.state() == CalibrationCaptureState::Collecting
            || session.state() == CalibrationCaptureState::Complete));
    // Rebuild only when count changes, avoiding selector churn on live frames.
    if (views_->count() != static_cast<int>(progress.count)) {
        const QSignalBlocker blocker(views_);
        views_->clear();
        for (std::size_t i = 0; i < progress.count; ++i) {
            const auto& view = session.views()[i];
            views_->addItem(QStringLiteral("Accepted view %1 — frame #%2, sharpness %3")
                .arg(i + 1).arg(view.frame->sequence).arg(view.detection.sharpness, 0, 'f', 0));
        }
        views_->setCurrentIndex(views_->count() - 1);
    }
    if (intrinsic) {
        for (std::size_t i = 0; i < intrinsic->perViewErrors.size(); ++i) {
            const auto& error = intrinsic->perViewErrors[i];
            views_->setItemText(static_cast<int>(i), QStringLiteral("View %1 — frame #%2 · RMS %3 px%4")
                .arg(i + 1).arg(error.frameSequence).arg(error.rmsPixels, 0, 'f', 4)
                .arg(error.rmsPixels > threshold ? QStringLiteral(" · WARNING") : QString{}));
        }
    }
}
void CalibrationWizard::bindWorkspaceCalibration(WorkspaceCalibrationController& controller)
{
    auto* action = new QPushButton(QStringLiteral("Tabletop / workspace calibration…"), this);
    action->setObjectName(QStringLiteral("openWorkspaceCalibration"));
    qobject_cast<QVBoxLayout*>(layout())->insertWidget(1, action);
    workspace_ = new WorkspaceCalibrationDialog(controller, this);
    workspace_->setAttribute(Qt::WA_QuitOnClose, false);
    connect(action, &QPushButton::clicked, workspace_, [dialog = workspace_] { dialog->show(); dialog->raise(); });
}
CalibrationWizard::~CalibrationWizard()
{
    controller_.setLivePreviewActive(false);
}
void CalibrationWizard::showEvent(QShowEvent* event)
{
    QDialog::showEvent(event);
    controller_.setLivePreviewActive(true);
}
void CalibrationWizard::hideEvent(QHideEvent* event)
{
    controller_.setLivePreviewActive(false);
    QDialog::hideEvent(event);
}
void CalibrationWizard::reject()
{
    if (workspace_) workspace_->close();
    controller_.setUndistortedPreview(false);
    if (controller_.session().state() != CalibrationCaptureState::Complete) controller_.cancel();
    QDialog::reject();
}
} // namespace micro_robot
