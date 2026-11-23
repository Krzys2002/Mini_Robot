#include "ui/CalibrationFlow.hpp"

#include "app/CalibrationController.hpp"
#include "app/WorkspaceCalibrationController.hpp"
#include "ui/CalibrationForm.hpp"
#include "ui/CalibrationWizard.hpp"
#include "ui/ElevatedLocalizationDialog.hpp"
#include "ui/ReferenceMarkersDialog.hpp"
#include "ui/WorkspaceCalibrationDialog.hpp"

#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <array>

#ifdef MICRO_ROBOT_DEBUG_CALIBRATION_PREVIEW
#include <QAbstractButton>
#include <QAbstractItemView>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QImage>
#include <QPainter>
#include <QPainterPath>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QTabWidget>
#endif

namespace micro_robot {

CalibrationFlow::CalibrationFlow(CalibrationController& camera, QWidget* parent)
    : QObject(parent), parent_(parent), camera_(camera),
      cameraDialog_(new CalibrationWizard(camera, parent))
{
    cameraDialog_->enableGuidedMode();
    cameraDialog_->setAttribute(Qt::WA_QuitOnClose, false);
    connect(cameraDialog_, &QDialog::accepted, this, [this] { showStage(Stage::Workspace); });
}

void CalibrationFlow::bindWorkspace(WorkspaceCalibrationController& workspace)
{
    workspace_ = &workspace;
    workspaceDialog_ = new WorkspaceCalibrationDialog(workspace, parent_);
    workspaceDialog_->setAttribute(Qt::WA_QuitOnClose, false);
    connect(workspace_, &WorkspaceCalibrationController::changed, this, [this] {
        if (!workspace_->boundaryConfirmed() && !workspace_->draftProfile()) {
            acceptedWorkspaceFrame_.clear();
            acceptedWorkspaceId_.clear();
        }
    });
    connect(workspaceDialog_, &QDialog::accepted, this, [this] {
        if (workspaceDialog_->stage() == WorkspaceCalibrationDialog::Stage::Workspace) {
            acceptedWorkspaceFrame_ = workspace_->frameId();
            acceptedWorkspaceId_ = workspace_->workspaceId().toStdString();
            showStage(Stage::Tabletop);
        } else {
            showStage(Stage::Robot);
        }
    });
}

bool CalibrationFlow::canStart(Stage stage) const
{
    if (!camera_.isStreaming()) return false;
    if (stage == Stage::Camera) return true;
    if (!workspace_ || !camera_.profile()) return false;
    if (stage == Stage::Workspace) return true;

    const auto& active = workspace_->profile();
    const auto& draft = workspace_->draftProfile();
    const bool acceptedBoundary = acceptedWorkspaceFrame_ == workspace_->frameId()
        && acceptedWorkspaceId_ == workspace_->workspaceId().toStdString()
        && (workspace_->boundaryConfirmed() || draft);
    const bool acceptedTable = active && draft && active->homography && draft->homography
        && active->frameId == draft->frameId && active->frameId == workspace_->frameId()
        && active->acquisitionId == draft->acquisitionId
        && active->homography->imageToWorld == draft->homography->imageToWorld;
    if (stage == Stage::Tabletop) return acceptedBoundary || acceptedTable;
    if (stage == Stage::Robot) return acceptedTable;
    return acceptedTable && active->localization && draft->localization;
}

void CalibrationFlow::open()
{
    if (!canStart(Stage::Camera)) return;
    if (current_ && current_->isVisible()) {
        current_->raise();
        current_->activateWindow();
        return;
    }
    if (!canStart(Stage::Workspace)) {
        showStage(Stage::Camera);
        return;
    }
    auto* chooser = new QDialog(parent_);
    current_ = chooser;
    chooser->setWindowTitle(QStringLiteral("Calibration"));
    chooser->setObjectName(QStringLiteral("calibrationChooser"));
    chooser->setAttribute(Qt::WA_DeleteOnClose);
    chooser->setAttribute(Qt::WA_QuitOnClose, false);
    auto* layout = new QVBoxLayout(chooser);
    auto* guidance = new QLabel(chooser);
    const auto updateGuidance = [this, guidance] {
        guidance->setText(camera_.isStreaming() ? QStringLiteral("Choose a calibration to start")
                                               : QStringLiteral("Start the camera to continue calibration."));
    };
    connect(&camera_, &CalibrationController::changed, guidance, updateGuidance);
    updateGuidance();
    layout->addWidget(guidance);
    constexpr std::array stages{Stage::Camera, Stage::Workspace, Stage::Tabletop, Stage::Robot, Stage::Other};
    constexpr std::array labels{"Camera", "Workspace", "Tabletop", "Robot", "Surveyed references"};
    constexpr std::array names{"chooseCameraCalibration", "chooseWorkspaceCalibration", "chooseTabletopCalibration",
                               "chooseRobotCalibration", "chooseOtherCalibration"};
    for (std::size_t i = 0; i < stages.size(); ++i) {
        const auto stage = stages[i];
        auto* button = new QPushButton(QString::fromLatin1(labels[i]), chooser);
        button->setObjectName(QString::fromLatin1(names[i]));
        const auto refresh = [this, stage, button] { button->setEnabled(canStart(stage)); };
        connect(&camera_, &CalibrationController::changed, button, refresh);
        connect(workspace_, &WorkspaceCalibrationController::changed, button, refresh);
        refresh();
        connect(button, &QPushButton::clicked, chooser, [this, chooser, stage] {
            if (!canStart(stage)) return;
            chooser->accept();
            showStage(stage);
        });
        layout->addWidget(button);
    }
    chooser->show();
}

void CalibrationFlow::showStage(Stage stage)
{
    if (!canStart(stage)) return;
    switch (stage) {
    case Stage::Camera:
        current_ = cameraDialog_;
        camera_.showLive();
        break;
    case Stage::Workspace:
        workspaceDialog_->setStage(WorkspaceCalibrationDialog::Stage::Workspace);
        current_ = workspaceDialog_;
        break;
    case Stage::Tabletop:
        workspaceDialog_->setStage(WorkspaceCalibrationDialog::Stage::Tabletop);
        current_ = workspaceDialog_;
        break;
    case Stage::Robot: {
        auto* dialog = new ElevatedLocalizationDialog(*workspace_, parent_, true);
        dialog->setObjectName(QStringLiteral("robotCalibration"));
        dialog->setAttribute(Qt::WA_DeleteOnClose);
        dialog->setAttribute(Qt::WA_QuitOnClose, false);
        connect(dialog, &QDialog::accepted, this, [this] { showStage(Stage::Other); });
        current_ = dialog;
        break;
    }
    case Stage::Other: {
        auto* dialog = new ReferenceMarkersDialog(*workspace_, parent_, true);
        dialog->setObjectName(QStringLiteral("otherCalibration"));
        dialog->setAttribute(Qt::WA_DeleteOnClose);
        dialog->setAttribute(Qt::WA_QuitOnClose, false);
        current_ = dialog;
        break;
    }
    }
    current_->show();
    current_->raise();
}

#ifdef MICRO_ROBOT_DEBUG_CALIBRATION_PREVIEW
namespace {
FramePtr makeCalibrationSampleFrame()
{
    constexpr unsigned width = 960;
    constexpr unsigned height = 600;
    QImage sample(width, height, QImage::Format_RGB888);
    sample.fill(QColor(30, 39, 50));
    {
        QPainter paint(&sample);
        paint.setRenderHint(QPainter::Antialiasing);
        const QPolygonF table{{168, 92}, {796, 132}, {902, 510}, {64, 538}};
        paint.setPen(QPen(QColor(193, 169, 119), 4));
        paint.setBrush(QColor(227, 210, 167));
        paint.drawPolygon(table);
        paint.save();
        QPainterPath footprint;
        footprint.addPolygon(table);
        paint.setClipPath(footprint);
        constexpr int gridSpacing = 32;
        paint.setPen(QPen(QColor(174, 153, 115), 1));
        for (int x = 0; x < static_cast<int>(width); x += gridSpacing)
            paint.drawLine(x, 0, x, static_cast<int>(height));
        for (int y = 0; y < static_cast<int>(height); y += gridSpacing)
            paint.drawLine(0, y, static_cast<int>(width), y);
        paint.restore();

        const std::array markers{QPointF(220, 144), QPointF(734, 183), QPointF(819, 460), QPointF(138, 480)};
        const std::array colors{QColor(172, 44, 50), QColor(47, 101, 158), QColor(53, 118, 71), QColor(100, 61, 138)};
        for (std::size_t i = 0; i < markers.size(); ++i) {
            const QRectF target(markers[i] - QPointF(24, 24), QSizeF(48, 48));
            paint.setPen(Qt::NoPen);
            paint.setBrush(colors[i]);
            paint.drawRoundedRect(target, 6, 6);
            paint.setPen(Qt::white);
            paint.drawText(target, Qt::AlignCenter, QString::number(i + 1));
        }
    }
    const auto* begin = reinterpret_cast<const std::byte*>(sample.constBits());
    auto pixels = std::make_shared<const std::vector<std::byte>>(begin, begin + sample.sizeInBytes());
    return std::make_shared<Frame>(Frame{0, {}, width, height, static_cast<std::size_t>(sample.bytesPerLine()),
                                         PixelFormat::Rgb8, std::move(pixels)});
}
} // namespace

void CalibrationFlow::openPreview()
{
    if (preview_) {
        preview_->raise();
        preview_->activateWindow();
        return;
    }

    auto* dialog = new QDialog(parent_);
    preview_ = dialog;
    dialog->setObjectName(QStringLiteral("calibrationUiPreview"));
    dialog->setWindowTitle(QStringLiteral("Debug calibration preview"));
    dialog->setAttribute(Qt::WA_DeleteOnClose);
    dialog->setAttribute(Qt::WA_QuitOnClose, false);
    dialog->setWindowModality(Qt::WindowModal);
    dialog->resize(940, 800);
    auto* layout = new QVBoxLayout(dialog);
    auto* description = new QLabel(QStringLiteral(
        "Calibration preview. Workspace and Tabletop have interactive sample frames and temporary points."), dialog);
    description->setWordWrap(true);
    layout->addWidget(description);
    auto* tabs = new QTabWidget(dialog);
    tabs->setObjectName(QStringLiteral("calibrationPreviewStages"));
    layout->addWidget(tabs);

    const auto addPage = [this, tabs](QDialog* page, const QString& label) {
        page->setWindowFlags(Qt::Widget);
        const auto lockControls = [page] {
            for (auto* button : page->findChildren<QAbstractButton*>()) {
                const auto& name = button->objectName();
                if (name != QStringLiteral("calibrationHelpToggle") && !name.startsWith(QStringLiteral("workspaceImage")))
                    button->setEnabled(false);
            }
            for (auto* field : page->findChildren<QLineEdit*>()) field->setReadOnly(true);
            for (auto* field : page->findChildren<QPlainTextEdit*>()) field->setReadOnly(true);
            for (auto* field : page->findChildren<QComboBox*>()) field->setEnabled(false);
            for (auto* table : page->findChildren<QAbstractItemView*>()) table->setEnabled(false);
            if (auto* image = page->findChild<QWidget*>(QStringLiteral("workspaceBoardZoom"))) image->setEnabled(false);
        };
        tabs->addTab(page, label);
        // Apply after each page's own refresh, which also runs during live updates.
        connect(&camera_, &CalibrationController::changed, page, lockControls);
        if (workspace_) connect(workspace_, &WorkspaceCalibrationController::changed, page, lockControls);
        lockControls();
    };
    auto* camera = new CalibrationWizard(camera_, tabs, true);
    camera->enableGuidedMode();
    addPage(camera, QStringLiteral("Camera"));
    if (workspace_) {
        const auto frame = makeCalibrationSampleFrame();
        auto* workspace = new WorkspaceCalibrationDialog(*workspace_, tabs);
        workspace->setStage(WorkspaceCalibrationDialog::Stage::Workspace);
        workspace->showSampleFrame(frame);
        addPage(workspace, QStringLiteral("Workspace"));
        auto* tabletop = new WorkspaceCalibrationDialog(*workspace_, tabs);
        tabletop->setStage(WorkspaceCalibrationDialog::Stage::Tabletop);
        tabletop->showSampleFrame(frame);
        addPage(tabletop, QStringLiteral("Tabletop"));
        addPage(new ElevatedLocalizationDialog(*workspace_, tabs, true), QStringLiteral("Robot"));
        addPage(new ReferenceMarkersDialog(*workspace_, tabs, true), QStringLiteral("Surveyed references"));
    }
    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Close, dialog);
    connect(buttons, &QDialogButtonBox::rejected, dialog, &QDialog::reject);
    layout->addWidget(buttons);
    dialog->show();
}
#endif

bool CalibrationFlow::close()
{
#ifdef MICRO_ROBOT_DEBUG_CALIBRATION_PREVIEW
    if (preview_) preview_->close();
#endif
    if (current_ && current_->isVisible() && !current_->close()) return false;
    if (camera_.result() || (workspace_ && (workspace_->draftBounds() || workspace_->frozenFrame()
        || workspace_->draftProfile() || workspace_->busy()))) {
        return confirmCalibrationLoss(parent_, "Close calibration session",
            "Any calibration work that has not been saved will be lost when the application closes.");
    }
    return true;
}

bool CalibrationFlow::confirmCameraChange()
{
    if (camera_.session().views().empty() && !camera_.busy() && !camera_.profile()) return true;
    return confirmCalibrationLoss(parent_, "Change camera input",
        "Changing or stopping the camera clears its current capture and dependent calibration from this session.");
}

} // namespace micro_robot
