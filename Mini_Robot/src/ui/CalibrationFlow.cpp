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
    layout->addWidget(new QLabel(QStringLiteral("Choose a calibration to start"), chooser));
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

bool CalibrationFlow::close()
{
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
