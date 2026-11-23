#pragma once

#include <QObject>
#include <QPointer>
#include <string>

class QDialog;
class QWidget;

namespace micro_robot {

class CalibrationController;
class CalibrationWizard;
class WorkspaceCalibrationController;
class WorkspaceCalibrationDialog;

class CalibrationFlow final : public QObject {
public:
    CalibrationFlow(CalibrationController& camera, QWidget* parent);
    void bindWorkspace(WorkspaceCalibrationController& workspace);
    void open();
    bool close();
    bool confirmCameraChange();

private:
    enum class Stage { Camera, Workspace, Tabletop, Robot, Other };
    [[nodiscard]] bool canStart(Stage stage) const;
    void showStage(Stage stage);

    QWidget* parent_;
    CalibrationController& camera_;
    WorkspaceCalibrationController* workspace_{};
    CalibrationWizard* cameraDialog_;
    WorkspaceCalibrationDialog* workspaceDialog_{};
    QPointer<QDialog> current_;
    std::string acceptedWorkspaceFrame_;
    std::string acceptedWorkspaceId_;
};

} // namespace micro_robot
