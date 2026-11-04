#pragma once

#include <QList>
#include <QMainWindow>
#include <QString>

class QVBoxLayout;
class QComboBox;
class QLabel;
class QPushButton;

namespace micro_robot {

class VisionController;
class VisionDebugDialog;
class CalibrationWizard;
class CalibrationController;
class WorkspaceCalibrationController;
class CameraSelectionController;
class DesktopController;
struct CameraChoiceItem;

class MainWindow final : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(DesktopController& controller, QWidget* parent = nullptr);

    void bindVision(VisionController& vision);

    // Opens the capture/review dialog for the injected calibration service.
    void bindCalibration(CalibrationController& calibration);
    void bindWorkspaceCalibration(WorkspaceCalibrationController& workspace);

    // Shows the controller's choices and selection and forwards user choices.
    void bindCameraSelection(CameraSelectionController& selection);

signals:
    void cameraSelectionRequested(const QString& id);
    void cameraStartRequested();
    void cameraStopRequested();

protected:
    void closeEvent(QCloseEvent* event) override;

private:
    void showCameraChoices(const QList<CameraChoiceItem>& cameras);
    void showSelectedCamera(const QString& id, const QString& description);

    // Non-owning; the Qt parent owns the widgets.
    QVBoxLayout* contentLayout_{};
    CalibrationWizard* calibrationWizard_{};
    VisionDebugDialog* visionDebug_{};
    QPushButton* visionAction_{};
    QComboBox* cameraSelector_{};
    QLabel* selectedCamera_{};
    QPushButton* startCamera_{};
    QPushButton* stopCamera_{};
    QString selectedCameraId_;
};

} // namespace micro_robot
