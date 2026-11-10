#pragma once
#include "camera/CameraTypes.hpp"
#include "calibration/CalibrationCapture.hpp"

#include <QDialog>

class QComboBox;
class QCheckBox;
class QLabel;
class QPushButton;

namespace micro_robot {
class CalibrationController;
class WorkspaceCalibrationController;
class WorkspaceCalibrationDialog;
class CalibrationPreview;
// Presentation only: renders owned frames/corners and forwards operator actions.
class CalibrationWizard final : public QDialog {
    Q_OBJECT
public:
    explicit CalibrationWizard(CalibrationController& controller, QWidget* parent = nullptr);
    ~CalibrationWizard() override;
    void bindWorkspaceCalibration(WorkspaceCalibrationController& controller);
protected:
    void reject() override;
    void showEvent(QShowEvent* event) override;
    void hideEvent(QHideEvent* event) override;
private:
    void refresh();
    WorkspaceCalibrationDialog* workspace_{};
    CalibrationController& controller_;
    CalibrationPreview* preview_{};
    QComboBox* views_{};
    QLabel* message_{};
    QLabel* progress_{};
    QLabel* quality_{};
    QLabel* profileStatus_{};
    QPushButton* saveProfile_{};
    QPushButton* reloadProfile_{};
    QCheckBox* undistorted_{};
    QPushButton* capture_{};
    QPushButton* remove_{};
    QPushButton* finish_{};
};
} // namespace micro_robot
