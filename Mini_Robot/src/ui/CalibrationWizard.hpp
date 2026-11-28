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
class CalibrationPreview;
// Presentation only: renders owned frames/corners and forwards operator actions.
class CalibrationWizard final : public QDialog {
    Q_OBJECT
public:
    explicit CalibrationWizard(CalibrationController& controller, QWidget* parent = nullptr,
                               bool previewOnly = false);
    ~CalibrationWizard() override;
    void enableGuidedMode();
protected:
    void reject() override;
    void showEvent(QShowEvent* event) override;
    void hideEvent(QHideEvent* event) override;
private:
    static constexpr double MillimetersPerMeter = 1000.0;
    void refresh();
    QPushButton* next_{};
    bool guided_{};
    const bool previewOnly_{};
    CalibrationController& controller_;
    CalibrationPreview* preview_{};
    QComboBox* views_{};
    QComboBox* overlayMode_{};
    QLabel* message_{};
    QLabel* progress_{};
    QLabel* guidance_{};
    QLabel* quality_{};
    QLabel* profileStatus_{};
    QPushButton* saveProfile_{};
    QPushButton* reloadProfile_{};
    QPushButton* saveFile_{};
    QPushButton* loadFile_{};
    QCheckBox* undistorted_{};
    QPushButton* restart_{};
    QPushButton* capture_{};
    QPushButton* remove_{};
    QPushButton* finish_{};
};
} // namespace micro_robot
