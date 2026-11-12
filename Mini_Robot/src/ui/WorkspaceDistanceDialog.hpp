#pragma once
#include "app/WorkspaceDistanceController.hpp"
#include <QDialog>
class QComboBox;
class QDoubleSpinBox;
class QCheckBox;
class QLabel;
class QLineEdit;
class QPushButton;
namespace micro_robot
{
inline constexpr int WorkspaceDistanceCoordinateDecimals = 6;
inline constexpr double MinimumWorkspaceDistanceEditorMagnitudeMeters = 1000;
inline constexpr double MaximumWorkspaceSelectionRadiusPixels = 1e6;
inline constexpr double MaximumWorkspaceSelectionRadiusMeters = 1000;
class WorkspaceDistanceView;
class WorkspaceDistanceDialog final : public QDialog
{
    Q_OBJECT
public:
    explicit WorkspaceDistanceDialog(WorkspaceCalibrationController&, QWidget* parent = nullptr);

private:
    void refresh();
    void selectionEdited(std::size_t index, bool spaceChanged = false);
    WorkspaceDistanceController controller_;
    WorkspaceDistanceView* view_{};
    QComboBox *viewMode_{}, *target_{};
    struct EndpointControls {
        QDoubleSpinBox *x{}, *y{}, *radius{};
        QCheckBox* known{};
        QComboBox* space{};
        QLabel* native{};
        QLineEdit* justification{};
        QPushButton* apply{};
    };
    std::array<EndpointControls, WorkspaceDistanceEndpointCount> controls_;
    std::array<std::optional<WorldPoint2D>, WorkspaceDistanceEndpointCount> shownWorld_;
    bool initialized_{};
    QLabel *identity_{}, *distance_{}, *uncertainty_{}, *status_{};
    QPushButton* freeze_{};
};
} // namespace micro_robot
