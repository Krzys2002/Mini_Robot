#pragma once
#include "app/WorkspaceCalibrationController.hpp"
#include <QDialog>
class QLineEdit;
class QDoubleSpinBox;
class QTableWidget;
class QLabel;
class QPushButton;

namespace micro_robot {
class WorkspaceImageView;

class WorkspaceCalibrationDialog final : public QDialog {
    Q_OBJECT
public:
    explicit WorkspaceCalibrationDialog(WorkspaceCalibrationController& controller,
                                        QWidget* parent = nullptr);

private:
    void refresh();
    void pointsChanged();
    void addPoint(std::optional<ImageCorner> point);
    WorkspaceCalibrationController& controller_;
    WorkspaceImageView* image_{};
    QLineEdit* identity_{};
    QDoubleSpinBox *width_{}, *height_{};
    QTableWidget* points_{};
    QLabel* message_{};
    QPushButton *freeze_{}, *calculate_{}, *save_{}, *load_{}, *verify_{}, *activate_{};
    FramePtr shownFrame_;
};
} // namespace micro_robot
