#pragma once
#include "app/WorkspaceCalibrationController.hpp"
#include <QDialog>
#include <limits>
class QLineEdit;
class QComboBox;
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
    void configure();
    void restoreReferences();
    WorkspaceCalibrationController& controller_;
    WorkspaceImageView* image_{};
    QLineEdit* identity_{};
    QDoubleSpinBox *width_{}, *height_{}, *originX_{}, *originY_{};
    QComboBox* originCorner_{};
    QTableWidget *points_{}, *corners_{};
    QLabel* message_{};
    QPushButton *freeze_{}, *calculate_{}, *save_{}, *load_{}, *verify_{}, *activate_{};
    QPushButton *confirm_{}, *undo_{}, *reset_{}, *reselect_{}, *originPick_{}, *add_{}, *remove_{};
    std::optional<std::size_t> replacingCorner_;
    std::uint64_t shownRevision_{std::numeric_limits<std::uint64_t>::max()};
    FramePtr shownFrame_;
    static constexpr int MaximumReferenceRows = 128;
    static constexpr double MaximumWorkspaceDimensionMeters = 1000;
    static constexpr int CoordinateDisplayDecimals = 9;
};
} // namespace micro_robot
