#pragma once
#include "app/WorkspaceCalibrationController.hpp"
#include <QDialog>
#include "ui/WorkspaceLengthUnit.hpp"
#include <QPointer>
#include <limits>
class QLineEdit;
class QComboBox;
class QDoubleSpinBox;
class QTableWidget;
class QTableView;
class QLabel;
class QPushButton;

namespace micro_robot {
class WorkspaceImageEditor;
class WorkspaceImageView;
class WorkspaceReferenceModel;
class WorkspaceBoardPanel;
class WorkspaceDiagnosticsDialog;
class WorkspaceDistanceDialog;

class WorkspaceCalibrationDialog final : public QDialog {
    Q_OBJECT
public:
    enum class Stage { Combined, Workspace, Tabletop };
    void setStage(Stage stage);
    [[nodiscard]] Stage stage() const { return stage_; }
#ifdef MICRO_ROBOT_DEBUG_CALIBRATION_PREVIEW
    void showSampleFrame(FramePtr frame);
#endif
    explicit WorkspaceCalibrationDialog(WorkspaceCalibrationController& controller,
                                        QWidget* parent = nullptr);

private:
    [[nodiscard]] bool hasGeometryDraft() const;
    void syncFields();
    void refresh();
    void addPoint(std::optional<ImageCorner> point);
    void configure(bool confirmLoss = true);
    void restoreReferences();
    WorkspaceCalibrationController& controller_;
    WorkspaceImageEditor* editor_{};
    WorkspaceImageView* image_{};
#ifdef MICRO_ROBOT_DEBUG_CALIBRATION_PREVIEW
    bool sampleFrame_{};
#endif
    QLineEdit* identity_{};
    QLineEdit *width_{}, *height_{}, *originX_{}, *originY_{};
    QComboBox* units_{};
    WorkspaceLengthUnit unit_{WorkspaceLengthUnit::Meters};
    std::optional<WorkspaceBounds> shownBounds_;
    bool configuring_{};
    Stage stage_{Stage::Combined};
    QWidget *geometryFields_{}, *cornerTools_{}, *referenceTools_{};
    QLabel *instructions_{}, *imageDetails_{};
    QPushButton *next_{}, *localization_{}, *surveyedReferences_{};

    QString inputError_;
    void changeUnit(int index);
    void updateUnitLabels();
    QComboBox* originCorner_{};
    QTableWidget* corners_{};
    QTableView* points_{};
    WorkspaceReferenceModel* referenceModel_{};
    WorkspaceBoardPanel* board_{};
    QPointer<WorkspaceDiagnosticsDialog> diagnosticsDialog_;
    QPointer<WorkspaceDistanceDialog> distanceDialog_;
    QPushButton* distance_{};
    QPushButton* diagnostics_{};
    QLabel* message_{};
    QPushButton *saveFile_{}, *loadFile_{};
    QPushButton *freeze_{}, *calculate_{}, *save_{}, *load_{}, *verify_{}, *activate_{};
    QPushButton *confirm_{}, *undo_{}, *reset_{}, *reselect_{}, *originPick_{}, *add_{}, *remove_{};
    std::optional<std::size_t> replacingCorner_;
    std::uint64_t shownRevision_{std::numeric_limits<std::uint64_t>::max()};
    FramePtr shownFrame_;
    static constexpr double MaximumWorkspaceDimensionMeters = 1000;
};
} // namespace micro_robot
