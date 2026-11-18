#pragma once
#include "vision/VisionDebugFrame.hpp"
#include "planning/MetricPath.hpp"
#include "common/RobotId.hpp"
#include <QWidget>
#include <QTimer>
#include <cstdint>
#include <optional>
#include <string>
class QLabel;
class QPushButton;
namespace micro_robot {
class LiveVisualController;
class VisionOverlay;
// Presentation only; coordinate validation, control and command ownership live
// in the application/domain services.
class VisualControlPanel final : public QWidget {
    Q_OBJECT
public:
    explicit VisualControlPanel(LiveVisualController& control, QWidget* parent = nullptr);
    void presentPreview(const QString& source, VisionDebugFramePtr preview);
    [[nodiscard]] bool beginPathEditing();
    [[nodiscard]] bool undoPathPoint();
    void clearPath();
    [[nodiscard]] bool finishPathEditing();
    void cancelPathEditing();
    [[nodiscard]] bool pathEditing() const noexcept { return pathEditing_; }
    [[nodiscard]] const MetricPath& draftPath() const noexcept { return draftPath_; }
    [[nodiscard]] const MetricPath& finishedPath() const noexcept { return finishedPath_; }
private:
    static constexpr int PreviewGateRefreshIntervalMs = 100;
    [[nodiscard]] bool currentPreviewCanEditPath() const;
    void refreshPathOverlay();
    void refreshPathControls();
    void invalidatePathScopeIfChanged();
    LiveVisualController& control_;
    VisionOverlay* view_{};
    QLabel* previewInstruction_{};
    QPushButton* pathBeginButton_{};
    QPushButton* pathUndoButton_{};
    QPushButton* pathClearButton_{};
    QPushButton* pathFinishButton_{};
    QPushButton* pathCancelButton_{};
    QPushButton* startButton_{};
    QTimer previewGateTimer_;
    QString previewSource_, previewWorkspaceRevision_;
    MetricPath draftPath_, finishedPath_;
    bool pathEditing_{};
    bool controlCanStart_{};
    std::optional<RobotId> pathRobot_;
    std::optional<std::string> pathFrame_;
    QString pathRevision_, pathSource_;
    std::uint32_t pathWidth_{}, pathHeight_{};
};
} // namespace micro_robot
