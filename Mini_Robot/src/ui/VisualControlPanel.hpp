#pragma once
#include "vision/VisionDebugFrame.hpp"
#include <QWidget>
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
private:
    LiveVisualController& control_;
    VisionOverlay* view_{};
    QString previewSource_, previewWorkspaceRevision_;
};
} // namespace micro_robot
