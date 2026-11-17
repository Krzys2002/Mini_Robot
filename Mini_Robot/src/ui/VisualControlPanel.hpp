#pragma once
#include <QWidget>
namespace micro_robot {
class LiveVisualController;
// Presentation only; coordinate validation, control and command ownership live
// in the application/domain services.
class VisualControlPanel final : public QWidget {
    Q_OBJECT
public:
    explicit VisualControlPanel(LiveVisualController& control, QWidget* parent = nullptr);
};
} // namespace micro_robot
