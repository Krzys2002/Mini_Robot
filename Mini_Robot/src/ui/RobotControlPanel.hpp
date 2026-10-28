#pragma once
#include <QWidget>
#include <QPointer>

namespace micro_robot {
class DesktopController;
// Presentation only. The controller bounds commands and owns connection/input
// state. Loss of focus, deactivation, hide and destruction release manual input.
class RobotControlPanel final : public QWidget {
    Q_OBJECT
public:
    explicit RobotControlPanel(DesktopController& controller, QWidget* parent = nullptr);
    ~RobotControlPanel() override;
protected:
    void showEvent(QShowEvent* event) override;
    bool eventFilter(QObject* watched, QEvent* event) override;
private:
    DesktopController& controller_;
    QPointer<QWidget> watchedWindow_;
};
} // namespace micro_robot
