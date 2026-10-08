#pragma once

#include <QMainWindow>

namespace micro_robot {

class DesktopController;

class MainWindow final : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(DesktopController& controller, QWidget* parent = nullptr);

signals:
    void cameraStartRequested();
    void cameraStopRequested();
    void robotConnectRequested();
    void robotDisconnectRequested();
};

} // namespace micro_robot
