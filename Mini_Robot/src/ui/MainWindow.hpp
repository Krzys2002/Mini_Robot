#pragma once

#include <QMainWindow>

class QLabel;

namespace micro_robot {

class DesktopController;

class MainWindow final : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(DesktopController& controller, QWidget* parent = nullptr);

public slots:
    void setCameraDevicesDescription(const QString& description);

signals:
    void cameraStartRequested();
    void cameraStopRequested();
    void robotConnectRequested();
    void robotDisconnectRequested();

private:
    QLabel* cameraDevicesDescription_{}; // Non-owning; Qt parent owns the label.
};

} // namespace micro_robot
