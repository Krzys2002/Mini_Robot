#pragma once

#include <QMainWindow>

class QLabel;
class QPushButton;

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
    void updateCameraButtons();

    // Non-owning; the Qt parent owns the widgets.
    QLabel* cameraDevicesDescription_{};
    QPushButton* startCamera_{};
    QPushButton* stopCamera_{};
    bool cameraPresent_{};
    bool cameraActive_{};
};

} // namespace micro_robot
