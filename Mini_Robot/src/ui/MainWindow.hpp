#pragma once

#include <QList>
#include <QMainWindow>
#include <QString>

class QComboBox;
class QLabel;
class QPushButton;

namespace micro_robot {

class CameraSelectionController;
class DesktopController;
struct CameraChoiceItem;

class MainWindow final : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(DesktopController& controller, QWidget* parent = nullptr);

    // Shows the controller's choices and selection and forwards user choices.
    void bindCameraSelection(CameraSelectionController& selection);

signals:
    void cameraSelectionRequested(const QString& id);
    void cameraStartRequested();
    void cameraStopRequested();
    void robotConnectRequested();
    void robotDisconnectRequested();

private:
    void showCameraChoices(const QList<CameraChoiceItem>& cameras);
    void showSelectedCamera(const QString& id, const QString& description);

    // Non-owning; the Qt parent owns the widgets.
    QComboBox* cameraSelector_{};
    QLabel* selectedCamera_{};
    QPushButton* startCamera_{};
    QPushButton* stopCamera_{};
    QString selectedCameraId_;
};

} // namespace micro_robot
