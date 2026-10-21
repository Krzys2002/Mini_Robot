#include "ui/MainWindow.hpp"
#include "ui/CalibrationWizard.hpp"
#include "app/CalibrationController.hpp"
#include "app/VisionController.hpp"
#include "app/CameraSelectionController.hpp"
#include "app/DesktopController.hpp"

#include <QComboBox>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QStandardItemModel>
#include <QStatusBar>
#include <QVBoxLayout>

namespace micro_robot {

MainWindow::MainWindow(DesktopController& controller, QWidget* parent)
    : QMainWindow(parent)
{
    setWindowTitle(QStringLiteral("Micro Robot"));
    resize(620, 460);

    auto* central = new QWidget(this);
    auto* layout = new QVBoxLayout(central);
    // Selection details can add wrapped lines; grow the window instead of
    // compressing controls below their usable size.
    layout->setSizeConstraint(QLayout::SetMinimumSize);
    auto* title = new QLabel(QStringLiteral("Micro Robot — Phase 0"), central);
    layout->addWidget(title);
    auto* applicationStatus = new QLabel(QStringLiteral("Starting…"), central);
    applicationStatus->setObjectName(QStringLiteral("applicationStatus"));
    layout->addWidget(applicationStatus);

    auto* deviceGroup = new QGroupBox(QStringLiteral("Camera selection"), central);
    auto* deviceLayout = new QVBoxLayout(deviceGroup);
    cameraSelector_ = new QComboBox(deviceGroup);
    cameraSelector_->setObjectName(QStringLiteral("cameraSelector"));
    cameraSelector_->setPlaceholderText(QStringLiteral("Looking for cameras…"));
    cameraSelector_->setEnabled(false);
    selectedCamera_ = new QLabel(QStringLiteral("No camera selected"), deviceGroup);
    selectedCamera_->setObjectName(QStringLiteral("selectedCamera"));
    selectedCamera_->setTextFormat(Qt::PlainText);
    selectedCamera_->setWordWrap(true);
    deviceLayout->addWidget(cameraSelector_);
    deviceLayout->addWidget(selectedCamera_);
    layout->addWidget(deviceGroup);

    auto* cameraGroup = new QGroupBox(QStringLiteral("Active camera source"), central);
    auto* cameraLayout = new QVBoxLayout(cameraGroup);
    auto* activeCamera = new QLabel(QStringLiteral("No camera selected"), cameraGroup);
    activeCamera->setObjectName(QStringLiteral("activeCamera"));
    activeCamera->setTextFormat(Qt::PlainText);
    auto* cameraStatus = new QLabel(QStringLiteral("No camera selected"), cameraGroup);
    cameraStatus->setObjectName(QStringLiteral("cameraStatus"));
    auto* frameDescription = new QLabel(QStringLiteral("No frames received"), cameraGroup);
    frameDescription->setObjectName(QStringLiteral("frameDescription"));
    auto* cameraButtons = new QHBoxLayout;
    startCamera_ = new QPushButton(QStringLiteral("Start camera"), cameraGroup);
    startCamera_->setObjectName(QStringLiteral("startCamera"));
    stopCamera_ = new QPushButton(QStringLiteral("Stop camera"), cameraGroup);
    stopCamera_->setObjectName(QStringLiteral("stopCamera"));
    startCamera_->setMinimumHeight(startCamera_->sizeHint().height());
    stopCamera_->setMinimumHeight(stopCamera_->sizeHint().height());
    startCamera_->setEnabled(false);
    stopCamera_->setEnabled(false);
    cameraButtons->addWidget(startCamera_);
    cameraButtons->addWidget(stopCamera_);
    cameraLayout->addWidget(activeCamera);
    cameraLayout->addWidget(cameraStatus);
    cameraLayout->addWidget(frameDescription);
    cameraLayout->addLayout(cameraButtons);
    layout->addWidget(cameraGroup);

    auto* transportGroup = new QGroupBox(QStringLiteral("Simulated robot transport"), central);
    auto* transportLayout = new QVBoxLayout(transportGroup);
    auto* connectionStatus = new QLabel(QStringLiteral("Disconnected"), transportGroup);
    connectionStatus->setObjectName(QStringLiteral("connectionStatus"));
    auto* telemetry = new QLabel(QStringLiteral("No active telemetry"), transportGroup);
    telemetry->setObjectName(QStringLiteral("telemetryDescription"));
    auto* transportButtons = new QHBoxLayout;
    auto* connectRobot = new QPushButton(QStringLiteral("Connect robot"), transportGroup);
    connectRobot->setObjectName(QStringLiteral("connectRobot"));
    auto* disconnectRobot = new QPushButton(QStringLiteral("Disconnect robot"), transportGroup);
    disconnectRobot->setObjectName(QStringLiteral("disconnectRobot"));
    disconnectRobot->setEnabled(false);
    transportButtons->addWidget(connectRobot);
    transportButtons->addWidget(disconnectRobot);
    transportLayout->addWidget(connectionStatus);
    transportLayout->addWidget(telemetry);
    transportLayout->addLayout(transportButtons);
    layout->addWidget(transportGroup);

    auto* worldGroup = new QGroupBox(QStringLiteral("WorldModel — static demo pose"), central);
    auto* worldLayout = new QVBoxLayout(worldGroup);
    auto* worldDescription = new QLabel(QStringLiteral("No registered robots"), worldGroup);
    worldDescription->setObjectName(QStringLiteral("worldDescription"));
    worldDescription->setWordWrap(true);
    worldLayout->addWidget(worldDescription);
    layout->addWidget(worldGroup);
    layout->addStretch();
    setCentralWidget(central);

    // activated fires for user choices only, never for programmatic updates.
    connect(cameraSelector_, &QComboBox::activated, this, [this](int index) {
        emit cameraSelectionRequested(cameraSelector_->itemData(index).toString());
    });
    connect(startCamera_, &QPushButton::clicked, this, &MainWindow::cameraStartRequested);
    connect(stopCamera_, &QPushButton::clicked, this, &MainWindow::cameraStopRequested);
    connect(connectRobot, &QPushButton::clicked, this, &MainWindow::robotConnectRequested);
    connect(disconnectRobot, &QPushButton::clicked, this, &MainWindow::robotDisconnectRequested);
    connect(this, &MainWindow::cameraStartRequested, &controller, &DesktopController::startCamera);
    connect(this, &MainWindow::cameraStopRequested, &controller, &DesktopController::stopCamera);
    connect(this, &MainWindow::robotConnectRequested, &controller, &DesktopController::connectRobot);
    connect(this, &MainWindow::robotDisconnectRequested, &controller, &DesktopController::disconnectRobot);

    connect(&controller, &DesktopController::applicationStatusChanged,
            applicationStatus, &QLabel::setText);
    connect(&controller, &DesktopController::frameDescriptionChanged,
            frameDescription, &QLabel::setText);
    connect(&controller, &DesktopController::telemetryDescriptionChanged, telemetry, &QLabel::setText);
    connect(&controller, &DesktopController::worldDescriptionChanged, worldDescription, &QLabel::setText);
    connect(&controller, &DesktopController::activeCameraChanged, this,
            [activeCamera](const QString& description, bool) { activeCamera->setText(description); });
    connect(&controller, &DesktopController::cameraStatusChanged, this,
            [cameraStatus](const QString& status, bool) { cameraStatus->setText(status); });
    connect(&controller, &DesktopController::cameraActionsChanged, this,
            [this](bool canStart, bool canStop) {
        startCamera_->setEnabled(canStart);
        stopCamera_->setEnabled(canStop);
    });
    connect(&controller, &DesktopController::connectionStatusChanged, this,
            [connectionStatus, connectRobot, disconnectRobot](const QString& status, bool, bool active) {
        connectionStatus->setText(status);
        connectRobot->setEnabled(!active);
        disconnectRobot->setEnabled(active);
    });
    connect(&controller, &DesktopController::errorOccurred, this, [this](const QString& message) {
        statusBar()->showMessage(message);
    });
}

void MainWindow::closeEvent(QCloseEvent* event)
{
    if (calibrationWizard_) calibrationWizard_->close();
    QMainWindow::closeEvent(event);
}

void MainWindow::bindVision(VisionController& vision)
{
    auto* status = new QLabel(vision.description(), this);
    status->setObjectName(QStringLiteral("visionStatus"));
    status->setTextFormat(Qt::PlainText);
    status->setWordWrap(true);
    auto* layout = qobject_cast<QVBoxLayout*>(centralWidget()->layout());
    layout->insertWidget(3, status);
    connect(&vision, &VisionController::descriptionChanged, status, &QLabel::setText);
}

void MainWindow::bindCalibration(CalibrationController& calibration)
{
    auto* action = new QPushButton(QStringLiteral("Camera calibration…"), this);
    action->setObjectName(QStringLiteral("openCalibration"));
    auto* layout = qobject_cast<QVBoxLayout*>(centralWidget()->layout());
    layout->insertWidget(3, action);
    // Keyboard focus follows the visual order, after the camera selector.
    setTabOrder(cameraSelector_, action);
    auto* wizard = new CalibrationWizard(calibration, this);
    calibrationWizard_ = wizard;
    // A child dialog must not trigger application shutdown on its own.
    wizard->setAttribute(Qt::WA_QuitOnClose, false);
    connect(action, &QPushButton::clicked, wizard, [wizard, &calibration] {
        wizard->show();
        wizard->raise();
        calibration.showLive();
    });
}

void MainWindow::bindCameraSelection(CameraSelectionController& selection)
{
    connect(this, &MainWindow::cameraSelectionRequested,
            &selection, &CameraSelectionController::selectCamera);
    connect(&selection, &CameraSelectionController::camerasChanged,
            this, &MainWindow::showCameraChoices);
    connect(&selection, &CameraSelectionController::selectionChanged,
            this, &MainWindow::showSelectedCamera);
    connect(&selection, &CameraSelectionController::errorOccurred, this, [this](const QString& message) {
        statusBar()->showMessage(message);
    });
}

void MainWindow::showCameraChoices(const QList<CameraChoiceItem>& cameras)
{
    cameraSelector_->clear();
    cameraSelector_->setPlaceholderText(cameras.isEmpty() ? QStringLiteral("No cameras available")
                                                          : QStringLiteral("Select a camera…"));
    cameraSelector_->setEnabled(!cameras.isEmpty());
    auto* model = qobject_cast<QStandardItemModel*>(cameraSelector_->model());
    for (const auto& camera : cameras) {
        cameraSelector_->addItem(camera.label, camera.id);
        const int index = cameraSelector_->count() - 1;
        cameraSelector_->setItemData(index, camera.detail, Qt::ToolTipRole);
        if (!camera.selectable && model) model->item(index)->setEnabled(false);
    }
    showSelectedCamera(selectedCameraId_, selectedCamera_->text());
}

void MainWindow::showSelectedCamera(const QString& id, const QString& description)
{
    selectedCameraId_ = id;
    cameraSelector_->setCurrentIndex(id.isEmpty() ? -1 : cameraSelector_->findData(id));
    selectedCamera_->setText(description);
}

} // namespace micro_robot
