#include "ui/MainWindow.hpp"
#include "app/DesktopController.hpp"

#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
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
    auto* title = new QLabel(QStringLiteral("Micro Robot — Phase 0"), central);
    layout->addWidget(title);
    auto* applicationStatus = new QLabel(QStringLiteral("Starting…"), central);
    applicationStatus->setObjectName(QStringLiteral("applicationStatus"));
    layout->addWidget(applicationStatus);

    auto* deviceGroup = new QGroupBox(QStringLiteral("Available system cameras"), central);
    auto* deviceLayout = new QVBoxLayout(deviceGroup);
    cameraDevicesDescription_ = new QLabel(QStringLiteral("Looking for cameras…"), deviceGroup);
    cameraDevicesDescription_->setObjectName(QStringLiteral("cameraDevicesDescription"));
    cameraDevicesDescription_->setTextFormat(Qt::PlainText);
    cameraDevicesDescription_->setWordWrap(true);
    deviceLayout->addWidget(cameraDevicesDescription_);
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
    updateCameraButtons();
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
            [this, activeCamera](const QString& description, bool present) {
        activeCamera->setText(description);
        cameraPresent_ = present;
        updateCameraButtons();
    });
    connect(&controller, &DesktopController::cameraStatusChanged, this,
            [this, cameraStatus](const QString& status, bool active) {
        cameraStatus->setText(status);
        cameraActive_ = active;
        updateCameraButtons();
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

void MainWindow::updateCameraButtons()
{
    startCamera_->setEnabled(cameraPresent_ && !cameraActive_);
    stopCamera_->setEnabled(cameraPresent_ && cameraActive_);
}

void MainWindow::setCameraDevicesDescription(const QString& description)
{
    cameraDevicesDescription_->setText(description);
}

} // namespace micro_robot
