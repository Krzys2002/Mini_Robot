#include "ui/MainWindow.hpp"
#include "ui/RobotControlPanel.hpp"
#include <QScrollArea>
#include "ui/CalibrationWizard.hpp"
#include "ui/VisionDebugView.hpp"
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
    resize(800, 760);

    auto* central = new QWidget(this);
    auto* rootLayout = new QVBoxLayout(central);
    auto* content = new QWidget(central);
    auto* layout = new QVBoxLayout(content);
    contentLayout_ = layout;
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

    auto* worldGroup = new QGroupBox(QStringLiteral("WorldModel — static demo pose"), central);
    auto* worldLayout = new QVBoxLayout(worldGroup);
    auto* worldDescription = new QLabel(QStringLiteral("No registered robots"), worldGroup);
    worldDescription->setObjectName(QStringLiteral("worldDescription"));
    worldDescription->setWordWrap(true);
    auto* convention = new QLabel(QStringLiteral(
        "Tabletop frame: origin at lower-left (0, 0) m; +X → right, +Y ↑ up.\n"
        "Heading: 0 rad along +X; positive counterclockwise (CCW)."), worldGroup);
    convention->setObjectName(QStringLiteral("worldCoordinateConvention"));
    convention->setWordWrap(true);
    auto* workspaceDescription = new QLabel(QStringLiteral("Workspace dimensions not configured"), worldGroup);
    workspaceDescription->setObjectName(QStringLiteral("workspaceDescription"));
    workspaceDescription->setWordWrap(true);
    // Wrapped labels must retain their text height when the window is resized.
    for (auto* label : {convention, workspaceDescription, worldDescription})
        label->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum);
    worldLayout->addWidget(convention);
    worldLayout->addWidget(workspaceDescription);
    worldLayout->addWidget(worldDescription);
    layout->addWidget(worldGroup);
    layout->addStretch();
    auto* scroll = new QScrollArea(central);
    scroll->setWidgetResizable(true);
    scroll->setMinimumHeight(160);
    scroll->setFocusPolicy(Qt::NoFocus);
    scroll->setWidget(content);
    rootLayout->addWidget(scroll, 1);
    // Manual Stop/E-stop stay visible when camera/world content must scroll.
    rootLayout->addWidget(new RobotControlPanel(controller, central));
    setCentralWidget(central);

    // activated fires for user choices only, never for programmatic updates.
    connect(cameraSelector_, &QComboBox::activated, this, [this](int index) {
        emit cameraSelectionRequested(cameraSelector_->itemData(index).toString());
    });
    connect(startCamera_, &QPushButton::clicked, this, &MainWindow::cameraStartRequested);
    connect(stopCamera_, &QPushButton::clicked, this, &MainWindow::cameraStopRequested);
    connect(this, &MainWindow::cameraStartRequested, &controller, &DesktopController::startCamera);
    connect(this, &MainWindow::cameraStopRequested, &controller, &DesktopController::stopCamera);

    connect(&controller, &DesktopController::applicationStatusChanged,
            applicationStatus, &QLabel::setText);
    connect(&controller, &DesktopController::frameDescriptionChanged,
            frameDescription, &QLabel::setText);
    connect(&controller, &DesktopController::worldDescriptionChanged, worldDescription, &QLabel::setText);
    connect(&controller, &DesktopController::workspaceDescriptionChanged, workspaceDescription, &QLabel::setText);
    connect(&controller, &DesktopController::activeCameraChanged, this,
            [activeCamera](const QString& description, bool) { activeCamera->setText(description); });
    connect(&controller, &DesktopController::cameraStatusChanged, this,
            [cameraStatus](const QString& status, bool) { cameraStatus->setText(status); });
    connect(&controller, &DesktopController::cameraActionsChanged, this,
            [this](bool canStart, bool canStop) {
        startCamera_->setEnabled(canStart);
        stopCamera_->setEnabled(canStop);
    });
    connect(&controller, &DesktopController::errorOccurred, this, [this](const QString& message) {
        statusBar()->showMessage(message);
    });
}

void MainWindow::closeEvent(QCloseEvent* event)
{
    if (calibrationWizard_) calibrationWizard_->close();
    if (visionDebug_) visionDebug_->close();
    QMainWindow::closeEvent(event);
}

void MainWindow::bindVision(VisionController& vision)
{
    auto* status = new QLabel(vision.description(), this);
    status->setObjectName(QStringLiteral("visionStatus"));
    status->setTextFormat(Qt::PlainText);
    status->setWordWrap(true);
    auto* layout = contentLayout_;
    layout->insertWidget(3, status);
    connect(&vision, &VisionController::descriptionChanged, status, &QLabel::setText);
    auto* action = new QPushButton(QStringLiteral("Vision debug…"), this);
    action->setObjectName(QStringLiteral("openVisionDebug"));
    visionAction_ = action;
    setTabOrder(cameraSelector_, action);
    layout->insertWidget(3, action);
    visionDebug_ = new VisionDebugDialog(vision, this);
    connect(action, &QPushButton::clicked, visionDebug_, [dialog = visionDebug_] {
        dialog->show();
        dialog->raise();
    });
}

void MainWindow::bindCalibration(CalibrationController& calibration)
{
    auto* action = new QPushButton(QStringLiteral("Camera calibration…"), this);
    action->setObjectName(QStringLiteral("openCalibration"));
    auto* layout = contentLayout_;
    layout->insertWidget(3, action);
    // Keyboard focus follows the visual order, after the camera selector.
    setTabOrder(cameraSelector_, action);
    if (visionAction_) setTabOrder(action, visionAction_);
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
