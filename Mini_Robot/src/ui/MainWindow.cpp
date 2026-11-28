#include "ui/MainWindow.hpp"
#include "ui/RobotControlPanel.hpp"
#include "ui/VisualControlPanel.hpp"
#include <QScrollArea>
#include "ui/CalibrationFlow.hpp"
#include <QCloseEvent>
#include "ui/VisionDebugView.hpp"
#include "ui/TopViewDialog.hpp"
#include "ui/WorldModelCanvas.hpp"
#include "app/CalibrationController.hpp"
#include "app/WorkspaceCalibrationController.hpp"
#include "app/VisionController.hpp"
#include "app/CameraSelectionController.hpp"
#include "app/DesktopController.hpp"

#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QStandardItemModel>
#include <QStatusBar>
#include <QVBoxLayout>

namespace micro_robot {
namespace {
QGroupBox* createWorldModelPanel(DesktopController& controller, QWidget* parent, const QString& nameSuffix = {})
{
    auto* worldGroup = new QGroupBox(QStringLiteral("World Model View"), parent);
    worldGroup->setObjectName(QStringLiteral("worldModelView") + nameSuffix);
    auto* worldLayout = new QVBoxLayout(worldGroup);
    auto* worldDescription = new QLabel(QStringLiteral("No registered robots"), worldGroup);
    worldDescription->setObjectName(QStringLiteral("worldDescription") + nameSuffix);
    worldDescription->setWordWrap(true);
    auto* convention = new QLabel(QStringLiteral(
        "Tabletop frame: explicit calibrated origin (0, 0) m; +X → right, +Y ↑ up in World Model View.\n"
        "Heading: 0 rad along +X; positive counterclockwise (CCW)."), worldGroup);
    convention->setObjectName(QStringLiteral("worldCoordinateConvention") + nameSuffix);
    convention->setWordWrap(true);
    auto* workspaceDescription = new QLabel(QStringLiteral("Workspace dimensions not configured"), worldGroup);
    workspaceDescription->setObjectName(QStringLiteral("workspaceDescription") + nameSuffix);
    workspaceDescription->setWordWrap(true);
    workspaceDescription->setTextFormat(Qt::PlainText);
    // Wrapped labels must retain their text height when the window is resized.
    for (auto* label : {convention, workspaceDescription, worldDescription}) {
        auto policy = label->sizePolicy();
        policy.setVerticalPolicy(QSizePolicy::Minimum);
        label->setSizePolicy(policy);
    }
    worldLayout->addWidget(convention);
    worldLayout->addWidget(workspaceDescription);
    worldLayout->addWidget(worldDescription);
    QObject::connect(&controller, &DesktopController::worldDescriptionChanged, worldDescription, &QLabel::setText);
    QObject::connect(&controller, &DesktopController::workspaceDescriptionChanged, workspaceDescription, &QLabel::setText);
    return worldGroup;
}
} // namespace

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

    viewActions_ = new QHBoxLayout;
    worldViewAction_ = new QPushButton(QStringLiteral("World View…"), content);
    worldViewAction_->setObjectName(QStringLiteral("openWorldView"));
    viewActions_->addWidget(worldViewAction_);
    layout->addLayout(viewActions_);
    layout->addWidget(createWorldModelPanel(controller, content));

    worldView_ = new QDialog(this);
    worldView_->setObjectName(QStringLiteral("worldModelViewDialog"));
    worldView_->setWindowTitle(QStringLiteral("World Model View"));
    worldView_->setAttribute(Qt::WA_QuitOnClose, false);
    worldView_->resize(800, 620);
    auto* worldViewLayout = new QVBoxLayout(worldView_);
    worldViewLayout->addWidget(new WorldModelCanvas(controller, worldView_), 1);
    auto* worldScroll = new QScrollArea(worldView_);
    worldScroll->setWidgetResizable(true);
    worldScroll->setMaximumHeight(150);
    auto* worldContent = new QWidget(worldScroll);
    auto* worldContentLayout = new QVBoxLayout(worldContent);
    worldContentLayout->addWidget(createWorldModelPanel(controller, worldContent, QStringLiteral("Dialog")));
    worldContentLayout->addStretch();
    worldScroll->setWidget(worldContent);
    worldViewLayout->addWidget(worldScroll);
    auto* worldButtons = new QDialogButtonBox(QDialogButtonBox::Close, worldView_);
    worldViewLayout->addWidget(worldButtons);
    connect(worldButtons, &QDialogButtonBox::rejected, worldView_, &QDialog::reject);
    connect(worldViewAction_, &QPushButton::clicked, worldView_, [view = worldView_] {
        view->show();
        view->raise();
        view->activateWindow();
    });
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
        const auto id = cameraSelector_->itemData(index).toString();
        if (id == selectedCameraId_) return;
        if (calibrationFlow_ && !calibrationFlow_->confirmCameraChange()) {
            cameraSelector_->setCurrentIndex(cameraSelector_->findData(selectedCameraId_));
            return;
        }
        emit cameraSelectionRequested(id);
    });
    connect(startCamera_, &QPushButton::clicked, this, &MainWindow::cameraStartRequested);
    connect(stopCamera_, &QPushButton::clicked, this, [this] {
        if (!calibrationFlow_ || calibrationFlow_->confirmCameraChange()) emit cameraStopRequested();
    });
    connect(this, &MainWindow::cameraStartRequested, &controller, &DesktopController::startCamera);
    connect(this, &MainWindow::cameraStopRequested, &controller, &DesktopController::stopCamera);

    connect(&controller, &DesktopController::applicationStatusChanged,
            applicationStatus, &QLabel::setText);
    connect(&controller, &DesktopController::frameDescriptionChanged,
            frameDescription, &QLabel::setText);
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
    if (calibrationFlow_ && !calibrationFlow_->close()) {
        event->ignore();
        return;
    }
    if (visionDebug_) visionDebug_->close();
    if (topView_) topView_->close();
    worldView_->close();
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

void MainWindow::bindTopView(DesktopController& desktop, WorkspaceCalibrationController& workspace,
                             std::unique_ptr<ICalibrationUndistorter> undistorter)
{
    auto* view = new TopViewDialog(std::move(undistorter), this);
    topView_ = view;
    auto* action = new QPushButton(QStringLiteral("Top View…"), worldViewAction_->parentWidget());
    action->setObjectName(QStringLiteral("openTopView"));
    viewActions_->insertWidget(0, action);
    setTabOrder(stopCamera_, action);
    setTabOrder(action, worldViewAction_);
    connect(action, &QPushButton::clicked, view, [view] {
        view->show();
        view->raise();
        view->activateWindow();
    });
    connect(&desktop, &DesktopController::calibrationInputChanged, view,
            [view](const QString& identity, bool streaming, bool) { view->setInput(identity, streaming); });
    connect(&desktop, &DesktopController::frameAvailable, view, &TopViewDialog::receiveFrame);
    connect(&workspace, &WorkspaceCalibrationController::profileChanged, view,
            [view, &workspace] { view->setCalibration(workspace.profile()); });
    view->setCalibration(workspace.profile());
}

void MainWindow::bindVisualControl(LiveVisualController& control, VisionController& vision)
{
    auto* panel = new VisualControlPanel(control, this);
    contentLayout_->insertWidget(contentLayout_->count() - 1, panel);
    connect(&vision, &VisionController::previewAvailable, panel, &VisualControlPanel::presentPreview);
}

void MainWindow::bindCalibration(CalibrationController& calibration)
{
    auto* action = new QPushButton(QStringLiteral("Calibration…"), this);
    action->setObjectName(QStringLiteral("openCalibration"));
    auto* layout = contentLayout_;
    layout->insertWidget(3, action);
    // Keyboard focus follows the visual order, after the camera selector.
    setTabOrder(cameraSelector_, action);
    if (visionAction_) setTabOrder(action, visionAction_);
    const auto updateCalibrationAction = [&calibration, action] {
        action->setEnabled(calibration.isStreaming());
        action->setToolTip(calibration.isStreaming() ? QString{}
            : QStringLiteral("Start the camera to enable calibration."));
    };
    connect(&calibration, &CalibrationController::changed, action, updateCalibrationAction);
    updateCalibrationAction();
    calibrationFlow_ = new CalibrationFlow(calibration, this);
    connect(action, &QPushButton::clicked, calibrationFlow_, &CalibrationFlow::open);
#ifdef MICRO_ROBOT_DEBUG_CALIBRATION_PREVIEW
    auto* preview = new QPushButton(QStringLiteral("Debug: preview calibration screens…"), this);
    preview->setObjectName(QStringLiteral("openCalibrationUiPreview"));
    layout->insertWidget(layout->indexOf(action) + 1, preview);
    setTabOrder(action, preview);
    if (visionAction_) setTabOrder(preview, visionAction_);
    connect(preview, &QPushButton::clicked, calibrationFlow_, &CalibrationFlow::openPreview);
#endif
}

void MainWindow::bindWorkspaceCalibration(WorkspaceCalibrationController& workspace)
{
    if (calibrationFlow_) calibrationFlow_->bindWorkspace(workspace);
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
