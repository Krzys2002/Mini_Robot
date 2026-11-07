#include "ui/RobotControlPanel.hpp"
#include "app/DesktopController.hpp"

#include <QApplication>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QEvent>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QSignalBlocker>
#include <QShowEvent>
#include <QStandardItemModel>

namespace micro_robot {
RobotControlPanel::RobotControlPanel(DesktopController& controller, QWidget* parent)
    : QWidget(parent), controller_(controller)
{
    auto* layout = new QGridLayout(this);
    auto* selector = new QComboBox(this);
    selector->setObjectName(QStringLiteral("robotSelector"));
    selector->setMinimumContentsLength(25);
    selector->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
    auto button = [this](const char* name, const QString& text) {
        auto* result = new QPushButton(text, this);
        result->setObjectName(QString::fromLatin1(name));
        result->setEnabled(false);
        return result;
    };
    auto* scan = button("scanRobots", QStringLiteral("Scan robots"));
    scan->setEnabled(true);
    auto* connectRobot = button("connectRobot", QStringLiteral("Connect"));
    auto* disconnectRobot = button("disconnectRobot", QStringLiteral("Disconnect"));
    auto* connection = new QLabel(QStringLiteral("Disconnected"), this);
    connection->setObjectName(QStringLiteral("connectionStatus"));
    auto* telemetry = new QLabel(QStringLiteral("No active telemetry"), this);
    telemetry->setObjectName(QStringLiteral("telemetryDescription"));
    telemetry->setTextFormat(Qt::PlainText);
    telemetry->setWordWrap(true);
    auto* status = new QLabel(QStringLiteral("Hold a direction to drive; release to stop"), this);
    status->setObjectName(QStringLiteral("manualStatus"));
    status->setTextFormat(Qt::PlainText);
    status->setWordWrap(true);
    const auto config = controller.manualControlConfig();
    auto speed = [this](const char* name, double limit, const QString& unit) {
        auto* value = new QDoubleSpinBox(this);
        value->setObjectName(QString::fromLatin1(name));
        value->setDecimals(3);
        value->setRange(0, limit);
        value->setSingleStep(limit / 10);
        value->setValue(limit);
        value->setSuffix(unit);
        value->setEnabled(false);
        return value;
    };
    auto* linear = speed("manualLinearSpeed", config.maximumLinearMetersPerSecond, QStringLiteral(" m/s"));
    auto* angular = speed("manualAngularSpeed", config.maximumAngularRadiansPerSecond, QStringLiteral(" rad/s"));
    linear->setAccessibleName(QStringLiteral("Manual linear speed"));
    angular->setAccessibleName(QStringLiteral("Manual angular speed"));
    auto* forward = button("manualForward", QStringLiteral("Forward"));
    auto* backward = button("manualBackward", QStringLiteral("Backward"));
    auto* left = button("manualLeft", QStringLiteral("Turn CCW"));
    auto* right = button("manualRight", QStringLiteral("Turn CW"));
    auto* stop = button("manualStop", QStringLiteral("Stop"));
    auto* emergency = button("emergencyStop", QStringLiteral("E-STOP ALL ROBOTS"));
    auto* clearEmergency = button("clearEmergencyStop", QStringLiteral("Clear E-stop"));
    clearEmergency->setToolTip(QStringLiteral("Explicitly request clear for the selected robot. Motion remains inhibited until the robot accepts; physical output is unconfirmed."));
    emergency->setFocusPolicy(Qt::ClickFocus);
    emergency->setStyleSheet(QStringLiteral("QPushButton { font-weight: bold; padding: 8px; } QPushButton:enabled { background: #a40000; color: white; } QPushButton:disabled { background: #dedede; color: #666666; }"));
    emergency->setToolTip(QStringLiteral("Requests a latched emergency stop for every registered robot. Use explicit clear only after resolving the cause and confirming conditions are safe. Physical execution is unconfirmed."));
    layout->addWidget(selector, 0, 0, 1, 3);
    layout->addWidget(scan, 0, 3);
    layout->addWidget(connectRobot, 0, 4);
    layout->addWidget(disconnectRobot, 0, 5);
    layout->addWidget(connection, 1, 0, 1, 2);
    layout->addWidget(telemetry, 1, 2, 1, 4);
    layout->addWidget(linear, 2, 0);
    layout->addWidget(angular, 2, 1);
    layout->addWidget(forward, 2, 2);
    layout->addWidget(backward, 2, 3);
    layout->addWidget(left, 2, 4);
    layout->addWidget(right, 2, 5);
    layout->addWidget(stop, 3, 0, 1, 2);
    layout->addWidget(emergency, 3, 2, 1, 3);
    layout->addWidget(clearEmergency, 3, 5);
    layout->addWidget(status, 4, 0, 1, 6);
    connect(selector, &QComboBox::activated, this, [&controller, selector](int index) {
        controller.selectRobot(selector->itemData(index).toUInt());
    });
    connect(&controller, &DesktopController::robotChoicesChanged, this,
        [selector](const QList<RobotChoiceItem>& choices, quint32 selected) {
        const QSignalBlocker blocker(selector);
        auto* model = qobject_cast<QStandardItemModel*>(selector->model());
        for (int row = 0; row < choices.size(); ++row) {
            const auto& item = choices[row];
            if (row == selector->count()) selector->addItem(item.label, item.id.value());
            else {
                selector->setItemText(row, item.label);
                selector->setItemData(row, item.id.value());
            }
            if (model) model->item(row)->setEnabled(!item.ambiguous);
        }
        while (selector->count() > choices.size()) selector->removeItem(selector->count() - 1);
        selector->setCurrentIndex(selector->findData(selected));
    });
    connect(&controller, &DesktopController::robotScanChanged, this, [scan](bool scanning) {
        scan->setEnabled(!scanning);
        scan->setText(scanning ? QStringLiteral("Scanning…") : QStringLiteral("Scan robots"));
    });
    connect(scan, &QPushButton::clicked, &controller, &DesktopController::scanRobots);
    connect(connectRobot, &QPushButton::clicked, &controller, &DesktopController::connectRobot);
    connect(disconnectRobot, &QPushButton::clicked, &controller, &DesktopController::disconnectRobot);
    connect(stop, &QPushButton::clicked, &controller, &DesktopController::stopManualDrive);
    connect(emergency, &QPushButton::clicked, &controller, &DesktopController::emergencyStop);
    connect(clearEmergency, &QPushButton::clicked, &controller, &DesktopController::clearEmergencyStop);
    connect(&controller, &DesktopController::emergencyClearAvailable, clearEmergency, &QPushButton::setEnabled);
    for (auto* direction : {forward, backward, left, right}) {
        direction->setAutoRepeat(false);
        connect(direction, &QPushButton::pressed, this, [&controller, direction, forward, backward, left, linear, angular] {
            controller.beginManualDrive(direction == forward ? linear->value() : direction == backward ? -linear->value() : 0,
                direction == left ? angular->value() : (direction == forward || direction == backward) ? 0 : -angular->value());
        });
        direction->installEventFilter(this);
        connect(direction, &QPushButton::released, &controller, &DesktopController::cancelManualDrive);
    }
    connect(linear, &QDoubleSpinBox::valueChanged, &controller, &DesktopController::cancelManualDrive);
    connect(angular, &QDoubleSpinBox::valueChanged, &controller, &DesktopController::cancelManualDrive);
    connect(&controller, &DesktopController::robotActionsChanged, this,
        [=](bool canConnect, bool canDisconnect, bool drive, bool canStop, bool estop) {
        connectRobot->setEnabled(canConnect);
        disconnectRobot->setEnabled(canDisconnect);
        for (auto* direction : {forward, backward, left, right}) direction->setEnabled(drive);
        linear->setEnabled(drive);
        angular->setEnabled(drive);
        stop->setEnabled(canStop);
        emergency->setEnabled(estop);
    });
    connect(&controller, &DesktopController::connectionStatusChanged, this,
        [connection](const QString& text, bool, bool) { connection->setText(text); });
    connect(&controller, &DesktopController::telemetryDescriptionChanged, telemetry, &QLabel::setText);
    connect(&controller, &DesktopController::manualStatusChanged, status, &QLabel::setText);
    installEventFilter(this);
    connect(qApp, &QGuiApplication::applicationStateChanged, this, [this](Qt::ApplicationState state) {
        if (state != Qt::ApplicationActive) controller_.cancelManualDrive();
    });
}
RobotControlPanel::~RobotControlPanel()
{
    if (watchedWindow_) watchedWindow_->removeEventFilter(this);
    controller_.cancelManualDrive();
}
void RobotControlPanel::showEvent(QShowEvent* event)
{
    if (watchedWindow_ != window()) {
        if (watchedWindow_) watchedWindow_->removeEventFilter(this);
        watchedWindow_ = window();
        watchedWindow_->installEventFilter(this);
    }
    QWidget::showEvent(event);
}
bool RobotControlPanel::eventFilter(QObject* watched, QEvent* event)
{
    if ((event->type() == QEvent::WindowDeactivate && watched == watchedWindow_)
        || (event->type() == QEvent::Hide && (watched == this || watched == window()))
        || (event->type() == QEvent::EnabledChange && watched == this && !isEnabled())
        || (event->type() == QEvent::FocusOut && qobject_cast<QPushButton*>(watched)
            && static_cast<QWidget*>(watched)->parentWidget() == this))
        controller_.cancelManualDrive();
    return QWidget::eventFilter(watched, event);
}
} // namespace micro_robot
