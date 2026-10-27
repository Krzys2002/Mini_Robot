#include "app/DesktopController.hpp"
#include "simulation/SimulationTransport.hpp"
#include "ui/MainWindow.hpp"
#include "ui/RobotControlPanel.hpp"
#include "ManualBackends.hpp"

#include <QApplication>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QEvent>
#include <QLabel>
#include <QPushButton>
#include <QScrollArea>
#include <QTest>
#include <gtest/gtest.h>
#include <limits>

namespace {
using namespace micro_robot;
using namespace std::chrono_literals;
constexpr RobotId one{1}, two{2};
struct Fixture {
    Logger logger;
    test::ManualTransport transport;
    WorldModel world;
    RobotManager robots{transport, world};
    DesktopController controller{robots, world, one, logger.channel("manual")};
    MainWindow window{controller};
    Fixture() { controller.initialize(MonotonicClock::now()); }
    void connected(RobotId id = one) {
        controller.connectRobot();
        transport.publishState(id, ConnectionState::Connected, MonotonicClock::now());
    }
    ~Fixture() { controller.shutdown(); }
    QPushButton* button(const char* name) { return window.findChild<QPushButton*>(name); }
    void expectStop() {
        ASSERT_FALSE(transport.commands.empty());
        EXPECT_TRUE(std::holds_alternative<StopCommand>(transport.commands.back().second.payload));
    }
};
TEST(RobotControlPanelTest, DisconnectedAndPendingSessionsCannotDrive)
{
    Fixture f;
    EXPECT_FALSE(f.button("manualForward")->isEnabled());
    EXPECT_FALSE(f.button("manualStop")->isEnabled());
    EXPECT_FALSE(f.button("emergencyStop")->isEnabled()); // unsupported capability
    f.controller.beginManualDrive(1, 1);
    EXPECT_TRUE(f.transport.commands.empty());
    f.controller.connectRobot();
    EXPECT_FALSE(f.button("connectRobot")->isEnabled());
    EXPECT_TRUE(f.button("disconnectRobot")->isEnabled());
    f.controller.beginManualDrive(1, 1);
    EXPECT_TRUE(f.transport.commands.empty());
    f.transport.publishState(one, ConnectionState::Connecting, MonotonicClock::now());
    EXPECT_FALSE(f.button("manualForward")->isEnabled());
    f.transport.publishState(one, ConnectionState::Connected, MonotonicClock::now());
    EXPECT_TRUE(f.button("manualForward")->isEnabled());
    EXPECT_TRUE(f.button("manualStop")->isEnabled());
}
TEST(RobotControlPanelTest, ServiceClampsBothSignsAndRejectsNonFiniteInput)
{
    Fixture f; f.connected();
    f.controller.beginManualDrive(1e9, -1e9);
    auto command = f.transport.commands.back().second;
    auto velocity = std::get<VelocityCommand>(command.payload);
    EXPECT_DOUBLE_EQ(velocity.linearMetersPerSecond, 0.1);
    EXPECT_DOUBLE_EQ(velocity.angularRadiansPerSecond, -1);
    EXPECT_EQ(command.validFor, 250ms);
    EXPECT_LE(command.issuedAt, MonotonicClock::now());
    f.controller.beginManualDrive(-1e9, 1e9);
    velocity = std::get<VelocityCommand>(f.transport.commands.back().second.payload);
    EXPECT_DOUBLE_EQ(velocity.linearMetersPerSecond, -0.1);
    EXPECT_DOUBLE_EQ(velocity.angularRadiansPerSecond, 1);
    f.controller.beginManualDrive(std::numeric_limits<double>::quiet_NaN(), 0);
    f.expectStop();
    f.controller.beginManualDrive(0, std::numeric_limits<double>::infinity());
    f.expectStop();
}
TEST(RobotControlPanelTest, HeldCommandsRefreshAndReleaseStopsWithoutAutomaticRestart)
{
    Fixture f; f.connected(); f.window.show(); QApplication::processEvents();
    auto* forward = f.button("manualForward");
    QTest::mousePress(forward, Qt::LeftButton);
    ASSERT_TRUE(std::holds_alternative<VelocityCommand>(f.transport.commands.back().second.payload));
    const auto first = f.transport.commands.back().second.issuedAt;
    ASSERT_TRUE(QTest::qWaitFor([&] { return f.transport.commands.back().second.issuedAt > first; }, 1000));
    QTest::mouseRelease(forward, Qt::LeftButton);
    f.expectStop();
    const auto count = f.transport.commands.size();
    QTest::qWait(150);
    EXPECT_EQ(f.transport.commands.size(), count);
    // Keyboard hold/release uses the same service, without button auto-repeat.
    forward->setFocus();
    QTest::keyPress(forward, Qt::Key_Space);
    EXPECT_TRUE(std::holds_alternative<VelocityCommand>(f.transport.commands.back().second.payload));
    QTest::keyRelease(forward, Qt::Key_Space);
    f.expectStop();
}
TEST(RobotControlPanelTest, AllDirectionsUseSignedSiValuesAndSpeedWidgetsAreBounded)
{
    Fixture f; f.connected();
    auto* linear = f.window.findChild<QDoubleSpinBox*>("manualLinearSpeed");
    auto* angular = f.window.findChild<QDoubleSpinBox*>("manualAngularSpeed");
    linear->setValue(99); angular->setValue(99);
    EXPECT_DOUBLE_EQ(linear->value(), 0.1); EXPECT_DOUBLE_EQ(angular->value(), 1);
    for (const auto& [name, target] : std::vector<std::pair<const char*, VelocityCommand>>{
        {"manualForward", {0.1, 0}}, {"manualBackward", {-0.1, 0}},
        {"manualLeft", {0, 1}}, {"manualRight", {0, -1}}}) {
        auto* button = f.button(name);
        QTest::mousePress(button, Qt::LeftButton);
        const auto actual = std::get<VelocityCommand>(f.transport.commands.back().second.payload);
        EXPECT_DOUBLE_EQ(actual.linearMetersPerSecond, target.linearMetersPerSecond);
        EXPECT_DOUBLE_EQ(actual.angularRadiansPerSecond, target.angularRadiansPerSecond);
        QTest::mouseRelease(button, Qt::LeftButton);
        f.expectStop();
    }
}
TEST(RobotControlPanelTest, DeactivationHidingAndDisablingPanelCancelManualInput)
{
    Fixture f; f.connected(); f.window.show(); QApplication::processEvents();
    auto* panel = f.window.findChild<RobotControlPanel*>();
    ASSERT_NE(panel, nullptr);
    f.controller.beginManualDrive(0.1, 0);
    QEvent deactivate{QEvent::ApplicationDeactivate};
    QApplication::sendEvent(qApp, &deactivate); f.expectStop();
    f.controller.beginManualDrive(0.1, 0);
    panel->setEnabled(false); f.expectStop();
    panel->setEnabled(true);
    f.controller.beginManualDrive(0.1, 0);
    panel->hide(); f.expectStop();
}
TEST(RobotControlPanelTest, SelectionStopsOldRobotAndTelemetryDoesNotCrossSessions)
{
    Fixture f; f.connected();
    f.controller.setDiscoveredRobots({{two, "MR-2", MonotonicClock::now(), false}});
    auto* selector = f.window.findChild<QComboBox*>("robotSelector");
    ASSERT_EQ(selector->count(), 2);
    f.controller.beginManualDrive(0.1, 0);
    f.controller.selectRobot(2);
    f.expectStop(); EXPECT_EQ(f.transport.commands.back().first, one);
    EXPECT_FALSE(f.button("manualForward")->isEnabled());
    f.connected(two);
    f.transport.publishTelemetry({one, MonotonicClock::now(), 9.9, {}, {}});
    EXPECT_FALSE(f.window.findChild<QLabel*>("telemetryDescription")->text().contains("9.90"));
    f.transport.publishTelemetry({two, MonotonicClock::now(), 3.8, {}, {}});
    EXPECT_TRUE(f.window.findChild<QLabel*>("telemetryDescription")->text().contains("3.80"));
    f.controller.beginManualDrive(0, 0.5);
    EXPECT_EQ(f.transport.commands.back().first, two);
    f.controller.selectRobot(99);
    EXPECT_EQ(f.controller.selectedRobot(), two);
    // Discovery cannot silently adopt a transport session owned elsewhere.
    f.controller.setDiscoveredRobots({{RobotId{3}, "Unmanaged", MonotonicClock::now(), false}});
    f.transport.publishState(RobotId{3}, ConnectionState::Connected, MonotonicClock::now());
    f.controller.selectRobot(3);
    EXPECT_EQ(f.controller.selectedRobot(), two);
}
TEST(RobotControlPanelTest, AmbiguousDiscoveryStopsAndDisablesDriveAndConnect)
{
    Fixture f; f.connected(); f.controller.beginManualDrive(0.1, 0);
    f.controller.setDiscoveredRobots({{one, "MR-1", MonotonicClock::now(), false},
                                     {one, "MR-1 duplicate", MonotonicClock::now(), false}});
    f.expectStop(); EXPECT_FALSE(f.button("manualForward")->isEnabled());
    f.controller.disconnectRobot();
    f.transport.publishState(one, ConnectionState::Disconnected, MonotonicClock::now());
    EXPECT_FALSE(f.button("connectRobot")->isEnabled());
    f.controller.connectRobot(); EXPECT_EQ(f.transport.connectRequests.size(), 1U);
}
TEST(RobotControlPanelTest, DisconnectFaultAndTransportErrorCancelDrive)
{
    Fixture f; f.connected(); f.controller.beginManualDrive(0.1, 0);
    f.controller.disconnectRobot(); f.expectStop();
    EXPECT_FALSE(f.button("manualForward")->isEnabled());
    f.transport.publishState(one, ConnectionState::Disconnected, MonotonicClock::now());
    f.connected(); f.controller.beginManualDrive(0.1, 0);
    f.transport.publishError({one, TransportOperation::SendCommand, TransportErrorCode::BackendFailure,
                             "write failed", MonotonicClock::now()});
    f.expectStop(); EXPECT_FALSE(f.button("manualForward")->isEnabled());
    f.controller.connectRobot(); // Existing connection intent cannot clear the fault.
    f.controller.beginManualDrive(0.1, 0);
    f.expectStop(); EXPECT_FALSE(f.button("manualForward")->isEnabled());
    f.transport.publishState(one, ConnectionState::Fault, MonotonicClock::now());
    f.controller.beginManualDrive(0.1, 0);
    f.expectStop();
}
TEST(RobotControlPanelTest, RobotFaultTelemetryDisablesManualDriveWhileStopStaysAvailable)
{
    Fixture f; f.connected(); f.controller.beginManualDrive(0.1, 0);
    f.transport.publishTelemetry({one, MonotonicClock::now(), 3.6, {}, {}, {}, RobotOperatingState::Fault});
    f.expectStop(); EXPECT_FALSE(f.button("manualForward")->isEnabled());
    EXPECT_TRUE(f.button("manualStop")->isEnabled());
    EXPECT_TRUE(f.window.findChild<QLabel*>("telemetryDescription")->text().contains("FAULT"));
}
TEST(RobotControlPanelTest, SimulationUsesSameServiceAndEmergencyStopLatchesAllSessions)
{
    Logger logger; SimulationTransport transport; WorldModel world;
    RobotManager robots{transport, world};
    DesktopController controller{robots, world, one, logger.channel("manual")};
    MainWindow window{controller};
    controller.initialize(MonotonicClock::now()); controller.connectRobot();
    ASSERT_TRUE(robots.addRobot(two, MonotonicClock::now())); ASSERT_TRUE(robots.connect(two));
    transport.advanceTo(MonotonicClock::now());
    controller.beginManualDrive(0.08, 0.2); transport.advanceTo(MonotonicClock::now());
    ASSERT_TRUE(transport.lastSentCommand(one));
    EXPECT_DOUBLE_EQ(std::get<VelocityCommand>(transport.lastSentCommand(one)->payload).linearMetersPerSecond, 0.08);
    auto* estop = window.findChild<QPushButton*>("emergencyStop");
    ASSERT_TRUE(estop->isEnabled()); estop->click(); transport.advanceTo(MonotonicClock::now());
    EXPECT_TRUE(robots.session(one)->emergencyStopRequested);
    EXPECT_TRUE(robots.session(two)->emergencyStopRequested);
    EXPECT_EQ(robots.session(two)->telemetry->operatingState, RobotOperatingState::EmergencyStop);
    EXPECT_DOUBLE_EQ(*robots.session(one)->telemetry->linearMetersPerSecond, 0);
    EXPECT_FALSE(window.findChild<QPushButton*>("manualForward")->isEnabled());
    EXPECT_TRUE(window.findChild<QPushButton*>("manualStop")->isEnabled());
    controller.disconnectRobot(); transport.advanceTo(MonotonicClock::now());
    EXPECT_TRUE(estop->isEnabled()); // Remains available while disconnected/faulted.
    controller.connectRobot(); transport.advanceTo(MonotonicClock::now());
    controller.beginManualDrive(0.1, 0); transport.advanceTo(MonotonicClock::now());
    EXPECT_FALSE(window.findChild<QPushButton*>("manualForward")->isEnabled());
    controller.shutdown(); transport.advanceTo(MonotonicClock::now());
}
TEST(RobotControlPanelTest, StopAndEmergencyStayVisibleAtSmallWindowSize)
{
    Fixture f; f.window.resize(800, 480); f.window.show(); QApplication::processEvents();
    for (const char* name : {"manualStop", "emergencyStop"}) {
        auto* control = f.button(name);
        const QRect bounds{control->mapTo(&f.window, QPoint{}), control->size()};
        EXPECT_TRUE(f.window.rect().contains(bounds));
        EXPECT_TRUE(control->isVisible());
    }
    const auto screenshot = qEnvironmentVariable("MICRO_ROBOT_MANUAL_SCREENSHOT");
    if (!screenshot.isEmpty()) EXPECT_TRUE(f.window.grab().save(screenshot));
}
TEST(RobotControlPanelTest, ScanAvailabilityAndNonSelectedConnectionStateFollowServiceUpdates)
{
    Fixture f;
    f.controller.setDiscoveredRobots({{two, "MR-2", MonotonicClock::now(), false}});
    f.controller.setRobotScanState(true);
    EXPECT_FALSE(f.button("scanRobots")->isEnabled());
    f.controller.connectRobot();
    f.transport.publishState(one, ConnectionState::Connected, MonotonicClock::now());
    EXPECT_FALSE(f.button("scanRobots")->isEnabled());
    ASSERT_TRUE(f.robots.addRobot(two, MonotonicClock::now()));
    ASSERT_TRUE(f.robots.connect(two));
    f.transport.publishState(two, ConnectionState::Connected, MonotonicClock::now());
    auto* selector = f.window.findChild<QComboBox*>("robotSelector");
    EXPECT_TRUE(selector->itemText(selector->findData(2U)).contains("Connected"));
    f.controller.setRobotScanState(false);
    EXPECT_TRUE(f.button("scanRobots")->isEnabled());
}
TEST(RobotControlPanelTest, InvalidConfigurationIsRejected)
{
    Logger logger; test::ManualTransport transport; WorldModel world; RobotManager robots{transport, world};
    ManualControlConfig config; config.maximumLinearMetersPerSecond = -1;
    EXPECT_THROW((DesktopController{robots, world, one, logger.channel("manual"), nullptr, config}), std::invalid_argument);
    config = {}; config.commandValidity = config.refreshInterval;
    EXPECT_THROW((DesktopController{robots, world, one, logger.channel("manual"), nullptr, config}), std::invalid_argument);
}
} // namespace
