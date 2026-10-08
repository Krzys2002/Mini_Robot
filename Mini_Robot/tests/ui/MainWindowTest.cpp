#include "ui/MainWindow.hpp"
#include "app/DesktopController.hpp"
#include "simulation/FakeCameraSource.hpp"
#include "simulation/SimulationTransport.hpp"
#include "ManualBackends.hpp"

#include <QApplication>
#include <QLabel>
#include <QPushButton>
#include <QStatusBar>
#include <gtest/gtest.h>

#include <chrono>
#include <memory>

namespace {

using namespace micro_robot;
using namespace std::chrono_literals;

struct DesktopFixture {
    FakeCameraSource camera;
    SimulationTransport transport;
    WorldModel world;
    DesktopController controller{camera, transport, world, RobotId{1}};
    MainWindow window{controller};
    Timestamp now{1s};

    DesktopFixture() { controller.initialize(now); }
    ~DesktopFixture()
    {
        controller.shutdown();
        advanceTo(now + 1ms);
    }
    void advanceTo(Timestamp at)
    {
        now = at;
        camera.advanceTo(at);
        transport.advanceTo(at);
    }
    QLabel* label(const char* name) { return window.findChild<QLabel*>(QString::fromLatin1(name)); }
    QPushButton* button(const char* name)
    {
        return window.findChild<QPushButton*>(QString::fromLatin1(name));
    }
};

TEST(MainWindowSmokeTest, OpensAndCloses)
{
    DesktopFixture fixture;
    fixture.window.show();
    QApplication::processEvents();

    ASSERT_TRUE(fixture.window.isVisible());
    ASSERT_NE(fixture.window.centralWidget(), nullptr);

    EXPECT_TRUE(fixture.window.close());
    QApplication::processEvents();
    EXPECT_FALSE(fixture.window.isVisible());
}

TEST(DesktopIntegrationTest, StartupRegistersRobotAndShowsIdleControls)
{
    DesktopFixture fixture;
    EXPECT_TRUE(fixture.label("applicationStatus")->text().contains(QStringLiteral("Ready")));
    EXPECT_EQ(fixture.label("cameraStatus")->text(), QStringLiteral("Stopped"));
    EXPECT_EQ(fixture.label("connectionStatus")->text(), QStringLiteral("Disconnected"));
    EXPECT_TRUE(fixture.button("startCamera")->isEnabled());
    EXPECT_FALSE(fixture.button("stopCamera")->isEnabled());
    EXPECT_TRUE(fixture.button("connectRobot")->isEnabled());
    EXPECT_FALSE(fixture.button("disconnectRobot")->isEnabled());
    ASSERT_EQ(fixture.world.robotCount(), 1U);
    ASSERT_TRUE(fixture.world.robot(RobotId{1}));
    EXPECT_FALSE(fixture.world.robot(RobotId{1})->pose);
    EXPECT_TRUE(fixture.label("worldDescription")->text().contains(QStringLiteral("Pose unknown")));
    fixture.controller.initialize(Timestamp{2s});
    EXPECT_EQ(fixture.world.robot(RobotId{1})->registeredAt, Timestamp{1s});
}

TEST(DesktopIntegrationTest, ButtonsRequestCameraChangesAndFrameEventsUpdateTheView)
{
    DesktopFixture fixture;
    fixture.button("startCamera")->click();
    EXPECT_EQ(fixture.camera.state(), CameraState::Stopped);
    EXPECT_EQ(fixture.label("frameDescription")->text(), QStringLiteral("No frames received"));
    fixture.advanceTo(Timestamp{1s + 100ms});
    EXPECT_EQ(fixture.label("cameraStatus")->text(), QStringLiteral("Streaming"));
    EXPECT_FALSE(fixture.button("startCamera")->isEnabled());
    EXPECT_TRUE(fixture.button("stopCamera")->isEnabled());
    EXPECT_TRUE(fixture.label("frameDescription")->text().contains(QStringLiteral("Frame #0")));
    // Camera frames do not infer poses or mutate world state.
    EXPECT_FALSE(fixture.world.robot(RobotId{1})->pose);
    const auto lastFrame = fixture.label("frameDescription")->text();
    fixture.button("stopCamera")->click();
    fixture.advanceTo(Timestamp{1s + 200ms});
    EXPECT_EQ(fixture.label("cameraStatus")->text(), QStringLiteral("Stopped"));
    EXPECT_EQ(fixture.label("frameDescription")->text(), lastFrame);
    EXPECT_TRUE(fixture.button("startCamera")->isEnabled());
    EXPECT_FALSE(fixture.button("stopCamera")->isEnabled());
}

TEST(DesktopIntegrationTest, TransportEventsAndExplicitPoseUpdatesReachTheView)
{
    DesktopFixture fixture;
    fixture.button("connectRobot")->click();
    EXPECT_EQ(fixture.transport.connectionState(RobotId{1}), ConnectionState::Disconnected);
    fixture.advanceTo(Timestamp{1s + 100ms});
    EXPECT_EQ(fixture.label("connectionStatus")->text(), QStringLiteral("Connected"));
    EXPECT_FALSE(fixture.button("connectRobot")->isEnabled());
    EXPECT_TRUE(fixture.button("disconnectRobot")->isEnabled());
    EXPECT_TRUE(fixture.label("telemetryDescription")->text().contains(QStringLiteral("3.70 V")));
    EXPECT_FALSE(fixture.world.robot(RobotId{1})->pose);
    EXPECT_EQ(fixture.controller.applyPose(RobotId{1},
        {Meters{0.25}, Meters{0.15}, Radians{0.0}}, fixture.now), PoseUpdateResult::Applied);
    ASSERT_TRUE(fixture.world.robot(RobotId{1})->pose);
    EXPECT_DOUBLE_EQ(fixture.world.robot(RobotId{1})->pose->x.value, 0.25);
    EXPECT_EQ(fixture.world.robot(RobotId{1})->poseUpdatedAt, fixture.now);
    EXPECT_TRUE(fixture.label("worldDescription")->text().contains(QStringLiteral("x 0.25 m")));
    const auto displayedWorld = fixture.label("worldDescription")->text();
    EXPECT_EQ(fixture.controller.applyPose(RobotId{1}, Pose2D{}, fixture.now),
              PoseUpdateResult::StaleTimestamp);
    EXPECT_EQ(fixture.label("worldDescription")->text(), displayedWorld);
    fixture.button("disconnectRobot")->click();
    fixture.advanceTo(Timestamp{1s + 200ms});
    EXPECT_EQ(fixture.label("connectionStatus")->text(), QStringLiteral("Disconnected"));
    EXPECT_EQ(fixture.label("telemetryDescription")->text(), QStringLiteral("No active telemetry"));
    EXPECT_TRUE(fixture.button("connectRobot")->isEnabled());
    EXPECT_FALSE(fixture.button("disconnectRobot")->isEnabled());
    // Last observed pose remains stored; disconnection does not erase it.
    EXPECT_EQ(fixture.label("worldDescription")->text(), displayedWorld);
}

TEST(DesktopIntegrationTest, ErrorsAreVisibleAndShutdownStopsBothBackends)
{
    DesktopFixture fixture;
    fixture.transport.sendCommand(RobotId{1}, {StopCommand{}, fixture.now, 1s});
    fixture.advanceTo(Timestamp{1s + 100ms});
    EXPECT_TRUE(fixture.window.statusBar()->currentMessage().contains(QStringLiteral("disconnected")));
    fixture.controller.startCamera();
    fixture.controller.connectRobot();
    fixture.advanceTo(Timestamp{1s + 200ms});
    fixture.controller.shutdown();
    fixture.controller.startCamera();
    fixture.controller.connectRobot();
    fixture.advanceTo(Timestamp{1s + 300ms});
    EXPECT_EQ(fixture.camera.state(), CameraState::Stopped);
    EXPECT_EQ(fixture.transport.connectionState(RobotId{1}), ConnectionState::Disconnected);
    EXPECT_EQ(fixture.label("applicationStatus")->text(), QStringLiteral("Shutting down"));
}

TEST(DesktopIntegrationTest, DestroyedControllerDetachesSubscriptions)
{
    FakeCameraSource camera;
    SimulationTransport transport;
    WorldModel world;
    {
        auto controller = std::make_unique<DesktopController>(camera, transport, world, RobotId{1});
        controller->initialize(Timestamp{1s});
        controller->startCamera();
        controller->connectRobot();
        camera.advanceTo(Timestamp{2s});
        transport.advanceTo(Timestamp{2s});
        controller->shutdown();
        camera.advanceTo(Timestamp{3s});
        transport.advanceTo(Timestamp{3s});
    }
    camera.start();
    transport.connect(RobotId{1});
    EXPECT_NO_THROW(camera.advanceTo(Timestamp{4s}));
    EXPECT_NO_THROW(transport.advanceTo(Timestamp{4s}));
}

TEST(DesktopIntegrationTest, AlternativeBackendsSupportPendingStatesAndCancellation)
{
    test::ManualCamera camera;
    test::ManualTransport transport;
    WorldModel world;
    DesktopController controller{camera, transport, world, RobotId{1}};
    MainWindow window{controller};
    controller.initialize(Timestamp{1s});
    auto button = [&](const char* name) {
        return window.findChild<QPushButton*>(QString::fromLatin1(name));
    };
    auto label = [&](const char* name) {
        return window.findChild<QLabel*>(QString::fromLatin1(name));
    };

    button("startCamera")->click();
    EXPECT_EQ(camera.startRequests, 1);
    camera.publishState(CameraState::Starting, Timestamp{2s});
    EXPECT_EQ(label("cameraStatus")->text(), QStringLiteral("Starting"));
    EXPECT_TRUE(button("stopCamera")->isEnabled());
    camera.publishState(CameraState::Streaming, Timestamp{3s});
    camera.publishFrame(std::make_shared<const Frame>(Frame{
        12, Timestamp{3s}, 1, 1, 1, PixelFormat::Gray8,
        std::make_shared<const std::vector<std::byte>>(1, std::byte{0x20})
    }));
    EXPECT_TRUE(label("frameDescription")->text().contains(QStringLiteral("Frame #12")));
    camera.publishError({CameraErrorCode::Unavailable, "Test camera unavailable", Timestamp{3s}});
    EXPECT_EQ(window.statusBar()->currentMessage(), QStringLiteral("Test camera unavailable"));
    button("stopCamera")->click();
    EXPECT_EQ(camera.stopRequests, 1);
    camera.publishState(CameraState::Stopped, Timestamp{4s});

    button("connectRobot")->click();
    ASSERT_EQ(transport.connectRequests.size(), 1U);
    EXPECT_EQ(transport.connectRequests.front(), RobotId{1});
    transport.publishState(RobotId{1}, ConnectionState::Connecting, Timestamp{2s});
    EXPECT_EQ(label("connectionStatus")->text(), QStringLiteral("Connecting"));
    EXPECT_FALSE(button("connectRobot")->isEnabled());
    EXPECT_TRUE(button("disconnectRobot")->isEnabled());
    button("disconnectRobot")->click();
    ASSERT_EQ(transport.disconnectRequests.size(), 1U);
    EXPECT_EQ(transport.disconnectRequests.front(), RobotId{1});
    transport.publishState(RobotId{1}, ConnectionState::Disconnected, Timestamp{3s});
    EXPECT_TRUE(button("connectRobot")->isEnabled());

    transport.publishState(RobotId{1}, ConnectionState::Connected, Timestamp{4s});
    transport.publishTelemetry({RobotId{1}, Timestamp{4s}, std::nullopt, std::nullopt, std::nullopt});
    EXPECT_TRUE(label("telemetryDescription")->text().contains(QStringLiteral("unknown battery")));
    const auto selectedTelemetry = label("telemetryDescription")->text();
    transport.publishState(RobotId{9}, ConnectionState::Disconnected, Timestamp{4s});
    transport.publishTelemetry({RobotId{9}, Timestamp{4s}, 5.0, 0.0, 0.0});
    EXPECT_EQ(label("connectionStatus")->text(), QStringLiteral("Connected"));
    EXPECT_EQ(label("telemetryDescription")->text(), selectedTelemetry);
    EXPECT_FALSE(world.robot(RobotId{1})->pose);
    controller.shutdown();
    transport.publishState(RobotId{1}, ConnectionState::Disconnected, Timestamp{5s});
}

TEST(DesktopIntegrationTest, InitializationCannotRestartAShutdownController)
{
    test::ManualCamera camera;
    test::ManualTransport transport;
    WorldModel world;
    DesktopController controller{camera, transport, world, RobotId{1}};
    MainWindow window{controller};
    controller.shutdown();
    controller.initialize(Timestamp{1s});
    EXPECT_EQ(window.findChild<QLabel*>(QStringLiteral("applicationStatus"))->text(),
              QStringLiteral("Shutting down"));
    EXPECT_EQ(world.robotCount(), 0U);
    controller.startCamera();
    controller.connectRobot();
    EXPECT_EQ(camera.startRequests, 0);
    EXPECT_TRUE(transport.connectRequests.empty());
}

} // namespace
