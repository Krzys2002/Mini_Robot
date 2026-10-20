#include "ui/MainWindow.hpp"
#include "app/DesktopController.hpp"
#include "infrastructure/logging/Logger.hpp"
#include "simulation/FakeCameraSourceFactory.hpp"
#include "simulation/SimulationTransport.hpp"
#include "ManualBackends.hpp"

#include <QApplication>
#include <QEventLoop>
#include <QLabel>
#include <QPushButton>
#include <QStatusBar>
#include <QTimer>
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace {

using namespace micro_robot;
using namespace std::chrono_literals;

class CapturingLogSink final : public ILogSink {
public:
    void write(const LogRecord& record) override { records.push_back(record); }
    [[nodiscard]] bool contains(LogLevel level, const std::string& message) const
    {
        return std::any_of(records.begin(), records.end(), [&](const LogRecord& record) {
            return record.level == level && record.module == "controller"
                && record.message.find(message) != std::string::npos;
        });
    }
    std::vector<LogRecord> records;
};

// Hands a manual camera to the controller and returns it for event publication.
test::ManualCamera& installManualCamera(DesktopController& controller,
                                        CameraInfo info = {"manual-camera", "Manual test camera"})
{
    auto camera = std::make_unique<test::ManualCamera>(std::move(info));
    auto& installed = *camera;
    controller.setCameraSource(std::move(camera));
    return installed;
}

struct DesktopFixture {
    Logger logger;
    FakeCameraSourceFactory cameras; // Outlives the source owned by the controller.
    SimulationTransport transport;
    WorldModel world;
    DesktopController controller{transport, world, RobotId{1}, logger.channel("controller")};
    MainWindow window{controller};
    ICameraSource* camera{};
    Timestamp now{1s};

    DesktopFixture()
    {
        auto source = cameras.create({"fake", "Fake camera"});
        camera = source.get();
        controller.setCameraSource(std::move(source));
        controller.initialize(now);
    }
    ~DesktopFixture()
    {
        controller.shutdown();
        advanceTo(now + 1ms);
    }
    void advanceTo(Timestamp at)
    {
        now = at;
        cameras.advanceTo(at);
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
    EXPECT_EQ(fixture.label("activeCamera")->text(), QStringLiteral("Fake camera (simulated feed)"));
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
    EXPECT_EQ(fixture.camera->state(), CameraState::Stopped);
    EXPECT_EQ(fixture.label("frameDescription")->text(), QStringLiteral("No frames received"));
    fixture.advanceTo(Timestamp{1s + 100ms});
    EXPECT_EQ(fixture.label("cameraStatus")->text(), QStringLiteral("Streaming"));
    EXPECT_FALSE(fixture.button("startCamera")->isEnabled());
    EXPECT_TRUE(fixture.button("stopCamera")->isEnabled());
    EXPECT_TRUE(fixture.label("frameDescription")->text().contains(QStringLiteral("Frame #0")));
    // Camera frames do not infer poses or mutate world state.
    EXPECT_FALSE(fixture.world.robot(RobotId{1})->pose);
    fixture.button("stopCamera")->click();
    fixture.advanceTo(Timestamp{1s + 200ms});
    EXPECT_EQ(fixture.label("cameraStatus")->text(), QStringLiteral("Stopped"));
    EXPECT_EQ(fixture.label("frameDescription")->text(), QStringLiteral("No frames received"));
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
    EXPECT_EQ(fixture.camera->state(), CameraState::Stopped);
    EXPECT_EQ(fixture.transport.connectionState(RobotId{1}), ConnectionState::Disconnected);
    EXPECT_EQ(fixture.label("applicationStatus")->text(), QStringLiteral("Shutting down"));
}

TEST(DesktopIntegrationTest, DestroyedControllerDetachesSubscriptionsAndReleasesCamera)
{
    Logger logger;
    FakeCameraSourceFactory cameras;
    SimulationTransport transport;
    WorldModel world;
    {
        auto controller = std::make_unique<DesktopController>(transport, world, RobotId{1},
                                                              logger.channel("controller"));
        controller->setCameraSource(cameras.create({"fake", "Fake camera"}));
        controller->initialize(Timestamp{1s});
        controller->startCamera();
        controller->connectRobot();
        cameras.advanceTo(Timestamp{2s});
        transport.advanceTo(Timestamp{2s});
        controller->shutdown();
        cameras.advanceTo(Timestamp{3s});
        transport.advanceTo(Timestamp{3s});
    }
    EXPECT_EQ(cameras.liveSourceCount(), 0U);
    transport.connect(RobotId{1});
    EXPECT_NO_THROW(transport.advanceTo(Timestamp{4s}));
}

FramePtr recoveryFrame(std::uint64_t sequence = 1)
{
    return std::make_shared<const Frame>(Frame{sequence, Timestamp{2s}, 1, 1, 1,
        PixelFormat::Gray8, std::make_shared<const std::vector<std::byte>>(1, std::byte{0})});
}

TEST(DesktopIntegrationTest, RepeatedSourceLossClearsFramesAndRejectsLateOrInvalidDelivery)
{
    DesktopFixture fixture;
    auto& camera = installManualCamera(fixture.controller);
    for (int cycle = 0; cycle < 5; ++cycle) {
        fixture.controller.startCamera();
        camera.publishState(CameraState::Streaming, Timestamp{2s});
        camera.publishFrame(recoveryFrame());
        const auto current = fixture.label("frameDescription")->text();
        EXPECT_TRUE(current.contains(QStringLiteral("Frame #1")));
        camera.publishFrame(nullptr);
        camera.publishFrame(std::make_shared<const Frame>());
        EXPECT_EQ(fixture.label("frameDescription")->text(), current);
        camera.publishError({CameraErrorCode::Unavailable, "Camera disconnected", Timestamp{3s}});
        camera.publishState(cycle % 2 ? CameraState::Fault : CameraState::Disconnected, Timestamp{3s});
        const auto lost = fixture.label("frameDescription")->text();
        EXPECT_TRUE(lost.contains(QStringLiteral("No current frame")));
        camera.publishFrame(recoveryFrame(99)); // Already queued when loss was reported.
        EXPECT_EQ(fixture.label("frameDescription")->text(), lost);
        EXPECT_EQ(fixture.window.statusBar()->currentMessage(), QStringLiteral("Camera disconnected"));
        EXPECT_TRUE(fixture.button("startCamera")->isEnabled());
        EXPECT_FALSE(fixture.button("stopCamera")->isEnabled());
    }
    EXPECT_EQ(camera.startRequests, 5);
}

TEST(DesktopIntegrationTest, SilentSourceCannotLeaveAnOldFramePresentedAsCurrent)
{
    DesktopFixture fixture;
    auto& camera = installManualCamera(fixture.controller);
    camera.publishState(CameraState::Streaming, Timestamp{2s});
    camera.publishFrame(recoveryFrame());
    ASSERT_TRUE(fixture.label("frameDescription")->text().contains(QStringLiteral("Frame #1")));
    QEventLoop loop;
    QTimer::singleShot(CameraFrameTimeout + 100ms, &loop, &QEventLoop::quit);
    loop.exec();
    EXPECT_TRUE(fixture.label("frameDescription")->text().contains(QStringLiteral("No current frame")));
    camera.publishFrame(recoveryFrame(2));
    EXPECT_TRUE(fixture.label("frameDescription")->text().contains(QStringLiteral("Frame #2")));
}

TEST(DesktopIntegrationTest, CameraTerminalStatesPermitRecoveryThroughTheInterface)
{
    Logger logger;
    SimulationTransport transport;
    WorldModel world;
    DesktopController controller{transport, world, RobotId{1}, logger.channel("controller")};
    auto& camera = installManualCamera(controller);
    MainWindow window{controller};
    controller.initialize(Timestamp{1s});
    auto* start = window.findChild<QPushButton*>(QStringLiteral("startCamera"));
    auto* stop = window.findChild<QPushButton*>(QStringLiteral("stopCamera"));
    auto* status = window.findChild<QLabel*>(QStringLiteral("cameraStatus"));
    ASSERT_NE(start, nullptr);
    ASSERT_NE(stop, nullptr);
    ASSERT_NE(status, nullptr);

    const CameraState terminalStates[]{CameraState::Disconnected, CameraState::Fault};
    int expectedRequests = 0;
    for (const auto terminal : terminalStates) {
        camera.publishState(CameraState::Streaming, Timestamp{2s});
        EXPECT_FALSE(start->isEnabled());
        EXPECT_TRUE(stop->isEnabled());
        const CameraError error{CameraErrorCode::BackendFailure,
            "Camera capture interrupted", Timestamp{3s}};
        camera.publishError(error);
        // Diagnostics and state are separate events.
        EXPECT_EQ(status->text(), QStringLiteral("Streaming"));
        EXPECT_EQ(window.statusBar()->currentMessage(), QString::fromStdString(error.message));
        camera.publishState(terminal, Timestamp{3s});
        EXPECT_EQ(status->text(), terminal == CameraState::Disconnected
            ? QStringLiteral("Disconnected") : QStringLiteral("Fault"));
        EXPECT_TRUE(start->isEnabled());
        EXPECT_FALSE(stop->isEnabled());
        start->click();
        EXPECT_EQ(camera.startRequests, ++expectedRequests);
        // Recovery remains asynchronous; requesting it does not invent a state.
        EXPECT_EQ(camera.state(), terminal);
    }
    controller.shutdown();
    camera.publishState(CameraState::Stopped, Timestamp{4s});
}

TEST(DesktopIntegrationTest, AlternativeBackendsSupportPendingStatesAndCancellation)
{
    Logger logger;
    test::ManualTransport transport;
    WorldModel world;
    DesktopController controller{transport, world, RobotId{1}, logger.channel("controller")};
    auto& camera = installManualCamera(controller);
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

TEST(DesktopIntegrationTest, ControllerLogsLifecycleStateChangesAndErrors)
{
    Logger logger{LogLevel::Debug};
    auto sink = std::make_shared<CapturingLogSink>();
    logger.addSink(sink);
    test::ManualTransport transport;
    WorldModel world;
    DesktopController controller{transport, world, RobotId{1}, logger.channel("controller")};
    auto& camera = installManualCamera(controller);

    controller.initialize(Timestamp{1s});
    EXPECT_TRUE(sink->contains(LogLevel::Info, "Controller initialized"));
    EXPECT_TRUE(sink->contains(LogLevel::Info, "Camera state: Stopped"));

    camera.publishState(CameraState::Streaming, Timestamp{2s});
    EXPECT_TRUE(sink->contains(LogLevel::Info, "Camera state: Streaming"));
    camera.publishError({CameraErrorCode::Unavailable, "Test camera unavailable", Timestamp{2s}});
    EXPECT_TRUE(sink->contains(LogLevel::Error, "Camera error: Test camera unavailable"));
    camera.publishState(CameraState::Fault, Timestamp{3s});
    EXPECT_TRUE(sink->contains(LogLevel::Error, "Camera state: Fault"));

    transport.publishState(RobotId{1}, ConnectionState::Connected, Timestamp{3s});
    ASSERT_FALSE(sink->records.empty());
    EXPECT_EQ(sink->records.back().message, "Connection state: Connected");
    EXPECT_EQ(sink->records.back().robotId, RobotId{1});

    controller.shutdown();
    EXPECT_TRUE(sink->contains(LogLevel::Info, "Shutdown requested"));
    transport.publishState(RobotId{1}, ConnectionState::Disconnected, Timestamp{4s});
}

TEST(DesktopIntegrationTest, InitializationCannotRestartAShutdownController)
{
    Logger logger;
    test::ManualTransport transport;
    WorldModel world;
    DesktopController controller{transport, world, RobotId{1}, logger.channel("controller")};
    auto& camera = installManualCamera(controller);
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

struct ReplacementFixture {
    Logger logger;
    test::ManualTransport transport;
    WorldModel world;
    DesktopController controller{transport, world, RobotId{1}, logger.channel("controller")};
    MainWindow window{controller};

    ReplacementFixture() { controller.initialize(Timestamp{1s}); }
    ~ReplacementFixture() { controller.shutdown(); }
    QLabel* label(const char* name) { return window.findChild<QLabel*>(QString::fromLatin1(name)); }
    QPushButton* button(const char* name)
    {
        return window.findChild<QPushButton*>(QString::fromLatin1(name));
    }
};

TEST(CameraReplacementTest, WithoutASourceCameraControlsAreDisabled)
{
    ReplacementFixture fixture;
    EXPECT_FALSE(fixture.controller.hasCameraSource());
    EXPECT_EQ(fixture.label("activeCamera")->text(), QStringLiteral("No camera selected"));
    EXPECT_EQ(fixture.label("cameraStatus")->text(), QStringLiteral("No camera selected"));
    EXPECT_FALSE(fixture.button("startCamera")->isEnabled());
    EXPECT_FALSE(fixture.button("stopCamera")->isEnabled());
    EXPECT_NO_THROW(fixture.controller.startCamera());
    EXPECT_NO_THROW(fixture.controller.stopCamera());
}

TEST(CameraReplacementTest, StoppedSourceIsReplacedImmediately)
{
    ReplacementFixture fixture;
    auto& first = installManualCamera(fixture.controller, {"a", "Camera A"});
    const auto firstAlive = first.alive;
    EXPECT_TRUE(first.hasSink());
    EXPECT_EQ(fixture.label("activeCamera")->text(), QStringLiteral("Camera A"));
    EXPECT_EQ(fixture.label("cameraStatus")->text(), QStringLiteral("Stopped"));
    EXPECT_TRUE(fixture.button("startCamera")->isEnabled());

    auto& second = installManualCamera(fixture.controller, {"b", "Camera B"});
    EXPECT_FALSE(*firstAlive);
    EXPECT_TRUE(second.hasSink());
    EXPECT_EQ(fixture.label("activeCamera")->text(), QStringLiteral("Camera B"));
    fixture.button("startCamera")->click();
    EXPECT_EQ(second.startRequests, 1);
    second.publishState(CameraState::Streaming, Timestamp{2s});
    EXPECT_EQ(fixture.label("cameraStatus")->text(), QStringLiteral("Streaming"));
    second.publishState(CameraState::Stopped, Timestamp{3s});

    const auto secondAlive = second.alive;
    fixture.controller.setCameraSource(nullptr);
    EXPECT_FALSE(*secondAlive);
    EXPECT_FALSE(fixture.controller.hasCameraSource());
    EXPECT_EQ(fixture.label("activeCamera")->text(), QStringLiteral("No camera selected"));
    EXPECT_FALSE(fixture.button("startCamera")->isEnabled());
}

TEST(CameraReplacementTest, RunningSourceStopsBeforeDeferredReplacement)
{
    ReplacementFixture fixture;
    auto& first = installManualCamera(fixture.controller, {"a", "Camera A"});
    const auto firstAlive = first.alive;
    fixture.controller.startCamera();
    first.publishState(CameraState::Streaming, Timestamp{2s});
    first.publishFrame(std::make_shared<const Frame>(Frame{
        3, Timestamp{2s}, 1, 1, 1, PixelFormat::Gray8,
        std::make_shared<const std::vector<std::byte>>(1, std::byte{0x10})
    }));

    auto superseded = std::make_unique<test::ManualCamera>(CameraInfo{"b", "Camera B"});
    const auto supersededAlive = superseded->alive;
    fixture.controller.setCameraSource(std::move(superseded));
    EXPECT_EQ(first.stopRequests, 1);
    EXPECT_TRUE(*firstAlive);
    EXPECT_EQ(fixture.label("activeCamera")->text(), QStringLiteral("Camera A"));
    // A later selection replaces the pending one; the stop request is idempotent.
    auto& third = installManualCamera(fixture.controller, {"c", "Camera C"});
    EXPECT_FALSE(*supersededAlive);
    EXPECT_EQ(first.stopRequests, 1);
    // Restarting the outgoing source would race the replacement.
    fixture.controller.startCamera();
    EXPECT_EQ(first.startRequests, 1);

    first.publishState(CameraState::Stopped, Timestamp{3s});
    // Never destroyed inside its own callback.
    EXPECT_TRUE(*firstAlive);
    EXPECT_EQ(fixture.label("cameraStatus")->text(), QStringLiteral("Stopped"));
    QApplication::processEvents();
    EXPECT_FALSE(*firstAlive);
    EXPECT_TRUE(third.hasSink());
    EXPECT_EQ(third.startRequests, 0);
    EXPECT_EQ(fixture.label("activeCamera")->text(), QStringLiteral("Camera C"));
    EXPECT_EQ(fixture.label("frameDescription")->text(), QStringLiteral("No frames received"));
    EXPECT_TRUE(fixture.button("startCamera")->isEnabled());
    fixture.controller.startCamera();
    EXPECT_EQ(third.startRequests, 1);
}

TEST(CameraReplacementTest, TerminalSourceIsReplacedImmediately)
{
    ReplacementFixture fixture;
    auto& first = installManualCamera(fixture.controller, {"a", "Camera A"});
    const auto firstAlive = first.alive;
    first.publishState(CameraState::Streaming, Timestamp{2s});
    first.publishState(CameraState::Disconnected, Timestamp{3s});
    installManualCamera(fixture.controller, {"b", "Camera B"});
    // Quiescent: released at once, without awaiting a stop.
    EXPECT_FALSE(*firstAlive);
    EXPECT_EQ(fixture.label("activeCamera")->text(), QStringLiteral("Camera B"));
}

TEST(CameraReplacementTest, ShutdownDropsAPendingReplacement)
{
    ReplacementFixture fixture;
    auto& first = installManualCamera(fixture.controller, {"a", "Camera A"});
    const auto firstAlive = first.alive;
    first.publishState(CameraState::Streaming, Timestamp{2s});
    auto pending = std::make_unique<test::ManualCamera>(CameraInfo{"b", "Camera B"});
    const auto pendingAlive = pending->alive;
    fixture.controller.setCameraSource(std::move(pending));
    fixture.controller.shutdown();
    EXPECT_FALSE(*pendingAlive);
    first.publishState(CameraState::Stopped, Timestamp{3s});
    QApplication::processEvents();
    EXPECT_TRUE(*firstAlive);
    EXPECT_EQ(fixture.label("activeCamera")->text(), QStringLiteral("Camera A"));
    fixture.controller.setCameraSource(std::make_unique<test::ManualCamera>());
    EXPECT_TRUE(*firstAlive);
}


TEST(CameraStreamControlTest, RepeatedCyclesStopFramesAndResumeWithoutRestartingApplication)
{
    DesktopFixture fixture;
    for (int cycle = 0; cycle < 4; ++cycle) {
        fixture.button("startCamera")->click();
        EXPECT_EQ(fixture.label("cameraStatus")->text(), QStringLiteral("Starting"));
        EXPECT_FALSE(fixture.button("startCamera")->isEnabled());
        fixture.advanceTo(fixture.now + 100ms);
        EXPECT_EQ(fixture.camera->state(), CameraState::Streaming);
        const auto frame = fixture.label("frameDescription")->text();
        EXPECT_TRUE(frame.contains(QStringLiteral("Frame #")));
        fixture.button("stopCamera")->click();
        EXPECT_EQ(fixture.label("cameraStatus")->text(), QStringLiteral("Stopping"));
        EXPECT_FALSE(fixture.button("startCamera")->isEnabled());
        EXPECT_FALSE(fixture.button("stopCamera")->isEnabled());
        fixture.advanceTo(fixture.now + 100ms);
        EXPECT_EQ(fixture.camera->state(), CameraState::Stopped);
        fixture.advanceTo(fixture.now + 100ms);
        EXPECT_EQ(fixture.label("frameDescription")->text(), QStringLiteral("No frames received"));
        EXPECT_TRUE(fixture.button("startCamera")->isEnabled());
    }
}

TEST(CameraStreamControlTest, CancelBeforeFirstBackendEventAcknowledgesStop)
{
    DesktopFixture fixture;
    fixture.controller.startCamera();
    fixture.controller.stopCamera();
    EXPECT_EQ(fixture.camera->state(), CameraState::Stopped);
    EXPECT_FALSE(fixture.button("startCamera")->isEnabled());
    fixture.advanceTo(fixture.now + 100ms);
    EXPECT_EQ(fixture.label("cameraStatus")->text(), QStringLiteral("Stopped"));
    EXPECT_TRUE(fixture.button("startCamera")->isEnabled());
    EXPECT_EQ(fixture.label("frameDescription")->text(), QStringLiteral("No frames received"));
}

TEST(CameraStreamControlTest, GuardsRequestsBeforeAndBetweenBackendEventsAndDuringShutdown)
{
    ReplacementFixture fixture;
    auto& camera = installManualCamera(fixture.controller);
    fixture.controller.stopCamera();
    EXPECT_EQ(camera.stopRequests, 0);
    fixture.controller.startCamera();
    fixture.controller.startCamera();
    EXPECT_EQ(camera.startRequests, 1);
    EXPECT_FALSE(fixture.button("startCamera")->isEnabled());
    EXPECT_TRUE(fixture.button("stopCamera")->isEnabled());
    fixture.controller.stopCamera();
    fixture.controller.stopCamera();
    fixture.controller.startCamera();
    EXPECT_EQ(camera.stopRequests, 1);
    EXPECT_EQ(camera.startRequests, 1);
    camera.publishState(CameraState::Starting, Timestamp{2s});
    camera.publishState(CameraState::Streaming, Timestamp{3s});
    EXPECT_EQ(fixture.label("cameraStatus")->text(), QStringLiteral("Stopping"));
    EXPECT_FALSE(fixture.button("startCamera")->isEnabled());
    EXPECT_FALSE(fixture.button("stopCamera")->isEnabled());
    camera.publishState(CameraState::Stopped, Timestamp{4s});
    fixture.controller.startCamera();
    EXPECT_EQ(camera.startRequests, 2);
    fixture.controller.shutdown();
    camera.publishState(CameraState::Stopped, Timestamp{5s});
    EXPECT_FALSE(fixture.button("startCamera")->isEnabled());
    EXPECT_FALSE(fixture.button("stopCamera")->isEnabled());
}

TEST(CameraStreamControlTest, SwitchingBeforeStartCompletesWaitsForCancellation)
{
    ReplacementFixture fixture;
    auto& first = installManualCamera(fixture.controller);
    const auto alive = first.alive;
    fixture.controller.startCamera();
    installManualCamera(fixture.controller, {"second", "Second camera"});
    EXPECT_TRUE(*alive);
    EXPECT_EQ(first.stopRequests, 1);
    EXPECT_FALSE(fixture.button("startCamera")->isEnabled());
    first.publishState(CameraState::Stopped, Timestamp{2s});
    // The outgoing callback cannot enable Start before replacement is applied.
    EXPECT_FALSE(fixture.button("startCamera")->isEnabled());
    QApplication::processEvents();
    EXPECT_FALSE(*alive);
    EXPECT_EQ(fixture.label("activeCamera")->text(), QStringLiteral("Second camera"));
    EXPECT_TRUE(fixture.button("startCamera")->isEnabled());
}

TEST(CameraStreamControlTest, FailedStartIsVisibleLoggedAndRetryable)
{
    Logger logger;
    auto logs = std::make_shared<CapturingLogSink>();
    logger.addSink(logs);
    test::ManualTransport transport;
    WorldModel world;
    DesktopController controller{transport, world, RobotId{1}, logger.channel("controller")};
    MainWindow window{controller};
    auto& camera = installManualCamera(controller);
    controller.startCamera(); // No requests before initialization.
    EXPECT_EQ(camera.startRequests, 0);
    controller.initialize(Timestamp{1s});
    controller.startCamera();
    camera.publishState(CameraState::Starting, Timestamp{2s});
    camera.publishError({CameraErrorCode::PermissionDenied, "Camera permission denied", Timestamp{2s}});
    camera.publishState(CameraState::Fault, Timestamp{2s});
    EXPECT_EQ(window.statusBar()->currentMessage(), QStringLiteral("Camera permission denied"));
    EXPECT_TRUE(logs->contains(LogLevel::Error, "Camera permission denied"));
    EXPECT_TRUE(window.findChild<QPushButton*>(QStringLiteral("startCamera"))->isEnabled());
    controller.startCamera();
    EXPECT_EQ(camera.startRequests, 2);
    camera.publishState(CameraState::Streaming, Timestamp{3s});
    controller.stopCamera();
    camera.publishState(CameraState::Stopped, Timestamp{4s});
    EXPECT_TRUE(logs->contains(LogLevel::Info, "Camera start requested: Manual test camera"));
    EXPECT_TRUE(logs->contains(LogLevel::Info, "Camera stop requested: Manual test camera"));
    controller.shutdown();
}

TEST(CameraStreamControlTest, StatusObserverCanCancelStartBeforeBackendDispatch)
{
    DesktopFixture fixture;
    QObject::connect(&fixture.controller, &DesktopController::cameraStatusChanged,
                     &fixture.window, [&](const QString& status, bool) {
        if (status == QStringLiteral("Starting")) fixture.controller.stopCamera();
    });
    fixture.controller.startCamera();
    fixture.advanceTo(fixture.now + 100ms);
    EXPECT_EQ(fixture.camera->state(), CameraState::Stopped);
    EXPECT_EQ(fixture.label("cameraStatus")->text(), QStringLiteral("Stopped"));
    EXPECT_TRUE(fixture.button("startCamera")->isEnabled());
    EXPECT_EQ(fixture.label("frameDescription")->text(), QStringLiteral("No frames received"));
}

} // namespace
