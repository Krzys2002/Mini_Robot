#include "app/LiveVisualController.hpp"
#include "app/ControlTraceWriter.hpp"
#include "app/VisionTracking.hpp"
#include "ui/VisualControlPanel.hpp"
#include "ui/VisionDebugView.hpp"
#include "../calibration/HomographyFixtures.hpp"
#include "calibration/OpenCvCalibrationUndistorter.hpp"
#include "VisionDebugFixtures.hpp"
#include "ManualBackends.hpp"
#include "simulation/SimulationTransport.hpp"
#include "calibration/TabletopHomography.hpp"

#include <QApplication>
#include <QColor>
#include <QFile>
#include <QImage>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QMouseEvent>
#include <QPushButton>
#include <QSignalSpy>
#include <QTemporaryDir>
#include <QTest>
#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include <future>
#include <filesystem>
#include <fstream>
#include <limits>

namespace {
using namespace micro_robot;
using namespace micro_robot::test;
using namespace std::chrono_literals;
void sendFractionalLeftClick(QWidget* target, QPointF position)
{
    const auto global = target->mapToGlobal(position.toPoint());
    QMouseEvent event{QEvent::MouseButtonPress, position, QPointF{global}, Qt::LeftButton,
                      Qt::LeftButton, Qt::NoModifier};
    QApplication::sendEvent(target, &event);
}
void useIdentityPointUndistorter(LiveVisualController& visual)
{
    visual.setImagePointUndistorter([](ImageCorner point, const IntrinsicCalibration&) {
        return std::optional<ImageCorner>{point};
    });
}
class NoMarkers final : public IMarkerDetector {
public: MarkerDetections detect(const Frame&) override { return {}; }
};
LiveVisualSettings simulationVisualSettings()
{
    auto settings = LiveVisualSettings::fromApplication({});
    settings.motionEnabled = true;
    return settings;
}
TEST(LiveVisualSettingsTest, SharesConfiguredManualSpeedCapsWithPolicyAndLoop)
{
    ApplicationConfig config;
    config.manual.maximumLinearMetersPerSecond = 0.025;
    config.manual.maximumAngularRadiansPerSecond = 0.4;

    const auto settings = LiveVisualSettings::fromApplication(config);
    EXPECT_DOUBLE_EQ(settings.goal.maximumLinearMetersPerSecond, 0.025);
    EXPECT_DOUBLE_EQ(settings.loop.maximumLinearMetersPerSecond, 0.025);
    EXPECT_DOUBLE_EQ(settings.goal.maximumAngularRadiansPerSecond, 0.4);
    EXPECT_DOUBLE_EQ(settings.loop.maximumAngularRadiansPerSecond, 0.4);
}
struct LiveVisualControlTest : testing::Test {
    Timestamp now{1s};
    Logger logger;
    WorldModel world;
    test::ManualTransport transport;
    RobotManager robots{transport, world};
    DesktopController desktop{robots, world, RobotId{1}, logger.channel("desktop")};
    RobotTracker tracker{world, {250ms, 1s}, {.25, .25}};
    VisionController vision{std::make_unique<NoMarkers>(), logger.channel("vision")};
    std::vector<std::string> traces;
    LiveVisualController visual{desktop, world, robots, simulationVisualSettings(),
        [this](std::string line) { traces.push_back(std::move(line)); }, [this] { return now; }};
    std::uint64_t sequence{};
    LiveVisualControlTest()
    {
        connectVisionTracking(vision, tracker, visual, [this] { desktop.refreshWorld(); }, [this] { return now; },
            [this](const MetricObservationBatchPtr& batch) { visual.receiveMeasurements(batch); });
        desktop.initialize(now);
        EXPECT_TRUE(robots.connect(RobotId{1}));
        transport.publishState(RobotId{1}, ConnectionState::Connected, now);
        world.setWorkspaceFrame(WorkspaceBounds{Meters{2}, Meters{2}}, "metric-frame");
        visual.setInput("camera-a", true);
        sample(0, 0);
        EXPECT_TRUE(visual.setTarget(1, 0, 0));
    }
    void sample(double x, double y, double heading = 0.0)
    {
        auto batch = std::make_shared<MetricObservationBatch>();
        batch->status = MetricObservationStatus::Valid;
        batch->measurements.push_back({RobotId{1}, {Meters{x}, Meters{y}, Radians{heading}}, now, ++sequence, 20, {}});
        emit vision.metricObservationsAvailable("camera-a", batch);
    }
    void run()
    {
        ASSERT_TRUE(visual.start());
        visual.refresh();
        ASSERT_TRUE(visual.running());
        ASSERT_FALSE(transport.commands.empty());
    }
};

WorkspaceCalibrationProfile calibratedPathProfile()
{
    auto profile = test::homographyInput().profile;
    profile.homography = test::knownHomography();
    return profile;
}

void preparePathCalibration(LiveVisualControlTest& fixture,
                            const WorkspaceCalibrationProfile& profile)
{
    fixture.visual.setImagePointUndistorter(OpenCvCalibrationUndistorter::undistortPoint);
    fixture.visual.setImagePointDistorter(OpenCvCalibrationUndistorter::distortPoint);
    fixture.desktop.setWorkspaceCalibration(profile);
    fixture.visual.refresh();
    fixture.now += 1ms;
    fixture.sample(0, 0);
}

std::optional<ImageCorner> rawPixelForWorld(const WorkspaceCalibrationProfile& profile,
                                            WorldPoint2D point)
{
    if (!profile.homography) return {};
    const auto ideal = unprojectWorkspacePoint(*profile.homography, point);
    if (!ideal) return {};
    return OpenCvCalibrationUndistorter::distortPoint(*ideal, profile.intrinsics);
}

void clickRawPixel(VisionOverlay* overlay, ImageCorner point)
{
    sendFractionalLeftClick(overlay, overlay->imageToView({point.u, point.v}));
}
TEST_F(LiveVisualControlTest, MetricBridgeUsesFilteredPoseAndLogsOriginalProvenanceAndDomainSequence)
{
    now += 50ms; sample(1, 0); // Filtered x=.25, not raw x=1 (which would already be at goal).
    run();
    ASSERT_TRUE(visual.lastCycle()->error);
    EXPECT_NEAR(visual.lastCycle()->error->xMeters, .75, 1e-12);
    ASSERT_TRUE(std::holds_alternative<VelocityCommand>(transport.commands.back().second.payload));
    EXPECT_GT(std::get<VelocityCommand>(transport.commands.back().second.payload).linearMetersPerSecond, 0);
    ASSERT_FALSE(traces.empty());
    const auto trace = QJsonDocument::fromJson(QByteArray::fromStdString(traces.back())).object();
    EXPECT_EQ(trace["frame_sequence"].toString(), QString::number(sequence));
    EXPECT_EQ(trace["camera_time_ns"].toString(), QString::number(std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count()));
    EXPECT_EQ(trace["domain_sequence"].toString().toULongLong(), *transport.commands.back().second.metadata.sequence);
    EXPECT_TRUE(trace["wire_sequence"].isNull());
    EXPECT_TRUE(trace["camera_timestamp_source"].toString().contains("unverified"));
    EXPECT_TRUE(visual.description().contains("Actual: x 0.250"));
    EXPECT_TRUE(visual.description().contains("Target: x 1.000"));
}
TEST_F(LiveVisualControlTest, StaleLostAndCameraInvalidationStopAndNeverAutoResume)
{
    EXPECT_EQ(visual.operationalState(), ControlOperationalState::Ready);
    run(); EXPECT_EQ(visual.operationalState(), ControlOperationalState::Running);
    now += 250ms; visual.refresh();
    EXPECT_FALSE(visual.running());
    EXPECT_EQ(visual.operationalState(), ControlOperationalState::LostTracking);
    EXPECT_TRUE(std::holds_alternative<StopCommand>(transport.commands.back().second.payload));
    now += 1ms; sample(0, 0);
    EXPECT_FALSE(visual.running());
    EXPECT_TRUE(visual.start()); visual.refresh();
    visual.setInput("camera-b", true);
    EXPECT_FALSE(visual.running()); EXPECT_FALSE(visual.start());
    now += 1ms; sample(0, 0);
    EXPECT_TRUE(visual.start());
    tracker.invalidate(now); visual.refresh();
    EXPECT_FALSE(visual.running()); EXPECT_FALSE(visual.start());
}
TEST_F(LiveVisualControlTest, DisconnectAndReconnectRequireNewStart)
{
    run(); now += 1ms;
    transport.publishState(RobotId{1}, ConnectionState::Disconnected, now);
    EXPECT_EQ(visual.operationalState(), ControlOperationalState::LostConnection);
    EXPECT_FALSE(visual.running()); EXPECT_FALSE(visual.start());
    EXPECT_TRUE(robots.connect(RobotId{1}));
    now += 1ms; transport.publishState(RobotId{1}, ConnectionState::Connected, now);
    EXPECT_FALSE(visual.running()); EXPECT_TRUE(visual.start());
}
TEST_F(LiveVisualControlTest, ManualInputAndStopTakeExclusiveMotionAuthority)
{
    run();
    desktop.beginManualDrive(-.05, 0);
    EXPECT_FALSE(visual.running());
    EXPECT_DOUBLE_EQ(std::get<VelocityCommand>(transport.commands.back().second.payload).linearMetersPerSecond, -.05);
    ASSERT_TRUE(visual.start()); visual.refresh();
    desktop.stopManualDrive(); EXPECT_FALSE(visual.running());
    EXPECT_EQ(visual.operationalState(), ControlOperationalState::Paused);
    EXPECT_TRUE(std::holds_alternative<StopCommand>(transport.commands.back().second.payload));
}
TEST_F(LiveVisualControlTest, SelectionAmbiguityFaultEstopAndShutdownInterruptVisualCommands)
{
    run();
    desktop.setDiscoveredRobots({{RobotId{1}, "conflict", now, true}});
    EXPECT_FALSE(visual.running()); EXPECT_FALSE(visual.start());
    desktop.setDiscoveredRobots({{RobotId{1}, "one", now, false}, {RobotId{2}, "two", now, false}});
    ASSERT_TRUE(visual.start()); visual.refresh();
    desktop.selectRobot(2);
    EXPECT_FALSE(visual.running()); EXPECT_FALSE(visual.start());
    EXPECT_EQ(visual.operationalState(), ControlOperationalState::LostConnection);
    desktop.selectRobot(1); ASSERT_TRUE(visual.setTarget(1, 0, 0));
    now += 1ms; sample(0, 0); ASSERT_TRUE(visual.start());
    desktop.emergencyStop(); EXPECT_FALSE(visual.running());
    EXPECT_EQ(visual.operationalState(), ControlOperationalState::EStop);
    EXPECT_FALSE(visual.start());
    desktop.shutdown(); EXPECT_FALSE(visual.running()); EXPECT_FALSE(visual.start());
}
TEST_F(LiveVisualControlTest, EmergencyStopLatchesManagerBeforeInterruptingVisualControl)
{
    run();
    bool latchObservedAtInterrupt = false;
    QObject::connect(&desktop, &DesktopController::motionInterruptRequested, &desktop, [&] {
        const auto session = robots.session(RobotId{1});
        latchObservedAtInterrupt = session && session->emergencyStopRequested;
    }, Qt::DirectConnection);
    desktop.emergencyStop();
    EXPECT_TRUE(latchObservedAtInterrupt);
    EXPECT_FALSE(visual.running());
    const auto commandCountAfterEmergencyStop = transport.commands.size();
    EXPECT_FALSE(visual.start());
    visual.refresh();
    EXPECT_EQ(transport.commands.size(), commandCountAfterEmergencyStop);
}
TEST_F(LiveVisualControlTest, FrameReplacementClearsTargetAndInvalidCoordinatesCannotArm)
{
    EXPECT_FALSE(visual.setTarget(-1, 0, 0));
    EXPECT_FALSE(visual.setTarget(std::numeric_limits<double>::infinity(), 0, 0));
    run();
    world.setWorkspaceFrame(WorkspaceBounds{Meters{2}, Meters{2}}, "new-frame");
    now += 1ms; sample(0, 0);
    EXPECT_FALSE(visual.running()); EXPECT_FALSE(visual.start());
    ASSERT_TRUE(visual.setTarget(1, 0, 0));
    EXPECT_TRUE(visual.start());
}
TEST_F(LiveVisualControlTest, ReportedRobotFaultImmediatelyStopsAndCannotRearm)
{
    run();
    transport.publishTelemetry({RobotId{1}, now, {}, {}, {}, {}, RobotOperatingState::Fault});
    EXPECT_EQ(visual.operationalState(), ControlOperationalState::Fault);
    EXPECT_FALSE(visual.running()); EXPECT_FALSE(visual.start());
    EXPECT_TRUE(std::holds_alternative<StopCommand>(transport.commands.back().second.payload));
}
TEST_F(LiveVisualControlTest, UnacceptedAndDuplicateMetadataCannotAuthorizeOrReplaceProvenance)
{
    tracker.clearMeasurements(now); visual.setInput("camera-a", false); visual.setInput("camera-a", true);
    auto batch = std::make_shared<MetricObservationBatch>();
    batch->status = MetricObservationStatus::Valid;
    batch->measurements.push_back({RobotId{1}, {}, now, 999, 20, {}});
    visual.receiveMeasurements(batch); // Not accepted by tracker.
    EXPECT_FALSE(visual.start());
    now += 1ms; sample(0, 0);
    run();
    batch->measurements[0].capturedAt = now;
    visual.receiveMeasurements(batch);
    now += 50ms; visual.refresh();
    const auto trace = QJsonDocument::fromJson(QByteArray::fromStdString(traces.back())).object();
    EXPECT_EQ(trace["frame_sequence"].toString(), QString::number(sequence));
}
TEST_F(LiveVisualControlTest, RejectedDuplicateCannotRestoreProvenanceAfterSourceChange)
{
    run(); visual.setInput("camera-b", true);
    auto duplicate = std::make_shared<MetricObservationBatch>();
    duplicate->status = MetricObservationStatus::Valid;
    duplicate->measurements.push_back({RobotId{1}, {}, now, 999, 20, {}});
    emit vision.metricObservationsAvailable("camera-b", duplicate);
    EXPECT_FALSE(visual.start());
    now += 1ms; sample(0, 0);
    EXPECT_TRUE(visual.start());
}
TEST_F(LiveVisualControlTest, TargetPanelAppliesCoordinatesStartsStopsAndRendersAtScale)
{
    VisualControlPanel panel{visual}; panel.resize(760, 320); panel.show();
    auto* x = panel.findChild<QLineEdit*>("visualTargetX");
    auto* y = panel.findChild<QLineEdit*>("visualTargetY");
    auto* h = panel.findChild<QLineEdit*>("visualTargetHeading");
    auto* acknowledge = panel.findChild<QPushButton*>("acknowledgeVisualFault");
    auto* pause = panel.findChild<QPushButton*>("pauseVisualControl");
    auto* cancel = panel.findChild<QPushButton*>("cancelVisualControl");
    ASSERT_TRUE(x && y && h && acknowledge && pause && cancel);
    EXPECT_FALSE(pause->isEnabled());
    EXPECT_TRUE(cancel->isEnabled());
    EXPECT_FALSE(acknowledge->isEnabled());
    x->setText("1.25"); y->setText("0.5"); h->setText("1.0");
    panel.findChild<QPushButton*>("applyVisualTarget")->click();
    auto* start = panel.findChild<QPushButton*>("startVisualControl"); ASSERT_TRUE(start->isEnabled()); start->click();
    visual.refresh(); EXPECT_TRUE(visual.running()); EXPECT_FALSE(x->isEnabled());
    EXPECT_TRUE(pause->isEnabled()); EXPECT_TRUE(cancel->isEnabled());
    const auto* status = panel.findChild<QLabel*>("visualControlStatus");
    ASSERT_TRUE(status);
    EXPECT_TRUE(status->text().contains("Target: x 1.250"));
    EXPECT_TRUE(status->text().contains("Position error: 1346.3 mm · heading error: 57.3°"));
    EXPECT_TRUE(status->text().contains("RUNNING"));
    if (const auto path = qEnvironmentVariable("MICRO_ROBOT_VISUAL_CONTROL_SCREENSHOT"); !path.isEmpty()) EXPECT_TRUE(panel.grab().save(path));
    pause->click();
    EXPECT_FALSE(visual.running()); EXPECT_TRUE(x->isEnabled());
    EXPECT_TRUE(visual.description().contains("Target: x 1.250"));
    EXPECT_TRUE(start->isEnabled()); EXPECT_FALSE(pause->isEnabled()); EXPECT_TRUE(cancel->isEnabled());
    start->click(); visual.refresh(); EXPECT_TRUE(visual.running());
    cancel->click();
    EXPECT_FALSE(visual.running()); EXPECT_TRUE(x->isEnabled());
    EXPECT_FALSE(visual.description().contains("Target:"));
    EXPECT_FALSE(start->isEnabled());
}
TEST_F(LiveVisualControlTest, ImageTargetUsesCalibratedPerspectiveTransformWithoutSubmittingMotion)
{
    auto profile = test::homographyInput().profile;
    constexpr double radialK1 = 0.2;
    profile.intrinsics.distortion = {radialK1, 0, 0, 0, 0};
    // This non-affine synthetic transform exercises the projective denominator.
    profile.homography = WorkspaceHomography{{1.0 / 600, 1.0 / 5400, -5.0 / 27,
                                               0, 1.0 / 450, -2.0 / 9,
                                               0, -1.0 / 900, 10.0 / 9}};
    visual.setImagePointUndistorter(OpenCvCalibrationUndistorter::undistortPoint);
    desktop.setWorkspaceCalibration(profile);
    visual.refresh();
    now += 1ms;
    sample(0, 0);

    const WorldPoint2D expected{Meters{0.625}, Meters{0.375}};
    const auto imagePoint = unprojectWorkspacePoint(*profile.homography, expected);
    ASSERT_TRUE(imagePoint);
    const double normalizedX = (imagePoint->u - 500.0) / 800.0;
    const double normalizedY = (imagePoint->v - 400.0) / 810.0;
    const double radiusSquared = normalizedX * normalizedX + normalizedY * normalizedY;
    const double radialScale = 1 + radialK1 * radiusSquared;
    const ImageCorner distortedPixel{
        500.0 + 800.0 * normalizedX * radialScale,
        400.0 + 810.0 * normalizedY * radialScale};
    EXPECT_GT(std::hypot(distortedPixel.u - imagePoint->u, distortedPixel.v - imagePoint->v), 1.0);
    const auto commandCountBeforeClick = transport.commands.size();
    EXPECT_TRUE(visual.setTargetFromImage(QStringLiteral("camera-a"), 1000, 800,
        distortedPixel.u, distortedPixel.v, 0.0, now, visual.workspaceCalibrationRevision()));
    const auto target = visual.targetPose();
    ASSERT_TRUE(target);
    EXPECT_NEAR(target->x.value, expected.x.value, 1e-8);
    EXPECT_NEAR(target->y.value, expected.y.value, 1e-8);
    EXPECT_NEAR(target->heading.value, 0.0, 1e-12);
    EXPECT_FALSE(visual.running());
    EXPECT_EQ(transport.commands.size(), commandCountBeforeClick);
}
TEST_F(LiveVisualControlTest, ImageTargetRejectsMismatchedCameraResolutionFrameAndStaleCapture)
{
    auto profile = test::homographyInput().profile;
    profile.homography = test::knownHomography();
    useIdentityPointUndistorter(visual);
    desktop.setWorkspaceCalibration(profile);
    visual.refresh();
    now += 1ms;
    sample(0, 0);
    const auto revision = visual.workspaceCalibrationRevision();
    const auto commandsBeforeRejectedClicks = transport.commands.size();

    EXPECT_FALSE(visual.setTargetFromImage(QStringLiteral("camera-b"), 1000, 800, 500, 400, 0.0, now, revision));
    EXPECT_FALSE(visual.setTargetFromImage(QStringLiteral("camera-a"), 999, 800, 500, 400, 0.0, now, revision));
    EXPECT_FALSE(visual.setTargetFromImage(QStringLiteral("camera-a"), 1000, 799, 500, 400, 0.0, now, revision));
    EXPECT_FALSE(visual.setTargetFromImage(QStringLiteral("camera-a"), 1000, 800, 500, 400, 0.0, now,
        QStringLiteral("another-workspace-frame")));
    EXPECT_FALSE(visual.setTargetFromImage(QStringLiteral("camera-a"), 1000, 800, 500, 400, 0.0,
        now - 1s, revision));
    EXPECT_FALSE(visual.description().contains("Target:"));
    EXPECT_FALSE(visual.running());
    EXPECT_EQ(transport.commands.size(), commandsBeforeRejectedClicks);
}
TEST_F(LiveVisualControlTest, ImageTargetRequiresActiveCalibrationAndNeverFallsBackToPixelCoordinates)
{
    auto profile = test::homographyInput().profile;
    profile.homography.reset();
    useIdentityPointUndistorter(visual);
    desktop.setWorkspaceCalibration(profile);
    visual.refresh();
    now += 1ms;
    sample(0, 0);
    const auto commandCountBeforeRejectedClick = transport.commands.size();

    EXPECT_FALSE(visual.setTargetFromImage(QStringLiteral("camera-a"), 1000, 800, 500, 400, 0.0,
        now, visual.workspaceCalibrationRevision()));
    EXPECT_FALSE(visual.description().contains("Target:"));
    EXPECT_FALSE(visual.running());
    EXPECT_EQ(transport.commands.size(), commandCountBeforeRejectedClick);
}
TEST_F(LiveVisualControlTest, QWidgetImageClicksKeepMetricTargetAcrossResizeAndIgnoreLetterboxMargins)
{
    auto profile = test::homographyInput().profile;
    profile.intrinsics.distortion = {0, 0, 0, 0, 0};
    profile.homography = test::knownHomography();
    useIdentityPointUndistorter(visual);
    desktop.setWorkspaceCalibration(profile);
    visual.refresh();
    now += 1ms;
    sample(0, 0);

    VisualControlPanel panel{visual};
    panel.resize(430, 900);
    panel.show();
    QApplication::processEvents();
    auto* overlay = panel.findChild<VisionOverlay*>("visionOverlay");
    auto* start = panel.findChild<QPushButton*>("startVisualControl");
    auto* x = panel.findChild<QLineEdit*>("visualTargetX");
    auto* y = panel.findChild<QLineEdit*>("visualTargetY");
    ASSERT_TRUE(overlay && start && x && y);
    panel.presentPreview(QStringLiteral("camera-a"), debugPreview(
        PixelFormat::Gray8, profile.intrinsics.width, profile.intrinsics.height, now));
    QApplication::processEvents();

    constexpr double pixelU = 350.25;
    constexpr double pixelV = 250.75;
    const double expectedX = 0.00125 * pixelU - 0.125;
    const double expectedY = -0.001 * pixelV + 0.7;
    const auto commandCountBeforeStaging = transport.commands.size();
    for (const QSize size : {QSize{430, 900}, QSize{1000, 600}}) {
        panel.resize(size);
        QApplication::processEvents();
        const auto imageRect = overlay->imageRect();
        ASSERT_FALSE(imageRect.isEmpty());
        ASSERT_TRUE(imageRect.left() > 1.0 || imageRect.top() > 1.0)
            << "Expected letterbox padding at overlay size " << overlay->size().width()
            << "x" << overlay->size().height();

        const auto click = overlay->imageToView({pixelU, pixelV});
        ASSERT_TRUE(imageRect.contains(click));
        sendFractionalLeftClick(overlay, click);
        const auto target = visual.targetPose();
        ASSERT_TRUE(target);
        EXPECT_NEAR(target->x.value, expectedX, 1e-12);
        EXPECT_NEAR(target->y.value, expectedY, 1e-12);
        EXPECT_NEAR(QLocale::c().toDouble(x->text()), expectedX, 1e-8);
        EXPECT_NEAR(QLocale::c().toDouble(y->text()), expectedY, 1e-8);
        EXPECT_TRUE(start->isEnabled());
        EXPECT_TRUE(transport.commands.empty());

        QPointF marginClick;
        if (imageRect.left() > 1.0)
            marginClick = {imageRect.left() / 2.0, imageRect.center().y()};
        else
            marginClick = {imageRect.center().x(), imageRect.top() / 2.0};
        ASSERT_TRUE(overlay->rect().contains(marginClick.toPoint()));
        sendFractionalLeftClick(overlay, marginClick);
        const auto afterMarginClick = visual.targetPose();
        ASSERT_TRUE(afterMarginClick);
        EXPECT_NEAR(afterMarginClick->x.value, expectedX, 1e-12);
        EXPECT_NEAR(afterMarginClick->y.value, expectedY, 1e-12);
        EXPECT_EQ(transport.commands.size(), commandCountBeforeStaging);
    }

    auto changedProfile = profile;
    changedProfile.frameId = "table-B";
    desktop.setWorkspaceCalibration(changedProfile);
    visual.refresh();
    EXPECT_FALSE(visual.targetPose());
    const auto oldRevisionClick = overlay->imageToView({pixelU, pixelV});
    sendFractionalLeftClick(overlay, oldRevisionClick);
    EXPECT_FALSE(visual.targetPose());
    EXPECT_EQ(transport.commands.size(), commandCountBeforeStaging);

    now += 1ms;
    sample(0, 0);
    panel.presentPreview(QStringLiteral("camera-a"), debugPreview(
        PixelFormat::Gray8, changedProfile.intrinsics.width, changedProfile.intrinsics.height, now));
    QApplication::processEvents();
    sendFractionalLeftClick(overlay, overlay->imageToView({pixelU, pixelV}));
    ASSERT_TRUE(visual.targetPose());
    EXPECT_TRUE(transport.commands.empty());
    start->click();
    visual.refresh();
    EXPECT_TRUE(visual.running());
    ASSERT_FALSE(transport.commands.empty());
}

TEST_F(LiveVisualControlTest, PathEditorLifecycleKeepsMetricDraftSeparateFromGoalsAndMotion)
{
    const auto profile = calibratedPathProfile();
    preparePathCalibration(*this, profile);
    ASSERT_TRUE(visual.setTarget(0.25, 0.2, 0.3));
    const auto initialTarget = visual.targetPose();
    ASSERT_TRUE(initialTarget);
    const auto commandCount = transport.commands.size();

    VisualControlPanel panel{visual};
    panel.resize(900, 520);
    panel.show();
    auto* overlay = panel.findChild<VisionOverlay*>("visionOverlay");
    auto* begin = panel.findChild<QPushButton*>("pathEditorBegin");
    auto* undo = panel.findChild<QPushButton*>("pathEditorUndo");
    auto* clear = panel.findChild<QPushButton*>("pathEditorClear");
    auto* finish = panel.findChild<QPushButton*>("pathEditorFinish");
    auto* cancel = panel.findChild<QPushButton*>("pathEditorCancel");
    ASSERT_TRUE(overlay && begin && undo && clear && finish && cancel);
    panel.presentPreview(QStringLiteral("camera-a"), debugPreview(
        PixelFormat::Gray8, profile.intrinsics.width, profile.intrinsics.height, now));
    QApplication::processEvents();

    const WorldPoint2D first{Meters{0.24}, Meters{0.18}};
    const WorldPoint2D second{Meters{0.73}, Meters{0.42}};
    const auto firstRaw = rawPixelForWorld(profile, first);
    const auto secondRaw = rawPixelForWorld(profile, second);
    ASSERT_TRUE(firstRaw && secondRaw);

    begin->click();
    ASSERT_TRUE(panel.pathEditing());
    clickRawPixel(overlay, *firstRaw);
    clickRawPixel(overlay, *secondRaw);
    ASSERT_EQ(panel.draftPath().size(), 2U);
    EXPECT_TRUE(panel.finishedPath().empty());
    EXPECT_NEAR(panel.draftPath().points()[0].x.value, first.x.value, 1e-6);
    EXPECT_NEAR(panel.draftPath().points()[0].y.value, first.y.value, 1e-6);
    EXPECT_NEAR(panel.draftPath().points()[1].x.value, second.x.value, 1e-6);
    EXPECT_NEAR(panel.draftPath().points()[1].y.value, second.y.value, 1e-6);
    EXPECT_TRUE(undo->isEnabled());
    EXPECT_TRUE(finish->isEnabled());
    EXPECT_EQ(transport.commands.size(), commandCount);

    undo->click();
    ASSERT_EQ(panel.draftPath().size(), 1U);
    EXPECT_NEAR(panel.draftPath().points().front().x.value, first.x.value, 1e-6);
    clear->click();
    EXPECT_TRUE(panel.draftPath().empty());
    EXPECT_FALSE(finish->isEnabled());

    clickRawPixel(overlay, *firstRaw);
    finish->click();
    EXPECT_FALSE(panel.pathEditing());
    EXPECT_TRUE(panel.draftPath().empty());
    ASSERT_EQ(panel.finishedPath().size(), 1U);
    EXPECT_NEAR(panel.finishedPath().points().front().x.value, first.x.value, 1e-6);

    begin->click();
    EXPECT_TRUE(panel.pathEditing());
    ASSERT_EQ(panel.draftPath().size(), 1U);
    ASSERT_EQ(panel.finishedPath().size(), 1U);
    clickRawPixel(overlay, *secondRaw);
    ASSERT_EQ(panel.draftPath().size(), 2U);
    cancel->click();
    EXPECT_FALSE(panel.pathEditing());
    EXPECT_TRUE(panel.draftPath().empty());
    ASSERT_EQ(panel.finishedPath().size(), 1U);
    clear->click();
    EXPECT_TRUE(panel.finishedPath().empty());

    begin->click();
    ASSERT_TRUE(panel.pathEditing());
    clickRawPixel(overlay, *secondRaw);
    ASSERT_EQ(panel.draftPath().size(), 1U);
    cancel->click();
    EXPECT_FALSE(panel.pathEditing());
    EXPECT_TRUE(panel.draftPath().empty());
    EXPECT_TRUE(panel.finishedPath().empty());
    ASSERT_TRUE(visual.targetPose());
    EXPECT_DOUBLE_EQ(visual.targetPose()->x.value, initialTarget->x.value);
    EXPECT_DOUBLE_EQ(visual.targetPose()->y.value, initialTarget->y.value);
    EXPECT_DOUBLE_EQ(visual.targetPose()->heading.value, initialTarget->heading.value);
    EXPECT_EQ(transport.commands.size(), commandCount);
}

TEST_F(LiveVisualControlTest, PathGeometryAndRawOverlayProjectionSurviveResizeAndLetterboxing)
{
    const auto profile = calibratedPathProfile();
    preparePathCalibration(*this, profile);

    VisualControlPanel panel{visual};
    panel.resize(430, 900);
    panel.show();
    auto* overlay = panel.findChild<VisionOverlay*>("visionOverlay");
    auto* begin = panel.findChild<QPushButton*>("pathEditorBegin");
    ASSERT_TRUE(overlay && begin);
    panel.presentPreview(QStringLiteral("camera-a"), debugPreview(
        PixelFormat::Gray8, profile.intrinsics.width, profile.intrinsics.height, now));
    QApplication::processEvents();
    begin->click();
    ASSERT_TRUE(panel.pathEditing());

    const WorldPoint2D first{Meters{0.2475}, Meters{0.3125}};
    const WorldPoint2D second{Meters{0.7125}, Meters{0.4875}};
    const auto firstRaw = rawPixelForWorld(profile, first);
    const auto secondRaw = rawPixelForWorld(profile, second);
    ASSERT_TRUE(firstRaw && secondRaw);
    const auto commandCount = transport.commands.size();

    for (const auto& [size, point] : std::array<std::pair<QSize, ImageCorner>, 2>{
             std::pair{QSize{430, 900}, *firstRaw}, std::pair{QSize{1000, 600}, *secondRaw}}) {
        panel.resize(size);
        QApplication::processEvents();
        const auto imageRect = overlay->imageRect();
        ASSERT_FALSE(imageRect.isEmpty());
        ASSERT_TRUE(imageRect.left() > 1.0 || imageRect.top() > 1.0)
            << "Expected letterbox padding at panel size " << size.width() << "x" << size.height();

        clickRawPixel(overlay, point);
        const auto expected = point.u == firstRaw->u ? first : second;
        ASSERT_EQ(panel.draftPath().size(), point.u == firstRaw->u ? 1U : 2U);
        EXPECT_NEAR(panel.draftPath().points().back().x.value, expected.x.value, 1e-6);
        EXPECT_NEAR(panel.draftPath().points().back().y.value, expected.y.value, 1e-6);

        QPointF marginClick;
        if (imageRect.left() > 1.0)
            marginClick = {imageRect.left() / 2.0, imageRect.center().y()};
        else
            marginClick = {imageRect.center().x(), imageRect.top() / 2.0};
        ASSERT_TRUE(overlay->rect().contains(marginClick.toPoint()));
        sendFractionalLeftClick(overlay, marginClick);
        EXPECT_EQ(panel.draftPath().size(), point.u == firstRaw->u ? 1U : 2U);

        QImage rendered(overlay->size(), QImage::Format_ARGB32);
        rendered.fill(Qt::transparent);
        overlay->render(&rendered);
        const auto mappedPoint = overlay->imageToView({point.u, point.v}).toPoint();
        EXPECT_EQ(rendered.pixelColor(mappedPoint), QColor(255, 40, 220));
    }

    ASSERT_EQ(panel.draftPath().size(), 2U);
    EXPECT_NEAR(panel.draftPath().points()[0].x.value, first.x.value, 1e-6);
    EXPECT_NEAR(panel.draftPath().points()[0].y.value, first.y.value, 1e-6);
    EXPECT_NEAR(panel.draftPath().points()[1].x.value, second.x.value, 1e-6);
    EXPECT_NEAR(panel.draftPath().points()[1].y.value, second.y.value, 1e-6);
    EXPECT_EQ(transport.commands.size(), commandCount);
}

TEST_F(LiveVisualControlTest, PathEditorRejectsStaleWrongSourceAndInvalidWorkspaceClicks)
{
    auto profile = calibratedPathProfile();
    preparePathCalibration(*this, profile);
    VisualControlPanel panel{visual};
    panel.resize(900, 520);
    panel.show();
    auto* overlay = panel.findChild<VisionOverlay*>("visionOverlay");
    auto* begin = panel.findChild<QPushButton*>("pathEditorBegin");
    ASSERT_TRUE(overlay && begin);
    const WorldPoint2D expected{Meters{0.45}, Meters{0.3}};
    const auto raw = rawPixelForWorld(profile, expected);
    ASSERT_TRUE(raw);
    const auto outsideWorkspace = OpenCvCalibrationUndistorter::distortPoint(
        {99.0, 700.0}, profile.intrinsics);
    ASSERT_TRUE(outsideWorkspace);
    const auto commandCount = transport.commands.size();

    panel.presentPreview(QStringLiteral("camera-a"), debugPreview(
        PixelFormat::Gray8, profile.intrinsics.width, profile.intrinsics.height, now));
    QApplication::processEvents();
    begin->click();
    ASSERT_TRUE(panel.pathEditing());
    clickRawPixel(overlay, *outsideWorkspace);
    EXPECT_TRUE(panel.draftPath().empty());
    panel.presentPreview(QStringLiteral("camera-a"), debugPreview(
        PixelFormat::Gray8, profile.intrinsics.width, profile.intrinsics.height, now - 1s));
    QApplication::processEvents();
    clickRawPixel(overlay, *raw);
    EXPECT_TRUE(panel.draftPath().empty());
    EXPECT_TRUE(panel.pathEditing());

    panel.presentPreview(QStringLiteral("camera-b"), debugPreview(
        PixelFormat::Gray8, profile.intrinsics.width, profile.intrinsics.height, now));
    QApplication::processEvents();
    EXPECT_FALSE(panel.pathEditing());
    EXPECT_TRUE(panel.draftPath().empty());
    panel.presentPreview(QStringLiteral("camera-a"), debugPreview(
        PixelFormat::Gray8, profile.intrinsics.width, profile.intrinsics.height, now));
    QApplication::processEvents();
    begin->click();
    ASSERT_TRUE(panel.pathEditing());
    panel.presentPreview(QStringLiteral("camera-b"), debugPreview(
        PixelFormat::Gray8, profile.intrinsics.width, profile.intrinsics.height, now));
    QApplication::processEvents();
    clickRawPixel(overlay, *raw);
    EXPECT_TRUE(panel.draftPath().empty());
    EXPECT_EQ(transport.commands.size(), commandCount);

    panel.cancelPathEditing();
    profile.homography.reset();
    desktop.setWorkspaceCalibration(profile);
    visual.refresh();
    panel.presentPreview(QStringLiteral("camera-a"), debugPreview(
        PixelFormat::Gray8, profile.intrinsics.width, profile.intrinsics.height, now));
    EXPECT_FALSE(panel.beginPathEditing());
    EXPECT_TRUE(panel.draftPath().empty());
    EXPECT_EQ(transport.commands.size(), commandCount);
}

TEST_F(LiveVisualControlTest, PathEditingScopeChangeDiscardsDraftAndFinishedGeometry)
{
    auto profile = calibratedPathProfile();
    preparePathCalibration(*this, profile);
    VisualControlPanel panel{visual};
    panel.resize(900, 520);
    panel.show();
    auto* overlay = panel.findChild<VisionOverlay*>("visionOverlay");
    auto* begin = panel.findChild<QPushButton*>("pathEditorBegin");
    ASSERT_TRUE(overlay && begin);
    panel.presentPreview(QStringLiteral("camera-a"), debugPreview(
        PixelFormat::Gray8, profile.intrinsics.width, profile.intrinsics.height, now));
    QApplication::processEvents();
    const auto raw = rawPixelForWorld(profile, {Meters{0.5}, Meters{0.3}});
    ASSERT_TRUE(raw);
    begin->click();
    clickRawPixel(overlay, *raw);
    ASSERT_EQ(panel.draftPath().size(), 1U);
    ASSERT_TRUE(panel.finishPathEditing());
    ASSERT_EQ(panel.finishedPath().size(), 1U);

    auto changedProfile = profile;
    changedProfile.frameId = "table-B";
    desktop.setWorkspaceCalibration(changedProfile);
    visual.refresh();
    panel.presentPreview(QStringLiteral("camera-a"), debugPreview(
        PixelFormat::Gray8, changedProfile.intrinsics.width, changedProfile.intrinsics.height, now));
    EXPECT_FALSE(panel.pathEditing());
    EXPECT_TRUE(panel.draftPath().empty());
    EXPECT_TRUE(panel.finishedPath().empty());
}

TEST_F(LiveVisualControlTest, ImagePreviewMatchingLiveSourceStillRejectsDifferentProfileCamera)
{
    auto profile = test::homographyInput().profile;
    profile.intrinsics.sourceId = "camera-b";
    profile.homography = test::knownHomography();
    useIdentityPointUndistorter(visual);
    desktop.setWorkspaceCalibration(profile);
    visual.refresh();
    now += 1ms;
    sample(0, 0);

    VisualControlPanel panel{visual};
    panel.resize(760, 640);
    panel.show();
    QApplication::processEvents();
    auto* overlay = panel.findChild<VisionOverlay*>("visionOverlay");
    ASSERT_TRUE(overlay);
    panel.presentPreview(QStringLiteral("camera-a"), debugPreview(
        PixelFormat::Gray8, profile.intrinsics.width, profile.intrinsics.height, now));
    QApplication::processEvents();
    const auto click = overlay->imageToView({500.25, 400.25});
    sendFractionalLeftClick(overlay, click);

    EXPECT_FALSE(visual.targetPose());
    EXPECT_FALSE(visual.running());
    EXPECT_TRUE(transport.commands.empty());
}
TEST_F(LiveVisualControlTest, PauseStopsAndRetainsTargetUntilExplicitRestart)
{
    run();
    const auto submittedBeforePause = transport.commands.size();

    visual.pause();

    EXPECT_FALSE(visual.running());
    EXPECT_EQ(visual.operationalState(), ControlOperationalState::Paused);
    EXPECT_TRUE(std::holds_alternative<StopCommand>(transport.commands.back().second.payload));
    EXPECT_TRUE(visual.description().contains("Target: x 1.000"));
    const auto submittedAfterPause = transport.commands.size();
    EXPECT_GT(submittedAfterPause, submittedBeforePause);

    now += 1ms;
    visual.refresh();
    EXPECT_FALSE(visual.running());
    EXPECT_EQ(transport.commands.size(), submittedAfterPause);

    ASSERT_TRUE(visual.start());
    visual.refresh();
    ASSERT_TRUE(visual.running());
    ASSERT_TRUE(visual.lastCycle()->targetPose);
    EXPECT_DOUBLE_EQ(visual.lastCycle()->targetPose->x.value, 1.0);
    ASSERT_TRUE(visual.lastCycle()->error);
    EXPECT_DOUBLE_EQ(visual.lastCycle()->error->xMeters, 1.0);
}
TEST_F(LiveVisualControlTest, CancelStopsAndClearsTargetBeforeRestartCanBeAccepted)
{
    run();
    visual.cancel();

    EXPECT_FALSE(visual.running());
    EXPECT_EQ(visual.operationalState(), ControlOperationalState::Paused);
    EXPECT_TRUE(std::holds_alternative<StopCommand>(transport.commands.back().second.payload));
    EXPECT_FALSE(visual.description().contains("Target:"));
    EXPECT_FALSE(visual.start());

    const auto submittedAfterCancel = transport.commands.size();
    visual.refresh();
    EXPECT_EQ(transport.commands.size(), submittedAfterCancel);

    ASSERT_TRUE(visual.setTarget(0.75, 0.5, 0.25));
    EXPECT_TRUE(visual.start());
    visual.refresh();
    ASSERT_TRUE(visual.running());
    ASSERT_TRUE(visual.lastCycle()->targetPose);
    EXPECT_DOUBLE_EQ(visual.lastCycle()->targetPose->x.value, 0.75);
    EXPECT_DOUBLE_EQ(visual.lastCycle()->targetPose->y.value, 0.5);
}
TEST_F(LiveVisualControlTest, ControllerFaultNeedsExplicitAcknowledgementBeforeRearm)
{
    VisualControlPanel panel{visual};
    auto* acknowledge = panel.findChild<QPushButton*>("acknowledgeVisualFault");
    ASSERT_TRUE(acknowledge);
    run();
    now -= 1ms;
    visual.refresh();
    EXPECT_FALSE(visual.running());
    EXPECT_EQ(visual.operationalState(), ControlOperationalState::Fault);
    EXPECT_FALSE(visual.start());
    EXPECT_FALSE(panel.findChild<QPushButton*>("startVisualControl")->isEnabled());
    EXPECT_FALSE(acknowledge->isEnabled());

    now = Timestamp{1s} + 1ms;
    sample(0, 0);
    EXPECT_TRUE(acknowledge->isEnabled());
    acknowledge->click();
    EXPECT_EQ(visual.operationalState(), ControlOperationalState::Ready);
    EXPECT_TRUE(visual.start());
}
TEST_F(LiveVisualControlTest, ArrivalSubmitsZeroThenStopAndDoesNotResume)
{
    ASSERT_TRUE(visual.setTarget(0, 0, 0));
    ASSERT_TRUE(visual.start()); visual.refresh();
    EXPECT_FALSE(visual.running());
    ASSERT_GE(transport.commands.size(), 2u);
    const auto& velocity = std::get<VelocityCommand>(transport.commands[transport.commands.size()-2].second.payload);
    EXPECT_DOUBLE_EQ(velocity.linearMetersPerSecond, 0); EXPECT_DOUBLE_EQ(velocity.angularRadiansPerSecond, 0);
    EXPECT_TRUE(std::holds_alternative<StopCommand>(transport.commands.back().second.payload));
    EXPECT_TRUE(visual.description().contains("Arrived"));
}
TEST_F(LiveVisualControlTest, HeadingArrivalUsesTheControllerExitTolerance)
{
    ASSERT_TRUE(visual.setTarget(0, 0, .2));
    ASSERT_TRUE(visual.start());
    ASSERT_TRUE(visual.running());
    now += LiveVisualSettings::period;
    // The fixture filters with alpha 0.25, yielding x=-0.007 m and heading=0.2 rad.
    sample(-.028, 0, .8);
    EXPECT_FALSE(visual.running());
    ASSERT_TRUE(std::holds_alternative<StopCommand>(transport.commands.back().second.payload));
    EXPECT_TRUE(visual.description().contains("Arrived"));
}
class BlockingSink final : public ILogSink {
public:
    std::promise<void> entered, release;
    std::shared_future<void> gate{release.get_future().share()};
    bool first{true}; std::size_t count{};
    void write(const LogRecord&) override { if (first) { first=false; entered.set_value(); gate.wait(); } ++count; }
};
TEST_F(LiveVisualControlTest, ExperimentRunExportsReconstructableJsonLinesAndConfiguration)
{
    QTemporaryDir directory;
    ASSERT_TRUE(directory.isValid());
    Logger exportLogger;
    {
        ControlTraceWriter exportWriter{exportLogger.channel("experiment"),
            std::filesystem::path{directory.path().toStdString()}};
        {
            LiveVisualController recorded{desktop, world, robots, simulationVisualSettings(), {},
                [this] { return now; }, nullptr, &exportWriter};
            recorded.setInput("camera-a", true);
            auto metadata = std::make_shared<MetricObservationBatch>();
            metadata->status = MetricObservationStatus::Valid;
            metadata->measurements.push_back({RobotId{1}, {}, now, 1, 20, {}});
            recorded.receiveMeasurements(metadata);
            ASSERT_TRUE(recorded.setTarget(1, 0, 0));
            ASSERT_TRUE(recorded.start());
            recorded.refresh();
            EXPECT_TRUE(recorded.description().contains("Experiment export"));
            recorded.stop();
        }
    }

    const auto files = QDir(directory.path()).entryList({QStringLiteral("*.jsonl")}, QDir::Files);
    ASSERT_EQ(files.size(), 1);
    QFile file(QDir(directory.path()).filePath(files.front()));
    ASSERT_TRUE(file.open(QIODevice::ReadOnly));
    const auto lines = file.readAll().split('\n');
    ASSERT_GE(lines.size(), 4);
    const auto header = QJsonDocument::fromJson(lines[0]).object();
    EXPECT_EQ(header["type"].toString(), QStringLiteral("run_start"));
    EXPECT_EQ(header["schema_version"].toInt(), 1);
    EXPECT_EQ(header["coordinate_units"].toString(), QStringLiteral("metres; radians"));
    EXPECT_EQ(header["position_gain_per_s"].toDouble(), simulationVisualSettings().goal.positionGainPerSecond);
    EXPECT_EQ(header["position_exit_tolerance_m"].toDouble(), simulationVisualSettings().goal.positionExitTolerance.value);
    EXPECT_EQ(header["target_x_m"].toDouble(), 1.0);
    const auto sample = QJsonDocument::fromJson(lines[1]).object();
    EXPECT_EQ(sample["type"].toString(), QStringLiteral("sample"));
    EXPECT_EQ(sample["pose_x_m"].toDouble(), 0.0);
    EXPECT_EQ(sample["target_x_m"].toDouble(), 1.0);
    EXPECT_EQ(sample["error_distance_m"].toDouble(), 1.0);
    EXPECT_TRUE(sample.contains("tracking_age_ns"));
    EXPECT_TRUE(sample.contains("tracking_confidence"));
    EXPECT_TRUE(sample.contains("measurement_quality"));
    EXPECT_TRUE(sample.contains("raw_pose_x_m"));
    EXPECT_EQ(sample["frame_id"].toString(), QStringLiteral("metric-frame"));
    EXPECT_TRUE(sample.contains("operational_state"));
    EXPECT_TRUE(sample.contains("skipped_periods"));
    EXPECT_TRUE(sample.contains("control_time_ns"));
    EXPECT_TRUE(sample.contains("submission_time_ns"));
    const auto footer = QJsonDocument::fromJson(lines[lines.size() - 2]).object();
    EXPECT_EQ(footer["type"].toString(), QStringLiteral("run_end"));
    EXPECT_EQ(footer["reason"].toString(), QStringLiteral("operator_stop"));
    EXPECT_TRUE(footer["queue_complete"].toBool());
    EXPECT_EQ(footer["dropped_samples"].toString(), QStringLiteral("0"));
}

TEST_F(LiveVisualControlTest, EmergencyStopRunExportRecordsLatchedStateAndReason)
{
    QTemporaryDir directory;
    ASSERT_TRUE(directory.isValid());
    Logger exportLogger;
    {
        ControlTraceWriter exportWriter{exportLogger.channel("experiment"),
            std::filesystem::path{directory.path().toStdString()}};
        {
            LiveVisualController recorded{desktop, world, robots, simulationVisualSettings(), {},
                [this] { return now; }, nullptr, &exportWriter};
            recorded.setInput("camera-a", true);
            auto metadata = std::make_shared<MetricObservationBatch>();
            metadata->status = MetricObservationStatus::Valid;
            metadata->measurements.push_back({RobotId{1}, {}, now, 1, 20, {}});
            recorded.receiveMeasurements(metadata);
            ASSERT_TRUE(recorded.setTarget(1, 0, 0));
            ASSERT_TRUE(recorded.start());
            desktop.emergencyStop();
            EXPECT_FALSE(recorded.running());
        }
    }

    const auto files = QDir(directory.path()).entryList({QStringLiteral("*.jsonl")}, QDir::Files);
    ASSERT_EQ(files.size(), 1);
    QFile file(QDir(directory.path()).filePath(files.front()));
    ASSERT_TRUE(file.open(QIODevice::ReadOnly));
    const auto lines = file.readAll().split('\n');
    ASSERT_GE(lines.size(), 3);
    const auto footer = QJsonDocument::fromJson(lines[lines.size() - 2]).object();
    EXPECT_EQ(footer["type"].toString(), QStringLiteral("run_end"));
    EXPECT_EQ(footer["final_state"].toString(), QStringLiteral("ESTOP"));
    EXPECT_EQ(footer["reason"].toString(), QStringLiteral("emergency_stop"));
}

TEST_F(LiveVisualControlTest, ManualTakeoverRunExportRecordsTheInterruptionCause)
{
    QTemporaryDir directory;
    ASSERT_TRUE(directory.isValid());
    Logger exportLogger;
    {
        ControlTraceWriter exportWriter{exportLogger.channel("experiment"),
            std::filesystem::path{directory.path().toStdString()}};
        {
            LiveVisualController recorded{desktop, world, robots, simulationVisualSettings(), {},
                [this] { return now; }, nullptr, &exportWriter};
            recorded.setInput("camera-a", true);
            auto metadata = std::make_shared<MetricObservationBatch>();
            metadata->status = MetricObservationStatus::Valid;
            metadata->measurements.push_back({RobotId{1}, {}, now, 1, 20, {}});
            recorded.receiveMeasurements(metadata);
            ASSERT_TRUE(recorded.setTarget(1, 0, 0));
            ASSERT_TRUE(recorded.start());
            desktop.beginManualDrive(-.05, 0);
            EXPECT_FALSE(recorded.running());
        }
    }
    const auto files = QDir(directory.path()).entryList({QStringLiteral("*.jsonl")}, QDir::Files);
    ASSERT_EQ(files.size(), 1);
    QFile file(QDir(directory.path()).filePath(files.front()));
    ASSERT_TRUE(file.open(QIODevice::ReadOnly));
    const auto lines = file.readAll().split('\n');
    ASSERT_GE(lines.size(), 3);
    const auto footer = QJsonDocument::fromJson(lines[lines.size() - 2]).object();
    EXPECT_EQ(footer["reason"].toString(), QStringLiteral("manual_takeover"));
    EXPECT_EQ(footer["final_state"].toString(), QStringLiteral("PAUSED"));
}

TEST_F(LiveVisualControlTest, FailedRunExportIsVisibleAndDoesNotShowAUsablePath)
{
    QTemporaryDir directory;
    ASSERT_TRUE(directory.isValid());
    QFile blocker(QDir(directory.path()).filePath(QStringLiteral("not-a-directory")));
    ASSERT_TRUE(blocker.open(QIODevice::WriteOnly));
    blocker.close();
    Logger exportLogger;
    {
        ControlTraceWriter exportWriter{exportLogger.channel("experiment"),
            std::filesystem::path{blocker.fileName().toStdString()}};
        LiveVisualController recorded{desktop, world, robots, simulationVisualSettings(), {},
            [this] { return now; }, nullptr, &exportWriter};
        recorded.setInput("camera-a", true);
        auto metadata = std::make_shared<MetricObservationBatch>();
        metadata->status = MetricObservationStatus::Valid;
        metadata->measurements.push_back({RobotId{1}, {}, now, 1, 20, {}});
        recorded.receiveMeasurements(metadata);
        ASSERT_TRUE(recorded.setTarget(1, 0, 0));
        ASSERT_TRUE(recorded.start());
        QTRY_VERIFY(recorded.description().contains("Experiment export failed"));
        EXPECT_FALSE(recorded.description().contains(blocker.fileName()));
        recorded.stop();
    }
}

TEST(LiveVisualSimulationTest, AcceptedMetricFeedbackMovesSimulationThenStaleDataStopsIt)
{
    Timestamp now{1s};
    Logger logger;
    WorldModel world;
    SimulationTransport transport;
    RobotManager robots{transport, world};
    DesktopController desktop{robots, world, RobotId{1}, logger.channel("desktop")};
    desktop.initialize(now);
    ASSERT_TRUE(robots.connect(RobotId{1})); transport.advanceTo(now);
    world.setWorkspaceFrame(WorkspaceBounds{Meters{2}, Meters{2}}, "metric-frame");
    RobotTracker tracker{world, {250ms, 1s}};
    LiveVisualController visual{desktop, world, robots, simulationVisualSettings(), {}, [&] { return now; }};
    visual.setInput("synthetic-camera", true);
    auto batch = std::make_shared<MetricObservationBatch>();
    batch->status = MetricObservationStatus::Valid;
    batch->measurements.push_back({RobotId{1}, {}, now, 1, 20, {}});
    visual.receiveMeasurements(ingestMetricBatch(tracker, batch, now));
    ASSERT_TRUE(visual.setTarget(1, 0, 0)); ASSERT_TRUE(visual.start()); visual.refresh(); transport.advanceTo(now);
    ASSERT_TRUE(transport.lastSentCommand(RobotId{1}));
    EXPECT_GT(std::get<VelocityCommand>(transport.lastSentCommand(RobotId{1})->payload).linearMetersPerSecond, 0);
    EXPECT_EQ(transport.robotStatus(RobotId{1})->state, SimulationRobotState::Moving);
    now += 250ms; visual.refresh(); transport.advanceTo(now);
    EXPECT_FALSE(visual.running());
    EXPECT_EQ(transport.robotStatus(RobotId{1})->state, SimulationRobotState::Idle);
    EXPECT_TRUE(std::holds_alternative<StopCommand>(transport.lastSentCommand(RobotId{1})->payload));
    now += 1ms; batch->measurements[0].capturedAt=now; batch->measurements[0].frameSequence=2;
    visual.receiveMeasurements(ingestMetricBatch(tracker, batch, now));
    EXPECT_FALSE(visual.running());
}
TEST(ControlTraceWriterTest, RunFooterSlotSurvivesSamplePressureAndExportIsJsonLines)
{
    QTemporaryDir directory;
    ASSERT_TRUE(directory.isValid());
    Logger logger;
    auto sink = std::make_shared<BlockingSink>();
    logger.addSink(sink);
    std::filesystem::path outputPath;
    {
        ControlTraceWriter writer{logger.channel("trace"), std::filesystem::path{directory.path().toStdString()}};
        const auto path = writer.beginRun("bounded.jsonl", R"({"type":"run_start"})");
        ASSERT_TRUE(path);
        outputPath = *path;
        EXPECT_TRUE(writer.enqueue(R"({"type":"sample","index":0})"));
        const auto status = sink->entered.get_future().wait_for(2s);
        if (status != std::future_status::ready) {
            sink->release.set_value();
            FAIL() << "Worker did not enter sink";
        }
        for (std::size_t i=0; i<ControlTraceWriter::capacity-2; ++i)
            EXPECT_TRUE(writer.enqueueSample(R"({"type":"sample","index":1})"));
        EXPECT_FALSE(writer.enqueueSample(R"({"type":"sample","index":"overflow"})"));
        EXPECT_EQ(writer.dropped(), 1u);
        EXPECT_TRUE(writer.finishRun(R"({"type":"run_end","queue_complete":false})"));
        sink->release.set_value();
    }
    std::ifstream exported(outputPath);
    ASSERT_TRUE(exported.is_open());
    std::string line;
    std::size_t lines{};
    while (std::getline(exported, line)) {
        EXPECT_TRUE(QJsonDocument::fromJson(QByteArray::fromStdString(line)).isObject());
        ++lines;
    }
    EXPECT_EQ(lines, ControlTraceWriter::capacity);
}

TEST(ControlTraceWriterTest, BlockedSinkCannotBlockProducerAndQueueAndBytesAreBounded)
{
    Logger logger; auto sink = std::make_shared<BlockingSink>(); logger.addSink(sink);
    {
        ControlTraceWriter writer{logger.channel("trace")};
        EXPECT_TRUE(writer.enqueue("first"));
        const auto status = sink->entered.get_future().wait_for(2s);
        if (status != std::future_status::ready) { sink->release.set_value(); FAIL() << "Worker did not enter sink"; }
        for (std::size_t i=0; i<ControlTraceWriter::capacity; ++i) EXPECT_TRUE(writer.enqueue("queued"));
        EXPECT_FALSE(writer.enqueue("overflow"));
        EXPECT_FALSE(writer.enqueue(std::string(ControlTraceWriter::maximumLineBytes+1, 'x')));
        EXPECT_EQ(writer.dropped(), 2u);
        sink->release.set_value();
    }
    EXPECT_EQ(sink->count, ControlTraceWriter::capacity+1);
}
} // namespace
