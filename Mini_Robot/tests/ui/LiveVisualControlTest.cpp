#include "app/LiveVisualController.hpp"
#include "app/ControlTraceWriter.hpp"
#include "app/VisionTracking.hpp"
#include "ui/VisualControlPanel.hpp"
#include "ManualBackends.hpp"
#include "simulation/SimulationTransport.hpp"

#include <QApplication>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSignalSpy>
#include <QTemporaryDir>
#include <QTest>
#include <gtest/gtest.h>
#include <future>
#include <filesystem>
#include <fstream>
#include <limits>

namespace {
using namespace micro_robot;
using namespace std::chrono_literals;
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
    ASSERT_TRUE(x && y && h && acknowledge);
    EXPECT_FALSE(acknowledge->isEnabled());
    x->setText("1.25"); y->setText("0.5"); h->setText("1.0");
    panel.findChild<QPushButton*>("applyVisualTarget")->click();
    auto* start = panel.findChild<QPushButton*>("startVisualControl"); ASSERT_TRUE(start->isEnabled()); start->click();
    visual.refresh(); EXPECT_TRUE(visual.running()); EXPECT_FALSE(x->isEnabled());
    EXPECT_TRUE(panel.findChild<QLabel*>("visualControlStatus")->text().contains("Target: x 1.250"));
    if (const auto path = qEnvironmentVariable("MICRO_ROBOT_VISUAL_CONTROL_SCREENSHOT"); !path.isEmpty()) EXPECT_TRUE(panel.grab().save(path));
    panel.findChild<QPushButton*>("stopVisualControl")->click();
    EXPECT_FALSE(visual.running()); EXPECT_TRUE(x->isEnabled());
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
    for (int i = 0; i < 8 && visual.running(); ++i) {
        now += LiveVisualSettings::period;
        sample(-.007, 0, .2);
    }
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
