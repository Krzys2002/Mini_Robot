#include "app/VisionController.hpp"
#include "app/DesktopController.hpp"
#include "ui/MainWindow.hpp"
#include "ManualBackends.hpp"

#include <QApplication>
#include <QElapsedTimer>
#include <QLabel>
#include <QTimer>
#include <gtest/gtest.h>
#include <atomic>
#include <future>
#include <functional>

namespace {
using namespace micro_robot;
using namespace micro_robot::test;
bool visionUntil(const std::function<bool()>& done)
{
    QElapsedTimer timer;
    timer.start();
    while (!done() && timer.elapsed() < 5000) {
        QApplication::processEvents();
        QThread::yieldCurrentThread();
    }
    return done();
}
FramePtr visionFrame(std::uint64_t sequence, std::uint32_t width = 2)
{
    auto f = std::make_shared<Frame>();
    f->sequence = sequence;
    f->capturedAt = Timestamp{} + std::chrono::milliseconds{sequence};
    f->width = width;
    f->height = 2;
    f->strideBytes = width;
    f->pixelFormat = PixelFormat::Gray8;
    f->pixels = std::make_shared<const std::vector<std::byte>>(width * 2);
    return f;
}
class GatedMarkerDetector final : public IMarkerDetector {
public:
    MarkerDetections detect(const Frame& frame) override
    {
        onWorker = QThread::currentThread() != QApplication::instance()->thread();
        ++calls;
        if (gate.valid()) gate.wait();
        if (fail && frame.sequence == 1) throw std::runtime_error("test detector failure");
        MarkerDetections result;
        result.markers.push_back({MarkerFamily::AprilTag36h11, static_cast<int>(frame.sequence), {}, {}});
        return result;
    }
    std::shared_future<void> gate;
    std::atomic<int> calls{};
    std::atomic<bool> onWorker{};
    bool fail{};
};
struct VisionFixture {
    Logger logger;
    std::promise<void> release;
    GatedMarkerDetector* detector{};
    std::unique_ptr<VisionController> vision;
    std::vector<MarkerDetectionsPtr> results;
    std::vector<VisionDebugFramePtr> previews;
    std::vector<QString> sources;
    int clears{};
    VisionFixture()
    {
        auto d = std::make_unique<GatedMarkerDetector>();
        detector = d.get();
        detector->gate = release.get_future().share();
        vision = std::make_unique<VisionController>(std::move(d), logger.channel("vision"));
        QObject::connect(vision.get(), &VisionController::observationsAvailable, vision.get(),
            [this](const QString& source, MarkerDetectionsPtr result) {
                if (result) { results.push_back(result); sources.push_back(source); }
                else ++clears;
            });
        QObject::connect(vision.get(), &VisionController::previewAvailable, vision.get(),
            [this](const QString&, VisionDebugFramePtr preview) { if (preview) previews.push_back(preview); });
        vision->setInput("camera-a", true);
    }
    void unblock() { if (!released) { released = true; release.set_value(); } }
    ~VisionFixture()
    {
        unblock();
        vision->shutdown();
        (void)visionUntil([&] { return vision->isStopped(); });
    }
    bool released{};
};
TEST(VisionControllerTest, KeepsOnlyNewestPendingFrameAndGuiProcessesEventsDuringDetection)
{
    VisionFixture f;
    f.vision->receiveFrame(visionFrame(1));
    ASSERT_TRUE(visionUntil([&] { return f.detector->calls == 1; }));
    int beats = 0;
    QTimer heartbeat;
    QObject::connect(&heartbeat, &QTimer::timeout, [&] { ++beats; });
    heartbeat.start(0);
    std::weak_ptr<const Frame> skipped;
    for (std::uint64_t i = 2; i <= 100; ++i) {
        auto frame = visionFrame(i);
        if (i == 2) skipped = frame;
        f.vision->receiveFrame(frame);
    }
    EXPECT_TRUE(skipped.expired());
    EXPECT_TRUE(visionUntil([&] { return beats >= 3; }));
    EXPECT_TRUE(f.detector->onWorker);
    EXPECT_EQ(f.detector->calls, 1);
    f.unblock();
    ASSERT_TRUE(visionUntil([&] { return f.results.size() == 2; }));
    EXPECT_EQ(f.detector->calls, 2);
    EXPECT_EQ(f.results[0]->frameSequence, 1U);
    EXPECT_EQ(f.results[1]->frameSequence, 100U);
    EXPECT_EQ(f.results[1]->capturedAt, visionFrame(100)->capturedAt);
    EXPECT_EQ(f.results[1]->width, 2U);
    EXPECT_GE(f.results[1]->processingTime, Duration::zero());
    ASSERT_EQ(f.previews.size(), 2U);
    EXPECT_EQ(f.previews[0]->frame->sequence, 1U);
    EXPECT_EQ(f.previews[1]->frame->sequence, 100U);
    EXPECT_EQ(f.previews[1]->frame->capturedAt, f.previews[1]->detections->capturedAt);
    EXPECT_EQ(f.previews[1]->detections.get(), f.results[1].get());
    EXPECT_FALSE(f.previews[0]->outputFramesPerSecond);
    EXPECT_TRUE(f.previews[1]->outputFramesPerSecond);
}
TEST(VisionControllerTest, SourceChangeDiscardsInFlightAndPendingOldFrames)
{
    VisionFixture f;
    f.vision->receiveFrame(visionFrame(5));
    ASSERT_TRUE(visionUntil([&] { return f.detector->calls == 1; }));
    f.vision->receiveFrame(visionFrame(6));
    f.vision->setInput("camera-b", true);
    f.vision->receiveFrame(visionFrame(1));
    f.unblock();
    ASSERT_TRUE(visionUntil([&] { return f.results.size() == 1; }));
    EXPECT_EQ(f.sources[0], "camera-b");
    ASSERT_EQ(f.previews.size(), 1U);
    EXPECT_EQ(f.previews[0]->frame->sequence, 1U);
    EXPECT_FALSE(f.previews[0]->outputFramesPerSecond);
    EXPECT_EQ(f.results[0]->frameSequence, 1U);
    EXPECT_EQ(f.detector->calls, 2);
}
TEST(VisionControllerTest, ResolutionChangeInvalidatesAndOldOrDuplicateFramesAreIgnored)
{
    VisionFixture f;
    f.vision->receiveFrame(visionFrame(1));
    ASSERT_TRUE(visionUntil([&] { return f.detector->calls == 1; }));
    f.vision->receiveFrame(visionFrame(2, 4));
    f.vision->receiveFrame(visionFrame(2, 4));
    f.vision->receiveFrame(visionFrame(1, 4));
    f.unblock();
    ASSERT_TRUE(visionUntil([&] { return f.results.size() == 1; }));
    EXPECT_EQ(f.results[0]->width, 4U);
    EXPECT_EQ(f.results[0]->frameSequence, 2U);
    EXPECT_EQ(f.detector->calls, 2);
}
TEST(VisionControllerTest, ShutdownIsPromptWhileDetectorRunsAndPublishesNoStaleResults)
{
    VisionFixture f;
    f.vision->receiveFrame(visionFrame(1));
    ASSERT_TRUE(visionUntil([&] { return f.detector->calls == 1; }));
    f.vision->receiveFrame(visionFrame(2));
    f.vision->shutdown();
    EXPECT_FALSE(f.vision->isStopped());
    f.vision->receiveFrame(visionFrame(3));
    f.unblock();
    ASSERT_TRUE(visionUntil([&] { return f.vision->isStopped(); }));
    QApplication::processEvents();
    EXPECT_TRUE(f.results.empty());
    EXPECT_EQ(f.detector->calls, 1);
}
TEST(VisionControllerTest, DetectorFailureIsReportedAndNextFrameRecovers)
{
    VisionFixture f;
    f.detector->fail = true;
    f.unblock();
    f.vision->receiveFrame(visionFrame(1));
    ASSERT_TRUE(visionUntil([&] { return f.results.size() == 1; }));
    EXPECT_FALSE(f.results[0]->error.empty());
    EXPECT_TRUE(f.results[0]->markers.empty());
    f.vision->receiveFrame(visionFrame(2));
    ASSERT_TRUE(visionUntil([&] { return f.results.size() == 2; }));
    EXPECT_TRUE(f.results[1]->error.empty());
    EXPECT_EQ(f.results[1]->markers.size(), 1U);
}
TEST(VisionControllerTest, StopClearsPendingWorkAndRestartAcceptsANewSourceSequence)
{
    VisionFixture f;
    f.vision->receiveFrame(visionFrame(10));
    ASSERT_TRUE(visionUntil([&] { return f.detector->calls == 1; }));
    f.vision->receiveFrame(visionFrame(11));
    f.vision->setInput("camera-a", false);
    f.vision->receiveFrame(visionFrame(12));
    f.unblock();
    ASSERT_TRUE(visionUntil([&] { return !f.vision->busy(); }));
    EXPECT_TRUE(f.results.empty());
    EXPECT_EQ(f.detector->calls, 1);
    f.vision->setInput("camera-a", true);
    f.vision->receiveFrame(visionFrame(1));
    ASSERT_TRUE(visionUntil([&] { return f.results.size() == 1; }));
    EXPECT_EQ(f.results[0]->frameSequence, 1U);
}
TEST(VisionControllerTest, DescriptionObserverCanInvalidateResultsBeforePublication)
{
    VisionFixture f;
    f.unblock();
    QObject::connect(f.vision.get(), &VisionController::descriptionChanged, f.vision.get(),
        [&](const QString& description) {
            EXPECT_EQ(QThread::currentThread(), QApplication::instance()->thread());
            if (description.contains("tags")) f.vision->setInput("camera-a", false);
        });
    f.vision->receiveFrame(visionFrame(1));
    ASSERT_TRUE(visionUntil([&] { return !f.vision->busy(); }));
    EXPECT_TRUE(f.results.empty());
    EXPECT_TRUE(f.vision->description().contains("inactive"));
}
TEST(VisionControllerTest, ObservationObserverCanInvalidateBeforePreviewPublication)
{
    VisionFixture f;
    f.unblock();
    QObject::connect(f.vision.get(), &VisionController::observationsAvailable, f.vision.get(),
        [&](const QString&, MarkerDetectionsPtr detections) { if (detections) f.vision->setInput("camera-a", false); });
    f.vision->receiveFrame(visionFrame(1));
    ASSERT_TRUE(visionUntil([&] { return !f.vision->busy(); }));
    EXPECT_TRUE(f.previews.empty());
}
TEST(VisionControllerTest, DesktopCameraEventsFeedServiceAndStatusWithoutWorldPoseChanges)
{
    VisionFixture f;
    f.unblock();
    ManualTransport transport;
    WorldModel world;
    RobotManager robots{transport, world};
    DesktopController desktop{robots, world, RobotId{1}, f.logger.channel("controller")};
    QObject::connect(&desktop, &DesktopController::cameraInputChanged, f.vision.get(), &VisionController::setInput);
    QObject::connect(&desktop, &DesktopController::frameAvailable, f.vision.get(), &VisionController::receiveFrame);
    MainWindow window{desktop};
    window.bindVision(*f.vision);
    desktop.initialize(MonotonicClock::now());
    auto camera = std::make_unique<ManualCamera>();
    auto* input = camera.get();
    desktop.setCameraSource(std::move(camera));
    input->publishState(CameraState::Streaming, MonotonicClock::now());
    auto frame = std::make_shared<Frame>(*visionFrame(1));
    frame->capturedAt = MonotonicClock::now();
    input->publishFrame(frame);
    ASSERT_TRUE(visionUntil([&] { return !f.results.empty(); }));
    EXPECT_EQ(f.sources.back(), "manual-camera");
    EXPECT_TRUE(window.findChild<QLabel*>("visionStatus")->text().contains("1 tags"));
    EXPECT_EQ(transport.commands.size(), 0U);
    ASSERT_TRUE(world.robot(RobotId{1}));
    EXPECT_FALSE(world.robot(RobotId{1})->pose);
    input->publishState(CameraState::Stopped, MonotonicClock::now());
    EXPECT_GT(f.clears, 1);
    desktop.shutdown();
}
} // namespace

namespace {
class TestMetricConverter final : public micro_robot::IMetricObservationConverter {
public:
    micro_robot::MetricObservationBatch convert(const micro_robot::MarkerDetections&, const std::string&,
        const micro_robot::RobotMarkerMap&, const std::optional<micro_robot::WorkspaceCalibrationProfile>&) override
    {
        return {micro_robot::MetricObservationStatus::Valid, {}, {}};
    }
};
TEST(VisionControllerMetricTest, CalibrationChangeDiscardsInFlightMeasurement)
{
    VisionFixture f;
    f.vision->setMetricConverter(std::make_unique<TestMetricConverter>());
    int batches = 0;
    QObject::connect(f.vision.get(), &VisionController::metricObservationsAvailable, f.vision.get(),
        [&](const QString&, MetricObservationBatchPtr result) { if (result) ++batches; });
    f.vision->receiveFrame(visionFrame(1));
    ASSERT_TRUE(visionUntil([&] { return f.detector->calls == 1; }));
    f.vision->setWorkspaceCalibration({});
    f.unblock();
    ASSERT_TRUE(visionUntil([&] { return !f.vision->busy(); }));
    EXPECT_EQ(batches, 0);
    f.vision->receiveFrame(visionFrame(2));
    ASSERT_TRUE(visionUntil([&] { return batches == 1; }));
}
}
