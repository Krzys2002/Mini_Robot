#include "app/CalibrationController.hpp"
#include "app/DesktopController.hpp"
#include "ui/CalibrationWizard.hpp"
#include "ui/MainWindow.hpp"
#include "infrastructure/logging/Logger.hpp"
#include "simulation/SimulationTransport.hpp"
#include "ManualBackends.hpp"
#include "../calibration/CaptureFixtures.hpp"
#include "../calibration/IntrinsicFixtures.hpp"

#include <QApplication>
#include <QComboBox>
#include <QElapsedTimer>
#include <QLabel>
#include <QImage>
#include <QPixmap>
#include <QPushButton>
#include <QThread>
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <future>

namespace {
using namespace micro_robot;
using namespace micro_robot::test;
using namespace std::chrono_literals;

bool processUntil(const std::function<bool()>& predicate)
{
    QElapsedTimer timer;
    timer.start();
    while (!predicate() && timer.elapsed() < 5000) {
        QApplication::processEvents();
        QThread::yieldCurrentThread();
    }
    return predicate();
}
class DeterministicDetector final : public ICalibrationDetector {
public:
    CalibrationDetection detect(const Frame& frame, const CalibrationTarget&) override
    {
        onWorker = QThread::currentThread() != QApplication::instance()->thread();
        ++calls;
        if (gate.valid()) gate.wait();
        return captureDetection(250 + (frame.sequence % 3) * 250);
    }
    std::shared_future<void> gate;
    std::atomic<int> calls{};
    std::atomic<bool> onWorker{};
};
class DeterministicSolver final : public IIntrinsicCalibrationSolver {
public:
    IntrinsicCalibrationOutcome solve(const IntrinsicCalibrationInput& input,
                                      const IntrinsicCalibrationConfig&) override
    {
        onWorker = QThread::currentThread() != QApplication::instance()->thread();
        ++calls;
        if (gate.valid()) gate.wait();
        if (throws) throw std::runtime_error("Synthetic solver failure");
        if (fails) return {{}, "Synthetic invalid observations"};
        auto result = validIntrinsicCalibration();
        result.sourceId = input.sourceId;
        result.width = input.width;
        result.height = input.height;
        result.perViewErrors.clear();
        for (const auto& view : input.observations)
            result.perViewErrors.push_back({view.frameSequence, view.corners.size(), rms});
        result.rmsPixels = rms;
        if (invalid) result.cameraMatrix[0] = -1;
        if (wrongCamera) result.sourceId = "wrong-camera";
        return {std::move(result), {}};
    }
    std::shared_future<void> gate;
    std::atomic<int> calls{};
    std::atomic<bool> onWorker{};
    double rms{0.5};
    bool fails{}, throws{}, invalid{}, wrongCamera{};
};
CalibrationCaptureConfig uiConfig()
{
    CalibrationCaptureConfig config;
    config.minimumViews = 3;
    config.minimumLocationCells = 1;
    config.minimumScaleSpread = config.minimumAngleSpreadRadians = 0;
    return config;
}
struct CaptureFixture {
    Logger logger;
    DeterministicDetector* detector{};
    DeterministicSolver* solver{};
    std::unique_ptr<CalibrationController> controller;
    std::unique_ptr<CalibrationWizard> wizard;
    explicit CaptureFixture(IntrinsicCalibrationConfig intrinsicConfig = {})
    {
        auto implementation = std::make_unique<DeterministicDetector>();
        detector = implementation.get();
        auto solverImplementation = std::make_unique<DeterministicSolver>();
        solver = solverImplementation.get();
        controller = std::make_unique<CalibrationController>(std::move(implementation),
            std::move(solverImplementation),
            logger.channel("calibration"), uiConfig(), intrinsicConfig);
        wizard = std::make_unique<CalibrationWizard>(*controller);
        controller->setInput(QStringLiteral("camera-a"), true);
    }
    ~CaptureFixture()
    {
        controller->shutdown();
        (void)processUntil([&] { return controller->isStopped(); });
    }
    void frame(std::uint64_t sequence, std::uint32_t width = 1000)
    {
        auto input = std::make_shared<Frame>(*captureFrame(sequence, width));
        input->capturedAt = MonotonicClock::now();
        controller->receiveFrame(input);
    }
    QPushButton* button(const char* name) { return wizard->findChild<QPushButton*>(QString::fromLatin1(name)); }
    void capture(std::uint64_t sequence)
    {
        frame(sequence);
        button("calibrationCapture")->click();
        ASSERT_TRUE(processUntil([&] { return !controller->busy(); }));
    }
};
TEST(CalibrationWizardTest, CompletesCaptureAndReviewsAcceptedDetectionsWithoutScripts)
{
    CaptureFixture fixture;
    fixture.wizard->show();
    EXPECT_FALSE(fixture.button("calibrationCapture")->isEnabled());
    fixture.button("calibrationRestart")->click();
    fixture.capture(1);
    fixture.capture(2);
    fixture.capture(3);
    EXPECT_TRUE(fixture.detector->onWorker);
    EXPECT_EQ(fixture.controller->session().views().size(), 3U);
    EXPECT_TRUE(fixture.button("calibrationFinish")->isEnabled());
    auto* views = fixture.wizard->findChild<QComboBox*>(QStringLiteral("calibrationViews"));
    ASSERT_EQ(views->count(), 3);
    FramePtr reviewed;
    std::size_t cornerCount = 0;
    QObject::connect(fixture.controller.get(), &CalibrationController::previewChanged,
        fixture.wizard.get(), [&](FramePtr frame, const std::vector<ImageCorner>& corners, bool accepted) {
            if (accepted) { reviewed = std::move(frame); cornerCount = corners.size(); }
        });
    fixture.controller->reviewView(0);
    ASSERT_TRUE(reviewed);
    EXPECT_EQ(reviewed->sequence, 1U);
    EXPECT_EQ(cornerCount, 54U);
    const auto rendered = fixture.wizard->findChild<QWidget*>(QStringLiteral("calibrationPreview"))->grab().toImage();
    int greenPixels = 0;
    for (int y = 0; y < rendered.height(); ++y) {
        for (int x = 0; x < rendered.width(); ++x) {
            const auto color = rendered.pixelColor(x, y);
            if (color.green() > 150 && color.red() < 80 && color.blue() < 80) ++greenPixels;
        }
    }
    EXPECT_GT(greenPixels, 50); // Accepted corners reach the actual rendered preview.
    fixture.button("calibrationFinish")->click();
    ASSERT_TRUE(processUntil([&] { return !fixture.controller->busy(); }));
    ASSERT_TRUE(fixture.controller->result());
    EXPECT_TRUE(fixture.solver->onWorker);
    EXPECT_TRUE(fixture.wizard->findChild<QLabel*>(QStringLiteral("calibrationQuality"))->text()
        .contains(QStringLiteral("Aggregate RMS: 0.5000 px")));
    EXPECT_TRUE(views->itemText(0).contains(QStringLiteral("RMS 0.5000 px")));
    QApplication::processEvents(); // Apply the changed multi-line quality label's layout.
    const auto* quality = fixture.wizard->findChild<QLabel*>(QStringLiteral("calibrationQuality"));
    EXPECT_GE(quality->height(), quality->heightForWidth(quality->width()));
    const auto screenshot = qEnvironmentVariable("MICRO_ROBOT_CALIBRATION_SCREENSHOT");
    if (!screenshot.isEmpty()) EXPECT_TRUE(fixture.wizard->grab().save(screenshot));
    EXPECT_EQ(fixture.controller->session().state(), CalibrationCaptureState::Complete);
    fixture.wizard->close();
    EXPECT_EQ(fixture.controller->session().views().size(), 3U);
}
TEST(CalibrationWizardTest, ShowsWarningAndClearsComputedModelOnInputChanges)
{
    IntrinsicCalibrationConfig config;
    config.warningRmsPixels = 1.25;
    CaptureFixture fixture{config};
    fixture.solver->rms = 1.5;
    fixture.controller->restart();
    fixture.capture(1); fixture.capture(2); fixture.capture(3);
    fixture.controller->finish();
    ASSERT_TRUE(processUntil([&] { return !fixture.controller->busy(); }));
    ASSERT_TRUE(fixture.controller->result());
    const auto text = fixture.wizard->findChild<QLabel*>(QStringLiteral("calibrationQuality"))->text();
    EXPECT_TRUE(text.contains(QStringLiteral("threshold: 1.25 px")));
    EXPECT_TRUE(text.contains(QStringLiteral("WARNING")));
    EXPECT_TRUE(fixture.wizard->findChild<QComboBox*>(QStringLiteral("calibrationViews"))->itemText(0)
        .contains(QStringLiteral("WARNING")));
    fixture.controller->restart();
    EXPECT_FALSE(fixture.controller->result());
    fixture.capture(4); fixture.capture(5); fixture.capture(6);
    fixture.controller->finish();
    ASSERT_TRUE(processUntil([&] { return !fixture.controller->busy(); }));
    ASSERT_TRUE(fixture.controller->result());
    fixture.frame(7, 5000); // Unsupported new resolution must also invalidate the model.
    EXPECT_FALSE(fixture.controller->result());
    EXPECT_FALSE(fixture.wizard->findChild<QLabel*>(QStringLiteral("calibrationQuality"))->text()
        .contains(QStringLiteral("Aggregate RMS")));
}
TEST(CalibrationWizardTest, RejectsSolverFailuresInvalidModelsAndMismatchedMetadata)
{
    for (int mode = 0; mode < 4; ++mode) {
        CaptureFixture fixture;
        fixture.solver->fails = mode == 0;
        fixture.solver->throws = mode == 1;
        fixture.solver->invalid = mode == 2;
        fixture.solver->wrongCamera = mode == 3;
        fixture.controller->restart();
        fixture.capture(1); fixture.capture(2); fixture.capture(3);
        fixture.controller->finish();
        ASSERT_TRUE(processUntil([&] { return !fixture.controller->busy(); }));
        EXPECT_FALSE(fixture.controller->result());
        EXPECT_TRUE(fixture.controller->message().contains(QStringLiteral("rejected")));
        EXPECT_TRUE(fixture.button("calibrationFinish")->isEnabled()); // Retry or restart.
    }
}
TEST(CalibrationWizardTest, SolvingIsBoundedAndInvalidatedWithoutBlockingTheGui)
{
    for (int mode = 0; mode < 5; ++mode) {
        CaptureFixture fixture;
        fixture.controller->restart();
        fixture.capture(1); fixture.capture(2); fixture.capture(3);
        std::promise<void> release;
        fixture.solver->gate = release.get_future().share();
        fixture.controller->finish();
        fixture.controller->finish(); // No second job.
        if (mode == 0) fixture.controller->cancel();
        if (mode == 1) fixture.controller->restart();
        if (mode == 2) fixture.controller->setInput(QStringLiteral("camera-b"), true);
        if (mode == 3) fixture.controller->setInput(QStringLiteral("camera-a"), false);
        if (mode == 4) fixture.controller->shutdown();
        EXPECT_TRUE(fixture.controller->busy());
        EXPECT_FALSE(fixture.controller->result());
        release.set_value();
        ASSERT_TRUE(processUntil([&] { return !fixture.controller->busy(); }));
        EXPECT_EQ(fixture.solver->calls, 1);
        EXPECT_FALSE(fixture.controller->result());
    }
}
TEST(CalibrationWizardTest, RemovalRecaptureRestartAndCancelUpdateProgress)
{
    CaptureFixture fixture;
    fixture.button("calibrationRestart")->click();
    fixture.capture(1);
    fixture.button("calibrationRemove")->click();
    EXPECT_TRUE(fixture.controller->session().views().empty());
    fixture.capture(2);
    EXPECT_EQ(fixture.controller->session().views().size(), 1U);
    fixture.button("calibrationRestart")->click();
    EXPECT_TRUE(fixture.controller->session().views().empty());
    fixture.capture(3);
    fixture.wizard->show();
    fixture.wizard->close();
    EXPECT_EQ(fixture.controller->session().state(), CalibrationCaptureState::Cancelled);
    EXPECT_TRUE(fixture.controller->session().views().empty());
}
TEST(CalibrationWizardTest, SourceStopChangeAndResolutionChangeClearSession)
{
    CaptureFixture fixture;
    fixture.controller->restart();
    fixture.capture(1);
    fixture.controller->setInput(QStringLiteral("camera-a"), false);
    EXPECT_TRUE(fixture.controller->session().views().empty());
    EXPECT_FALSE(fixture.controller->canCapture());
    fixture.controller->setInput(QStringLiteral("camera-b"), true);
    fixture.controller->restart();
    fixture.capture(2);
    EXPECT_EQ(fixture.controller->session().sourceId(), "camera-b");
    fixture.frame(3, 1200);
    EXPECT_TRUE(fixture.controller->session().views().empty());
    EXPECT_EQ(fixture.controller->session().state(), CalibrationCaptureState::Cancelled);
}
TEST(CalibrationWizardTest, PendingWorkIsBoundedAndCancelledResultIsDiscarded)
{
    CaptureFixture fixture;
    std::promise<void> release;
    fixture.detector->gate = release.get_future().share();
    fixture.controller->restart();
    fixture.frame(1);
    fixture.controller->capture();
    EXPECT_TRUE(fixture.controller->busy());
    fixture.controller->capture(); // Must not queue another job.
    fixture.frame(2); // Latest camera frame can advance while worker is busy.
    fixture.controller->cancel();
    release.set_value();
    ASSERT_TRUE(processUntil([&] { return !fixture.controller->busy(); }));
    EXPECT_EQ(fixture.detector->calls, 1);
    EXPECT_TRUE(fixture.controller->session().views().empty());
    EXPECT_EQ(fixture.controller->session().state(), CalibrationCaptureState::Cancelled);
}
TEST(CalibrationWizardTest, ShutdownDoesNotWaitForInFlightDetection)
{
    CaptureFixture fixture;
    std::promise<void> release;
    fixture.detector->gate = release.get_future().share();
    fixture.controller->restart();
    fixture.frame(1);
    fixture.controller->capture();
    const bool began = processUntil([&] { return fixture.detector->calls.load() == 1; });
    // Release even on failure so the fixture can always join its worker.
    if (!began) { release.set_value(); FAIL() << "Worker did not start"; }
    fixture.controller->shutdown();
    EXPECT_FALSE(fixture.controller->isStopped());
    EXPECT_FALSE(fixture.controller->canCapture());
    release.set_value();
    ASSERT_TRUE(processUntil([&] { return fixture.controller->isStopped(); }));
    EXPECT_TRUE(fixture.controller->session().views().empty());
}
TEST(CalibrationWizardTest, ClosingMainWindowClosesItsCalibrationDialog)
{
    CaptureFixture fixture;
    SimulationTransport transport;
    WorldModel world;
    DesktopController desktop{transport, world, RobotId{1}, fixture.logger.channel("controller")};
    MainWindow window{desktop};
    window.bindCalibration(*fixture.controller);
    window.show();
    auto* open = window.findChild<QPushButton*>(QStringLiteral("openCalibration"));
    ASSERT_NE(open, nullptr);
    open->click();
    auto* dialog = window.findChild<CalibrationWizard*>();
    ASSERT_NE(dialog, nullptr);
    EXPECT_TRUE(dialog->isVisible());
    window.close();
    EXPECT_FALSE(dialog->isVisible());
    EXPECT_EQ(fixture.controller->session().state(), CalibrationCaptureState::Cancelled);
}
TEST(CalibrationWizardTest, StaleAndOutOfOrderFramesCannotBeCaptured)
{
    CaptureFixture fixture;
    fixture.controller->restart();
    auto stale = std::make_shared<Frame>(*captureFrame());
    stale->capturedAt = MonotonicClock::now() - 3s;
    fixture.controller->receiveFrame(stale);
    fixture.controller->capture();
    EXPECT_FALSE(fixture.controller->busy());
    EXPECT_TRUE(fixture.controller->message().contains(QStringLiteral("stale")));
    fixture.frame(2);
    fixture.controller->receiveFrame(stale);
    fixture.controller->capture();
    ASSERT_TRUE(processUntil([&] { return !fixture.controller->busy(); }));
    ASSERT_EQ(fixture.controller->session().views().size(), 1U);
    EXPECT_EQ(fixture.controller->session().views()[0].frame->sequence, 2U);
}
TEST(CalibrationWizardTest, DesktopRoutesActualFramesAndStopInvalidatesCalibration)
{
    CaptureFixture fixture;
    SimulationTransport transport;
    WorldModel world;
    DesktopController desktop{transport, world, RobotId{1}, fixture.logger.channel("controller")};
    QObject::connect(&desktop, &DesktopController::frameAvailable,
                     fixture.controller.get(), &CalibrationController::receiveFrame);
    QObject::connect(&desktop, &DesktopController::cameraInputChanged,
                     fixture.controller.get(), &CalibrationController::setInput);
    auto source = std::make_unique<ManualCamera>();
    auto* camera = source.get();
    desktop.setCameraSource(std::move(source));
    desktop.initialize(MonotonicClock::now());
    camera->publishState(CameraState::Streaming, MonotonicClock::now());
    fixture.controller->restart();
    auto frame = std::make_shared<Frame>(*captureFrame());
    frame->capturedAt = MonotonicClock::now();
    camera->publishFrame(frame);
    EXPECT_TRUE(fixture.controller->canCapture());
    fixture.controller->capture();
    ASSERT_TRUE(processUntil([&] { return !fixture.controller->busy(); }));
    EXPECT_EQ(fixture.controller->session().views().size(), 1U);
    desktop.stopCamera();
    EXPECT_TRUE(fixture.controller->session().views().empty());
    camera->publishState(CameraState::Stopped, MonotonicClock::now());
    desktop.shutdown();
}
} // namespace
