#include "app/CalibrationController.hpp"
#include "calibration/QtCalibrationProfileStore.hpp"
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
#include <QCheckBox>
#include <QTemporaryDir>
#include <QElapsedTimer>
#include <QLabel>
#include <QImage>
#include <QPixmap>
#include <QPushButton>
#include <QThread>
#include <QTimer>
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
        if (missing) return {};
        return captureDetection(250 + (frame.sequence % 3) * 250, 400, 40,
            distinctFourth && frame.sequence == 4 ? 0.4 : 0.0);
    }
    bool distinctFourth{};
    bool missing{};
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
class DeterministicUndistorter final : public ICalibrationUndistorter {
public:
    UndistortedPreviewOutcome undistort(const Frame& input, const IntrinsicCalibration&) override
    {
        onWorker = QThread::currentThread() != QApplication::instance()->thread();
        ++calls;
        if (gate.valid()) gate.wait();
        if (fails) throw std::runtime_error("Synthetic remap failure");
        auto result = std::make_shared<Frame>(input);
        auto pixels = std::make_shared<std::vector<std::byte>>(*input.pixels);
        (*pixels)[0] = std::byte{127};
        result->pixels = pixels;
        return {result, {}};
    }
    std::shared_future<void> gate;
    std::atomic<int> calls{};
    std::atomic<bool> onWorker{};
    bool fails{};
};
class ControlledProfileStore final : public ICalibrationProfileStore {
public:
    ProfileLoadOutcome load(std::string_view id, std::uint32_t width, std::uint32_t height) override
    {
        ++calls;
        onWorker = QThread::currentThread() != QApplication::instance()->thread();
        if (gate.valid()) gate.wait();
        if (throws) throw std::runtime_error("Synthetic read error");
        auto c = validIntrinsicCalibration();
        c.sourceId = wrongCamera ? "wrong-camera" : std::string{id};
        c.width = width; c.height = height;
        return {ProfileLoadState::Loaded, c, {}};
    }
    std::string save(const IntrinsicCalibration&) override { return "Synthetic write error"; }
    std::shared_future<void> gate;
    std::atomic<int> calls{};
    std::atomic<bool> onWorker{};
    bool wrongCamera{}, throws{};
};
struct CaptureFixture {
    Logger logger;
    DeterministicDetector* detector{};
    DeterministicSolver* solver{};
    DeterministicUndistorter* undistorter{};
    std::unique_ptr<CalibrationController> controller;
    std::unique_ptr<CalibrationWizard> wizard;
    explicit CaptureFixture(IntrinsicCalibrationConfig intrinsicConfig = {}, QString profileDirectory = {},
                            std::unique_ptr<ICalibrationProfileStore> storeOverride = {})
    {
        auto implementation = std::make_unique<DeterministicDetector>();
        detector = implementation.get();
        auto solverImplementation = std::make_unique<DeterministicSolver>();
        solver = solverImplementation.get();
        controller = std::make_unique<CalibrationController>(std::move(implementation),
            std::move(solverImplementation),
            logger.channel("calibration"), uiConfig(), intrinsicConfig);
        if (!profileDirectory.isEmpty() || storeOverride) {
            auto remapper = std::make_unique<DeterministicUndistorter>();
            undistorter = remapper.get();
            if (!storeOverride) storeOverride = std::make_unique<QtCalibrationProfileStore>(profileDirectory);
            controller->setProfileServices(std::move(storeOverride), std::move(remapper));
        }
        wizard = std::make_unique<CalibrationWizard>(*controller);
        controller->setInput(QStringLiteral("camera-a"), true, true);
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
        controller->showLive();
        ASSERT_TRUE(processUntil([&] { return !controller->profileBusy(); }));
        button("calibrationCapture")->click();
        ASSERT_TRUE(processUntil([&] { return !controller->busy(); }));
    }
};
TEST(CalibrationProfileUiTest, UnidentifiedCameraCannotReloadOrSaveButCanCalibrateInSession)
{
    auto store = std::make_unique<ControlledProfileStore>();
    auto* observed = store.get();
    CaptureFixture f{{}, {}, std::move(store)};
    f.controller->setInput("/dev/video0", true, false);
    f.frame(1); QApplication::processEvents();
    f.controller->reloadProfile();
    EXPECT_EQ(observed->calls, 0);
    EXPECT_FALSE(f.controller->canReloadProfile());
    f.controller->restart();
    f.capture(1); f.capture(2); f.capture(3);
    f.controller->finish();
    ASSERT_TRUE(processUntil([&] { return !f.controller->busy(); }));
    ASSERT_TRUE(f.controller->result());
    EXPECT_FALSE(f.controller->canSaveProfile());
    EXPECT_TRUE(f.controller->canUndistort());
    f.controller->setInput("v4l2-usb:camera-a", true, true);
    f.frame(4);
    ASSERT_TRUE(processUntil([&] { return !f.controller->profileBusy(); }));
    ASSERT_TRUE(f.controller->profile());
    EXPECT_EQ(f.controller->profile()->sourceId, "v4l2-usb:camera-a");
    f.controller->setInput("v4l2-usb:camera-b", true, true);
    EXPECT_FALSE(f.controller->profile());
    f.frame(5);
    ASSERT_TRUE(processUntil([&] { return !f.controller->profileBusy(); }));
    ASSERT_TRUE(f.controller->profile());
    EXPECT_EQ(f.controller->profile()->sourceId, "v4l2-usb:camera-b");
}
TEST(CalibrationWizardTest, RemovingSolvedViewKeepsOtherCapturesAndRequiresRecalculation)
{
    QTemporaryDir directory;
    CaptureFixture f{{}, directory.path()};
    f.controller->restart();
    f.detector->distinctFourth = true;
    f.capture(1); f.capture(2); f.capture(3); f.capture(4);
    f.controller->finish();
    ASSERT_TRUE(processUntil([&] { return !f.controller->busy(); }));
    ASSERT_TRUE(f.controller->result());
    ASSERT_TRUE(f.controller->canSaveProfile());
    EXPECT_TRUE(f.button("calibrationRemove")->isEnabled());
    const auto keptFrame = f.controller->session().views()[1].frame;
    f.controller->removeView(0);
    EXPECT_EQ(f.controller->session().state(), CalibrationCaptureState::Collecting);
    ASSERT_EQ(f.controller->session().views().size(), 3U);
    EXPECT_EQ(f.controller->session().views()[0].frame, keptFrame);
    EXPECT_FALSE(f.controller->result());
    EXPECT_FALSE(f.controller->profile());
    EXPECT_FALSE(f.controller->canSaveProfile());
    EXPECT_FALSE(f.controller->canUndistort());
    f.controller->finish();
    ASSERT_TRUE(processUntil([&] { return !f.controller->busy(); }));
    ASSERT_TRUE(f.controller->result());
    EXPECT_EQ(f.controller->result()->perViewErrors.size(), 3U);
    EXPECT_EQ(f.solver->calls, 2);
}
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
        if (mode == 2) fixture.controller->setInput(QStringLiteral("camera-b"), true, true);
        if (mode == 3) fixture.controller->setInput(QStringLiteral("camera-a"), false, true);
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
    fixture.controller->setInput(QStringLiteral("camera-a"), false, true);
    EXPECT_TRUE(fixture.controller->session().views().empty());
    EXPECT_FALSE(fixture.controller->canCapture());
    fixture.controller->setInput(QStringLiteral("camera-b"), true, true);
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
    RobotManager robots{transport, world};
    DesktopController desktop{robots, world, RobotId{1}, fixture.logger.channel("controller")};
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
    RobotManager robots{transport, world};
    DesktopController desktop{robots, world, RobotId{1}, fixture.logger.channel("controller")};
    QObject::connect(&desktop, &DesktopController::frameAvailable,
                     fixture.controller.get(), &CalibrationController::receiveFrame);
    QObject::connect(&desktop, &DesktopController::calibrationInputChanged,
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

TEST(CalibrationProfileUiTest, SavesCalculatedProfileAndReloadsAfterControllerRestart)
{
    QTemporaryDir directory;
    ASSERT_TRUE(directory.isValid());
    {
        CaptureFixture fixture{{}, directory.path()};
        fixture.controller->restart();
        fixture.capture(1); fixture.capture(2); fixture.capture(3);
        fixture.controller->finish();
        ASSERT_TRUE(processUntil([&] { return !fixture.controller->busy(); }));
        ASSERT_TRUE(fixture.controller->result());
        ASSERT_TRUE(fixture.button("calibrationSaveProfile")->isEnabled());
        fixture.button("calibrationSaveProfile")->click();
        ASSERT_TRUE(processUntil([&] { return !fixture.controller->profileBusy(); }));
        EXPECT_TRUE(fixture.controller->profileMessage().contains(QStringLiteral("saved")));
    }
    CaptureFixture restarted{{}, directory.path()};
    auto* toggle = restarted.wizard->findChild<QCheckBox*>(QStringLiteral("calibrationUndistorted"));
    EXPECT_FALSE(toggle->isEnabled()); // Delivered size is not known until first frame.
    restarted.frame(1);
    ASSERT_TRUE(processUntil([&] { return !restarted.controller->profileBusy(); }));
    ASSERT_TRUE(restarted.controller->profile());
    EXPECT_FALSE(restarted.controller->result()); // Loading does not fabricate a capture session.
    EXPECT_EQ(restarted.controller->profile()->cameraMatrix, validIntrinsicCalibration().cameraMatrix);
    EXPECT_TRUE(toggle->isEnabled());
    EXPECT_FALSE(toggle->isChecked());
    EXPECT_TRUE(restarted.controller->profileMessage().contains(QStringLiteral("loaded")));
    restarted.controller->setInput(QStringLiteral("camera-b"), true, true);
    EXPECT_FALSE(restarted.controller->profile());
    restarted.frame(1);
    ASSERT_TRUE(processUntil([&] { return !restarted.controller->profileBusy(); }));
    EXPECT_FALSE(restarted.controller->profile());
    EXPECT_FALSE(toggle->isEnabled());
    restarted.controller->setInput(QStringLiteral("camera-a"), true, true);
    restarted.frame(1, 1200);
    ASSERT_TRUE(processUntil([&] { return !restarted.controller->profileBusy(); }));
    EXPECT_FALSE(restarted.controller->profile()); // No automatic scaling/reuse.
}
TEST(CalibrationProfileUiTest, ToggleRemapsOnlyPreviewAndPreservesRawCapturePixels)
{
    QTemporaryDir directory;
    QtCalibrationProfileStore store{directory.path()};
    ASSERT_TRUE(store.save(validIntrinsicCalibration()).empty());
    CaptureFixture fixture{{}, directory.path()};
    fixture.controller->restart();
    fixture.frame(1);
    ASSERT_TRUE(processUntil([&] { return !fixture.controller->profileBusy(); }));
    FramePtr presented;
    std::size_t corners = 99;
    QObject observer; // Disconnect before captured locals and fixture shutdown.
    QObject::connect(fixture.controller.get(), &CalibrationController::previewChanged,
        &observer, [&](FramePtr frame, const auto& overlay, bool) { presented = std::move(frame); corners = overlay.size(); });
    auto* toggle = fixture.wizard->findChild<QCheckBox*>(QStringLiteral("calibrationUndistorted"));
    toggle->click();
    ASSERT_TRUE(processUntil([&] { return presented && (*presented->pixels)[0] == std::byte{127}; }));
    EXPECT_TRUE(fixture.undistorter->onWorker);
    EXPECT_EQ(corners, 0U); // Raw corners cannot be overlaid in corrected coordinates.
    fixture.controller->capture();
    ASSERT_TRUE(processUntil([&] { return !fixture.controller->busy(); }));
    ASSERT_EQ(fixture.controller->session().views().size(), 1U);
    EXPECT_EQ((*fixture.controller->session().views().front().frame->pixels)[0], std::byte{0});
    toggle->click();
    ASSERT_TRUE(presented);
    EXPECT_EQ((*presented->pixels)[0], std::byte{0});
    EXPECT_EQ(corners, 54U);
}
TEST(CalibrationProfileUiTest, RemapUsesOneActiveAndNewestPendingFrameWithoutBlockingGui)
{
    QTemporaryDir directory;
    QtCalibrationProfileStore store{directory.path()};
    ASSERT_TRUE(store.save(validIntrinsicCalibration()).empty());
    CaptureFixture fixture{{}, directory.path()};
    fixture.frame(1);
    ASSERT_TRUE(processUntil([&] { return !fixture.controller->profileBusy(); }));
    std::promise<void> release;
    fixture.undistorter->gate = release.get_future().share();
    fixture.controller->setUndistortedPreview(true);
    const bool began = processUntil([&] { return fixture.undistorter->calls.load() == 1; });
    if (!began) { release.set_value(); FAIL() << "Remap did not start"; }
    for (std::uint64_t seq = 2; seq <= 20; ++seq) fixture.frame(seq);
    QApplication::processEvents();
    EXPECT_EQ(fixture.undistorter->calls, 1); // GUI events continue with worker blocked.
    std::uint64_t presentedSequence = 0;
    QObject observer;
    QObject::connect(fixture.controller.get(), &CalibrationController::previewChanged, &observer,
        [&](FramePtr frame, const auto&, bool) { if (frame) presentedSequence = frame->sequence; });
    release.set_value();
    ASSERT_TRUE(processUntil([&] { return presentedSequence == 20; }));
    EXPECT_EQ(fixture.undistorter->calls, 2);
}
TEST(CalibrationProfileUiTest, LateRemapsAreDiscardedAfterModeSourceResolutionStopOrShutdown)
{
    for (int mode = 0; mode < 5; ++mode) {
        QTemporaryDir directory;
        QtCalibrationProfileStore store{directory.path()};
        ASSERT_TRUE(store.save(validIntrinsicCalibration()).empty());
        CaptureFixture fixture{{}, directory.path()};
        fixture.frame(1);
        ASSERT_TRUE(processUntil([&] { return !fixture.controller->profileBusy(); }));
        std::promise<void> release;
        fixture.undistorter->gate = release.get_future().share();
        fixture.controller->setUndistortedPreview(true);
        const bool began = processUntil([&] { return fixture.undistorter->calls.load() == 1; });
        if (!began) { release.set_value(); FAIL() << "Remap did not start"; }
        int corrected = 0;
        QObject observer;
        QObject::connect(fixture.controller.get(), &CalibrationController::previewChanged, &observer,
            [&](FramePtr frame, const auto&, bool) { if (frame && (*frame->pixels)[0] == std::byte{127}) ++corrected; });
        if (mode == 0) fixture.controller->setUndistortedPreview(false);
        if (mode == 1) fixture.controller->setInput(QStringLiteral("camera-b"), true, true);
        if (mode == 2) fixture.frame(2, 1200);
        if (mode == 3) fixture.controller->setInput(QStringLiteral("camera-a"), false, true);
        if (mode == 4) fixture.controller->shutdown();
        EXPECT_FALSE(fixture.controller->undistortedPreview());
        release.set_value();
        // A queued observer runs after the worker remap and its GUI completion.
        fixture.controller->shutdown();
        ASSERT_TRUE(processUntil([&] { return fixture.controller->isStopped(); }));
        QApplication::processEvents();
        EXPECT_EQ(corrected, 0);
    }
}
TEST(CalibrationProfileUiTest, RemapFailureReturnsToRawPreviewAndShowsError)
{
    QTemporaryDir directory;
    QtCalibrationProfileStore store{directory.path()};
    ASSERT_TRUE(store.save(validIntrinsicCalibration()).empty());
    CaptureFixture fixture{{}, directory.path()};
    fixture.frame(1);
    ASSERT_TRUE(processUntil([&] { return !fixture.controller->profileBusy(); }));
    fixture.undistorter->fails = true;
    fixture.controller->setUndistortedPreview(true);
    ASSERT_TRUE(processUntil([&] { return !fixture.controller->undistortedPreview(); }));
    EXPECT_TRUE(fixture.controller->profileMessage().contains(QStringLiteral("failed")));
    EXPECT_FALSE(fixture.wizard->findChild<QCheckBox*>(QStringLiteral("calibrationUndistorted"))->isChecked());
    EXPECT_TRUE(fixture.controller->profile()); // A preview failure does not damage the saved model.
}

TEST(CalibrationProfileUiTest, SlowProfileLookupFollowsNewestCameraAndResolution)
{
    auto store = std::make_unique<ControlledProfileStore>();
    auto* backend = store.get();
    std::promise<void> release;
    backend->gate = release.get_future().share();
    CaptureFixture fixture{{}, {}, std::move(store)};
    fixture.frame(1);
    const bool began = processUntil([&] { return backend->calls.load() == 1; });
    if (!began) { release.set_value(); FAIL() << "Profile lookup did not start"; }
    fixture.controller->setInput(QStringLiteral("camera-b"), true, true);
    fixture.frame(1, 1200);
    EXPECT_FALSE(fixture.controller->profile());
    EXPECT_EQ(backend->calls, 1); // At most one lookup plus a coalesced pending request.
    release.set_value();
    ASSERT_TRUE(processUntil([&] { return !fixture.controller->profileBusy(); }));
    ASSERT_TRUE(fixture.controller->profile());
    EXPECT_EQ(fixture.controller->profile()->sourceId, "camera-b");
    EXPECT_EQ(fixture.controller->profile()->width, 1200U);
    EXPECT_EQ(backend->calls, 2);
    EXPECT_TRUE(backend->onWorker);
}
TEST(CalibrationProfileUiTest, CaptureRestartDoesNotInvalidateSameCameraProfileLookup)
{
    auto store = std::make_unique<ControlledProfileStore>();
    auto* backend = store.get();
    std::promise<void> release;
    backend->gate = release.get_future().share();
    CaptureFixture fixture{{}, {}, std::move(store)};
    fixture.frame(1);
    const bool began = processUntil([&] { return backend->calls.load() == 1; });
    if (!began) { release.set_value(); FAIL() << "Profile lookup did not start"; }
    fixture.controller->restart();
    release.set_value();
    ASSERT_TRUE(processUntil([&] { return !fixture.controller->profileBusy(); }));
    ASSERT_TRUE(fixture.controller->profile());
    EXPECT_EQ(fixture.controller->profile()->sourceId, "camera-a");
}
TEST(CalibrationProfileUiTest, RejectsIncompatibleOrThrowingStoresAndReportsSaveFailure)
{
    for (bool throws : {false, true}) {
        auto store = std::make_unique<ControlledProfileStore>();
        store->throws = throws;
        store->wrongCamera = !throws;
        CaptureFixture fixture{{}, {}, std::move(store)};
        fixture.frame(1);
        ASSERT_TRUE(processUntil([&] { return !fixture.controller->profileBusy(); }));
        EXPECT_FALSE(fixture.controller->profile());
        EXPECT_FALSE(fixture.controller->canUndistort());
        EXPECT_TRUE(fixture.controller->profileMessage().contains(QStringLiteral("rejected")));
    }
    CaptureFixture fixture{{}, {}, std::make_unique<ControlledProfileStore>()};
    fixture.frame(1);
    ASSERT_TRUE(processUntil([&] { return !fixture.controller->profileBusy(); }));
    ASSERT_TRUE(fixture.controller->profile());
    fixture.controller->saveProfile();
    ASSERT_TRUE(processUntil([&] { return !fixture.controller->profileBusy(); }));
    EXPECT_TRUE(fixture.controller->profileMessage().contains(QStringLiteral("save failed")));
    EXPECT_TRUE(fixture.controller->profile());
}
} // namespace

namespace {
TEST(CalibrationLiveTest, HiddenWizardDoesNotDetectAndCaptureReusesDisplayedSample)
{
    CaptureFixture f;
    f.controller->restart();
    f.frame(1);
    QApplication::processEvents();
    EXPECT_EQ(f.detector->calls, 0);
    FramePtr displayed;
    std::vector<ImageCorner> points;
    QObject observer;
    QObject::connect(f.controller.get(), &CalibrationController::previewChanged, &observer,
        [&](FramePtr frame, const std::vector<ImageCorner>& corners, bool) {
            displayed = frame; points = corners;
        });
    f.wizard->show();
    ASSERT_TRUE(processUntil([&] { return !points.empty(); }));
    ASSERT_TRUE(displayed);
    const auto selected = displayed;
    EXPECT_EQ(selected->sequence, 1U);
    EXPECT_TRUE(f.controller->session().views().empty());
    f.controller->capture();
    ASSERT_EQ(f.controller->session().views().size(), 1U);
    EXPECT_EQ(f.controller->session().views().front().frame, selected);
    EXPECT_EQ(f.detector->calls, 1);
    f.controller->showLive();
    f.frame(2);
    EXPECT_EQ(displayed->sequence, 2U); // Video advances independently of 5 Hz detection.
    EXPECT_TRUE(points.empty());
    f.wizard->hide();
    f.frame(3);
    QApplication::processEvents();
    EXPECT_LE(f.detector->calls, 2); // At most the job started by Show Live; hidden frames add none.
}
TEST(CalibrationLiveTest, SlowLiveWorkIsBoundedAndCancelDiscardsItWithoutBlockingGui)
{
    CaptureFixture f;
    std::promise<void> release;
    f.detector->gate = release.get_future().share();
    f.controller->restart();
    f.frame(1);
    f.wizard->show();
    // Always release even if a test assertion fails, so teardown cannot hang.
    EXPECT_TRUE(processUntil([&] { return f.detector->calls.load() == 1; }));
    int overlays = 0;
    QObject observer;
    QObject::connect(f.controller.get(), &CalibrationController::previewChanged, &observer,
        [&](FramePtr, const std::vector<ImageCorner>& corners, bool) { if (!corners.empty()) ++overlays; });
    for (int i = 2; i < 102; ++i) f.frame(i);
    bool guiRan = false;
    QTimer::singleShot(0, [&] { guiRan = true; });
    EXPECT_TRUE(processUntil([&] { return guiRan; }));
    EXPECT_EQ(f.detector->calls, 1);
    f.controller->cancel();
    f.wizard->hide();
    release.set_value();
    f.controller->shutdown();
    EXPECT_TRUE(processUntil([&] { return f.controller->isStopped(); }));
    EXPECT_EQ(overlays, 0);
    EXPECT_TRUE(f.controller->session().views().empty());
}
TEST(CalibrationLiveTest, CameraChangeRejectsInFlightLiveResult)
{
    CaptureFixture f;
    std::promise<void> release;
    f.detector->gate = release.get_future().share();
    f.frame(1);
    f.wizard->show();
    EXPECT_TRUE(processUntil([&] { return f.detector->calls.load() == 1; }));
    int overlays = 0;
    QObject observer;
    QObject::connect(f.controller.get(), &CalibrationController::previewChanged, &observer,
        [&](FramePtr, const std::vector<ImageCorner>& corners, bool) { if (!corners.empty()) ++overlays; });
    f.controller->setInput(QStringLiteral("camera-b"), true);
    release.set_value();
    f.controller->shutdown();
    EXPECT_TRUE(processUntil([&] { return f.controller->isStopped(); }));
    EXPECT_EQ(overlays, 0);
}
TEST(CalibrationLiveTest, CaptureDuringLiveJobRetainsSelectedFrameAndNoLiveQueue)
{
    CaptureFixture f;
    std::promise<void> release;
    f.detector->gate = release.get_future().share();
    f.controller->restart();
    f.frame(1);
    f.wizard->show();
    EXPECT_TRUE(processUntil([&] { return f.detector->calls.load() == 1; }));
    // Before the first result, raw preview still follows acquisition.
    f.frame(2);
    f.controller->capture();
    EXPECT_TRUE(f.controller->busy());
    for (int i = 3; i < 50; ++i) f.frame(i);
    release.set_value();
    EXPECT_TRUE(processUntil([&] { return !f.controller->busy(); }));
    ASSERT_EQ(f.controller->session().views().size(), 1U);
    EXPECT_EQ(f.controller->session().views().front().frame->sequence, 2U);
    EXPECT_EQ(f.detector->calls, 2); // One running live + one explicit capture only.
}
TEST(CalibrationLiveTest, ResolutionRestartAndDialogDestructionDiscardInFlightResult)
{
    CaptureFixture f;
    std::promise<void> release;
    f.detector->gate = release.get_future().share();
    f.frame(1);
    f.wizard->show();
    EXPECT_TRUE(processUntil([&] { return f.detector->calls.load() == 1; }));
    int overlays = 0;
    QObject observer;
    QObject::connect(f.controller.get(), &CalibrationController::previewChanged, &observer,
        [&](FramePtr, const std::vector<ImageCorner>& corners, bool) { if (!corners.empty()) ++overlays; });
    f.frame(2, 900);
    f.controller->restart();
    f.wizard.reset();
    release.set_value();
    f.controller->shutdown();
    EXPECT_TRUE(processUntil([&] { return f.controller->isStopped(); }));
    EXPECT_EQ(overlays, 0);
}
} // namespace

namespace {
TEST(CalibrationHeldOverlayTest, VideoAdvancesWithFixedPointsUntilNextDetectionCompletes)
{
    CaptureFixture f;
    f.controller->restart();
    FramePtr displayed;
    std::vector<ImageCorner> points;
    bool held = false;
    QObject observer;
    QObject::connect(f.controller.get(), &CalibrationController::previewChanged, &observer,
        [&](FramePtr frame, const std::vector<ImageCorner>& corners, bool, bool cached) {
            displayed = frame; points = corners; held = cached;
        });
    f.frame(1); f.wizard->show();
    ASSERT_TRUE(processUntil([&] { return !points.empty(); }));
    const auto oldU = points.front().u;
    std::promise<void> release;
    f.detector->gate = release.get_future().share();
    f.frame(2);
    EXPECT_TRUE(displayed);
    if (displayed) EXPECT_EQ(displayed->sequence, 2U);
    EXPECT_FALSE(points.empty());
    if (!points.empty()) EXPECT_EQ(points.front().u, oldU);
    EXPECT_TRUE(held);
    bool advanced = false;
    QTimer::singleShot(210, &observer, [&] { f.frame(3); advanced = true; });
    EXPECT_TRUE(processUntil([&] { return advanced && f.detector->calls.load() >= 2; }));
    EXPECT_EQ(displayed->sequence, 3U);
    EXPECT_FALSE(points.empty());
    if (!points.empty()) EXPECT_EQ(points.front().u, oldU); // In-flight detection holds the overlay.
    release.set_value();
    EXPECT_TRUE(processUntil([&] { return !points.empty() && points.front().u != oldU; }));
    EXPECT_EQ(displayed->sequence, 3U); // Delayed results update points, never rewind video.
}
TEST(CalibrationHeldOverlayTest, CaptureRedetectsDisplayedFrameInsteadOfSavingHeldPoints)
{
    CaptureFixture f;
    f.controller->restart();
    std::vector<ImageCorner> points;
    bool held = false;
    QObject observer;
    QObject::connect(f.controller.get(), &CalibrationController::previewChanged, &observer,
        [&](FramePtr, const std::vector<ImageCorner>& corners, bool, bool cached) { points = corners; held = cached; });
    f.frame(1); f.wizard->show();
    ASSERT_TRUE(processUntil([&] { return !points.empty(); }));
    const auto oldU = points.front().u;
    f.frame(2);
    EXPECT_TRUE(held);
    EXPECT_EQ(points.front().u, oldU);
    f.controller->capture();
    ASSERT_TRUE(processUntil([&] { return !f.controller->busy(); }));
    ASSERT_EQ(f.controller->session().views().size(), 1U);
    const auto& view = f.controller->session().views().front();
    EXPECT_EQ(view.frame->sequence, 2U);
    EXPECT_NE(view.detection.corners.front().u, oldU);
    EXPECT_FALSE(held);
    EXPECT_GE(f.detector->calls, 2);
}
TEST(CalibrationHeldOverlayTest, CameraChangeClearsHeldPointsAndBothWorkersDrainOnce)
{
    CaptureFixture f;
    std::vector<ImageCorner> points;
    int stops = 0;
    QObject observer;
    QObject::connect(f.controller.get(), &CalibrationController::previewChanged, &observer,
        [&](FramePtr, const std::vector<ImageCorner>& corners, bool) { points = corners; });
    QObject::connect(f.controller.get(), &CalibrationController::stopped, &observer, [&] { ++stops; });
    f.frame(1); f.wizard->show();
    ASSERT_TRUE(processUntil([&] { return !points.empty(); }));
    f.frame(2);
    EXPECT_FALSE(points.empty());
    f.controller->setInput(QStringLiteral("new-camera"), true);
    EXPECT_TRUE(points.empty());
    f.wizard.reset();
    f.controller->shutdown();
    EXPECT_TRUE(processUntil([&] { return stops == 1; }));
    QApplication::processEvents();
    EXPECT_EQ(stops, 1);
}
TEST(CalibrationHeldOverlayTest, NextDetectionWithoutCornersClearsTheHeldOverlay)
{
    CaptureFixture f;
    bool hasCorners = false, advanced = false;
    QObject observer;
    QObject::connect(f.controller.get(), &CalibrationController::previewChanged, &observer,
        [&](FramePtr, const std::vector<ImageCorner>& corners, bool) { hasCorners = !corners.empty(); });
    f.frame(1); f.wizard->show();
    ASSERT_TRUE(processUntil([&] { return hasCorners; }));
    f.detector->missing = true;
    QTimer::singleShot(210, &observer, [&] { f.frame(2); advanced = true; });
    EXPECT_TRUE(processUntil([&] { return advanced && !hasCorners; }));
    EXPECT_GE(f.detector->calls, 2);
}
} // namespace
