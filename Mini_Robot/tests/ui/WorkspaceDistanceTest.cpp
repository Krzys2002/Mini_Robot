#include "../calibration/CaptureFixtures.hpp"
#include "../calibration/WorkspaceDiagnosticsFixtures.hpp"
#include "app/WorkspaceCalibrationController.hpp"
#include "app/WorkspaceDiagnosticsController.hpp"
#include "app/WorkspaceDistanceController.hpp"
#include "ui/WorkspaceCalibrationDialog.hpp"
#include "ui/WorkspaceDistanceDialog.hpp"
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QLineEdit>
#include <QMouseEvent>
#include <QPushButton>
#include <QTest>
#include <atomic>
#include <condition_variable>
#include <cstdlib>
#include <gtest/gtest.h>
#include <mutex>
namespace
{
using namespace micro_robot;
using namespace micro_robot::test;
struct Gate {
    std::mutex mutex;
    std::condition_variable cv;
    std::atomic<bool> entered{};
    bool released{};
    void wait()
    {
        entered = true;
        std::unique_lock lock(mutex);
        cv.wait(lock, [&] { return released; });
    }
    void release()
    {
        std::lock_guard lock(mutex);
        released = true;
        cv.notify_all();
    }
};
class Solver final : public IWorkspaceHomographySolver
{
    WorkspaceHomographyOutcome solve(const WorkspaceHomographyInput&) override
    {
        return {{}, "Not used in stored-profile fixture."};
    }
};
class Undistorter final : public ICalibrationUndistorter
{
public:
    std::shared_ptr<Gate> gate;
    bool bad{};
    std::atomic<int> calls{};
    UndistortedPreviewOutcome undistort(const Frame& frame, const IntrinsicCalibration&) override
    {
        ++calls;
        if (gate)
            gate->wait();
        auto copy = std::make_shared<Frame>(frame);
        if (bad)
            ++copy->sequence;
        return {copy, {}};
    }
};
class Store final : public IWorkspaceProfileStore
{
public:
    std::shared_ptr<WorkspaceAcquisitionRecord> record;
    explicit Store(std::shared_ptr<WorkspaceAcquisitionRecord> a) : record(std::move(a)) {}
    WorkspaceProfileLoadOutcome load(std::string_view, const IntrinsicCalibration&) override
    {
        return {ProfileLoadState::Loaded, record->profile, {}, record};
    }
    std::string save(const WorkspaceCalibrationProfile&) override { return "Unused fixture write."; }
};
struct Fixture {
    Logger logger;
    std::shared_ptr<WorkspaceAcquisitionRecord> acquisition = diagnosticsAcquisition(0.001);
    std::shared_ptr<Gate> gate = std::make_shared<Gate>();
    Undistorter* undistorter{};
    Store* store{};
    WorkspaceCalibrationController source{std::make_unique<Solver>(), makeUndistorter(), makeStore(),
                                          logger.channel("distance")};
    std::unique_ptr<Undistorter> makeUndistorter()
    {
        auto p = std::make_unique<Undistorter>();
        undistorter = p.get();
        return p;
    }
    std::unique_ptr<Store> makeStore()
    {
        auto p = std::make_unique<Store>(acquisition);
        store = p.get();
        return p;
    }
    Fixture()
    {
        source.setInput(validIntrinsicCalibration(), true);
        source.configure("table-A", 1, 0.6);
        source.load();
        if (!QTest::qWaitFor([&] { return !source.busy(); }))
            return;
        source.activate();
        (void)QTest::qWaitFor([&] { return !source.diagnostics().busy(); });
    }
    void surveyed()
    {
        source.diagnostics().request(diagnosticsEvidence(*acquisition));
        ASSERT_TRUE(QTest::qWaitFor([&] { return !source.diagnostics().busy(); }));
        ASSERT_TRUE(source.diagnostics().snapshot()->prediction);
    }
    void fresh(std::uint64_t sequence = 1, Duration age = Duration::zero())
    {
        auto frame = std::make_shared<Frame>(*captureFrame(sequence));
        frame->capturedAt = MonotonicClock::now() - age;
        source.receiveFrame(frame);
    }
    ~Fixture()
    {
        gate->release();
        source.shutdown();
        (void)QTest::qWaitFor([&] { return source.isStopped(); });
    }
};
void clickAt(QWidget* widget, QPointF p)
{
    QMouseEvent press(QEvent::MouseButtonPress, p, widget->mapToGlobal(p), Qt::LeftButton, Qt::LeftButton,
                      Qt::NoModifier);
    QApplication::sendEvent(widget, &press);
}
TEST(WorkspaceDistanceUiTest, MeasurementFreezePreservesAcquisitionAndDraftWhileRefreezeClearsPicks)
{
    Fixture f;
    ASSERT_TRUE(f.source.profile());
    f.surveyed();
    f.fresh();
    WorkspaceDistanceController c(f.source);
    const auto active = f.source.activeAcquisition(), draft = f.source.draftAcquisition();
    const auto revision = f.source.referencesRevision();
    const auto count = f.source.references().size();
    c.freeze();
    ASSERT_TRUE(QTest::qWaitFor([&] { return !f.source.busy(); }));
    ASSERT_TRUE(c.frame());
    EXPECT_EQ(c.frame()->sequence, 1u);
    EXPECT_EQ(f.undistorter->calls, 1);
    EXPECT_FALSE(f.source.frozenFrame());
    EXPECT_EQ(f.source.references().size(), count);
    EXPECT_EQ(f.source.referencesRevision(), revision);
    EXPECT_EQ(f.source.activeAcquisition(), active);
    EXPECT_EQ(f.source.draftAcquisition(), draft);
    ASSERT_TRUE(c.pickImage(0, {380, 450}));
    ASSERT_TRUE(c.pickImage(1, {620, 300}));
    ASSERT_TRUE(c.measurement().result);
    EXPECT_NEAR(c.measurement().result->distanceMeters, std::hypot(0.3, 0.15), 1e-12);
    EXPECT_FALSE(c.measurement().result->plusMinusMeters);
    for (std::size_t i = 0; i < 2; ++i)
        c.setSelection(i, {WorkspaceSelectionSpace::NativeUndistortedPixels, 1,
                           "Synthetic fixed maximum pointer disk."});
    ASSERT_TRUE(c.measurement().result->plusMinusMeters);
    const auto distance = c.measurement().result->distanceMeters;
    f.source.configure("draft-only", 2, 1);
    EXPECT_TRUE(c.frame());
    EXPECT_EQ(c.measurement().result->distanceMeters, distance);
    f.fresh(2);
    c.freeze();
    EXPECT_FALSE(c.endpoints()[0]);
    EXPECT_FALSE(c.endpoints()[1]);
    ASSERT_TRUE(QTest::qWaitFor([&] { return !f.source.busy(); }));
    ASSERT_TRUE(c.frame());
    EXPECT_EQ(c.frame()->sequence, 2u);
    EXPECT_EQ(f.source.references().size(),
              0u); // Draft edit cleared its references, measurement did not restore them.
    EXPECT_EQ(f.source.activeAcquisition(), active);
}
TEST(WorkspaceDistanceUiTest, LateFrameCannotRestoreOldOpticsAndGuiRemainsResponsive)
{
    Fixture f;
    f.fresh();
    f.undistorter->gate = f.gate;
    WorkspaceDistanceController c(f.source);
    c.freeze();
    ASSERT_TRUE(QTest::qWaitFor([&] { return f.gate->entered.load(); }));
    bool delivered = false;
    QMetaObject::invokeMethod(&c, [&] { delivered = true; }, Qt::QueuedConnection);
    EXPECT_TRUE(QTest::qWaitFor([&] { return delivered; }));
    auto optical = validIntrinsicCalibration();
    optical.cameraMatrix[0] += 1;
    f.source.setInput(optical, true);
    f.gate->release();
    ASSERT_TRUE(QTest::qWaitFor([&] { return !f.source.busy(); }));
    EXPECT_FALSE(c.frame());
    EXPECT_FALSE(c.geometry());
    EXPECT_FALSE(c.measurement().result);
    EXPECT_FALSE(c.pickImage(0, {380, 450}));
}
TEST(WorkspaceDistanceUiTest, StaleAndMalformedUndistortionAreRejectedAndRecoverable)
{
    Fixture f;
    WorkspaceDistanceController c(f.source);
    f.fresh(1, std::chrono::seconds{3});
    c.freeze();
    EXPECT_FALSE(f.source.busy());
    EXPECT_FALSE(c.frame());
    EXPECT_TRUE(c.message().contains("stale"));
    f.fresh(2);
    f.undistorter->bad = true;
    c.freeze();
    ASSERT_TRUE(QTest::qWaitFor([&] { return !f.source.busy(); }));
    EXPECT_FALSE(c.frame());
    EXPECT_TRUE(c.message().contains("metadata"));
    f.undistorter->bad = false;
    f.fresh(3);
    c.freeze();
    ASSERT_TRUE(QTest::qWaitFor([&] { return !f.source.busy(); }));
    ASSERT_TRUE(c.frame());
    EXPECT_EQ(c.frame()->sequence, 3u);
}
TEST(WorkspaceDistanceUiTest, EvidenceRecomputeWithholdsOldUncertaintyButKeepsWorldEndpoints)
{
    Fixture f;
    f.surveyed();
    WorkspaceDistanceController c(f.source);
    ASSERT_TRUE(c.setWorld(0, {Meters{0.35}, Meters{0.25}}));
    ASSERT_TRUE(c.setWorld(1, {Meters{0.65}, Meters{0.4}}));
    for (std::size_t i = 0; i < 2; ++i)
        c.setSelection(i, {WorkspaceSelectionSpace::WorldMeters, 0, "Synthetic exact endpoint."});
    ASSERT_TRUE(c.measurement().result->plusMinusMeters);
    f.source.diagnostics().request();
    ASSERT_TRUE(c.measurement().result);
    EXPECT_FALSE(c.measurement().result->plusMinusMeters);
    EXPECT_TRUE(c.endpoints()[0]);
    EXPECT_TRUE(c.endpoints()[1]);
    ASSERT_TRUE(QTest::qWaitFor([&] { return !f.source.diagnostics().busy(); }));
    ASSERT_TRUE(c.measurement().result);
    EXPECT_FALSE(c.measurement().result->plusMinusMeters);
    f.source.setInput({}, false);
    EXPECT_FALSE(c.endpoints()[0]);
    EXPECT_FALSE(c.endpoints()[1]);
    EXPECT_FALSE(c.measurement().result);
}
TEST(WorkspaceDistanceUiTest, DialogPicksEditsSwitchesViewsAndClearsWithExplicitSelectionBounds)
{
    Fixture f;
    f.surveyed();
    f.fresh();
    WorkspaceDistanceDialog dialog(f.source);
    dialog.show();
    QApplication::processEvents();
    auto* freeze = dialog.findChild<QPushButton*>("distanceFreeze");
    ASSERT_NE(freeze, nullptr);
    freeze->click();
    ASSERT_TRUE(QTest::qWaitFor([&] { return !f.source.busy(); }));
    QApplication::processEvents();
    auto* view = dialog.findChild<QWidget*>("distanceView");
    ASSERT_NE(view, nullptr);
    auto vp = workspaceCameraViewport(1000, 800, view->width(), view->height());
    ASSERT_TRUE(vp);
    for (auto p : {ImageCorner{380, 450}, ImageCorner{620, 300}}) {
        auto at = workspaceCameraDisplay(*vp, p);
        ASSERT_TRUE(at);
        clickAt(view, {at->u, at->v});
    }
    auto* value = dialog.findChild<QLabel*>("distanceValue");
    ASSERT_NE(value, nullptr);
    EXPECT_TRUE(value->text().contains("33.541"));
    auto* uncertainty = dialog.findChild<QLabel*>("distanceUncertainty");
    EXPECT_TRUE(uncertainty->text().contains("unavailable"));
    for (int i = 0; i < 2; ++i) {
        const auto prefix = QStringLiteral("distance%1").arg(i);
        dialog.findChild<QLineEdit*>(prefix + "Method")
            ->setText("Synthetic fixed maximum disk in native pixels.");
        dialog.findChild<QCheckBox*>(prefix + "Known")->setChecked(true);
        dialog.findChild<QDoubleSpinBox*>(prefix + "Radius")->setValue(1);
    }
    EXPECT_TRUE(uncertainty->text().contains("95%"));
    EXPECT_FALSE(uncertainty->text().contains("unavailable"));
    const auto distance = value->text(), bound = uncertainty->text();
    auto* mode = dialog.findChild<QComboBox*>("distanceViewMode");
    mode->setCurrentIndex(1);
    QApplication::processEvents();
    EXPECT_EQ(value->text(), distance);
    EXPECT_EQ(uncertainty->text(), bound);
    if (const auto* screenshot = std::getenv("MICRO_ROBOT_DISTANCE_SCREENSHOT"))
        EXPECT_TRUE(dialog.grab().save(QString::fromLocal8Bit(screenshot)));
    mode->setCurrentIndex(0);
    QApplication::processEvents();
    EXPECT_EQ(value->text(), distance);
    if (const auto* screenshot = std::getenv("MICRO_ROBOT_DISTANCE_SCREENSHOT"))
        EXPECT_TRUE(dialog.grab().save(QString::fromLocal8Bit(screenshot) + ".camera.png"));
    auto* x = dialog.findChild<QDoubleSpinBox*>("distance1X");
    auto* y = dialog.findChild<QDoubleSpinBox*>("distance1Y");
    x->setValue(620);
    y->setValue(400);
    dialog.findChild<QPushButton*>("distance1Apply")->click();
    EXPECT_FALSE(dialog.findChild<QCheckBox*>("distance1Known")->isChecked());
    EXPECT_TRUE(dialog.findChild<QLineEdit*>("distance1Method")->text().isEmpty());
    EXPECT_TRUE(uncertainty->text().contains("unavailable"));
    mode->setCurrentIndex(1);
    clickAt(view, {view->width() / 2.0, view->height() / 2.0});
    EXPECT_NEAR(x->value(), 500, 1e-6);
    EXPECT_NEAR(y->value(), 300, 1e-6);
    auto* selectionSpace = dialog.findChild<QComboBox*>("distance1Space");
    auto* selectionKnown = dialog.findChild<QCheckBox*>("distance1Known");
    auto* selectionMethod = dialog.findChild<QLineEdit*>("distance1Method");
    selectionMethod->setText("Synthetic one millimetre world-space disk.");
    selectionKnown->setChecked(true);
    dialog.findChild<QDoubleSpinBox*>("distance1Radius")->setValue(1);
    selectionSpace->setCurrentIndex(0);
    EXPECT_FALSE(selectionKnown->isChecked());
    EXPECT_TRUE(selectionMethod->text().isEmpty());
    dialog.findChild<QPushButton*>("distance0Clear")->click();
    EXPECT_TRUE(value->text().contains("Select two"));
    dialog.findChild<QPushButton*>("distanceClearBoth")->click();
    EXPECT_TRUE(dialog.findChild<QLabel*>("distance1Native")->text().contains("not selected"));
}
TEST(WorkspaceDistanceUiTest, CalibrationDialogEntryAndResolutionInvalidationClearMeasurement)
{
    Fixture f;
    f.fresh();
    WorkspaceCalibrationDialog calibration(f.source);
    auto* entry = calibration.findChild<QPushButton*>("workspaceDistance");
    ASSERT_NE(entry, nullptr);
    EXPECT_TRUE(entry->isEnabled());
    entry->click();
    QApplication::processEvents();
    auto* distance = calibration.findChild<WorkspaceDistanceDialog*>();
    ASSERT_NE(distance, nullptr);
    distance->findChild<QPushButton*>("distanceFreeze")->click();
    ASSERT_TRUE(QTest::qWaitFor([&] { return !f.source.busy(); }));
    ASSERT_TRUE(f.source.measurementFrame());
    auto bad = std::make_shared<Frame>(*captureFrame(2, 640, 480));
    bad->capturedAt = MonotonicClock::now();
    f.source.receiveFrame(bad);
    EXPECT_FALSE(f.source.measurementFrame());
    EXPECT_FALSE(entry->isEnabled());
    EXPECT_TRUE(distance->findChild<QLabel*>("distanceValue")->text().contains("Select two"));
}

TEST(WorkspaceDistanceUiTest, ActivatingAnInteriorOriginClearsOldEndpointsButLoadingDraftDoesNot)
{
    Fixture f;
    WorkspaceDistanceController c(f.source);
    ASSERT_TRUE(c.setWorld(0, {Meters{0.35}, Meters{0.25}}));
    ASSERT_TRUE(c.setWorld(1, {Meters{0.65}, Meters{0.4}}));
    const double distance = c.measurement().result->distanceMeters;
    auto shifted = std::make_shared<WorkspaceAcquisitionRecord>(*f.acquisition);
    shifted->id = "interior-origin";
    shifted->profile.acquisitionId = shifted->id;
    shifted->profile.frameId = "signed-interior-frame";
    shifted->profile.bounds = WorkspaceBounds{Meters{1}, Meters{0.6}, {Meters{-0.5}, Meters{-0.3}}};
    shifted->profile.homography->imageToWorld[2] -= 0.5;
    shifted->profile.homography->imageToWorld[5] -= 0.3;
    for (auto& r : shifted->references) {
        r.point.world.x.value -= 0.5;
        r.point.world.y.value -= 0.3;
    }
    f.store->record = shifted;
    f.source.load();
    ASSERT_TRUE(QTest::qWaitFor([&] { return !f.source.busy(); }));
    ASSERT_TRUE(c.measurement().result);
    EXPECT_EQ(c.measurement().result->distanceMeters, distance);
    f.source.activate();
    EXPECT_FALSE(c.endpoints()[0]);
    EXPECT_FALSE(c.endpoints()[1]);
    EXPECT_FALSE(c.measurement().result);
    ASSERT_TRUE(QTest::qWaitFor([&] { return !f.source.diagnostics().busy(); }));
    ASSERT_TRUE(c.geometry());
    EXPECT_EQ(c.geometry()->profile.frameId, "signed-interior-frame");
    ASSERT_TRUE(c.setWorld(0, {Meters{-0.15}, Meters{-0.05}}));
    ASSERT_TRUE(c.setWorld(1, {Meters{0.15}, Meters{0.1}}));
    EXPECT_NEAR(c.measurement().result->distanceMeters, distance, 1e-12);
}

TEST(WorkspaceDistanceUiTest, EditorsRetainCoordinatesInLargerValidLoadedWorkspaces)
{
    Fixture f;
    auto large = std::make_shared<WorkspaceAcquisitionRecord>(*f.acquisition);
    large->id = "large-frame";
    large->profile.acquisitionId = large->id;
    large->profile.bounds = WorkspaceBounds{Meters{2500}, Meters{1500}};
    for (std::size_t i = 0; i < 6; ++i)
        large->profile.homography->imageToWorld[i] *= 2500;
    for (auto& r : large->references) {
        r.point.world.x.value *= 2500;
        r.point.world.y.value *= 2500;
    }
    f.store->record = large;
    f.source.load();
    ASSERT_TRUE(QTest::qWaitFor([&] { return !f.source.busy(); }));
    f.source.activate();
    ASSERT_TRUE(QTest::qWaitFor([&] { return !f.source.diagnostics().busy(); }));
    WorkspaceDistanceDialog dialog(f.source);
    for (int i = 0; i < 2; ++i) {
        const auto prefix = QStringLiteral("distance%1").arg(i);
        auto* x = dialog.findChild<QDoubleSpinBox*>(prefix + "X");
        auto* y = dialog.findChild<QDoubleSpinBox*>(prefix + "Y");
        x->setValue(i == 0 ? 1750000 : 2000000);
        y->setValue(1000000);
        dialog.findChild<QPushButton*>(prefix + "Apply")->click();
        EXPECT_EQ(x->value(), i == 0 ? 1750000 : 2000000);
    }
    EXPECT_TRUE(dialog.findChild<QLabel*>("distanceValue")->text().contains("25000 cm"));
}
} // namespace
