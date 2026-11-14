#include <QMessageBox>
#include <QTimer>
#include "../calibration/CaptureFixtures.hpp"
#include "../calibration/HomographyFixtures.hpp"
#include "app/WorkspaceCalibrationController.hpp"
#include "app/WorkspaceDiagnosticsController.hpp"
#include "ui/WorkspaceDiagnosticsDialog.hpp"
#include "ui/WorkspaceCalibrationDialog.hpp"
#include "world/WorldModel.hpp"
#include <QApplication>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QLineEdit>
#include <QMouseEvent>
#include <QPushButton>
#include <QTableWidget>
#include <QTableView>
#include "calibration/WorkspaceAcquisition.hpp"
#include <QTest>
#include <atomic>
#include <functional>
#include <condition_variable>
#include <gtest/gtest.h>
#include <mutex>

namespace {
using namespace micro_robot;
using namespace micro_robot::test;

struct Gate {
    std::mutex mutex;
    std::condition_variable condition;
    bool released{};
    std::atomic<bool> entered{};

    void wait()
    {
        entered = true;
        std::unique_lock lock(mutex);
        condition.wait(lock, [&] { return released; });
    }

    void release()
    {
        {
            std::lock_guard lock(mutex);
            released = true;
        }
        condition.notify_all();
    }
};

class Solver final : public IWorkspaceHomographySolver {
public:
    std::shared_ptr<Gate> gate;
    std::function<void(WorkspaceHomographyResult&)> alter;

    WorkspaceHomographyOutcome solve(const WorkspaceHomographyInput& input) override
    {
        if (gate)
            gate->wait();
        auto h = knownHomography();
        for (std::size_t i=0;i<3;++i) h.imageToWorld[i]*=input.profile.bounds.width().value;
        for (std::size_t i=3;i<6;++i) h.imageToWorld[i]*=input.profile.bounds.height().value/0.6;
        h.imageToWorld[2] += input.profile.bounds.minimum().x.value;
        h.imageToWorld[5] += input.profile.bounds.minimum().y.value;
        auto result = evaluateWorkspaceHomography(input, h);
        if (alter && result.result) alter(*result.result);
        return result;
    }
};

class Undistorter final : public ICalibrationUndistorter {
public:
    bool wrongMetadata{};

    UndistortedPreviewOutcome undistort(const Frame& frame, const IntrinsicCalibration&) override
    {
        auto copy = std::make_shared<Frame>(frame);
        if (wrongMetadata)
            ++copy->width;
        return {std::move(copy), {}};
    }
};

class Store final : public IWorkspaceProfileStore {
public:
    std::optional<WorkspaceCalibrationProfile> saved;
    std::shared_ptr<WorkspaceAcquisitionRecord> acquisition;
    std::string saveError;
    std::shared_ptr<Gate> gate;

    WorkspaceProfileLoadOutcome load(std::string_view, const IntrinsicCalibration&) override
    {
        if (gate)
            gate->wait();
        return saved ? WorkspaceProfileLoadOutcome{ProfileLoadState::Loaded, saved, {}, acquisition}
                     : WorkspaceProfileLoadOutcome{};
    }

    std::string saveAcquisition(const WorkspaceCalibrationProfile& p,const WorkspaceAcquisitionRecord& record) override
    {
        const auto error=save(p);
        if (error.empty()) acquisition=std::make_shared<WorkspaceAcquisitionRecord>(record);
        return error;
    }
    std::string save(const WorkspaceCalibrationProfile& p) override
    {
        if (!saveError.empty())
            return saveError;
        saved = p;
        return {};
    }
};

struct Fixture {
    Logger logger;
    Solver* solver{};
    Undistorter* undistorter{};
    Store* store{};
    std::shared_ptr<Gate> gate = std::make_shared<Gate>();
    WorkspaceCalibrationController controller{makeSolver(), makeUndistorter(), makeStore(),
                                              logger.channel("workspace")};

    Fixture()
    {
        controller.setInput(validIntrinsicCalibration(), true);
        controller.configure("table-A", 1, 0.6);
        fresh();
    }

    std::unique_ptr<Solver> makeSolver()
    {
        auto p = std::make_unique<Solver>();
        solver = p.get();
        return p;
    }

    std::unique_ptr<Undistorter> makeUndistorter()
    {
        auto p = std::make_unique<Undistorter>();
        undistorter = p.get();
        return p;
    }

    std::unique_ptr<Store> makeStore()
    {
        auto p = std::make_unique<Store>();
        store = p.get();
        return p;
    }

    void fresh(std::uint64_t seq = 1)
    {
        auto frame = std::make_shared<Frame>(*captureFrame(seq));
        frame->capturedAt = MonotonicClock::now();
        controller.receiveFrame(frame);
    }

    void freeze()
    {
        controller.freeze();
        ASSERT_TRUE(QTest::qWaitFor([&] { return !controller.busy(); }));
        ASSERT_TRUE(controller.frozenFrame()) << controller.message().toStdString();
    }

    void boundary()
    {
        controller.clearCorners();
        for (const auto& p : homographyInput().fitPoints) controller.selectCorner(p.image);
        controller.confirmBoundary();
        ASSERT_TRUE(controller.boundaryConfirmed()) << controller.message().toStdString();
    }
    void calculate()
    {
        boundary();
        const auto input = homographyInput();
        controller.setPoints(input.fitPoints, input.checkPoints);
        controller.calculate();
        ASSERT_TRUE(QTest::qWaitFor([&] { return !controller.busy(); }));
        ASSERT_TRUE(controller.draftProfile()) << controller.message().toStdString();
    }

    ~Fixture()
    {
        gate->release();
        controller.shutdown();
        (void)QTest::qWaitFor([&] { return controller.isStopped(); });
    }
};

TEST(WorkspaceDiagnosticsIntegrationTest, ActiveEvidenceBindingSurvivesDraftEditsAndClearsWithOptics)
{
    Fixture f; f.freeze(); f.calculate();
    WorkspaceCalibrationDialog dialog(f.controller);
    auto* maps=dialog.findChild<QPushButton*>("workspaceDiagnostics");ASSERT_NE(maps,nullptr);EXPECT_FALSE(maps->isEnabled());
    f.controller.activate();EXPECT_TRUE(maps->isEnabled());
    auto& diagnostics=f.controller.diagnostics();ASSERT_TRUE(QTest::qWaitFor([&]{return !diagnostics.busy();}));ASSERT_TRUE(diagnostics.snapshot())<<diagnostics.message().toStdString();
    EXPECT_EQ(diagnostics.snapshot()->profile.acquisitionId,f.controller.profile()->acquisitionId);
    EXPECT_EQ(diagnostics.snapshot()->acquisition,f.controller.activeAcquisition());
    const auto active=diagnostics.snapshot();f.controller.configure("draft-change",2,1);EXPECT_EQ(diagnostics.snapshot(),active);
    maps->click();QApplication::processEvents();ASSERT_NE(dialog.findChild<WorkspaceDiagnosticsDialog*>(),nullptr);
    auto optics=validIntrinsicCalibration();optics.cameraMatrix[0]+=1;f.controller.setInput(optics,true);
    EXPECT_FALSE(diagnostics.snapshot());EXPECT_FALSE(diagnostics.profile());EXPECT_FALSE(maps->isEnabled());
}

TEST(WorkspaceCalibrationUiTest, FreezeSolveAndStoreRoundTripThroughWorker)
{
    Fixture f;
    f.freeze();
    f.calculate();
    EXPECT_TRUE(f.controller.canSave());
    EXPECT_TRUE(f.controller.message().contains("Independent check"));
    f.controller.save();
    ASSERT_TRUE(QTest::qWaitFor([&] { return !f.controller.busy(); }));
    ASSERT_TRUE(f.store->saved);
    const auto matrix = f.controller.draftProfile()->homography->imageToWorld;
    f.controller.configure("table-A", 2, 1);
    EXPECT_FALSE(f.controller.draftProfile());
    f.controller.load();
    ASSERT_TRUE(QTest::qWaitFor([&] { return !f.controller.busy(); }));
    ASSERT_TRUE(f.controller.draftProfile());
    EXPECT_EQ(f.controller.draftProfile()->homography->imageToWorld, matrix);
    EXPECT_DOUBLE_EQ(f.controller.draftProfile()->bounds.width().value, 1);
    EXPECT_FALSE(f.controller.frozenFrame());
    EXPECT_FALSE(f.controller.canCalculate());
}

TEST(WorkspaceCalibrationUiTest, InvalidGeometryAndStorageErrorsHaveVisibleFeedback)
{
    Fixture f;
    f.freeze();
    auto points = homographyInput();
    for (const auto& p : points.fitPoints) f.controller.selectCorner(p.image);
    f.controller.selectCorner(points.fitPoints[0].image, 1);
    f.controller.confirmBoundary();
    EXPECT_FALSE(f.controller.draftProfile());
    EXPECT_TRUE(f.controller.message().contains("duplicate"));
    f.calculate();
    f.store->saveError = "disk full";
    f.controller.save();
    ASSERT_TRUE(QTest::qWaitFor([&] { return !f.controller.busy(); }));
    EXPECT_TRUE(f.controller.message().contains("disk full"));
    f.controller.setInput(validIntrinsicCalibration(), false);
    EXPECT_FALSE(f.controller.canSave());
    EXPECT_FALSE(f.controller.canLoad());
}

TEST(WorkspaceCalibrationUiTest, CameraOpticsResolutionAndPointEditsInvalidateResult)
{
    Fixture f;
    f.freeze();
    f.calculate();
    auto optical = validIntrinsicCalibration();
    optical.cameraMatrix[0] += 1;
    f.controller.setInput(optical, true);
    EXPECT_FALSE(f.controller.draftProfile());
    EXPECT_FALSE(f.controller.frozenFrame());
    f.fresh(2);
    f.freeze();
    f.calculate();
    f.controller.setPoints({}, {});
    EXPECT_FALSE(f.controller.draftProfile());
    f.controller.receiveFrame(captureFrame(3, 500, 400));
    EXPECT_FALSE(f.controller.frozenFrame());
    f.controller.setInput({}, false);
    EXPECT_FALSE(f.controller.canFreeze());
}

TEST(WorkspaceCalibrationUiTest, LateSolverCompletionCannotRestoreOldInputAndGuiRemainsResponsive)
{
    Fixture f;
    f.freeze();
    f.boundary();
    f.solver->gate = f.gate;
    const auto points = homographyInput();
    f.controller.setPoints(points.fitPoints, {});
    f.controller.calculate();
    ASSERT_TRUE(QTest::qWaitFor([&] { return f.gate->entered.load(); }));
    f.controller.setInput({}, false);
    EXPECT_FALSE(f.controller.draftProfile());
    f.gate->release();
    ASSERT_TRUE(QTest::qWaitFor([&] { return !f.controller.busy(); }));
    EXPECT_FALSE(f.controller.draftProfile());
    EXPECT_FALSE(f.controller.frozenFrame());
}

TEST(WorkspaceCalibrationUiTest, LateLoadCannotRestoreAWorkspaceForAnotherCamera)
{
    Fixture f;
    f.store->saved = homographyInput().profile;
    f.store->saved->homography = knownHomography();
    f.store->gate = f.gate;
    f.controller.load();
    ASSERT_TRUE(QTest::qWaitFor([&] { return f.gate->entered.load(); }));
    auto optical = validIntrinsicCalibration();
    optical.sourceId = "other-camera";
    f.controller.setInput(optical, true);
    f.gate->release();
    ASSERT_TRUE(QTest::qWaitFor([&] { return !f.controller.busy(); }));
    EXPECT_FALSE(f.controller.draftProfile());
}

TEST(WorkspaceCalibrationUiTest, StaleFramesBadRemapMetadataAndIncompatibleLoadsAreRejected)
{
    Fixture f;
    f.controller.setInput({}, false);
    f.controller.setInput(validIntrinsicCalibration(), true);
    f.controller.receiveFrame(captureFrame(2));
    f.controller.freeze();
    EXPECT_FALSE(f.controller.frozenFrame());
    EXPECT_TRUE(f.controller.message().contains("stale"));
    f.fresh(3);
    f.undistorter->wrongMetadata = true;
    f.controller.freeze();
    ASSERT_TRUE(QTest::qWaitFor([&] { return !f.controller.busy(); }));
    EXPECT_FALSE(f.controller.frozenFrame());
    f.store->saved = homographyInput().profile;
    f.store->saved->homography = knownHomography();
    f.store->saved->intrinsics.distortion[0] += 0.1;
    f.controller.load();
    ASSERT_TRUE(QTest::qWaitFor([&] { return !f.controller.busy(); }));
    EXPECT_FALSE(f.controller.draftProfile());
    EXPECT_TRUE(f.controller.message().contains("rejected"));
}

TEST(WorkspaceCalibrationUiTest, DialogAcquiresPairsReportsInvalidInputAndRendersFrozenFrame)
{
    Fixture f;
    WorkspaceCalibrationDialog dialog{f.controller};
    dialog.show();
    QApplication::processEvents();
    dialog.findChild<QLineEdit*>("workspaceIdentity")->setText("table-A");
    dialog.findChild<QLineEdit*>("workspaceWidth")->setText(QString::number(1));
    QMetaObject::invokeMethod(dialog.findChild<QLineEdit*>("workspaceWidth"),"editingFinished");
    dialog.findChild<QLineEdit*>("workspaceHeight")->setText(QString::number(0.6));
    QMetaObject::invokeMethod(dialog.findChild<QLineEdit*>("workspaceHeight"),"editingFinished");
    dialog.findChild<QPushButton*>("workspaceFreeze")->click();
    ASSERT_TRUE(QTest::qWaitFor([&] { return !f.controller.busy(); }));
    ASSERT_TRUE(f.controller.frozenFrame());
    auto* table = dialog.findChild<QTableView*>("workspacePoints");
    const auto points = homographyInput();
    EXPECT_FALSE(dialog.findChild<QPushButton*>("workspaceAddPoint")->isEnabled());
    for (const auto& p : points.fitPoints) f.controller.selectCorner(p.image);
    dialog.findChild<QPushButton*>("workspaceConfirmBoundary")->click();
    ASSERT_EQ(table->model()->rowCount(), 4);
    EXPECT_FALSE(table->model()->flags(table->model()->index(0,0)).testFlag(Qt::ItemIsEditable));
    dialog.findChild<QPushButton*>("workspaceCalculate")->click();
    ASSERT_TRUE(QTest::qWaitFor([&] { return !f.controller.busy(); }));
    ASSERT_TRUE(f.controller.draftProfile());
    EXPECT_TRUE(dialog.findChild<QLabel*>("workspaceMessage")->text().contains("No independent check"));
    const auto screenshot = qEnvironmentVariable("MICRO_ROBOT_WORKSPACE_CALIBRATION_SCREENSHOT");
    if (!screenshot.isEmpty())
        EXPECT_TRUE(dialog.grab().save(screenshot));
    // Clicking the letterboxed centre reports native image pixels, not display pixels.
    auto* image = dialog.findChild<QWidget*>("workspaceImage");
    const QSizeF fitted = QSizeF(1000,800).scaled(image->size(),Qt::KeepAspectRatio);
    const QPointF start{(image->width()-fitted.width())/2,(image->height()-fitted.height())/2};
    const QPointF position = start + QPointF{500.5*fitted.width()/1000,400.5*fitted.height()/800};
    QMouseEvent event{QEvent::MouseButtonPress,position,position,Qt::LeftButton,Qt::LeftButton,Qt::NoModifier};
    QApplication::sendEvent(image,&event);
    ASSERT_EQ(table->model()->rowCount(), 5);
    EXPECT_NEAR(table->model()->index(4,0).data().toString().toDouble(), 500, 1e-10);
    EXPECT_NEAR(table->model()->index(4,1).data().toString().toDouble(), 400, 1e-10);
    table->model()->setData(table->model()->index(4,2),"invalid");
    f.controller.verify();
    ASSERT_TRUE(QTest::qWaitFor([&]{return !f.controller.busy();}));
    EXPECT_TRUE(f.controller.message().contains("rejected", Qt::CaseInsensitive));
    f.controller.selectCorner({100,700}, 1);
    EXPECT_FALSE(f.controller.draftProfile());
    f.controller.confirmBoundary();
    EXPECT_TRUE(dialog.findChild<QLabel*>("workspaceMessage")->text().contains("duplicate"));

}
} // namespace

TEST(WorkspaceCalibrationReviewTest, DraftEditsAndFailedLoadsKeepActiveWorldBounds)
{
    Fixture f;
    f.freeze();
    f.calculate();
    EXPECT_FALSE(f.controller.profile());
    WorldModel world;
    QObject::connect(&f.controller, &WorkspaceCalibrationController::profileChanged, &f.controller, [&] {
        world.setWorkspaceBounds(f.controller.profile() ? std::optional{f.controller.profile()->bounds}
                                                        : std::nullopt);
    });
    f.controller.activate();
    ASSERT_TRUE(f.controller.profile());
    ASSERT_TRUE(world.workspaceBounds());
    const auto active = f.controller.profile()->homography->imageToWorld;
    f.controller.configure("draft-table", 2, 1);
    f.controller.setPoints({}, {});
    EXPECT_EQ(f.controller.profile()->homography->imageToWorld, active);
    EXPECT_DOUBLE_EQ(world.workspaceBounds()->width().value, 1);
    f.controller.load();
    ASSERT_TRUE(QTest::qWaitFor([&] { return !f.controller.busy(); }));
    EXPECT_EQ(f.controller.profile()->homography->imageToWorld, active);
    f.controller.setInput({}, false);
    EXPECT_FALSE(f.controller.profile());
    EXPECT_FALSE(world.workspaceBounds());
}

TEST(WorkspaceCalibrationReviewTest, LoadedTransformCanBeRecheckedWithoutRefittingOrActivation)
{
    Fixture f;
    f.store->saved = homographyInput().profile;
    f.store->saved->homography = knownHomography();
    f.controller.load();
    ASSERT_TRUE(QTest::qWaitFor([&] { return !f.controller.busy(); }));
    ASSERT_TRUE(f.controller.draftProfile());
    EXPECT_FALSE(f.controller.profile());
    const auto matrix = f.controller.draftProfile()->homography->imageToWorld;
    f.fresh(2);
    f.freeze();
    auto checks = homographyInput().checkPoints;
    checks.front().world.x.value += 0.02;
    f.controller.setPoints({}, checks);
    ASSERT_TRUE(f.controller.canVerify());
    f.controller.verify();
    ASSERT_TRUE(QTest::qWaitFor([&]{return !f.controller.busy();}));
    EXPECT_TRUE(f.controller.message().contains("0.02"));
    EXPECT_EQ(f.controller.draftProfile()->homography->imageToWorld, matrix);
    EXPECT_FALSE(f.controller.profile());
    f.controller.activate();
    ASSERT_TRUE(f.controller.profile());
    EXPECT_EQ(f.controller.profile()->homography->imageToWorld, matrix);
    f.controller.setPoints({}, {});
    EXPECT_FALSE(f.controller.canVerify());
    WorkspaceCalibrationDialog reopened{f.controller};
    EXPECT_EQ(reopened.findChild<QLineEdit*>("workspaceIdentity")->text(), "table-A");
}

TEST(WorkspaceCalibrationReviewTest, RepeatedResolutionMismatchPublishesOnlyOnce)
{
    Fixture f;
    f.freeze();
    f.calculate();
    f.controller.activate();
    int changes = 0, activeChanges = 0;
    QObject::connect(&f.controller, &WorkspaceCalibrationController::changed, &f.controller,
                     [&] { ++changes; });
    QObject::connect(&f.controller, &WorkspaceCalibrationController::profileChanged, &f.controller,
                     [&] { ++activeChanges; });
    for (int i = 0; i < 30; ++i)
        f.controller.receiveFrame(captureFrame(10 + i, 500, 400));
    EXPECT_EQ(changes, 1);
    EXPECT_EQ(activeChanges, 1);
}

TEST(WorkspaceCalibrationReviewTest, LoadedDraftCanBeVerifiedThroughDialogWithIntegerPixelCentres)
{
    Fixture f;
    f.store->saved = homographyInput().profile;
    f.store->saved->homography = knownHomography();
    f.controller.load();
    ASSERT_TRUE(QTest::qWaitFor([&] { return !f.controller.busy(); }));
    f.fresh(2);
    f.freeze();
    WorkspaceCalibrationDialog dialog{f.controller};
    dialog.show();
    QApplication::processEvents();
    auto* image = dialog.findChild<QWidget*>("workspaceImage");
    const QSizeF fitted = QSizeF(1000, 800).scaled(image->size(), Qt::KeepAspectRatio);
    const QPointF topLeft{(image->width() - fitted.width()) / 2, (image->height() - fitted.height()) / 2};
    const QPointF click = topLeft + QPointF{500.5 * fitted.width() / 1000, 400.5 * fitted.height() / 800};
    QMouseEvent event{QEvent::MouseButtonPress, click, click, Qt::LeftButton, Qt::LeftButton, Qt::NoModifier};
    QApplication::sendEvent(image, &event);
    auto* table = dialog.findChild<QTableView*>("workspacePoints");
    ASSERT_EQ(table->model()->rowCount(), 1);
    EXPECT_NEAR(table->model()->index(0,0).data().toString().toDouble(), 500, 1e-10);
    EXPECT_NEAR(table->model()->index(0,1).data().toString().toDouble(), 400, 1e-10);
    EXPECT_EQ(table->model()->index(0,4).data(Qt::CheckStateRole).toInt(), Qt::Checked);
    table->model()->setData(table->model()->index(0,2),"0.5");
    table->model()->setData(table->model()->index(0,3),"0.3");
    ASSERT_TRUE(f.controller.draftProfile());
    auto* verify = dialog.findChild<QPushButton*>("workspaceVerify");
    ASSERT_TRUE(verify->isEnabled());
    verify->click();
    ASSERT_TRUE(QTest::qWaitFor([&]{return !f.controller.busy();}));
    EXPECT_TRUE(f.controller.message().contains("Transform unchanged"));
    EXPECT_FALSE(f.controller.profile());
    WorkspaceCalibrationDialog reopened{f.controller};
    EXPECT_EQ(reopened.findChild<QTableView*>("workspacePoints")->model()->rowCount(), 1);
    dialog.findChild<QPushButton*>("workspaceActivate")->click();
    EXPECT_TRUE(f.controller.profile());
}

TEST(WorkspaceCornerWorkflowTest, CornerAndInteriorOriginsRoundTripWithoutReusingOldReferences)
{
    Fixture f;
    f.freeze();
    for (const auto origin : std::array<std::array<double,2>,5>{{{0,0},{1,0},{1,0.6},{0,0.6},{0.4,0.2}}}) {
        f.controller.configure("table-A",1,0.6,origin[0],origin[1]);
        f.boundary();
        f.controller.calculate();
        ASSERT_TRUE(QTest::qWaitFor([&]{ return !f.controller.busy(); }));
        ASSERT_TRUE(f.controller.draftProfile()) << f.controller.message().toStdString();
        const auto p = *f.controller.draftProfile();
        ASSERT_TRUE(p.boundary);
        EXPECT_FALSE(p.frameId.empty());
        EXPECT_NE(p.frameId,"legacy-lower-left");
        EXPECT_DOUBLE_EQ(p.bounds.minimum().x.value,-origin[0]);
        EXPECT_DOUBLE_EQ(p.bounds.minimum().y.value,-origin[1]);
        const auto mapped = projectWorkspacePoint(*p.homography,{100,700});
        ASSERT_TRUE(mapped);
        EXPECT_NEAR(mapped->x.value,-origin[0],1e-12);
        EXPECT_NEAR(mapped->y.value,-origin[1],1e-12);
        const auto saved = parseWorkspaceProfile(serializeWorkspaceProfile(p));
        ASSERT_TRUE(saved.profile);
        EXPECT_TRUE(saved.profile->bounds == p.bounds);
        EXPECT_EQ(saved.profile->frameId,p.frameId);
        EXPECT_EQ(saved.profile->homography->imageToWorld,p.homography->imageToWorld);
    }
    f.controller.activate();
    const auto activeId = f.controller.profile()->frameId;
    f.controller.setOriginFromImage({500,400});
    EXPECT_FALSE(f.controller.draftProfile());
    EXPECT_TRUE(f.controller.fitPoints().empty());
    EXPECT_TRUE(f.controller.checkPoints().empty());
    EXPECT_FALSE(f.controller.boundaryConfirmed());
    EXPECT_EQ(f.controller.profile()->frameId,activeId);
    EXPECT_NEAR(f.controller.draftBounds()->minimum().x.value,-0.5,1e-12);
    EXPECT_NEAR(f.controller.draftBounds()->minimum().y.value,-0.3,1e-12);
}

TEST(WorkspaceCornerWorkflowTest, UndoCrossingAndReselectionInvalidateOnlyDraft)
{
    Fixture f; f.freeze(); f.calculate(); f.controller.activate();
    const auto active = f.controller.profile()->frameId;
    f.controller.undoCorner();
    EXPECT_EQ(f.controller.corners().size(),3);
    EXPECT_FALSE(f.controller.boundaryConfirmed());
    EXPECT_FALSE(f.controller.draftProfile());
    EXPECT_TRUE(f.controller.fitPoints().empty());
    f.controller.confirmBoundary();
    EXPECT_FALSE(f.controller.boundaryConfirmed());
    f.controller.selectCorner({100,100});
    f.controller.selectCorner({900,100},1); // C2/C3 duplicate.
    f.controller.confirmBoundary();
    EXPECT_FALSE(f.controller.boundaryConfirmed());
    f.controller.selectCorner({900,700},1);
    f.controller.confirmBoundary();
    EXPECT_TRUE(f.controller.boundaryConfirmed());
    f.controller.selectCorner({-1,700},1); // Invalid correction must also invalidate the reviewed draft boundary.
    EXPECT_FALSE(f.controller.boundaryConfirmed());
    EXPECT_EQ(f.controller.profile()->frameId,active);
}

TEST(WorkspaceCornerWorkflowTest, OriginChangeDuringSolveRejectsLateResultWithoutPublishingActiveChange)
{
    Fixture f; f.freeze(); f.boundary();
    int activeChanges{};
    QObject::connect(&f.controller,&WorkspaceCalibrationController::profileChanged,&f.controller,[&]{ ++activeChanges; });
    f.solver->gate = f.gate;
    f.controller.calculate();
    ASSERT_TRUE(QTest::qWaitFor([&]{ return f.gate->entered.load(); }));
    f.controller.configure("table-A",1,0.6,0.5,0.3);
    f.gate->release();
    ASSERT_TRUE(QTest::qWaitFor([&]{ return !f.controller.busy(); }));
    EXPECT_FALSE(f.controller.draftProfile());
    EXPECT_EQ(activeChanges,0);
    EXPECT_TRUE(f.controller.fitPoints().empty());
}

TEST(WorkspaceCornerWorkflowTest, RejectsSolverOutputWithWrongOriginFrameOrBoundary)
{
    Fixture f; f.freeze();
    for (int mutation = 0; mutation < 3; ++mutation) {
        f.boundary();
        f.solver->alter = [mutation](WorkspaceHomographyResult& result) {
            if (mutation == 0) result.profile.bounds = WorkspaceBounds{Meters{1},Meters{0.6},{Meters{-0.5},Meters{0}}};
            if (mutation == 1) result.profile.frameId = "other";
            if (mutation == 2) (*result.profile.boundary)[0].u += 1;
        };
        f.controller.calculate();
        ASSERT_TRUE(QTest::qWaitFor([&]{ return !f.controller.busy(); }));
        EXPECT_FALSE(f.controller.draftProfile());
        EXPECT_TRUE(f.controller.message().contains("rejected",Qt::CaseInsensitive));
    }
}

TEST(WorkspaceCornerWorkflowTest, DialogPicksLabelledCornersAndMeasuredInteriorOrigin)
{
    Fixture f; f.freeze();
    WorkspaceCalibrationDialog dialog{f.controller}; dialog.show(); QApplication::processEvents();
    auto* image = dialog.findChild<QWidget*>("workspaceImage");
    auto click = [&](ImageCorner p) {
        const QSizeF fitted = QSizeF(1000,800).scaled(image->size(),Qt::KeepAspectRatio);
        const QPointF topLeft{(image->width()-fitted.width())/2,(image->height()-fitted.height())/2};
        const QPointF position = topLeft + QPointF{(p.u+0.5)*fitted.width()/1000,(p.v+0.5)*fitted.height()/800};
        QMouseEvent event{QEvent::MouseButtonPress,position,position,Qt::LeftButton,Qt::LeftButton,Qt::NoModifier};
        QApplication::sendEvent(image,&event);
    };
    for (const auto& p : homographyInput().fitPoints) click(p.image);
    ASSERT_EQ(f.controller.corners().size(),4);
    dialog.findChild<QLineEdit*>("workspaceOriginX")->setText(QString::number(0.5));
    QMetaObject::invokeMethod(dialog.findChild<QLineEdit*>("workspaceOriginX"),"editingFinished");
    dialog.findChild<QLineEdit*>("workspaceOriginY")->setText(QString::number(0.3));
    QMetaObject::invokeMethod(dialog.findChild<QLineEdit*>("workspaceOriginY"),"editingFinished");
    dialog.findChild<QPushButton*>("workspaceConfirmBoundary")->click();
    ASSERT_TRUE(f.controller.boundaryConfirmed());
    auto* table = dialog.findChild<QTableView*>("workspacePoints");
    ASSERT_EQ(table->model()->rowCount(),4);
    EXPECT_DOUBLE_EQ(table->model()->index(0,2).data().toString().toDouble(),-0.5);
    EXPECT_DOUBLE_EQ(table->model()->index(0,3).data().toString().toDouble(),-0.3);
    dialog.findChild<QPushButton*>("workspaceCalculate")->click();
    ASSERT_TRUE(QTest::qWaitFor([&]{ return !f.controller.busy(); }));
    ASSERT_TRUE(f.controller.draftProfile());
    const auto screenshot = qEnvironmentVariable("MICRO_ROBOT_WORKSPACE_SIGNED_SCREENSHOT");
    if (!screenshot.isEmpty()) EXPECT_TRUE(dialog.grab().save(screenshot));
    dialog.findChild<QPushButton*>("workspacePickOrigin")->click();
    click({300,500});
    EXPECT_FALSE(f.controller.draftProfile());
    EXPECT_EQ(table->model()->rowCount(),0);
    EXPECT_NEAR(dialog.findChild<QLineEdit*>("workspaceOriginX")->text().toDouble(),0.25,1e-8);
    EXPECT_NEAR(dialog.findChild<QLineEdit*>("workspaceOriginY")->text().toDouble(),0.2,1e-8);
}

#include <QCheckBox>
#include <QComboBox>
#include <QGroupBox>
#include <QSpinBox>
#include <QElapsedTimer>
#include <iostream>
#include <QTimer>
#include <QScrollArea>

namespace {
class WorkspaceBoardDetector final : public ICalibrationDetector {
public:
    std::shared_ptr<Gate> gate;
    std::atomic<double> anchorX{0.2},anchorY{0.1},widthMeters{1};
    std::atomic<std::uint64_t> observedSequence{};
    CalibrationDetection detect(const Frame& frame,const CalibrationTarget& target) override
    {
        observedSequence=frame.sequence;
        if (gate) gate->wait();
        CalibrationDetection result;
        for (int row=0;row<target.rows;++row) for (int col=0;col<target.columns;++col)
            result.corners.push_back({100+800/widthMeters.load()*(anchorX.load()+col*target.squareSizeMeters),700-1000*(anchorY.load()+row*target.squareSizeMeters)});
        return result;
    }
};
WorkspaceBoardDetector* installBoardDetector(Fixture& f)
{
    auto detector=std::make_unique<WorkspaceBoardDetector>(); auto* result=detector.get();
    f.controller.setAcquisitionDetector(std::move(detector)); return result;
}
WorkspaceBoardPlacement boardSettings(double x=0.2,double y=0.1)
{
    WorkspaceBoardPlacement p; p.anchor={Meters{x},Meters{y}}; p.columnAngle=Radians{0}; p.rowDirection=1; p.note="surveyed flat board"; return p;
}
void reviewBoard(Fixture& f,WorkspaceBoardPlacement settings=boardSettings())
{
    f.controller.detectBoard(); ASSERT_TRUE(QTest::qWaitFor([&]{return !f.controller.busy();}));
    ASSERT_TRUE(f.controller.detectedBoard())<<f.controller.message().toStdString();
    f.controller.reviewBoardPlacement(std::move(settings)); ASSERT_TRUE(QTest::qWaitFor([&]{return !f.controller.busy();}));
    ASSERT_TRUE(f.controller.boardReview())<<f.controller.message().toStdString();
}
}
TEST(WorkspaceBoardAcquisitionUiTest, RepeatedPlacementsRelativeAnchorsCorrectionsAndSaveLoadRetainProvenance)
{
    Fixture f; f.freeze(); f.boundary(); auto* detector=installBoardDetector(f);
    reviewBoard(f);
    f.controller.importBoardPlacement(false,false);
    EXPECT_EQ(f.controller.references().size(),4);
    f.controller.importBoardPlacement(false,true);
    ASSERT_EQ(f.controller.references().size(),58); ASSERT_EQ(f.controller.placements().size(),1);
    EXPECT_EQ(f.controller.placements()[0].frameSequence,1);
    f.controller.calculate(); ASSERT_TRUE(QTest::qWaitFor([&]{return !f.controller.busy();}));
    ASSERT_TRUE(f.controller.draftProfile())<<f.controller.message().toStdString();
    f.controller.activate(); const auto active=f.controller.profile()->acquisitionId;
    f.fresh(2); f.controller.freezeBoard(); ASSERT_TRUE(QTest::qWaitFor([&]{return !f.controller.busy();}));
    ASSERT_EQ(f.controller.references().size(),58);
    detector->anchorX=0.6; detector->anchorY=0.3;
    auto second=boardSettings(); second.relativeReferenceId=f.controller.references()[4].id;
    second.measuredOffset={Meters{0.4},Meters{0.2}};
    reviewBoard(f,second);
    EXPECT_NEAR(f.controller.boardReview()->anchor.x.value,0.6,1e-14);
    EXPECT_NEAR(f.controller.boardReview()->anchor.y.value,0.3,1e-14);
    f.controller.importBoardPlacement(true,true);
    ASSERT_EQ(f.controller.references().size(),112);
    EXPECT_EQ(f.controller.fitPoints().size(),58); EXPECT_EQ(f.controller.checkPoints().size(),54);
    EXPECT_EQ(f.controller.profile()->acquisitionId,active); // Draft check acquisition never silently replaces active.
    f.controller.save(); ASSERT_TRUE(QTest::qWaitFor([&]{return !f.controller.busy();}));
    ASSERT_TRUE(f.store->acquisition)<<f.controller.message().toStdString();
    EXPECT_EQ(f.store->acquisition->placements.size(),2);
    EXPECT_TRUE(validateWorkspaceAcquisition(*f.store->acquisition).empty());
    EXPECT_DOUBLE_EQ(f.store->acquisition->placements[1].relativeReferenceSnapshot.x.value,0.2);
    f.controller.load(); ASSERT_TRUE(QTest::qWaitFor([&]{return !f.controller.busy();}));
    ASSERT_TRUE(f.controller.draftProfile())<<f.controller.message().toStdString();
    ASSERT_EQ(f.controller.references().size(),112); EXPECT_EQ(f.controller.placements().size(),2);
    EXPECT_FALSE(f.controller.frozenFrame());
    auto corrected=f.controller.references()[4].point; corrected.world.x.value+=0.001;
    f.controller.editReference(4,corrected,false);
    EXPECT_FALSE(f.controller.draftProfile());
    EXPECT_DOUBLE_EQ(f.controller.placements()[0].anchor.x.value,0.2);
    EXPECT_DOUBLE_EQ(f.controller.placements()[0].detected[0].u,260);
    EXPECT_EQ(f.controller.profile()->acquisitionId,active);
}
TEST(WorkspaceBoardAcquisitionUiTest, ExactFrozenFrameAndLateDetectionReviewAreGenerationFenced)
{
    Fixture f; f.freeze(); f.boundary(); auto* detector=installBoardDetector(f); detector->gate=f.gate;
    f.controller.detectBoard(); ASSERT_TRUE(QTest::qWaitFor([&]{return f.gate->entered.load();}));
    f.fresh(99); EXPECT_EQ(detector->observedSequence.load(),1);
    f.controller.configure("table-A",1,0.6,0.5,0.3);
    f.gate->release(); ASSERT_TRUE(QTest::qWaitFor([&]{return !f.controller.busy();}));
    EXPECT_FALSE(f.controller.detectedBoard()); EXPECT_FALSE(f.controller.boardReview());
    EXPECT_TRUE(f.controller.references().empty());
    f.controller.configure("table-A",1,0.6); f.boundary(); detector->gate.reset();
    f.controller.detectBoard(); ASSERT_TRUE(QTest::qWaitFor([&]{return !f.controller.busy();}));
    ASSERT_TRUE(f.controller.detectedBoard()); EXPECT_EQ(f.controller.detectedBoard()->frameSequence,1); // Still the same frozen image.
    f.controller.reviewBoardPlacement(boardSettings());
    f.controller.configure("table-A",1,0.6,0.2,0.1);
    ASSERT_TRUE(QTest::qWaitFor([&]{return !f.controller.busy();}));
    EXPECT_FALSE(f.controller.boardReview()); EXPECT_TRUE(f.controller.placements().empty());
}
TEST(WorkspaceBoardAcquisitionUiTest, CapacityRejectionIsAtomicAndCheckRoleRemainsSeparate)
{
    Fixture f; f.freeze(); f.boundary(); installBoardDetector(f);
    auto fit=f.controller.fitPoints();
    for (std::size_t i=0;i<MaximumWorkspaceFitPoints-WorkspaceCornerCount;++i) {
        const double x=0.001+0.998*(i%64)/63, y=0.001+0.598*(i/64)/63;
        fit.push_back({{100+800*x,700-1000*y},{Meters{x},Meters{y}}});
    }
    f.controller.setPoints(fit,{}); ASSERT_EQ(f.controller.fitPoints().size(),4096);
    reviewBoard(f); f.controller.importBoardPlacement(false,true);
    EXPECT_EQ(f.controller.references().size(),4096); EXPECT_TRUE(f.controller.placements().empty());
    EXPECT_TRUE(f.controller.message().contains("atomically"));
    f.controller.importBoardPlacement(true,true);
    EXPECT_EQ(f.controller.fitPoints().size(),4096); EXPECT_EQ(f.controller.checkPoints().size(),54);
    EXPECT_EQ(f.controller.references().size(),4150);
}
TEST(WorkspaceBoardAcquisitionUiTest, PanelRequiresOrientationReviewAndPhysicalIdentityConfirmation)
{
    Fixture f; f.freeze(); f.boundary(); installBoardDetector(f);
    WorkspaceCalibrationDialog dialog{f.controller}; dialog.show(); QApplication::processEvents();
    auto* panel=dialog.findChild<QGroupBox*>("workspaceBoardPanel"); ASSERT_NE(panel,nullptr); panel->setChecked(true);
    dialog.findChild<QPushButton*>("workspaceBoardDetect")->click(); ASSERT_TRUE(QTest::qWaitFor([&]{return !f.controller.busy();}));
    dialog.findChild<QDoubleSpinBox*>("workspaceBoardAnchorX")->setValue(0.2);
    dialog.findChild<QDoubleSpinBox*>("workspaceBoardAnchorY")->setValue(0.1);
    dialog.findChild<QPushButton*>("workspaceBoardReview")->click(); ASSERT_TRUE(QTest::qWaitFor([&]{return !f.controller.busy();}));
    EXPECT_FALSE(f.controller.boardReview()); // No default physical row direction is inferred.
    dialog.findChild<QComboBox*>("workspaceBoardRowDirection")->setCurrentIndex(1);
    dialog.findChild<QPushButton*>("workspaceBoardReview")->click(); ASSERT_TRUE(QTest::qWaitFor([&]{return !f.controller.busy();}));
    ASSERT_TRUE(f.controller.boardReview())<<f.controller.message().toStdString();
    auto* import=dialog.findChild<QPushButton*>("workspaceBoardImport"); EXPECT_FALSE(import->isEnabled());
    // Zoom is a display crop; a hit changes the logical anchor index, never pixel coordinates.
    auto* zoom=dialog.findChild<QWidget*>("workspaceBoardZoom"); ASSERT_NE(zoom,nullptr);
    const QSizeF source{192,157}; const QSizeF fitted=source.scaled(QSizeF(zoom->width(),zoom->height()-25),Qt::KeepAspectRatio);
    const QPointF start{(zoom->width()-fitted.width())/2,25+(zoom->height()-25-fitted.height())/2};
    const QPointF position=start+QPointF{(340.5-244.5)*fitted.width()/source.width(),(575.5-459.5)*fitted.height()/source.height()};
    QMouseEvent pick{QEvent::MouseButtonPress,position,position,Qt::LeftButton,Qt::LeftButton,Qt::NoModifier}; QApplication::sendEvent(zoom,&pick);
    EXPECT_EQ(dialog.findChild<QSpinBox*>("workspaceBoardAnchorIndex")->value(),13);
    EXPECT_FALSE(f.controller.boardReview());
    dialog.findChild<QDoubleSpinBox*>("workspaceBoardAnchorX")->setValue(0.3);
    dialog.findChild<QDoubleSpinBox*>("workspaceBoardAnchorY")->setValue(0.125);
    dialog.findChild<QPushButton*>("workspaceBoardReview")->click(); ASSERT_TRUE(QTest::qWaitFor([&]{return !f.controller.busy();}));
    ASSERT_TRUE(f.controller.boardReview());
    auto* table=dialog.findChild<QTableWidget*>("workspaceBoardPoints"); ASSERT_EQ(table->rowCount(),54);
    table->item(0,5)->setCheckState(Qt::Unchecked);
    dialog.findChild<QCheckBox*>("workspaceBoardIdentityConfirmed")->setChecked(true); EXPECT_TRUE(import->isEnabled());
    const auto screenshot=qEnvironmentVariable("MICRO_ROBOT_WORKSPACE_BOARD_SCREENSHOT");
    if (!screenshot.isEmpty()) {
        dialog.findChild<QScrollArea*>("workspaceScroll")->ensureWidgetVisible(dialog.findChild<QWidget*>("workspaceBoardZoom"));
        QApplication::processEvents();
        EXPECT_TRUE(dialog.grab().save(screenshot));
    }
    import->click(); EXPECT_EQ(f.controller.references().size(),57);
    EXPECT_FALSE(f.controller.placements()[0].included[0]);
    dialog.findChild<QLineEdit*>("workspaceOriginX")->setText(QString::number(0.5));
    QMetaObject::invokeMethod(dialog.findChild<QLineEdit*>("workspaceOriginX"),"editingFinished");
    EXPECT_TRUE(f.controller.references().empty()); EXPECT_TRUE(f.controller.placements().empty());
    EXPECT_EQ(dialog.findChild<QComboBox*>("workspaceBoardRowDirection")->currentIndex(),0);
}
TEST(WorkspaceBoardAcquisitionUiTest, ModelShowsRequestedAndCapacityDatasetsWithoutGuiBlocking)
{
    Fixture f; f.freeze(); f.boundary();
    const auto corners=f.controller.fitPoints();
    for (int total : {1525,8192}) {
        auto fit=corners; std::vector<WorkspaceCorrespondence> check;
        if (total==1525) {
            for (int y=0;y<25;++y) for (int x=0;x<61;++x) {
                if ((x==0 || x==60) && (y==0 || y==24)) continue;
                const double wx=x/60.0,wy=0.6*y/24;
                fit.push_back({{100+800*wx,700-1000*wy},{Meters{wx},Meters{wy}}});
            }
        } else {
            for (std::size_t i=0;i<MaximumWorkspaceFitPoints;++i) {
                const double x=(i%64+0.2)/64,y=0.6*(i/64+0.2)/64;
                const double cx=(i%64+0.7)/64,cy=0.6*(i/64+0.7)/64;
                if (i<MaximumWorkspaceFitPoints-WorkspaceCornerCount) fit.push_back({{100+800*x,700-1000*y},{Meters{x},Meters{y}}});
                check.push_back({{100+800*cx,700-1000*cy},{Meters{cx},Meters{cy}}});
            }
        }
        f.controller.setPoints(fit,check); ASSERT_EQ(f.controller.references().size(),static_cast<std::size_t>(total));
        QElapsedTimer timer; timer.start();
        WorkspaceCalibrationDialog dialog{f.controller}; dialog.show(); QApplication::processEvents();
        auto* table=dialog.findChild<QTableView*>("workspacePoints"); ASSERT_NE(table,nullptr); EXPECT_EQ(table->model()->rowCount(),total);
        EXPECT_EQ(dialog.findChild<QTableWidget*>("workspacePoints"),nullptr);
        bool tick{}; QTimer::singleShot(0,&dialog,[&]{tick=true;}); ASSERT_TRUE(QTest::qWaitFor([&]{return tick;}));
        std::cout<<total<<"-row model show + event delivery: "<<timer.elapsed()<<" ms\n";
        table->model()->setData(table->model()->index(total-1,2),"0.333");
        EXPECT_NEAR(f.controller.references()[static_cast<std::size_t>(total-1)].point.world.x.value,0.333,1e-15);
    }
}

TEST(WorkspaceBoardAcquisitionUiTest, ActiveEvidenceRemainsImmutableAcrossDraftEditsAndCameraInvalidation)
{
    Fixture f; f.freeze(); f.calculate();
    ASSERT_TRUE(f.controller.draftAcquisition()); EXPECT_FALSE(f.controller.activeAcquisition());
    f.controller.activate(); ASSERT_TRUE(f.controller.activeAcquisition());
    const auto old=f.controller.activeAcquisition();
    auto changed=f.controller.references()[4].point; changed.world.x.value+=0.01;
    f.controller.editReference(4,changed,true);
    EXPECT_FALSE(f.controller.canActivate()); EXPECT_FALSE(f.controller.draftAcquisition());
    EXPECT_DOUBLE_EQ(old->references[4].point.world.x.value,0.5);
    EXPECT_EQ(f.controller.activeAcquisition(),old);
    f.controller.verify(); ASSERT_TRUE(QTest::qWaitFor([&]{return !f.controller.busy();}));
    ASSERT_TRUE(f.controller.draftAcquisition()); EXPECT_TRUE(f.controller.canActivate());
    f.controller.activate(); ASSERT_TRUE(f.controller.activeAcquisition());
    EXPECT_NE(f.controller.activeAcquisition()->id,old->id);
    EXPECT_NEAR(f.controller.activeAcquisition()->references[4].point.world.x.value,0.51,1e-14);
    EXPECT_EQ(f.controller.activeAcquisition()->id,f.controller.profile()->acquisitionId);
    f.controller.configure("table-A",1,0.6,0.5,0.3);
    EXPECT_TRUE(f.controller.activeAcquisition()); EXPECT_FALSE(f.controller.draftAcquisition());
    f.controller.setInput({},false); EXPECT_FALSE(f.controller.activeAcquisition());
}

TEST(WorkspaceBoardAcquisitionUiTest, FortyPlacementWorkflowSolvesAndStoresAll1525UniqueLocations)
{
    Fixture f; f.controller.configure("table-A",1.5,0.6); f.freeze(); f.boundary();
    auto* detector=installBoardDetector(f); detector->widthMeters=1.5;
    std::uint64_t sequence=1;
    for (int row : {0,5,10,15,19}) for (int column : {0,8,16,24,32,40,48,52}) {
        f.fresh(++sequence); f.controller.freezeBoard(); ASSERT_TRUE(QTest::qWaitFor([&]{return !f.controller.busy();}));
        detector->anchorX=column*0.025; detector->anchorY=row*0.025;
        reviewBoard(f,boardSettings(column*0.025,row*0.025));
        ASSERT_TRUE(f.controller.boardReview()); f.controller.importBoardPlacement(false,true);
    }
    ASSERT_EQ(f.controller.placements().size(),40);
    ASSERT_EQ(f.controller.references().size(),1525);
    ASSERT_EQ(f.controller.fitPoints().size(),1525);
    EXPECT_TRUE(f.controller.checkPoints().empty());
    f.controller.calculate(); ASSERT_TRUE(QTest::qWaitFor([&]{return !f.controller.busy();}));
    ASSERT_TRUE(f.controller.draftProfile())<<f.controller.message().toStdString();
    ASSERT_TRUE(f.controller.draftAcquisition()); EXPECT_EQ(f.controller.draftAcquisition()->references.size(),1525);
    f.controller.save(); ASSERT_TRUE(QTest::qWaitFor([&]{return !f.controller.busy();}));
    ASSERT_TRUE(f.store->acquisition)<<f.controller.message().toStdString();
    EXPECT_EQ(f.store->acquisition->placements.size(),40);
    EXPECT_EQ(f.store->acquisition->placements[0].reviewRejections[0],WorkspaceReferenceRejection::Duplicate);
    const auto restored=parseWorkspaceAcquisition(serializeWorkspaceAcquisition(*f.store->acquisition));
    ASSERT_TRUE(restored.record)<<restored.error;
    EXPECT_EQ(restored.record->references.size(),1525);
    EXPECT_EQ(restored.record->placements.back().frameSequence,41);
}

TEST(WorkspaceBoardAcquisitionUiTest, InteriorOriginKeepsNegativeBoardCoordinatesAndFrameBinding)
{
    Fixture f; f.controller.configure("table-A",1,0.6,0.5,0.3); f.freeze(); f.boundary(); installBoardDetector(f);
    reviewBoard(f,boardSettings(-0.3,-0.2));
    f.controller.importBoardPlacement(false,true);
    ASSERT_EQ(f.controller.references().size(),58);
    EXPECT_NEAR(f.controller.references()[4].point.world.x.value,-0.3,1e-14);
    EXPECT_NEAR(f.controller.references()[4].point.world.y.value,-0.2,1e-14);
    f.controller.calculate(); ASSERT_TRUE(QTest::qWaitFor([&]{return !f.controller.busy();}));
    ASSERT_TRUE(f.controller.draftAcquisition())<<f.controller.message().toStdString();
    const auto record=parseWorkspaceAcquisition(serializeWorkspaceAcquisition(*f.controller.draftAcquisition()));
    ASSERT_TRUE(record.record)<<record.error;
    EXPECT_EQ(record.record->profile.frameId,f.controller.draftProfile()->frameId);
    EXPECT_DOUBLE_EQ(record.record->profile.bounds.minimum().x.value,-0.5);
    EXPECT_DOUBLE_EQ(record.record->placements[0].anchor.x.value,-0.3);
}

namespace {
void changeWorkspaceUnit(WorkspaceCalibrationDialog& dialog,int index,QMessageBox::StandardButton answer)
{
    QTimer::singleShot(0,[answer] {
        auto* box=qobject_cast<QMessageBox*>(QApplication::activeModalWidget());
        ASSERT_NE(box,nullptr);
        box->button(answer)->click();
    });
    dialog.findChild<QComboBox*>("workspaceLengthUnit")->setCurrentIndex(index);
}
}
TEST(WorkspaceInputUiTest, DimensionsStartBlankAndOnlyValidateAfterEditingFinishes)
{
    Fixture f; f.controller.configure("table-A",0,0);
    WorkspaceCalibrationDialog dialog(f.controller); dialog.show(); QApplication::processEvents();
    auto* width=dialog.findChild<QLineEdit*>("workspaceWidth");
    auto* height=dialog.findChild<QLineEdit*>("workspaceHeight");
    ASSERT_NE(width,nullptr); ASSERT_NE(height,nullptr);
    EXPECT_TRUE(width->text().isEmpty()); EXPECT_TRUE(height->text().isEmpty());
    bool labelled=false;
    for (auto* label:dialog.findChildren<QLabel*>()) if (label->buddy()==width) {
        labelled=true; EXPECT_TRUE(label->text().contains("(m)"));
    }
    EXPECT_TRUE(labelled);
    width->setText("1"); height->setText("0.5");
    EXPECT_FALSE(f.controller.draftBounds());
    QMetaObject::invokeMethod(height,"editingFinished");
    ASSERT_TRUE(f.controller.draftBounds());
    EXPECT_DOUBLE_EQ(f.controller.draftBounds()->width().value,1);
    const auto revision=f.controller.frameId();
    width->setText("invalid");
    EXPECT_EQ(f.controller.frameId(),revision);
    QMetaObject::invokeMethod(width,"editingFinished");
    EXPECT_FALSE(f.controller.draftBounds());
    EXPECT_EQ(width->text(),"invalid");
    EXPECT_FALSE(width->styleSheet().isEmpty());
    width->setText("1"); QMetaObject::invokeMethod(width,"editingFinished");
    EXPECT_TRUE(f.controller.draftBounds()); EXPECT_TRUE(width->styleSheet().isEmpty());
    width->clear(); QMetaObject::invokeMethod(width,"editingFinished");
    EXPECT_TRUE(width->text().isEmpty()); EXPECT_FALSE(f.controller.draftBounds());
}
TEST(WorkspaceInputUiTest, YesConvertsAllLengthsAndPreservesCalibrationAndUnknowns)
{
    Fixture f; f.freeze(); f.boundary(); f.calculate(); f.controller.activate();
    const auto active=f.controller.profile(); const auto frame=f.controller.frameId();
    WorkspaceCalibrationDialog dialog(f.controller); dialog.show(); QApplication::processEvents();
    auto* anchor=dialog.findChild<QDoubleSpinBox*>("workspaceBoardAnchorX"); anchor->setValue(0.2);
    changeWorkspaceUnit(dialog,1,QMessageBox::Yes);
    EXPECT_DOUBLE_EQ(dialog.findChild<QLineEdit*>("workspaceWidth")->text().toDouble(),100);
    EXPECT_DOUBLE_EQ(dialog.findChild<QLineEdit*>("workspaceHeight")->text().toDouble(),60);
    EXPECT_DOUBLE_EQ(anchor->value(),20);
    EXPECT_DOUBLE_EQ(dialog.findChild<QDoubleSpinBox*>("workspaceBoardSpacing")->value(),2.5);
    EXPECT_TRUE(dialog.findChild<QLineEdit*>("workspaceBoardSurfaceElevation")->text().isEmpty());
    auto* model=dialog.findChild<QTableView*>("workspacePoints")->model();
    EXPECT_EQ(model->headerData(2,Qt::Horizontal).toString(),"x (cm)");
    EXPECT_DOUBLE_EQ(model->index(1,2).data().toString().toDouble(),100);
    EXPECT_EQ(f.controller.frameId(),frame);
    EXPECT_EQ(f.controller.profile()->frameId,active->frameId);
    EXPECT_TRUE(f.controller.draftProfile());
    for (auto* label:dialog.findChildren<QLabel*>()) if (label->buddy()==anchor)
        EXPECT_TRUE(label->text().contains("(cm)"));
    const auto screenshot=qEnvironmentVariable("MICRO_ROBOT_WORKSPACE_UNITS_SCREENSHOT");
    if (!screenshot.isEmpty()) EXPECT_TRUE(dialog.grab().save(screenshot));
    changeWorkspaceUnit(dialog,0,QMessageBox::Yes);
    EXPECT_DOUBLE_EQ(dialog.findChild<QLineEdit*>("workspaceWidth")->text().toDouble(),1);
    EXPECT_DOUBLE_EQ(anchor->value(),0.2);
    EXPECT_EQ(f.controller.frameId(),frame);
}
TEST(WorkspaceInputUiTest, NoKeepsNumbersAndReinterpretsDraftWithoutChangingActiveCalibration)
{
    Fixture f; f.freeze(); f.boundary(); f.calculate(); f.controller.activate();
    const auto active=f.controller.profile(); const auto frame=f.controller.frameId();
    WorkspaceCalibrationDialog dialog(f.controller); dialog.show(); QApplication::processEvents();
    auto* anchor=dialog.findChild<QDoubleSpinBox*>("workspaceBoardAnchorX"); anchor->setValue(0.2);
    changeWorkspaceUnit(dialog,1,QMessageBox::No);
    EXPECT_DOUBLE_EQ(dialog.findChild<QLineEdit*>("workspaceWidth")->text().toDouble(),1);
    EXPECT_DOUBLE_EQ(anchor->value(),0.2);
    ASSERT_TRUE(f.controller.draftBounds());
    EXPECT_DOUBLE_EQ(f.controller.draftBounds()->width().value,0.01);
    EXPECT_NE(f.controller.frameId(),frame);
    EXPECT_TRUE(f.controller.references().empty());
    EXPECT_EQ(f.controller.profile()->frameId,active->frameId);
    EXPECT_DOUBLE_EQ(f.controller.profile()->bounds.width().value,1);
}
TEST(WorkspaceInputUiTest, CentimeterReferenceEditsAreConvertedAtThePresentationBoundary)
{
    Fixture f; f.freeze(); f.boundary(); f.controller.appendReference(ImageCorner{500,400});
    WorkspaceCalibrationDialog dialog(f.controller);
    changeWorkspaceUnit(dialog,1,QMessageBox::Yes);
    auto* model=dialog.findChild<QTableView*>("workspacePoints")->model();
    ASSERT_TRUE(model->setData(model->index(4,2),"25"));
    ASSERT_TRUE(model->setData(model->index(4,3),"10"));
    EXPECT_DOUBLE_EQ(f.controller.references()[4].point.world.x.value,0.25);
    EXPECT_DOUBLE_EQ(f.controller.references()[4].point.world.y.value,0.1);
    EXPECT_DOUBLE_EQ(model->index(4,2).data().toString().toDouble(),25);
    EXPECT_EQ(model->headerData(0,Qt::Horizontal).toString(),"u (px)");
}

TEST(WorkspaceInputUiTest, CentimeterBoardReviewStoresMetersAndKeepsUnknownMetadataUnknown)
{
    Fixture f; f.freeze(); f.boundary(); installBoardDetector(f);
    f.controller.detectBoard(); ASSERT_TRUE(QTest::qWaitFor([&]{return !f.controller.busy();}));
    ASSERT_TRUE(f.controller.detectedBoard());
    WorkspaceCalibrationDialog dialog(f.controller); dialog.show();
    dialog.findChild<QGroupBox*>("workspaceBoardPanel")->setChecked(true);
    changeWorkspaceUnit(dialog,1,QMessageBox::Yes);
    dialog.findChild<QDoubleSpinBox*>("workspaceBoardAnchorX")->setValue(20);
    dialog.findChild<QDoubleSpinBox*>("workspaceBoardAnchorY")->setValue(10);
    dialog.findChild<QComboBox*>("workspaceBoardRowDirection")->setCurrentIndex(1);
    dialog.findChild<QLineEdit*>("workspaceBoardSurfaceElevation")->setText("0.4");
    dialog.findChild<QLineEdit*>("workspaceBoardElevationMethod")->setText("caliper survey");
    dialog.findChild<QPushButton*>("workspaceBoardReview")->click();
    ASSERT_TRUE(QTest::qWaitFor([&]{return !f.controller.busy();}));
    ASSERT_TRUE(f.controller.boardReview())<<f.controller.message().toStdString();
    EXPECT_DOUBLE_EQ(f.controller.boardReview()->anchor.x.value,0.2);
    EXPECT_DOUBLE_EQ(f.controller.boardReview()->target.squareSizeMeters,0.025);
    EXPECT_EQ(f.controller.boardReview()->surfaceElevationMeters,0.004);
    EXPECT_FALSE(f.controller.boardReview()->backingThicknessMeters);
    auto* table=dialog.findChild<QTableWidget*>("workspaceBoardPoints");
    EXPECT_DOUBLE_EQ(table->item(0,3)->text().toDouble(),20);
    EXPECT_TRUE(table->horizontalHeaderItem(3)->text().contains("cm"));
    dialog.findChild<QCheckBox*>("workspaceBoardIdentityConfirmed")->setChecked(true);
    const auto screenshot=qEnvironmentVariable("MICRO_ROBOT_WORKSPACE_BOARD_UNITS_SCREENSHOT");
    if (!screenshot.isEmpty()) {
        dialog.findChild<QScrollArea*>("workspaceScroll")->ensureWidgetVisible(dialog.findChild<QLineEdit*>("workspaceBoardSurfaceElevation"));
        QApplication::processEvents(); EXPECT_TRUE(dialog.grab().save(screenshot));
    }
    const auto* review=&*f.controller.boardReview();
    changeWorkspaceUnit(dialog,0,QMessageBox::Yes);
    EXPECT_EQ(&*f.controller.boardReview(),review);
    EXPECT_TRUE(dialog.findChild<QCheckBox*>("workspaceBoardIdentityConfirmed")->isChecked());
}
TEST(WorkspaceInputUiTest, NoDoesNotClampExistingNumbersWhenReturningToMeters)
{
    Fixture f; WorkspaceCalibrationDialog dialog(f.controller);
    changeWorkspaceUnit(dialog,1,QMessageBox::Yes);
    auto* spacing=dialog.findChild<QDoubleSpinBox*>("workspaceBoardSpacing");
    EXPECT_DOUBLE_EQ(spacing->value(),2.5);
    changeWorkspaceUnit(dialog,0,QMessageBox::No);
    EXPECT_DOUBLE_EQ(spacing->value(),2.5);
    EXPECT_DOUBLE_EQ(dialog.findChild<QLineEdit*>("workspaceWidth")->text().toDouble(),100);
}
TEST(WorkspaceInputUiTest, UnitRoundTripsAndFinishingAnUnchangedFieldPreserveExactFrameIdentity)
{
    Fixture f; f.controller.configure("table-A",0.123456789012345,0.5);
    WorkspaceCalibrationDialog dialog(f.controller);
    const auto frame=f.controller.frameId();
    const double original=f.controller.draftBounds()->width().value;
    changeWorkspaceUnit(dialog,1,QMessageBox::Yes);
    QMetaObject::invokeMethod(dialog.findChild<QLineEdit*>("workspaceWidth"),"editingFinished");
    EXPECT_EQ(f.controller.frameId(),frame);
    EXPECT_DOUBLE_EQ(f.controller.draftBounds()->width().value,original);
    changeWorkspaceUnit(dialog,0,QMessageBox::Yes);
    QMetaObject::invokeMethod(dialog.findChild<QLineEdit*>("workspaceWidth"),"editingFinished");
    EXPECT_EQ(f.controller.frameId(),frame);
    EXPECT_DOUBLE_EQ(f.controller.draftBounds()->width().value,original);
    EXPECT_EQ(dialog.findChild<QLineEdit*>("workspaceHeight")->text(),"0.5");
}

TEST(WorkspaceInputUiTest, PlainLengthInputsRetainDecimalCommaSupportWithoutAcceptingGrouping)
{
    const auto previous=QLocale();
    struct RestoreLocale { QLocale value; ~RestoreLocale() { QLocale::setDefault(value); } } restore{previous};
    QLocale::setDefault(QLocale{QLocale::Polish,QLocale::Poland});
    Fixture f; WorkspaceCalibrationDialog dialog(f.controller);
    auto* width=dialog.findChild<QLineEdit*>("workspaceWidth");
    auto* height=dialog.findChild<QLineEdit*>("workspaceHeight");
    width->setText("1,5"); height->setText("0,5");
    QMetaObject::invokeMethod(height,"editingFinished");
    ASSERT_TRUE(f.controller.draftBounds());
    EXPECT_DOUBLE_EQ(f.controller.draftBounds()->width().value,1.5);
    EXPECT_DOUBLE_EQ(f.controller.draftBounds()->height().value,0.5);
    width->setText("1.25"); QMetaObject::invokeMethod(width,"editingFinished");
    ASSERT_TRUE(f.controller.draftBounds());
    EXPECT_DOUBLE_EQ(f.controller.draftBounds()->width().value,1.25);
    width->setText("1 000"); QMetaObject::invokeMethod(width,"editingFinished");
    EXPECT_FALSE(f.controller.draftBounds());
}
