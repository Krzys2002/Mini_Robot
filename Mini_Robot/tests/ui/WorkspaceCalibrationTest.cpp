#include "../calibration/CaptureFixtures.hpp"
#include "../calibration/HomographyFixtures.hpp"
#include "app/WorkspaceCalibrationController.hpp"
#include "ui/WorkspaceCalibrationDialog.hpp"
#include "world/WorldModel.hpp"
#include <QApplication>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QLineEdit>
#include <QMouseEvent>
#include <QPushButton>
#include <QTableWidget>
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
    std::string saveError;
    std::shared_ptr<Gate> gate;

    WorkspaceProfileLoadOutcome load(std::string_view, const IntrinsicCalibration&) override
    {
        if (gate)
            gate->wait();
        return saved ? WorkspaceProfileLoadOutcome{ProfileLoadState::Loaded, saved, {}}
                     : WorkspaceProfileLoadOutcome{};
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
    dialog.findChild<QDoubleSpinBox*>("workspaceWidth")->setValue(1);
    dialog.findChild<QDoubleSpinBox*>("workspaceHeight")->setValue(0.6);
    dialog.findChild<QPushButton*>("workspaceFreeze")->click();
    ASSERT_TRUE(QTest::qWaitFor([&] { return !f.controller.busy(); }));
    ASSERT_TRUE(f.controller.frozenFrame());
    auto* table = dialog.findChild<QTableWidget*>("workspacePoints");
    const auto points = homographyInput();
    EXPECT_FALSE(dialog.findChild<QPushButton*>("workspaceAddPoint")->isEnabled());
    for (const auto& p : points.fitPoints) f.controller.selectCorner(p.image);
    dialog.findChild<QPushButton*>("workspaceConfirmBoundary")->click();
    ASSERT_EQ(table->rowCount(), 4);
    EXPECT_FALSE(table->item(0,0)->flags().testFlag(Qt::ItemIsEditable));
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
    ASSERT_EQ(table->rowCount(), 5);
    EXPECT_NEAR(table->item(4, 0)->text().toDouble(), 500, 1e-10);
    EXPECT_NEAR(table->item(4, 1)->text().toDouble(), 400, 1e-10);
    table->item(4, 2)->setText("invalid");
    f.controller.verify();
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
    auto* table = dialog.findChild<QTableWidget*>("workspacePoints");
    ASSERT_EQ(table->rowCount(), 1);
    EXPECT_NEAR(table->item(0, 0)->text().toDouble(), 500, 1e-10);
    EXPECT_NEAR(table->item(0, 1)->text().toDouble(), 400, 1e-10);
    EXPECT_EQ(table->item(0, 4)->checkState(), Qt::Checked);
    table->item(0, 2)->setText("0.5");
    table->item(0, 3)->setText("0.3");
    ASSERT_TRUE(f.controller.draftProfile());
    auto* verify = dialog.findChild<QPushButton*>("workspaceVerify");
    ASSERT_TRUE(verify->isEnabled());
    verify->click();
    EXPECT_TRUE(f.controller.message().contains("Transform unchanged"));
    EXPECT_FALSE(f.controller.profile());
    WorkspaceCalibrationDialog reopened{f.controller};
    EXPECT_EQ(reopened.findChild<QTableWidget*>("workspacePoints")->rowCount(), 1);
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
    dialog.findChild<QDoubleSpinBox*>("workspaceOriginX")->setValue(0.5);
    dialog.findChild<QDoubleSpinBox*>("workspaceOriginY")->setValue(0.3);
    dialog.findChild<QPushButton*>("workspaceConfirmBoundary")->click();
    ASSERT_TRUE(f.controller.boundaryConfirmed());
    auto* table = dialog.findChild<QTableWidget*>("workspacePoints");
    ASSERT_EQ(table->rowCount(),4);
    EXPECT_DOUBLE_EQ(table->item(0,2)->text().toDouble(),-0.5);
    EXPECT_DOUBLE_EQ(table->item(0,3)->text().toDouble(),-0.3);
    dialog.findChild<QPushButton*>("workspaceCalculate")->click();
    ASSERT_TRUE(QTest::qWaitFor([&]{ return !f.controller.busy(); }));
    ASSERT_TRUE(f.controller.draftProfile());
    const auto screenshot = qEnvironmentVariable("MICRO_ROBOT_WORKSPACE_SIGNED_SCREENSHOT");
    if (!screenshot.isEmpty()) EXPECT_TRUE(dialog.grab().save(screenshot));
    dialog.findChild<QPushButton*>("workspacePickOrigin")->click();
    click({300,500});
    EXPECT_FALSE(f.controller.draftProfile());
    EXPECT_EQ(table->rowCount(),0);
    EXPECT_NEAR(dialog.findChild<QDoubleSpinBox*>("workspaceOriginX")->value(),0.25,1e-8);
    EXPECT_NEAR(dialog.findChild<QDoubleSpinBox*>("workspaceOriginY")->value(),0.2,1e-8);
}
