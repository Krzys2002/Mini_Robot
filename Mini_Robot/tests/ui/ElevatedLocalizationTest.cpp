#include "../calibration/ElevatedFixtures.hpp"
#include "app/WorkspaceCalibrationController.hpp"
#include "app/VisionController.hpp"
#include "app/VisionTracking.hpp"
#include "calibration/QtWorkspaceProfileStore.hpp"
#include "ui/ElevatedLocalizationDialog.hpp"
#include <QApplication>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QFile>
#include <QDir>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QTemporaryDir>
#include <QTest>
#include <atomic>
#include <future>
#include <gtest/gtest.h>
using namespace micro_robot;
using namespace micro_robot::test;
namespace {
class NoopHomography final:public IWorkspaceHomographySolver {
    WorkspaceHomographyOutcome solve(const WorkspaceHomographyInput&) override { return {{},"unused"}; }
};
class NoopUndistorter final:public ICalibrationUndistorter {
    UndistortedPreviewOutcome undistort(const Frame& frame,const IntrinsicCalibration&) override { return {std::make_shared<Frame>(frame),{}}; }
};
class MemoryWorkspace final:public IWorkspaceProfileStore {
public:
    WorkspaceCalibrationProfile stored=elevatedProfile();
    std::shared_ptr<WorkspaceAcquisitionRecord> acquisition;
    WorkspaceProfileLoadOutcome load(std::string_view,const IntrinsicCalibration&) override { return {ProfileLoadState::Loaded,stored,{},acquisition}; }
    std::string save(const WorkspaceCalibrationProfile& p) override { stored=p; return {}; }
};
class GatedExtrinsics final:public IExtrinsicsSolver {
public:
    std::shared_future<void> gate;
    std::atomic<int> calls{};
    std::atomic<bool> offGui{};
    ExtrinsicSolveOutcome solve(const WorkspaceCalibrationProfile& p,const ExtrinsicSolveInput& input) override {
        ++calls; offGui=QThread::currentThread()!=QApplication::instance()->thread();
        if (gate.valid()) gate.wait();
        auto e=*elevatedProfile().localization;
        e.tabletopBinding=tabletopProfileBinding(p); e.survey=input.survey; e.mounts=input.mounts; e.revision=input.revision;
        return {e,{}};
    }
};
struct LocalFixture {
    Logger log;
    MemoryWorkspace* store{}; GatedExtrinsics* solver{};
    std::promise<void> release; bool released{};
    WorkspaceCalibrationController controller{std::make_unique<NoopHomography>(),std::make_unique<NoopUndistorter>(),makeStore(),log.channel("localization")};
    std::unique_ptr<MemoryWorkspace> makeStore() { auto s=std::make_unique<MemoryWorkspace>(); store=s.get(); s->stored.localization.reset(); return s; }
    LocalFixture() {
        controller.setInput(store->stored.intrinsics,true);
        controller.configure("elevated-table",1,.5,.5,.25);
        controller.load();
        EXPECT_TRUE(QTest::qWaitFor([&] { return !controller.busy(); }));
        EXPECT_TRUE(controller.draftProfile());
        auto s=std::make_unique<GatedExtrinsics>(); solver=s.get(); controller.setExtrinsicsSolver(std::move(s));
    }
    ExtrinsicSolveInput input(std::string rev="new-r2") { const auto e=*elevatedProfile().localization; return {e.survey,e.mounts,std::move(rev)}; }
    void unblock() { if (!released) { released=true; release.set_value(); } }
    ~LocalFixture() { unblock(); controller.shutdown(); (void)QTest::qWaitFor([&] { return controller.isStopped(); }); }
};
}
TEST(ElevatedLocalizationUiTest, WorkerCreatesDraftThenExplicitActivationAndSavePreserveBinding) {
    LocalFixture f;
    f.controller.calculateLocalization(f.input()); ASSERT_TRUE(QTest::qWaitFor([&]{return !f.controller.busy();}));
    ASSERT_TRUE(f.controller.draftProfile()->localization)<<f.controller.message().toStdString();
    EXPECT_TRUE(f.solver->offGui); EXPECT_FALSE(f.controller.profile());
    f.controller.activate(); ASSERT_TRUE(f.controller.profile()->localization);
    f.controller.save(); ASSERT_TRUE(QTest::qWaitFor([&]{return !f.controller.busy();}));
    EXPECT_EQ(f.store->stored.localization->revision,"new-r2");
    const auto loaded=parseWorkspaceProfile(serializeWorkspaceProfile(f.store->stored)); ASSERT_TRUE(loaded.profile)<<loaded.error;
    EXPECT_EQ(loaded.profile->localization->mounts[0].revision,"mount-r1");
}
TEST(ElevatedLocalizationUiTest, DraftEditAndCameraChangeFenceLateExtrinsicsWithoutRelabellingActiveSnapshot) {
    LocalFixture f;
    f.controller.calculateLocalization(f.input()); ASSERT_TRUE(QTest::qWaitFor([&]{return !f.controller.busy();})); f.controller.activate();
    f.solver->gate=f.release.get_future().share();
    f.controller.calculateLocalization(f.input("pending-r3")); ASSERT_TRUE(QTest::qWaitFor([&]{return f.solver->calls==2;}));
    f.controller.localizationInputsChanged(); f.unblock(); ASSERT_TRUE(QTest::qWaitFor([&]{return !f.controller.busy();}));
    ASSERT_TRUE(f.controller.draftProfile()); EXPECT_FALSE(f.controller.draftProfile()->localization);
    ASSERT_TRUE(f.controller.profile()->localization); EXPECT_EQ(f.controller.profile()->localization->revision,"new-r2");
    auto k=f.store->stored.intrinsics; k.cameraMatrix[0]+=1; f.controller.setInput(k,true);
    EXPECT_FALSE(f.controller.profile()); EXPECT_FALSE(f.controller.draftProfile());
}
TEST(ElevatedLocalizationUiTest, SurveyImportRejectsUnconfirmedMalformedAndOverCapacityRowsAtomically) {
    LocalFixture f; ElevatedLocalizationDialog dialog{f.controller}; dialog.show(); QApplication::processEvents();
    auto* calculate=dialog.findChild<QPushButton*>("estimateExtrinsics"); ASSERT_TRUE(calculate);
    calculate->click(); EXPECT_EQ(f.solver->calls,0);
    auto* points=dialog.findChild<QPlainTextEdit*>("extrinsicReferences");
    auto* mounts=dialog.findChild<QPlainTextEdit*>("robotMounts");
    auto* confirm=dialog.findChild<QCheckBox*>("surveyMeasurementsConfirmed");
    ASSERT_TRUE(points && mounts && confirm);
    points->setPlainText("Fit wrong row"); confirm->setChecked(true); calculate->click(); EXPECT_EQ(f.solver->calls,0);
    QString tooMany;
    for(std::size_t i=0;i<MaximumExtrinsicReferences+1;++i) tooMany+="Fit 1 2 3 4\n";
    points->setPlainText(tooMany); EXPECT_EQ(points->toPlainText(),tooMany); // Never silently discard pasted rows.
    confirm->setChecked(true); calculate->click(); EXPECT_EQ(f.solver->calls,0);
    EXPECT_FALSE(f.controller.busy());
    dialog.resize(640,480); QApplication::processEvents();
    const auto screenshot=qEnvironmentVariable("MICRO_ROBOT_ELEVATED_SCREENSHOT");
    if(!screenshot.isEmpty()) EXPECT_TRUE(dialog.grab().save(screenshot));
}
TEST(ElevatedLocalizationUiTest, V4UpgradeResaveReloadLeavesOriginalV3AcquisitionBytesUntouched) {
    QTemporaryDir directory; ASSERT_TRUE(directory.isValid());
    auto p=elevatedProfile(); auto e=*p.localization; p.localization.reset(); p.acquisitionId="original-v3-evidence";
    WorkspaceAcquisitionRecord record{p.acquisitionId,p,{},{}};
    std::uint64_t id=1;
    // Acquisition records keep their original surface's uncorrected tabletop semantics.
    for(const auto& ref:e.survey.references) {
        const auto mapped=projectWorkspacePoint(*p.homography,ref.image); ASSERT_TRUE(mapped);
        record.references.push_back({id++,{ref.image,*mapped},ref.check,false,{}});
    }
    QtWorkspaceProfileStore store{directory.path()}; ASSERT_TRUE(store.saveAcquisition(p,record).empty());
    const auto files=QDir(directory.path()).entryList({"*.acquisition"},QDir::Files); ASSERT_EQ(files.size(),1);
    auto bytes=[&] { QFile file{directory.filePath(files[0])}; EXPECT_TRUE(file.open(QIODevice::ReadOnly)); return file.readAll(); };
    const auto original=bytes();
    e.tabletopBinding=tabletopProfileBinding(p); p.localization=e;
    ASSERT_TRUE(store.saveAcquisition(p,record).empty()); EXPECT_EQ(bytes(),original);
    const auto loaded=store.load(p.workspaceId,p.intrinsics); ASSERT_TRUE(loaded.profile)<<loaded.error;
    ASSERT_TRUE(loaded.profile->localization); ASSERT_TRUE(loaded.acquisition); EXPECT_FALSE(loaded.acquisition->profile.localization);
    EXPECT_EQ(loaded.acquisition->id,p.acquisitionId);
    auto changed=p; changed.localization->mounts[0].revision="new-mount"; changed.localization->mounts[0].tagHeightMeters=.07;
    EXPECT_TRUE(store.save(changed).empty()); EXPECT_EQ(bytes(),original);
}

TEST(ElevatedLocalizationUiTest, ChangedMountRequiresNewMountAndCalibrationRevisions) {
    LocalFixture f;
    f.controller.calculateLocalization(f.input()); ASSERT_TRUE(QTest::qWaitFor([&]{return !f.controller.busy();})); f.controller.activate();
    f.controller.localizationInputsChanged(); auto changed=f.input("changed-r3"); changed.mounts[0].tagHeightMeters=.07;
    f.controller.calculateLocalization(changed); EXPECT_FALSE(f.controller.busy()); EXPECT_EQ(f.solver->calls,1);
    EXPECT_TRUE(f.controller.message().contains("mounting revision"));
    changed.mounts[0].revision="mount-r2";
    f.controller.calculateLocalization(changed); ASSERT_TRUE(QTest::qWaitFor([&]{return !f.controller.busy();}));
    ASSERT_TRUE(f.controller.draftProfile()->localization); EXPECT_EQ(f.solver->calls,2);
    EXPECT_EQ(f.controller.draftProfile()->localization->mounts[0].tagHeightMeters,.07);
    EXPECT_EQ(f.controller.profile()->localization->mounts[0].tagHeightMeters,.05);
}
namespace {
class RevisionGateDetector final:public IMarkerDetector {
public:
    std::shared_future<void> gate; std::atomic<int> calls{};
    MarkerDetections detect(const Frame&) override { ++calls; if(gate.valid()) gate.wait(); return {}; }
};
class RevisionMetricConverter final:public IMetricObservationConverter {
public:
    MetricObservationBatch convert(const MarkerDetections& d,const std::string&,const RobotMarkerMap&,
        const std::optional<WorkspaceCalibrationProfile>& p) override {
        RobotObservation o{}; o.frameSequence=d.frameSequence; o.capturedAt=d.capturedAt;
        auto result=metricObservation(o,elevatedCorners(0,0,.05,0),p);
        MetricObservationBatch batch{result.status,{},{}};
        if(result.measurement) batch.measurements.push_back(*result.measurement);
        return batch;
    }
};
}
TEST(ElevatedLocalizationUiTest, MountReplacementClearsTrackingAndDropsOldWorkerRevisionBeforePublication) {
    using namespace std::chrono_literals;
    Logger log; WorldModel world; RobotTracker tracker{world,{500ms,2s}};
    Timestamp now=Timestamp{}+10s;
    auto detector=std::make_unique<RevisionGateDetector>(); auto* gateDetector=detector.get();
    std::promise<void> release; gateDetector->gate=release.get_future().share();
    VisionController vision{std::move(detector),log.channel("revision")};
    struct ReleaseOnExit { std::promise<void>& p; bool done{}; ~ReleaseOnExit(){if(!done) p.set_value();} } guard{release};
    vision.setMetricConverter(std::make_unique<RevisionMetricConverter>());
    auto p=elevatedProfile(); vision.setWorkspaceCalibration(p); vision.setInput(QString::fromStdString(p.intrinsics.sourceId),true);
    QObject context; int published=0; std::string revision;
    connectVisionTracking(vision,tracker,context,[]{},[&]{return now;});
    QObject::connect(&vision,&VisionController::metricObservationsAvailable,&context,[&](const QString&,MetricObservationBatchPtr b) {
        if(b && !b->measurements.empty()) { ++published; revision=b->measurements[0].mountingRevision; }
    });
    ASSERT_EQ(tracker.observe(RobotId{0},{Meters{.1},Meters{.1},Radians{0}},now+10ms,now+10ms),TrackerUpdate::Applied);
    auto frame=[&](std::uint64_t seq) {
        auto f=std::make_shared<Frame>(); f->sequence=seq; f->capturedAt=now;
        f->width=p.intrinsics.width; f->height=p.intrinsics.height; f->strideBytes=f->width; f->pixelFormat=PixelFormat::Gray8;
        f->pixels=std::make_shared<const std::vector<std::byte>>(static_cast<std::size_t>(f->width)*f->height); return f;
    };
    now+=20ms; vision.receiveFrame(frame(1)); ASSERT_TRUE(QTest::qWaitFor([&]{return gateDetector->calls==1;}));
    p.localization->revision="new-calibration"; p.localization->mounts[0].revision="new-mount"; p.localization->mounts[0].tagHeightMeters=.07;
    now+=20ms; vision.setWorkspaceCalibration(p);
    EXPECT_FALSE(world.robot(RobotId{0})->pose); EXPECT_EQ(world.tracking(RobotId{0},now).status,TrackingStatus::Unknown);
    guard.done=true; release.set_value(); ASSERT_TRUE(QTest::qWaitFor([&]{return !vision.busy();})); EXPECT_EQ(published,0);
    now+=20ms; vision.receiveFrame(frame(2)); ASSERT_TRUE(QTest::qWaitFor([&]{return !vision.busy();}));
    EXPECT_EQ(published,1); EXPECT_EQ(revision,"new-mount"); EXPECT_EQ(world.tracking(RobotId{0},now).status,TrackingStatus::Fresh);
    vision.shutdown(); ASSERT_TRUE(QTest::qWaitFor([&]{return vision.isStopped();}));
}

TEST(ElevatedLocalizationUiTest, VerifyLoadedV4PreservesBodyGeometryAndOriginalAcquisition) {
    LocalFixture f; auto p=elevatedProfile(); auto geometry=*p.localization;
    p.localization.reset(); p.acquisitionId="verification-evidence";
    WorkspaceBoundary boundary;
    const auto corners=workspaceCorners(p.bounds);
    auto record=std::make_shared<WorkspaceAcquisitionRecord>(WorkspaceAcquisitionRecord{p.acquisitionId,p,{},{}});
    for(std::size_t i=0;i<corners.size();++i) {
        const auto pixel=unprojectWorkspacePoint(*p.homography,corners[i]); ASSERT_TRUE(pixel);
        boundary[i]=*pixel; record->references.push_back({i+1,{*pixel,corners[i]},false,true,{}});
    }
    p.boundary=boundary; record->profile=p;
    std::uint64_t id=5;
    for(const auto& ref:geometry.survey.references) if(ref.check)
        record->references.push_back({id++,{ref.image,*projectWorkspacePoint(*p.homography,ref.image)},true,false,{}});
    geometry.tabletopBinding=tabletopProfileBinding(p); p.localization=geometry;
    f.store->stored=p; f.store->acquisition=record;
    f.controller.load(); ASSERT_TRUE(QTest::qWaitFor([&]{return !f.controller.busy();}));
    ASSERT_TRUE(f.controller.draftProfile())<<f.controller.message().toStdString(); f.controller.activate();
    auto frame=std::make_shared<Frame>(); frame->width=p.intrinsics.width; frame->height=p.intrinsics.height;
    frame->strideBytes=frame->width; frame->pixelFormat=PixelFormat::Gray8; frame->sequence=1; frame->capturedAt=MonotonicClock::now();
    frame->pixels=std::make_shared<const std::vector<std::byte>>(static_cast<std::size_t>(frame->width)*frame->height);
    f.controller.receiveFrame(frame); f.controller.freezeBoard(); ASSERT_TRUE(QTest::qWaitFor([&]{return !f.controller.busy();}));
    ASSERT_TRUE(f.controller.canVerify()); const auto original=serializeWorkspaceAcquisition(*record);
    f.controller.verify(); ASSERT_TRUE(QTest::qWaitFor([&]{return !f.controller.busy();}));
    EXPECT_TRUE(f.controller.message().startsWith("Independent check RMS"))<<f.controller.message().toStdString();
    EXPECT_EQ(serializeWorkspaceProfile(*f.controller.draftProfile()),serializeWorkspaceProfile(p));
    EXPECT_EQ(serializeWorkspaceProfile(*f.controller.profile()),serializeWorkspaceProfile(p));
    EXPECT_EQ(serializeWorkspaceAcquisition(*f.controller.draftAcquisition()),original);
}
