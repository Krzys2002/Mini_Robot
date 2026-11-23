#include "../calibration/ReferenceFixtures.hpp"
#include "app/WorkspaceCalibrationController.hpp"
#include "app/VisionTracking.hpp"
#include "calibration/OpenCvExtrinsicsSolver.hpp"
#include "calibration/QtWorkspaceProfileStore.hpp"
#include "vision/OpenCvMetricObservation.hpp"
#include "ui/ReferenceMarkersDialog.hpp"
#include <QApplication>
#include <QCheckBox>
#include <QFile>
#include <QDir>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QTemporaryDir>
#include <QTest>
#include <atomic>
#include <algorithm>
#include <future>
#include <gtest/gtest.h>
using namespace micro_robot;
using namespace micro_robot::test;
using namespace std::chrono_literals;
namespace {
class ReferenceNoopHomography final:public IWorkspaceHomographySolver {
    WorkspaceHomographyOutcome solve(const WorkspaceHomographyInput&) override { return {{},"unused"}; }
};
class ReferenceNoopUndistorter final:public ICalibrationUndistorter {
    UndistortedPreviewOutcome undistort(const Frame&,const IntrinsicCalibration&) override { return {{},"unused"}; }
};
class ReferenceDetector final:public IMarkerDetector {
public:
    WorkspaceCalibrationProfile geometry=referenceProfile();
    std::shared_future<void> gate; std::atomic<int> calls{}; std::atomic<double> cameraShift{}; std::atomic<bool> corruptChecks{};
    MarkerDetections detect(const Frame& input) override {
        ++calls; if(gate.valid()) gate.wait();
        auto actual=*geometry.localization; actual.translationMeters[0]+=cameraShift.load();
        auto f=referenceFrame(geometry,input.capturedAt,&actual);
        MarkerDetections result;
        for(const auto& row:f.observations) {
            auto corners=row.raw;
            if(corruptChecks && std::any_of(geometry.references->markers.begin(),geometry.references->markers.end(),[&](const auto& m){return m.id==row.id && m.role==ReferenceRole::IndependentCheck;}))
                for(auto& corner:corners) corner.u+=20;
            result.markers.push_back({MarkerFamily::AprilTag36h11,row.id,corners,{}});
        }
        std::array<MarkerCorner,4> corners;
        const double front[]{1,1,-1,-1},right[]{-1,1,1,-1};
        for(std::size_t i=0;i<4;++i) {
            auto pixel=projectReference(actual,geometry.intrinsics,{Meters{.016*front[i]},Meters{-.016*right[i]}},.05);
            corners[i]={pixel->u,pixel->v};
        }
        result.markers.push_back({MarkerFamily::AprilTag36h11,0,corners,{}});
        return result;
    }
};
struct ReferenceWorkflow {
    Logger log; QTemporaryDir directory;
    WorkspaceCalibrationProfile original=referenceProfile();
    std::unique_ptr<WorkspaceCalibrationController> workspace;
    std::unique_ptr<VisionController> vision;
    ReferenceDetector* detector{};
    WorldModel world; RobotTracker tracker{world,{300ms,2s}};
    std::promise<void> release; bool released{};
    std::uint64_t sequence{};
    std::vector<MetricObservationBatchPtr> batches;
    ReferenceWorkflow() {
        original.references.reset(); original.localization->mounts[0].robotId=RobotId{1};
        QtWorkspaceProfileStore store{directory.path()}; EXPECT_TRUE(store.save(original).empty());
        workspace=std::make_unique<WorkspaceCalibrationController>(std::make_unique<ReferenceNoopHomography>(),std::make_unique<ReferenceNoopUndistorter>(),
            std::make_unique<QtWorkspaceProfileStore>(directory.path()),log.channel("references"));
        workspace->setExtrinsicsSolver(std::make_unique<OpenCvExtrinsicsSolver>());
        workspace->setInput(original.intrinsics,true); workspace->configure("elevated-table",1,.5,.5,.25); workspace->load();
        EXPECT_TRUE(QTest::qWaitFor([&]{return !workspace->busy();})); EXPECT_TRUE(workspace->draftProfile());
        auto d=std::make_unique<ReferenceDetector>(); detector=d.get();
        vision=std::make_unique<VisionController>(std::move(d),log.channel("vision"));
        vision->setMetricConverter(std::make_unique<OpenCvMetricObservationConverter>()); vision->setInput(QString::fromStdString(original.intrinsics.sourceId),true);
        QObject::connect(workspace.get(),&WorkspaceCalibrationController::changed,vision.get(),[this]{vision->setReferenceDraft(workspace->draftProfile());});
        QObject::connect(vision.get(),&VisionController::referenceObservationsAvailable,workspace.get(),&WorkspaceCalibrationController::receiveReferenceFrame);
        QObject::connect(workspace.get(),&WorkspaceCalibrationController::profileChanged,vision.get(),[this]{vision->setWorkspaceCalibration(workspace->profile());});
        QObject::connect(vision.get(),&VisionController::metricObservationsAvailable,vision.get(),[this](const QString&,MetricObservationBatchPtr b){if(b) batches.push_back(b);});
        connectVisionTracking(*vision,tracker,*workspace,[]{});
        workspace->configureReferences(*referenceProfile().references);
    }
    void frame() {
        auto f=std::make_shared<Frame>(); f->sequence=++sequence; f->capturedAt=MonotonicClock::now(); f->width=original.intrinsics.width; f->height=original.intrinsics.height;
        f->strideBytes=f->width; f->pixelFormat=PixelFormat::Gray8; f->pixels=std::make_shared<const std::vector<std::byte>>(static_cast<std::size_t>(f->width)*f->height);
        vision->receiveFrame(f);
    }
    void commission() {
        frame(); ASSERT_TRUE(QTest::qWaitFor([&]{return bool(workspace->latestReferenceFrame());}))<<workspace->message().toStdString();
        workspace->calculateReferences(); ASSERT_TRUE(QTest::qWaitFor([&]{return !workspace->busy();}));
        ASSERT_TRUE(workspace->canActivate())<<workspace->message().toStdString();
    }
    void unblock() {if(!released){released=true;release.set_value();}}
    ~ReferenceWorkflow() {unblock();vision->shutdown();workspace->shutdown();(void)QTest::qWaitFor([&]{return vision->isStopped()&&workspace->isStopped();});}
};
}
TEST(ReferenceWorkflowTest, DetectorAcquisitionMixedHeightSolveExplicitActivationAndRealStoreRoundtrip) {
    ReferenceWorkflow f; EXPECT_FALSE(f.workspace->canActivate()); f.commission();
    ASSERT_TRUE(f.workspace->draftProfile()->references->acquisition);
    EXPECT_FALSE(f.workspace->profile());
    const auto acquired=f.workspace->draftProfile()->references->acquisition;
    const auto inputRevision=acquired->inputCalibrationRevision;
    f.workspace->activate(); ASSERT_TRUE(f.workspace->profile());
    f.workspace->save(); ASSERT_TRUE(QTest::qWaitFor([&]{return !f.workspace->busy();}));
    QtWorkspaceProfileStore store{f.directory.path()}; const auto restored=store.load(f.original.workspaceId,f.original.intrinsics);
    ASSERT_TRUE(restored.profile)<<restored.error; ASSERT_TRUE(restored.profile->references);
    EXPECT_EQ(restored.profile->references->acquisition->inputCalibrationRevision,inputRevision);
    EXPECT_EQ(serializeWorkspaceProfile(*restored.profile),serializeWorkspaceProfile(*f.workspace->profile()));
    f.frame(); ASSERT_TRUE(QTest::qWaitFor([&]{return !f.vision->busy();}));
    ASSERT_FALSE(f.batches.empty()); EXPECT_EQ(f.batches.back()->measurements.size(),1U)<<f.batches.back()->error;
    EXPECT_EQ(f.world.robotCount(),1U); EXPECT_TRUE(f.world.robot(RobotId{1})); EXPECT_FALSE(f.world.robot(RobotId{100}));
    const auto bounds=f.workspace->profile()->bounds;
    EXPECT_DOUBLE_EQ(bounds.width().value,1); EXPECT_DOUBLE_EQ(bounds.height().value,.5);
}
TEST(ReferenceWorkflowTest, MonitoringExpiryInvalidatesTrackingEvenWithoutAnotherCameraFrame) {
    ReferenceWorkflow f; f.commission(); f.workspace->activate(); f.frame(); ASSERT_TRUE(QTest::qWaitFor([&]{return !f.vision->busy();}));
    ASSERT_TRUE(f.world.robot(RobotId{1})); EXPECT_EQ(f.world.tracking(RobotId{1},MonotonicClock::now()).status,TrackingStatus::Fresh);
    ASSERT_TRUE(QTest::qWaitFor([&]{return !f.batches.empty() && f.batches.back()->status==MetricObservationStatus::PlacementUnverified;},2000));
    EXPECT_NE(f.batches.back()->placement->message.find("expired"),std::string::npos);
    EXPECT_TRUE(f.batches.back()->measurements.empty()); EXPECT_NE(f.world.tracking(RobotId{1},MonotonicClock::now()).status,TrackingStatus::Fresh);
    EXPECT_TRUE(f.vision->description().contains("expired"));
}
TEST(ReferenceWorkflowTest, ConfirmedDriftCannotRecoverUntilNewReferenceSolveAndExplicitActivation) {
    ReferenceWorkflow f; f.commission(); f.workspace->activate(); f.frame(); ASSERT_TRUE(QTest::qWaitFor([&]{return !f.vision->busy();}));
    const auto oldRevision=f.workspace->profile()->localization->revision;
    f.detector->cameraShift=.012;
    for(int i=0;i<3;++i) { QTest::qWait(210); f.frame(); ASSERT_TRUE(QTest::qWaitFor([&]{return !f.vision->busy();})); }
    ASSERT_EQ(f.batches.back()->status,MetricObservationStatus::PlacementDrift)<<f.batches.back()->error;
    EXPECT_EQ(f.workspace->profile()->localization->revision,oldRevision);
    ASSERT_TRUE(f.workspace->latestReferenceFrame());
    f.workspace->calculateReferences(); ASSERT_TRUE(QTest::qWaitFor([&]{return !f.workspace->busy();}));
    ASSERT_TRUE(f.workspace->canActivate())<<f.workspace->message().toStdString();
    EXPECT_EQ(f.workspace->profile()->localization->revision,oldRevision);
    f.frame(); ASSERT_TRUE(QTest::qWaitFor([&]{return !f.vision->busy();})); EXPECT_EQ(f.batches.back()->status,MetricObservationStatus::PlacementDrift);
    f.workspace->activate(); QTest::qWait(30); f.frame(); ASSERT_TRUE(QTest::qWaitFor([&]{return !f.vision->busy();}));
    EXPECT_EQ(f.batches.back()->status,MetricObservationStatus::Valid)<<f.batches.back()->error;
    EXPECT_EQ(f.batches.back()->measurements.size(),1U);
    EXPECT_NE(f.workspace->profile()->localization->revision,oldRevision);
}
TEST(ReferenceWorkflowTest, RegistryEditFencesInFlightSnapshotAndCoalescesToNewestPendingFrame) {
    ReferenceWorkflow f; f.detector->gate=f.release.get_future().share(); f.frame();
    ASSERT_TRUE(QTest::qWaitFor([&]{return f.detector->calls==1;}));
    auto c=*f.workspace->draftProfile()->references; c.policy.cadence=300ms; f.workspace->configureReferences(c);
    const auto revision=f.workspace->draftProfile()->references->revision;
    f.frame(); f.frame(); f.frame(); f.unblock();
    ASSERT_TRUE(QTest::qWaitFor([&]{return !f.vision->busy();}));
    ASSERT_TRUE(f.workspace->latestReferenceFrame()); EXPECT_EQ(f.workspace->latestReferenceFrame()->registryRevision,revision);
    EXPECT_EQ(f.workspace->latestReferenceFrame()->frameSequence,4U); EXPECT_EQ(f.detector->calls,2);
    auto changed=c; changed.markers[0].surfaceHeightMeters+=.001;
    f.workspace->configureReferences(changed); EXPECT_EQ(f.workspace->draftProfile()->references->revision,revision);
    EXPECT_TRUE(f.workspace->message().contains("revisions"));
}
TEST(ReferenceWorkflowTest, RegistryEditorRejectsUnconfirmedAndOversizedPasteAndRemainsScrollable) {
    ReferenceWorkflow f; ReferenceMarkersDialog dialog{*f.workspace}; dialog.show(); QApplication::processEvents();
    auto* apply=dialog.findChild<QPushButton*>("applyReferenceRegistry"); auto* rows=dialog.findChild<QPlainTextEdit*>("referenceRegistryRows");
    auto* confirmed=dialog.findChild<QCheckBox*>("referenceSurveyConfirmed"); ASSERT_TRUE(apply&&rows&&confirmed);
    const auto revision=f.workspace->draftProfile()->references->revision; apply->click(); EXPECT_EQ(f.workspace->draftProfile()->references->revision,revision);
    const auto line=rows->toPlainText().section('\n',0,0); QString over;
    for(std::size_t i=0;i<MaximumReferenceMarkers+1;++i) over+=line+'\n';
    rows->setPlainText(over); EXPECT_EQ(rows->toPlainText(),over); confirmed->setChecked(true); apply->click();
    EXPECT_EQ(f.workspace->draftProfile()->references->revision,revision);
    dialog.resize(640,480); QApplication::processEvents();
    const auto image=qEnvironmentVariable("MICRO_ROBOT_REFERENCE_SCREENSHOT"); if(!image.isEmpty()) EXPECT_TRUE(dialog.grab().save(image));
}

TEST(ReferenceWorkflowTest, GuidedFinishRequiresValidatedRegistryAndAcceptsIt)
{
    ReferenceWorkflow f;
    ReferenceMarkersDialog dialog{*f.workspace, nullptr, true};
    auto* finish = dialog.findChild<QPushButton*>("otherCalibrationFinish");
    auto* rows = dialog.findChild<QPlainTextEdit*>("referenceRegistryRows");
    ASSERT_TRUE(finish && rows);
    EXPECT_FALSE(finish->isEnabled());
    f.commission();
    ASSERT_TRUE(finish->isEnabled());
    const auto registry = rows->toPlainText();
    rows->setPlainText(registry + "new unreviewed row");
    EXPECT_FALSE(finish->isEnabled());
    EXPECT_FALSE(f.workspace->profile());
    rows->setPlainText(registry);
    EXPECT_TRUE(finish->isEnabled());
    finish->click();
    EXPECT_EQ(dialog.result(), QDialog::Accepted);
    ASSERT_TRUE(f.workspace->profile() && f.workspace->profile()->references);
    EXPECT_EQ(f.workspace->profile()->references->validatedLocalizationRevision,
              f.workspace->profile()->localization->revision);
}

TEST(ReferenceWorkflowTest, V5ReferenceUpgradePreservesImmutableV3SidecarByteForByte) {
    QTemporaryDir directory; auto runtime=referenceProfile(); auto mounts=runtime.localization;
    runtime.references.reset(); runtime.localization.reset(); runtime.acquisitionId="original-v3-sidecar";
    WorkspaceAcquisitionRecord record{runtime.acquisitionId,runtime,{},{}};
    std::uint64_t id=1;
    for(const auto& reference:mounts->survey.references) {
        const auto mapped=projectWorkspacePoint(*runtime.homography,reference.image); ASSERT_TRUE(mapped);
        record.references.push_back({id++,{reference.image,*mapped},reference.check,false,{}});
    }
    QtWorkspaceProfileStore store{directory.path()}; ASSERT_TRUE(store.saveAcquisition(runtime,record).empty());
    const auto files=QDir(directory.path()).entryList({"*.acquisition"},QDir::Files); ASSERT_EQ(files.size(),1);
    auto bytes=[&]{QFile f{directory.filePath(files[0])}; EXPECT_TRUE(f.open(QIODevice::ReadOnly)); return f.readAll();}; const auto original=bytes();
    runtime.localization=mounts; runtime.localization->tabletopBinding=tabletopProfileBinding(runtime);
    runtime.references=referenceProfile().references; runtime.references->tabletopBinding=tabletopProfileBinding(runtime);
    const auto frame=referenceFrame(runtime); std::string error;
    const auto input=referenceExtrinsicInput(frame,*runtime.references,runtime,"v5-commissioned",error); ASSERT_TRUE(input)<<error;
    runtime.localization->revision=input->revision; runtime.localization->survey=input->survey;
    runtime.references->acquisition=frame; runtime.references->validatedLocalizationRevision=runtime.localization->revision;
    ASSERT_TRUE(store.saveAcquisition(runtime,record).empty()); EXPECT_EQ(bytes(),original);
    const auto loaded=store.load(runtime.workspaceId,runtime.intrinsics); ASSERT_TRUE(loaded.profile)<<loaded.error;
    ASSERT_TRUE(loaded.acquisition); EXPECT_FALSE(loaded.acquisition->profile.references); EXPECT_FALSE(loaded.acquisition->profile.localization);
    EXPECT_EQ(serializeWorkspaceProfile(*loaded.profile),serializeWorkspaceProfile(runtime));
    EXPECT_TRUE(store.save(*loaded.profile).empty()); EXPECT_EQ(bytes(),original);
}

TEST(ReferenceWorkflowTest, ExpiryObserverInputChangeCannotRestoreObsoleteStatusOrReferenceSnapshot) {
    ReferenceWorkflow f; f.commission(); f.workspace->activate(); f.frame(); ASSERT_TRUE(QTest::qWaitFor([&]{return !f.vision->busy();}));
    bool switched=false;
    QObject::connect(f.vision.get(),&VisionController::metricObservationsAvailable,f.vision.get(),[&](const QString&,MetricObservationBatchPtr b) {
        if(b && b->status==MetricObservationStatus::PlacementUnverified) { switched=true; f.vision->setInput("replacement-source",false); }
    });
    ASSERT_TRUE(QTest::qWaitFor([&]{return switched;},2000));
    EXPECT_TRUE(f.vision->description().contains("inactive")); EXPECT_FALSE(f.workspace->latestReferenceFrame());
}

TEST(ReferenceWorkflowTest, RejectedFitPreservesRetryGeometryAndAcquisitionWithoutPermittingActivation) {
    ReferenceWorkflow f; f.detector->corruptChecks=true; f.frame();
    ASSERT_TRUE(QTest::qWaitFor([&]{return bool(f.workspace->latestReferenceFrame());}));
    const auto originalGeometry=f.workspace->draftProfile()->localization;
    f.workspace->calculateReferences(); ASSERT_TRUE(QTest::qWaitFor([&]{return !f.workspace->busy();}));
    ASSERT_TRUE(f.workspace->message().contains("rejected"))<<f.workspace->message().toStdString();
    ASSERT_TRUE(f.workspace->draftProfile()->localization)<<"Rejected fit must retain the acquisition/mount context for retry.";
    EXPECT_EQ(f.workspace->draftProfile()->localization->revision,originalGeometry->revision);
    EXPECT_FALSE(f.workspace->canActivate()); EXPECT_FALSE(f.workspace->canSave());
    EXPECT_FALSE(f.workspace->profile());
    const auto sequence=f.sequence; f.detector->corruptChecks=false; QTest::qWait(210); f.frame();
    ASSERT_TRUE(QTest::qWaitFor([&]{return f.workspace->latestReferenceFrame() && f.workspace->latestReferenceFrame()->frameSequence>sequence;}));
    f.workspace->calculateReferences(); ASSERT_TRUE(QTest::qWaitFor([&]{return !f.workspace->busy();}));
    ASSERT_TRUE(f.workspace->canActivate())<<f.workspace->message().toStdString(); EXPECT_TRUE(f.workspace->canSave());
    EXPECT_EQ(f.workspace->draftProfile()->references->validatedLocalizationRevision,f.workspace->draftProfile()->localization->revision);
    EXPECT_FALSE(f.workspace->profile()); // Successful retry still needs explicit Activate.
}
