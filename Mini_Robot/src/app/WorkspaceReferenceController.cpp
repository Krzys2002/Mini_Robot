#include "app/WorkspaceCalibrationController.hpp"
#include <QUuid>
namespace micro_robot {
void WorkspaceCalibrationController::configureReferences(ReferenceConfiguration c) {
    if(shuttingDown_ || busy_ || !draft_ || !draft_->localization) { publish(QStringLiteral("Load/calculate a measured height-aware draft before configuring references.")); return; }
    c.revision=QUuid::createUuid().toString(QUuid::WithoutBraces).toStdString();
    c.tabletopBinding=tabletopProfileBinding(*draft_); c.acquisition.reset(); c.validatedLocalizationRevision.clear();
    if(auto error=validateReferenceConfiguration(c,*draft_); !error.empty()) { publish(QString::fromStdString(error)); return; }
    if(draft_->references) for(const auto& before:draft_->references->markers) for(const auto& after:c.markers) {
        if(before.id!=after.id || before.family!=after.family) continue;
        bool placementChanged=before.surfaceHeightMeters!=after.surfaceHeightMeters;
        for(std::size_t i=0;i<4;++i) placementChanged=placementChanged || before.corners[i].x.value!=after.corners[i].x.value || before.corners[i].y.value!=after.corners[i].y.value;
        const bool surveyChanged=placementChanged || before.blackEdgeMeters!=after.blackEdgeMeters || before.standardUncertaintyMeters!=after.standardUncertaintyMeters || before.method!=after.method;
        if((placementChanged && before.installationRevision==after.installationRevision) || (surveyChanged && before.surveyRevision==after.surveyRevision)) {
            publish(QStringLiteral("Changed physical placement/survey data requires new installation/survey revisions.")); return;
        }
    }
    ++generation_; latestReferenceFrame_.reset(); pendingReferenceRevision_.clear();
    draft_->references=std::move(c);
    publish(QStringLiteral("Reference draft configured. Stream the matching camera; acquire distributed Fit/Check markers, then explicitly Fit references, Activate and Save."));
}
void WorkspaceCalibrationController::receiveReferenceFrame(ReferenceFramePtr frame) {
    if(shuttingDown_) return;
    if(!frame) { latestReferenceFrame_.reset(); return; }
    if(!draft_ || !draft_->references || frame->registryRevision!=draft_->references->revision) return;
    if(auto error=validateReferenceFrame(*frame,*draft_->references,*draft_); !error.empty()) { publish(QString::fromStdString(error)); return; }
    if(latestReferenceFrame_ && latestReferenceFrame_->capturedAt>=frame->capturedAt) return;
    latestReferenceFrame_=std::move(frame);
    publish(QStringLiteral("Reference frame %1 acquired with explicit per-marker inclusion/rejection rows. Review support, then Fit references.").arg(latestReferenceFrame_->frameSequence));
}
void WorkspaceCalibrationController::calculateReferences() {
    if(shuttingDown_ || busy_ || !draft_ || !draft_->references || !latestReferenceFrame_) { publish(QStringLiteral("Configure references and acquire a matching camera frame first.")); return; }
    const auto now=MonotonicClock::now();
    if(now<latestReferenceFrame_->capturedAt || now-latestReferenceFrame_->capturedAt>MaximumFrozenFrameAge) {
        publish(QStringLiteral("Reference acquisition expired; acquire a fresh matching frame.")); return;
    }
    auto c=*draft_->references; c.validatedLocalizationRevision.clear(); c.acquisition.reset();
    std::string error;
    const auto revision=QUuid::createUuid().toString(QUuid::WithoutBraces).toStdString();
    auto input=referenceExtrinsicInput(*latestReferenceFrame_,c,*draft_,revision,error);
    if(!input) { publish(QString::fromStdString(error)); return; }
    draft_->references->acquisition=*latestReferenceFrame_; draft_->references->validatedLocalizationRevision.clear();
    pendingReferenceRevision_=revision;
    calculateLocalization(std::move(*input));
}
} // namespace micro_robot
