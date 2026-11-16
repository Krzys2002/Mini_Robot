#include "app/WorkspaceCalibrationController.hpp"
#include <QMetaObject>
#include <QUuid>
#include <algorithm>
#include <cmath>
#include <limits>

namespace micro_robot {
void WorkspaceCalibrationController::clearBoardReview()
{
    ++boardRevision_;
    detectedBoard_.reset(); boardReview_.reset(); boardPoints_.clear(); boardAssessment_.clear();
}
void WorkspaceCalibrationController::setAcquisitionDetector(std::unique_ptr<ICalibrationDetector> detector)
{
    if (shuttingDown_) return;
    ++generation_; clearBoardReview(); detector_=std::move(detector); emit changed();
}
std::shared_ptr<WorkspaceAcquisitionRecord> WorkspaceCalibrationController::acquisitionSnapshot(
    const WorkspaceCalibrationProfile& profile) const
{
    return std::make_shared<WorkspaceAcquisitionRecord>(WorkspaceAcquisitionRecord{
        profile.acquisitionId,profile,references_,placements_});
}
void WorkspaceCalibrationController::synchronizeReferences()
{
    references_.clear(); placements_.clear(); nextReferenceId_=1; nextPlacementId_=1;
    for (std::size_t i=0;i<fit_.size();++i)
        references_.push_back({nextReferenceId_++,fit_[i],false,boundaryConfirmed_ && i<WorkspaceCornerCount,{}});
    for (const auto& p:check_) references_.push_back({nextReferenceId_++,p,true,false,{}});
    ++referencesRevision_;
    if (draft_ && boundaryConfirmed_) {
        draft_->acquisitionId=QUuid::createUuid().toString(QUuid::WithoutBraces).toStdString(); draftAcquisition_.reset();
    }
}
void WorkspaceCalibrationController::referencesEdited()
{
    std::vector<WorkspaceCorrespondence> fit,check;
    for (const auto& r:references_) (r.check ? check : fit).push_back(r.point);
    const bool sameFit=fit.size()==fit_.size() && std::equal(fit.begin(),fit.end(),fit_.begin(),[](const auto& a,const auto& b) {
        return a.image.u==b.image.u && a.image.v==b.image.v && a.world.x.value==b.world.x.value && a.world.y.value==b.world.y.value;
    });
    ++generation_; ++referencesRevision_;
    fit_=std::move(fit); check_=std::move(check);
    if (!sameFit) { draft_.reset(); draftAcquisition_.reset(); }
    else if (draft_ && boundaryConfirmed_) {
        draft_->acquisitionId=QUuid::createUuid().toString(QUuid::WithoutBraces).toStdString(); draftAcquisition_.reset();
    }
    // A reviewed detection can be used again only after reassessing conflicts.
    ++boardRevision_;
    boardReview_.reset(); boardPoints_.clear(); boardAssessment_.clear();
    publish(boundaryConfirmed_
        ? QStringLiteral("References changed. Fit edits invalidate draft H; Check edits require renewed verification. Save records this reviewed acquisition.")
        : QStringLiteral("Verification-only references are session diagnostics. Save retains the original transform/acquisition. Use the complete dataset and Freeze next board frame to record new acquisition checks."));
}
void WorkspaceCalibrationController::editReference(std::size_t index, WorkspaceCorrespondence point, bool check)
{
    if (shuttingDown_ || busy_ || index>=references_.size() || references_[index].boundary) return;
    if (check!=references_[index].check && ((check && check_.size()>=MaximumWorkspaceCheckPoints)
        || (!check && fit_.size()>=MaximumWorkspaceFitPoints))) {
        publish(QStringLiteral("Role change rejected: 4096-reference role capacity reached.")); return;
    }
    references_[index].point=point; references_[index].check=check; referencesEdited();
}
void WorkspaceCalibrationController::appendReference(std::optional<ImageCorner> point)
{
    if (nextReferenceId_==std::numeric_limits<std::uint64_t>::max() || shuttingDown_ || busy_ || !frozen_ || !canAddReferences() || references_.size()>=MaximumWorkspaceReferences) return;
    const bool check=draft_.has_value();
    if ((check && check_.size()>=MaximumWorkspaceCheckPoints) || (!check && fit_.size()>=MaximumWorkspaceFitPoints)) {
        publish(QStringLiteral("Reference role capacity reached (4096 each). No row was added.")); return;
    }
    const auto nan=std::numeric_limits<double>::quiet_NaN();
    WorkspaceCorrespondence pair{point.value_or(ImageCorner{nan,nan}),{Meters{nan},Meters{nan}}};
    references_.push_back({nextReferenceId_++,pair,check,false,{}}); referencesEdited();
}
void WorkspaceCalibrationController::removeReference(std::size_t index)
{
    if (shuttingDown_ || busy_ || index>=references_.size() || references_[index].boundary) return;
    if (const auto source=references_[index].source)
        for (auto& p:placements_) if (p.id==source->placementId) p.included[source->cornerIndex]=false;
    references_.erase(references_.begin()+static_cast<std::ptrdiff_t>(index)); referencesEdited();
}
void WorkspaceCalibrationController::detectBoard()
{
    if (!canDetectBoard()) return;
    clearBoardReview();
    const auto frame=frozen_;
    const auto detector=detector_;
    const auto generation=++generation_; busy_=true;
    publish(QStringLiteral("Detecting the shared flat chessboard on this exact frozen undistorted frame…"));
    QMetaObject::invokeMethod(workerContext_,[this,frame,detector,generation] {
        CalibrationDetection result;
        try { result=detector->detect(*frame,WorkspaceBoardTarget); }
        catch (const std::exception& e) { result.error=e.what(); }
        catch (...) { result.error="Unknown board detection failure."; }
        QMetaObject::invokeMethod(this,[this,frame,generation,result=std::move(result)] {
            busy_=false;
            if (shuttingDown_) return;
            if (generation!=generation_ || frozen_!=frame) { emit changed(); return; }
            if (!result.error.empty() || result.corners.size()!=static_cast<std::size_t>(WorkspaceBoardTarget.columns*WorkspaceBoardTarget.rows)
                || !std::all_of(result.corners.begin(),result.corners.end(),[&](const auto& p) {
                    return std::isfinite(p.u) && std::isfinite(p.v) && p.u>=0 && p.v>=0 && p.u<=frame->width-1 && p.v<=frame->height-1;
                })) {
                publish(QStringLiteral("Complete board detection unavailable: %1. Keep the board flat on the table, visible and in focus.")
                    .arg(QString::fromStdString(result.error))); return;
            }
            WorkspaceBoardPlacement p; p.detected=std::move(result.corners); p.included.assign(p.detected.size(),true);
            p.frameSequence=frame->sequence;
            p.capturedTicksNanoseconds=std::chrono::duration_cast<std::chrono::nanoseconds>(frame->capturedAt.time_since_epoch()).count();
            detectedBoard_=std::move(p); ++boardRevision_;
            publish(QStringLiteral("54 exact-frame corners detected. Set measured anchor, orientation and ordering; Review before import. Never infer a new board displacement."));
        },Qt::QueuedConnection);
    },Qt::QueuedConnection);
}
void WorkspaceCalibrationController::discardBoardReview()
{
    if (shuttingDown_) return;
    ++generation_; ++boardRevision_;
    boardReview_.reset(); boardPoints_.clear(); boardAssessment_.clear(); emit changed();
}
void WorkspaceCalibrationController::reviewBoardPlacement(WorkspaceBoardPlacement settings)
{
    if (shuttingDown_ || busy_ || !detectedBoard_ || !frozen_ || !boundaryConfirmed_ || !bounds_ || !intrinsics_) return;
    settings.id=nextPlacementId_; settings.detected=detectedBoard_->detected;
    settings.frameSequence=detectedBoard_->frameSequence; settings.capturedTicksNanoseconds=detectedBoard_->capturedTicksNanoseconds;
    settings.included.assign(settings.detected.size(),true);
    if (settings.relativeReferenceId) {
        const auto r=std::find_if(references_.begin(),references_.end(),[&](const auto& r){return r.id==*settings.relativeReferenceId;});
        if (r==references_.end()) { publish(QStringLiteral("Select an existing measured reference for the offset.")); return; }
        settings.relativeReferenceSnapshot=r->point.world;
        settings.anchor={Meters{r->point.world.x.value+settings.measuredOffset.x.value},Meters{r->point.world.y.value+settings.measuredOffset.y.value}};
    }
    boardReview_.reset(); boardPoints_.clear(); boardAssessment_.clear();
    WorkspaceCalibrationProfile profile{WorkspaceCalibrationProfileVersion,workspaceId_.toStdString(),*bounds_,*intrinsics_};
    std::vector<WorkspaceCorrespondence> existing=fit_; existing.insert(existing.end(),check_.begin(),check_.end());
    const auto generation=++generation_; busy_=true;
    publish(QStringLiteral("Reviewing placement geometry and all existing-reference conflicts…"));
    QMetaObject::invokeMethod(workerContext_,[this,settings=std::move(settings),profile,existing=std::move(existing),generation] () mutable {
        WorkspaceBoardGeneration generated;
        WorkspaceReferenceAssessment assessment;
        try {
            generated=generateWorkspaceBoard(settings,profile.bounds);
            if (generated.error.empty()) assessment=assessWorkspaceReferences(profile,existing,generated.points);
        } catch (const std::exception& e) { generated.error=e.what(); }
        catch (...) { generated.error="Unknown placement validation failure."; }
        QMetaObject::invokeMethod(this,[this,settings=std::move(settings),generated=std::move(generated),assessment=std::move(assessment),generation] () mutable {
            busy_=false; if (shuttingDown_) return;
            if (generation!=generation_) { emit changed(); return; }
            const auto error=generated.error.empty() ? assessment.error : generated.error;
            if (!error.empty()) { publish(QString::fromStdString(error)); return; }
            for (std::size_t i=0;i<settings.included.size();++i) settings.included[i]=assessment.candidates[i]==WorkspaceReferenceRejection::None;
            settings.reviewRejections=assessment.candidates;
            const auto included=std::count(settings.included.begin(),settings.included.end(),true);
            ++boardRevision_;
            boardReview_=std::move(settings); boardPoints_=std::move(generated.points); boardAssessment_=std::move(assessment.candidates);
            publish(QStringLiteral("Review %1 new points; %2 outside/duplicate points explicitly excluded. Confirm physical anchor/order/+columns/+rows and inclusion mask; no cap truncation is performed.")
                .arg(included).arg(boardPoints_.size()-static_cast<std::size_t>(included)));
        },Qt::QueuedConnection);
    },Qt::QueuedConnection);
}
void WorkspaceCalibrationController::setBoardPointIncluded(std::size_t index, bool included)
{
    if (!canImportBoard() || index>=boardReview_->included.size()) return;
    if (included && boardAssessment_[index]!=WorkspaceReferenceRejection::None) return;
    boardReview_->included[index]=included; emit changed();
}
void WorkspaceCalibrationController::importBoardPlacement(bool check, bool physicalIdentityConfirmed)
{
    if (!canImportBoard()) return;
    if (!physicalIdentityConfirmed) { publish(QStringLiteral("Explicitly confirm the physical anchor, ordering, axis arrows and inclusion mask before import.")); return; }
    const auto count=static_cast<std::size_t>(std::count(boardReview_->included.begin(),boardReview_->included.end(),true));
    if (nextReferenceId_>std::numeric_limits<std::uint64_t>::max()-count || nextPlacementId_==std::numeric_limits<std::uint64_t>::max()
        || !count || placements_.size()>=MaximumWorkspacePlacements || references_.size()+count>MaximumWorkspaceReferences
        || (check ? check_.size()+count>MaximumWorkspaceCheckPoints : fit_.size()+count>MaximumWorkspaceFitPoints)) {
        publish(QStringLiteral("Import rejected atomically: use at most 4096 Fit, 4096 Check references and 128 placements. No observations were truncated; revise the mask or dataset.")); return;
    }
    auto placement=*boardReview_;
    for (std::size_t i=0;i<boardPoints_.size();++i) if (placement.included[i])
        references_.push_back({nextReferenceId_++,boardPoints_[i],check,false,WorkspaceBoardSource{placement.id,i}});
    placements_.push_back(std::move(placement)); ++nextPlacementId_;
    referencesEdited();
    publish(QStringLiteral("Placement imported with measured anchor/orientation and original exact-frame detections. Freeze next board frame to retain this dataset; Calculate before saving/activation."));
}
} // namespace micro_robot
