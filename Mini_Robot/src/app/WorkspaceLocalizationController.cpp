#include "app/WorkspaceCalibrationController.hpp"
#include <QMetaObject>

namespace micro_robot {
void WorkspaceCalibrationController::setExtrinsicsSolver(std::unique_ptr<IExtrinsicsSolver> solver) {
    if (shuttingDown_) return;
    localizationInputsChanged();
    extrinsicsSolver_=std::move(solver);
}
void WorkspaceCalibrationController::localizationInputsChanged() {
    if (shuttingDown_) return;
    ++generation_;
    if (draft_ && draft_->localization) {
        lastLocalizationDraft_=draft_->localization;
        localizationValidationRequired_=true;
        draft_->localization.reset();
    }
    publish(QStringLiteral("Localization draft invalidated. Active calibration is unchanged until explicit Activate."));
}
void WorkspaceCalibrationController::calculateLocalization(ExtrinsicSolveInput input) {
    if (shuttingDown_ || busy_ || !draft_ || !draft_->homography || !extrinsicsSolver_) {
        publish(QStringLiteral("Load/calculate a tabletop draft and wait for the worker before estimating camera geometry."));
        return;
    }
    // A reused human mounting revision must never silently acquire different geometry.
    const auto sameMountGeometry=[](const RobotMount& a,const RobotMount& b) {
        return a.tagHeightMeters==b.tagHeightMeters && a.bodyForwardMeters==b.bodyForwardMeters
            && a.bodyLeftMeters==b.bodyLeftMeters && a.bodyHeadingOffsetRadians==b.bodyHeadingOffsetRadians
            && a.measurementMethod==b.measurementMethod;
    };
    for (const auto* old:{lastLocalizationDraft_ ? &*lastLocalizationDraft_ : nullptr,
                         profile_ && profile_->localization ? &*profile_->localization : nullptr,
                         draft_->localization ? &*draft_->localization : nullptr}) {
        if (!old) continue;
        if (old->revision==input.revision) {
            publish(QStringLiteral("Allocate a new calibration revision before re-estimating geometry.")); return;
        }
        for (const auto& before:old->mounts) for (const auto& after:input.mounts)
            if (before.robotId==after.robotId && before.revision==after.revision && !sameMountGeometry(before,after)) {
                publish(QStringLiteral("Changed mounting measurements require a new mounting revision.")); return;
            }
    }
    auto profile=*draft_; profile.localization.reset();
    profile.references.reset(); // Solver consumes the frozen measured input, not a stale registry activation.
    const auto solver=extrinsicsSolver_;
    const auto generation=++generation_; busy_=true;
    localizationValidationRequired_=true; // Keep the previous geometry for acquisition/retry only.
    publish(QStringLiteral("Estimating both planar camera poses and checking measured references on the calibration worker…"));
    QMetaObject::invokeMethod(workerContext_,[this,solver,profile,input=std::move(input),generation] {
        ExtrinsicSolveOutcome outcome;
        try {
            outcome=solver->solve(profile,input);
            if (outcome.calibration && outcome.error.empty()) {
                auto expected=*outcome.calibration;
                expected.survey=input.survey; expected.mounts=input.mounts;
                expected.revision=input.revision; expected.tabletopBinding=tabletopProfileBinding(profile);
                if (serializeElevatedLocalization(expected)!=serializeElevatedLocalization(*outcome.calibration))
                    outcome.error="Solver changed survey, mounting or revision binding.";
                else outcome.error=validateElevatedLocalization(*outcome.calibration,profile);
            }
        } catch(const std::exception& e) { outcome.error=e.what(); }
        catch(...) { outcome.error="Unknown extrinsics solver failure."; }
        QMetaObject::invokeMethod(this,[this,generation,outcome=std::move(outcome)] {
            busy_=false;
            if (shuttingDown_) return;
            if (generation!=generation_) { emit changed(); return; }
            if (!outcome.calibration || !outcome.error.empty()) {
                publish(QStringLiteral("Localization rejected: %1").arg(QString::fromStdString(outcome.error)));
                return;
            }
            draft_->localization=std::move(outcome.calibration);
            localizationValidationRequired_=false;
            if (draft_->references && pendingReferenceRevision_ == draft_->localization->revision) {
                draft_->references->validatedLocalizationRevision = draft_->localization->revision;
            } else if (draft_->references) {
                // The manual robot survey replaced the localization to which the registry was bound.
                draft_->references.reset();
                latestReferenceFrame_.reset();
            }
            pendingReferenceRevision_.clear();
            lastLocalizationDraft_=draft_->localization;
            publish(QStringLiteral("Height-aware draft passed reference/cheirality/ambiguity checks. Review the survey and mounts, then explicitly Activate and Save. Physical ±5 mm/body-centre acceptance is NOT RUN (#224/#63)."));
        },Qt::QueuedConnection);
    },Qt::QueuedConnection);
}
} // namespace micro_robot
