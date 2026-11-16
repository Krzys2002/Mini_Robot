#include "app/WorkspaceCalibrationController.hpp"
#include "app/WorkspaceDiagnosticsController.hpp"
#include <QMetaObject>
#include <QUuid>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace micro_robot {
WorkspaceCalibrationController::WorkspaceCalibrationController(
    std::unique_ptr<IWorkspaceHomographySolver> solver, std::unique_ptr<ICalibrationUndistorter> undistorter,
    std::unique_ptr<IWorkspaceProfileStore> store, LogChannel log, QObject* parent)
    : QObject(parent), solver_(std::move(solver)), undistorter_(std::move(undistorter)),
      store_(std::move(store)), log_(std::move(log))
{
    if (!solver_ || !undistorter_ || !store_)
        throw std::invalid_argument("Workspace calibration services are required.");
    diagnostics_ = std::make_unique<WorkspaceDiagnosticsController>();
    connect(this, &WorkspaceCalibrationController::profileChanged, this, [this] {
        ++measurementGeneration_; measurementFrame_.reset();
        emit measurementFrameChanged();
        diagnostics_->setCalibration(profile_, activeAcquisition_);
    });
    connect(diagnostics_.get(), &WorkspaceDiagnosticsController::stopped, this, &WorkspaceCalibrationController::stopped);
    workerContext_ = new QObject;
    workerContext_->moveToThread(&worker_);
    connect(&worker_, &QThread::finished, workerContext_, &QObject::deleteLater);
    connect(&worker_, &QThread::finished, this, &WorkspaceCalibrationController::stopped);
    worker_.start();
}

WorkspaceCalibrationController::~WorkspaceCalibrationController()
{
    worker_.quit();
    worker_.wait();
}

WorkspaceDiagnosticsController& WorkspaceCalibrationController::diagnostics() const { return *diagnostics_; }
bool WorkspaceCalibrationController::isStopped() const { return worker_.isFinished() && diagnostics_->isStopped(); }

void WorkspaceCalibrationController::publish(QString message)
{
    if (message_ != message)
        log_.debug(message.toStdString());
    message_ = std::move(message);
    emit changed();
}

void WorkspaceCalibrationController::invalidate(bool clearFrame)
{
    ++generation_;
    draft_.reset();
    if (clearFrame) {
        ++referencesRevision_;
        references_.clear(); placements_.clear(); clearBoardReview(); draftAcquisition_.reset();
        latest_.reset();
        frozen_.reset();
        fit_.clear();
        check_.clear();
        corners_.clear();
        boundaryConfirmed_ = false;
    }
}

void WorkspaceCalibrationController::setInput(std::optional<IntrinsicCalibration> input, bool persistent)
{
    if (shuttingDown_)
        return;
    if (input && !validateIntrinsicCalibration(*input).empty())
        input.reset();
    bool same = !input && !intrinsics_;
    if (input && intrinsics_) {
        same = input->sourceId == intrinsics_->sourceId && input->width == intrinsics_->width &&
               input->height == intrinsics_->height && input->cameraMatrix == intrinsics_->cameraMatrix &&
               input->distortion == intrinsics_->distortion;
    }
    if (same && persistent == persistent_)
        return;
    invalidate(true);
    resolutionMismatch_ = false;
    if (profile_) {
        profile_.reset(); activeAcquisition_.reset();
        emit profileChanged();
    }
    intrinsics_ = std::move(input);
    persistent_ = persistent;
    publish(
        intrinsics_
            ? QStringLiteral("Intrinsic binding ready. Freeze an undistorted frame; enter measured workspace "
                             "dimensions.")
            : QStringLiteral("No compatible intrinsic calibration. Workspace transform cleared."));
}

void WorkspaceCalibrationController::receiveFrame(FramePtr frame)
{
    if (shuttingDown_ || !intrinsics_ || !frame || !isValidFrame(*frame))
        return;
    if (frame->width != intrinsics_->width || frame->height != intrinsics_->height) {
        if (!resolutionMismatch_) {
            resolutionMismatch_ = true;
            invalidate(true);
            if (profile_) {
                profile_.reset(); activeAcquisition_.reset();
                emit profileChanged();
            }
            publish(QStringLiteral("Resolution mismatch; workspace transform cleared."));
        }
        return;
    }
    resolutionMismatch_ = false;
    if (latest_ && (frame->sequence <= latest_->sequence || frame->capturedAt <= latest_->capturedAt))
        return;
    latest_ = std::move(frame);
    emit changed();
}

void WorkspaceCalibrationController::geometryChanged()
{
    invalidate(false);
    ++referencesRevision_;
    references_.clear(); placements_.clear(); clearBoardReview(); draftAcquisition_.reset();
    fit_.clear();
    check_.clear();
    boundaryConfirmed_ = false;
    frameId_ = QUuid::createUuid().toString(QUuid::WithoutBraces).toStdString();
}

void WorkspaceCalibrationController::configure(QString id, double width, double height,
                                               double originX, double originY)
{
    if (shuttingDown_) return;
    if (id == workspaceId_ && bounds_ && width == bounds_->width().value && height == bounds_->height().value
        && -originX == bounds_->minimum().x.value && -originY == bounds_->minimum().y.value) return;
    geometryChanged();
    workspaceId_ = std::move(id);
    bounds_.reset();
    try {
        if (!std::isfinite(originX) || !std::isfinite(originY) || originX < 0 || originY < 0
            || originX > width || originY > height) throw std::invalid_argument("origin");
        bounds_.emplace(Meters{width}, Meters{height}, WorldPoint2D{Meters{-originX}, Meters{-originY}});
    } catch (const std::invalid_argument&) {
        publish(QStringLiteral("Enter positive measured dimensions and an origin offset inside the rectangle (metres)."));
        return;
    }
    publish(QStringLiteral("Frame changed; draft references cleared. Review and confirm base, +X, opposite, +Y corners."));
}

void WorkspaceCalibrationController::selectCorner(ImageCorner point, std::optional<std::size_t> replace)
{
    if (shuttingDown_ || !frozen_) return;
    if (!std::isfinite(point.u) || !std::isfinite(point.v) || point.u < 0 || point.v < 0
        || point.u > frozen_->width - 1 || point.v > frozen_->height - 1) {
        if (replace && *replace < corners_.size()) geometryChanged();
        publish(QStringLiteral("Select a finite corner inside the undistorted native image.")); return;
    }
    if (replace && *replace >= corners_.size()) return;
    if (!replace && corners_.size() == WorkspaceCornerCount) {
        publish(QStringLiteral("Four corners selected. Confirm or reselect a labelled corner.")); return;
    }
    geometryChanged();
    if (replace) corners_[*replace] = point;
    else corners_.push_back(point);
    publish(QStringLiteral("%1/4 corners selected: base, +X neighbour, opposite, +Y neighbour. Confirm the labelled perimeter.")
        .arg(corners_.size()));
}
void WorkspaceCalibrationController::undoCorner()
{
    if (shuttingDown_ || corners_.empty()) return;
    geometryChanged();
    corners_.pop_back();
    publish(QStringLiteral("Last corner removed. Reselect it before confirming the boundary."));
}
void WorkspaceCalibrationController::clearCorners()
{
    if (shuttingDown_) return;
    geometryChanged();
    corners_.clear();
    publish(QStringLiteral("Select physical base, +X neighbour, opposite, +Y neighbour in that order."));
}
void WorkspaceCalibrationController::confirmBoundary()
{
    if (shuttingDown_ || busy_ || !frozen_ || !bounds_ || corners_.size() != WorkspaceCornerCount) {
        publish(QStringLiteral("Freeze a compatible frame, enter dimensions/origin and select four corners first.")); return;
    }
    WorkspaceBoundary boundary;
    std::copy(corners_.begin(), corners_.end(), boundary.begin());
    if (auto error = validateWorkspaceBoundary(boundary, frozen_->width, frozen_->height); !error.empty()) {
        publish(QString::fromStdString(error)); return;
    }
    geometryChanged();
    const auto world = workspaceCorners(*bounds_);
    for (std::size_t i = 0; i < boundary.size(); ++i) fit_.push_back({boundary[i], world[i]});
    boundaryConfirmed_ = true;
    synchronizeReferences();
    publish(QStringLiteral("Boundary confirmed. Review axes/dimensions, then add measured references or Calculate. "
                           "After calculating, an image pick can set the origin; this clears draft results/references."));
}
void WorkspaceCalibrationController::setOriginFromImage(ImageCorner point)
{
    if (shuttingDown_ || busy_ || !draft_ || !draft_->homography || !bounds_ || !frozen_) return;
    if (!std::isfinite(point.u) || !std::isfinite(point.v) || point.u < 0 || point.v < 0
        || point.u > frozen_->width - 1 || point.v > frozen_->height - 1) return;
    const auto world = projectWorkspacePoint(*draft_->homography, point);
    if (!world || !bounds_->contains(*world)) {
        publish(QStringLiteral("Origin pick must map inside the reviewed workspace.")); return;
    }
    const auto b = *bounds_;
    configure(workspaceId_, b.width().value, b.height().value,
              world->x.value - b.minimum().x.value, world->y.value - b.minimum().y.value);
}

void WorkspaceCalibrationController::setPoints(std::vector<WorkspaceCorrespondence> fit,
                                               std::vector<WorkspaceCorrespondence> check)
{
    if (shuttingDown_)
        return;
    if (fit.size()>MaximumWorkspaceFitPoints || check.size()>MaximumWorkspaceCheckPoints) {
        publish(QStringLiteral("Reference count exceeds 4096 per role; no rows were replaced.")); return;
    }
    if (!placements_.empty()) {
        publish(QStringLiteral("Use reference-row edits to preserve placement provenance; bulk manual replacement is unavailable while placements exist.")); return;
    }
    if (!boundaryConfirmed_ && !fit.empty()) {
        publish(QStringLiteral("Confirm the four labelled corners before adding fitting references.")); return;
    }
    if (boundaryConfirmed_ && !fit.empty()) {
        if (fit.size() < WorkspaceCornerCount) return;
        for (std::size_t i = 0; i < WorkspaceCornerCount; ++i) {
            if (fit[i].image.u != fit_[i].image.u || fit[i].image.v != fit_[i].image.v
                || fit[i].world.x.value != fit_[i].world.x.value || fit[i].world.y.value != fit_[i].world.y.value) {
                publish(QStringLiteral("Boundary rows are fixed. Use corner reselection or edit the origin/dimensions.")); return;
            }
        }
    }
    const auto sameFit = fit.size() == fit_.size() &&
                         std::equal(fit.begin(), fit.end(), fit_.begin(), [](const auto& a, const auto& b) {
                             return a.image.u == b.image.u && a.image.v == b.image.v &&
                                    a.world.x.value == b.world.x.value && a.world.y.value == b.world.y.value;
                         });
    ++generation_;
    if (!sameFit)
        draft_.reset();
    if (fit.empty()) boundaryConfirmed_ = false;
    fit_ = std::move(fit);
    check_ = std::move(check);
    synchronizeReferences();
    publish(QStringLiteral("Draft points changed. Calculate a fit, or verify check points against the "
                           "current draft transform."));
}

void WorkspaceCalibrationController::freeze() { freezeFrame(false); }
void WorkspaceCalibrationController::freezeBoard()
{
    if (!boundaryConfirmed_) { publish(QStringLiteral("Confirm the physical workspace boundary before retaining placement data.")); return; }
    freezeFrame(true);
}
void WorkspaceCalibrationController::freezeFrame(bool preserveReferences)
{
    if (!canFreeze())
        return;
    const auto age = MonotonicClock::now() - latest_->capturedAt;
    if (age < Duration::zero() || age > MaximumFrozenFrameAge) {
        publish(QStringLiteral("Frame is stale. Wait for a fresh frame."));
        return;
    }
    const auto frame = latest_;
    const auto optical = *intrinsics_;
    ++generation_;
    frozen_.reset();
    clearBoardReview();
    if (!preserveReferences) {
        ++referencesRevision_;
        references_.clear(); placements_.clear();
        fit_.clear(); check_.clear(); corners_.clear(); boundaryConfirmed_ = false;
    }
    const auto generation = generation_;
    busy_ = true;
    publish(QStringLiteral("Freezing an undistorted native-K image…"));
    QMetaObject::invokeMethod(
        workerContext_,
        [this, frame, optical, generation, preserveReferences] {
            UndistortedPreviewOutcome result;
            try {
                result = undistorter_->undistort(*frame, optical);
            } catch (const std::exception& e) {
                result.error = e.what();
            } catch (...) {
                result.error = "Unknown undistortion failure.";
            }
            if (result.frame &&
                (!isValidFrame(*result.frame) || result.frame->width != frame->width ||
                 result.frame->height != frame->height || result.frame->sequence != frame->sequence ||
                 result.frame->capturedAt != frame->capturedAt ||
                 result.frame->pixelFormat != frame->pixelFormat))
                result.error = "Undistorter returned incompatible frame metadata.";
            QMetaObject::invokeMethod(
                this,
                [this, generation, preserveReferences, result = std::move(result)] {
                    busy_ = false;
                    if (shuttingDown_)
                        return;
                    if (generation != generation_) {
                        emit changed();
                        return;
                    }
                    if (result.frame && result.error.empty()) {
                        frozen_ = result.frame;
                        publish(preserveReferences
                            ? QStringLiteral("Frozen next placement frame; stored references retained. Keep camera/table fixed. Detect this exact board image.")
                            : QStringLiteral("Frozen undistorted pixels. Select base, +X, opposite, +Y corners, or add Check points to verify a loaded transform."));
                    } else
                        publish(QStringLiteral("Undistortion failed: %1")
                                    .arg(QString::fromStdString(result.error)));
                },
                Qt::QueuedConnection);
        },
        Qt::QueuedConnection);
}

void WorkspaceCalibrationController::calculate()
{
    if (!canCalculate()) {
        publish(QStringLiteral("Freeze a compatible frame and enter workspace identity/dimensions first."));
        return;
    }
    WorkspaceHomographyInput input{
        {WorkspaceCalibrationProfileVersion, workspaceId_.toStdString(), *bounds_, *intrinsics_},
        fit_,
        check_};
    input.profile.frameId = frameId_;
    WorkspaceBoundary boundary;
    std::copy(corners_.begin(), corners_.end(), boundary.begin());
    input.profile.boundary = boundary;
    input.profile.acquisitionId = QUuid::createUuid().toString(QUuid::WithoutBraces).toStdString();
    const auto acquisition = acquisitionSnapshot(input.profile);
    const auto generation = ++generation_;
    draft_.reset();
    busy_ = true;
    publish(QStringLiteral("Calculating image-to-world homography…"));
    QMetaObject::invokeMethod(
        workerContext_,
        [this, input, generation, acquisition] {
            WorkspaceHomographyOutcome outcome;
            try {
                if (auto error = validateWorkspaceHomographyInput(input); !error.empty()) outcome.error=error;
                else outcome = solver_->solve(input);
            } catch (const std::exception& e) {
                outcome.error = e.what();
            } catch (...) {
                outcome.error = "Unknown homography solver failure.";
            }
            // Even an injected adapter's output must match the requested binding and
            // pass geometry validation before becoming consumable.
            if (outcome.result && outcome.error.empty()) {
                if (auto error = workspaceCalibrationCompatibilityError(outcome.result->profile,
                                                                        input.profile.intrinsics);
                    !error.empty())
                    outcome.error = error;
                else if (outcome.result->profile.workspaceId != input.profile.workspaceId ||
                         !(outcome.result->profile.bounds == input.profile.bounds) ||
                         outcome.result->profile.frameId != input.profile.frameId ||
                         outcome.result->profile.acquisitionId != input.profile.acquisitionId ||
                         outcome.result->profile.boundary.has_value() != input.profile.boundary.has_value() ||
                         !outcome.result->profile.homography)
                    outcome.error = "Solver returned another workspace or no transform.";
                else if (!std::equal(input.profile.boundary->begin(), input.profile.boundary->end(),
                                     outcome.result->profile.boundary->begin(), [](const auto& a, const auto& b) {
                                         return a.u == b.u && a.v == b.v;
                                     }))
                    outcome.error = "Solver returned another boundary.";
                else
                    outcome = evaluateWorkspaceHomography(input, *outcome.result->profile.homography);
            }
            if (outcome.result && outcome.error.empty()) {
                acquisition->profile=outcome.result->profile;
                outcome.error=validateWorkspaceAcquisition(*acquisition);
            }
            QMetaObject::invokeMethod(
                this,
                [this, generation, acquisition, outcome = std::move(outcome)] {
                    busy_ = false;
                    if (shuttingDown_)
                        return;
                    if (generation != generation_) {
                        emit changed();
                        return;
                    }
                    if (!outcome.result || !outcome.error.empty()) {
                        publish(QStringLiteral("Calibration rejected: %1")
                                    .arg(QString::fromStdString(outcome.error)));
                        return;
                    }
                    draft_ = outcome.result->profile;
                    draftAcquisition_ = acquisition;
                    auto text = QStringLiteral("Fit RMS %1 m, maximum %2 m (%3 points). ")
                                    .arg(outcome.result->fit.rmsMeters, 0, 'g', 6)
                                    .arg(outcome.result->fit.maximumMeters, 0, 'g', 6)
                                    .arg(outcome.result->fit.distancesMeters.size());
                    text += outcome.result->check
                                ? QStringLiteral("Independent check RMS %1 m, maximum %2 m (%3 points).")
                                      .arg(outcome.result->check->rmsMeters, 0, 'g', 6)
                                      .arg(outcome.result->check->maximumMeters, 0, 'g', 6)
                                      .arg(outcome.result->check->distancesMeters.size())
                                : QStringLiteral("No independent check points; four-point fit error does not "
                                                 "establish accuracy.");
                    text += QStringLiteral(" Fit hull covers %1% of the declared workspace (%2% of the native image). "
                                           "%3 Check point(s) are extrapolated outside the image/world fit hull. "
                                           "Spread references across near/far edges; accuracy outside the fit hull is unsupported.")
                                .arg(outcome.result->coverage.workspaceFraction * 100, 0, 'g', 4)
                                .arg(outcome.result->coverage.imageFraction * 100, 0, 'g', 4)
                                .arg(outcome.result->coverage.extrapolatedCheckCount);
                    text += QStringLiteral(
                        " Review these errors against your accuracy requirement before explicitly "
                        "activating. No automatic accuracy threshold is applied.");
                    publish(text);
                },
                Qt::QueuedConnection);
        },
        Qt::QueuedConnection);
}

void WorkspaceCalibrationController::verify()
{
    if (!canVerify()) return;
    const auto profile=*draft_;
    const auto fit=fit_, check=check_;
    const auto acquisition=boundaryConfirmed_ && !profile.acquisitionId.empty()?acquisitionSnapshot(profile):nullptr;
    const auto generation=++generation_; busy_=true;
    publish(QStringLiteral("Verifying independent Check references on the calibration worker…"));
    QMetaObject::invokeMethod(workerContext_,[this,profile,fit,check,generation,acquisition] {
        std::string error;
        std::optional<WorkspaceMappingErrors> result;
        try {
            if (!fit.empty()) error=validateWorkspaceHomographyInput({profile,fit,check});
            if (error.empty()) result=evaluateWorkspaceChecks(profile,check);
            if (error.empty() && acquisition) error=validateWorkspaceAcquisition(*acquisition);
            if (error.empty() && !result) error="Use finite distinct Check points inside the native image and workspace.";
        } catch (const std::exception& e) { error=e.what(); }
        catch (...) { error="Unknown Check verification failure."; }
        QMetaObject::invokeMethod(this,[this,generation,acquisition,error=std::move(error),result=std::move(result)] {
            busy_=false; if (shuttingDown_) return;
            if (generation!=generation_) { emit changed(); return; }
            if (!result || !error.empty()) { publish(QStringLiteral("Verification rejected: %1").arg(QString::fromStdString(error))); return; }
            if (acquisition) draftAcquisition_=acquisition;
            publish(QStringLiteral("Independent check RMS %1 m, maximum %2 m (%3 points). Transform unchanged. "
                                   "Review errors against your accuracy requirement before Activate. Fit coverage requires acquisition references; no extrapolation accuracy is established.")
                .arg(result->rmsMeters,0,'g',6).arg(result->maximumMeters,0,'g',6).arg(result->distancesMeters.size()));
        },Qt::QueuedConnection);
    },Qt::QueuedConnection);
}

void WorkspaceCalibrationController::activate()
{
    if (!canActivate())
        return;
    profile_ = draft_;
    activeAcquisition_=draftAcquisition_;
    emit profileChanged();
    log_.info("Workspace transform explicitly activated");
    publish(QStringLiteral("Workspace transform activated. Operator acceptance of physical placement and "
                           "measured errors is required; no automatic accuracy threshold is applied."));
}

void WorkspaceCalibrationController::save()
{
    if (!canSave())
        return;
    const auto profile = *draft_;
    const auto acquisition = !profile.acquisitionId.empty()
        ? (boundaryConfirmed_ ? acquisitionSnapshot(profile) : draftAcquisition_) : nullptr;
    const auto generation = generation_;
    busy_ = true;
    publish(QStringLiteral("Saving workspace profile…"));
    QMetaObject::invokeMethod(
        workerContext_,
        [this, profile, generation, acquisition] {
            std::string error;
            try {
                if (!profile.acquisitionId.empty()) {
                    error=acquisition ? validateWorkspaceAcquisition(*acquisition) : "Acquisition provenance is unavailable; calculate a complete dataset before saving.";
                    if (error.empty()) error=store_->saveAcquisition(profile,*acquisition);
                }
                else error = store_->save(profile);
            } catch (const std::exception& e) {
                error = e.what();
            } catch (...) {
                error = "Unknown workspace save failure.";
            }
            QMetaObject::invokeMethod(
                this,
                [this, generation, error, acquisition] {
                    busy_ = false;
                    if (shuttingDown_)
                        return;
                    if (generation == generation_) {
                        if (error.empty() && acquisition) draftAcquisition_=acquisition;
                        publish(error.empty() ? QStringLiteral("Workspace transform saved atomically.")
                                              : QStringLiteral("Workspace save failed: %1")
                                                    .arg(QString::fromStdString(error)));
                    } else
                        emit changed();
                },
                Qt::QueuedConnection);
        },
        Qt::QueuedConnection);
}

void WorkspaceCalibrationController::load()
{
    if (!canLoad())
        return;
    const auto id = workspaceId_.toStdString();
    const auto active = *intrinsics_;
    invalidate(false);
    const auto generation = generation_;
    busy_ = true;
    publish(QStringLiteral("Loading workspace transform…"));
    QMetaObject::invokeMethod(
        workerContext_,
        [this, id, active, generation] {
            WorkspaceProfileLoadOutcome result;
            try {
                result = store_->load(id, active);
                if (result.profile) {
                    result.error = workspaceCalibrationCompatibilityError(*result.profile, active);
                    if (result.profile->workspaceId != id || !result.profile->homography)
                        result.error = "Loaded workspace identity/transform mismatch.";
                    if (!result.profile->acquisitionId.empty()) {
                        if (!result.acquisition) result.error="Linked acquisition provenance is missing.";
                        else if (serializeWorkspaceProfile(result.acquisition->profile)!=serializeWorkspaceProfile(*result.profile))
                            result.error="Linked acquisition/profile binding mismatch.";
                        else if (auto error=validateWorkspaceAcquisition(*result.acquisition); !error.empty()) result.error=error;
                    }
                }
            } catch (const std::exception& e) {
                result.error = e.what();
            } catch (...) {
                result.error = "Unknown workspace load failure.";
            }
            QMetaObject::invokeMethod(
                this,
                [this, generation, result = std::move(result)] {
                    busy_ = false;
                    if (shuttingDown_)
                        return;
                    if (generation != generation_) {
                        emit changed();
                        return;
                    }
                    if (result.state == ProfileLoadState::Loaded && result.profile && result.error.empty()) {
                        ++referencesRevision_;
                        draft_ = result.profile;
                        bounds_ = draft_->bounds;
                        frameId_ = draft_->frameId;
                        corners_.clear();
                        boundaryConfirmed_ = false;
                        // A stored H has no current captured correspondences/check evidence.
                        frozen_.reset();
                        fit_.clear();
                        check_.clear();
                        references_.clear(); placements_.clear(); clearBoardReview();
                        draftAcquisition_=result.acquisition;
                        if (draftAcquisition_) {
                            references_=draftAcquisition_->references; placements_=draftAcquisition_->placements;
                            if (draft_->boundary) {
                                corners_.assign(draft_->boundary->begin(),draft_->boundary->end());
                                boundaryConfirmed_=true;
                            }
                            nextReferenceId_=1; nextPlacementId_=1;
                            for (const auto& r:references_) {
                                nextReferenceId_=std::max(nextReferenceId_,r.id+1);
                                (r.check ? check_ : fit_).push_back(r.point);
                            }
                            for (const auto& placement:placements_) nextPlacementId_=std::max(nextPlacementId_,placement.id+1);
                        }
                        publish(draftAcquisition_
                            ? QStringLiteral("Draft transform and acquisition loaded (%1 references, %2 placements). Use Freeze next board frame to retain the dataset, or Freeze/reset for session-only rechecks. Verify independently before explicit Activate.")
                                .arg(references_.size()).arg(placements_.size())
                            : QStringLiteral("Draft transform loaded without acquisition provenance. Freeze/reset a fresh frame, add Check points and Verify; explicitly Activate only after reviewing measured errors."));
                    } else
                        publish(
                            result.state == ProfileLoadState::Missing && result.error.empty()
                                ? QStringLiteral("No saved workspace transform for this camera/resolution.")
                                : QStringLiteral("Workspace profile rejected: %1")
                                      .arg(QString::fromStdString(result.error)));
                },
                Qt::QueuedConnection);
        },
        Qt::QueuedConnection);
}

void WorkspaceCalibrationController::shutdown()
{
    diagnostics_->shutdown();
    if (shuttingDown_)
        return;
    shuttingDown_ = true;
    invalidate(true);
    profile_.reset(); activeAcquisition_.reset();
    emit profileChanged();
    worker_.quit();
    emit changed();
}
} // namespace micro_robot
