#include "app/WorkspaceCalibrationController.hpp"
#include <QMetaObject>
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
        latest_.reset();
        frozen_.reset();
        fit_.clear();
        check_.clear();
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
        profile_.reset();
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
                profile_.reset();
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

void WorkspaceCalibrationController::configure(QString id, double width, double height)
{
    if (shuttingDown_)
        return;
    if (id == workspaceId_ && bounds_ && width == bounds_->width().value && height == bounds_->height().value)
        return;
    invalidate(false);
    workspaceId_ = std::move(id);
    bounds_.reset();
    try {
        bounds_.emplace(Meters{width}, Meters{height});
    } catch (const std::invalid_argument&) {
        publish(QStringLiteral("Enter finite positive measured workspace dimensions in metres."));
        return;
    }
    publish(QStringLiteral("Workspace changed. Calculate or explicitly reload its saved transform."));
}

void WorkspaceCalibrationController::setPoints(std::vector<WorkspaceCorrespondence> fit,
                                               std::vector<WorkspaceCorrespondence> check)
{
    if (shuttingDown_)
        return;
    const auto sameFit = fit.size() == fit_.size() &&
                         std::equal(fit.begin(), fit.end(), fit_.begin(), [](const auto& a, const auto& b) {
                             return a.image.u == b.image.u && a.image.v == b.image.v &&
                                    a.world.x.value == b.world.x.value && a.world.y.value == b.world.y.value;
                         });
    ++generation_;
    if (!sameFit)
        draft_.reset();
    fit_ = std::move(fit);
    check_ = std::move(check);
    publish(QStringLiteral("Draft points changed. Calculate a fit, or verify check points against the "
                           "current draft transform."));
}

void WorkspaceCalibrationController::freeze()
{
    if (!canFreeze())
        return;
    const auto age = MonotonicClock::now() - latest_->capturedAt;
    if (age < Duration::zero() || age > std::chrono::seconds{2}) {
        publish(QStringLiteral("Frame is stale. Wait for a fresh frame."));
        return;
    }
    const auto frame = latest_;
    const auto optical = *intrinsics_;
    ++generation_;
    frozen_.reset();
    fit_.clear();
    check_.clear();
    const auto generation = generation_;
    busy_ = true;
    publish(QStringLiteral("Freezing an undistorted native-K image…"));
    QMetaObject::invokeMethod(
        workerContext_,
        [this, frame, optical, generation] {
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
                [this, generation, result = std::move(result)] {
                    busy_ = false;
                    if (shuttingDown_)
                        return;
                    if (generation != generation_) {
                        emit changed();
                        return;
                    }
                    if (result.frame && result.error.empty()) {
                        frozen_ = result.frame;
                        publish(QStringLiteral("Frozen undistorted pixels. Pair measured points in any "
                                               "order, or add Check points to verify a loaded transform."));
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
    if (const auto error = validateWorkspaceHomographyInput(input); !error.empty()) {
        publish(QString::fromStdString(error));
        return;
    }
    const auto generation = ++generation_;
    draft_.reset();
    emit profileChanged();
    busy_ = true;
    publish(QStringLiteral("Calculating image-to-world homography…"));
    QMetaObject::invokeMethod(
        workerContext_,
        [this, input, generation] {
            WorkspaceHomographyOutcome outcome;
            try {
                outcome = solver_->solve(input);
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
                         outcome.result->profile.bounds.width().value != input.profile.bounds.width().value ||
                         outcome.result->profile.bounds.height().value !=
                             input.profile.bounds.height().value ||
                         !outcome.result->profile.homography)
                    outcome.error = "Solver returned another workspace or no transform.";
                else
                    outcome = evaluateWorkspaceHomography(input, *outcome.result->profile.homography);
            }
            QMetaObject::invokeMethod(
                this,
                [this, generation, outcome = std::move(outcome)] {
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
    if (!canVerify())
        return;
    // At most 64 simple projections; no image processing or refit on the GUI thread.
    if (!fit_.empty()) {
        if (const auto error = validateWorkspaceHomographyInput({*draft_, fit_, check_}); !error.empty()) {
            publish(QStringLiteral("Verification rejected: %1").arg(QString::fromStdString(error)));
            return;
        }
    }
    const auto result = evaluateWorkspaceChecks(*draft_, check_);
    if (!result) {
        publish(QStringLiteral("Verification rejected: use finite, distinct Check points inside the native "
                               "image and workspace."));
        return;
    }
    publish(QStringLiteral("Independent check RMS %1 m, maximum %2 m (%3 points). Transform unchanged. "
                           "Review errors against your accuracy requirement before Activate.")
                .arg(result->rmsMeters, 0, 'g', 6)
                .arg(result->maximumMeters, 0, 'g', 6)
                .arg(result->distancesMeters.size()));
}

void WorkspaceCalibrationController::activate()
{
    if (!canActivate())
        return;
    profile_ = draft_;
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
    const auto generation = generation_;
    busy_ = true;
    publish(QStringLiteral("Saving workspace profile…"));
    QMetaObject::invokeMethod(
        workerContext_,
        [this, profile, generation] {
            std::string error;
            try {
                error = store_->save(profile);
            } catch (const std::exception& e) {
                error = e.what();
            } catch (...) {
                error = "Unknown workspace save failure.";
            }
            QMetaObject::invokeMethod(
                this,
                [this, generation, error] {
                    busy_ = false;
                    if (shuttingDown_)
                        return;
                    if (generation == generation_)
                        publish(error.empty() ? QStringLiteral("Workspace transform saved atomically.")
                                              : QStringLiteral("Workspace save failed: %1")
                                                    .arg(QString::fromStdString(error)));
                    else
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
                        draft_ = result.profile;
                        bounds_ = draft_->bounds;
                        // A stored H has no current captured correspondences/check evidence.
                        frozen_.reset();
                        fit_.clear();
                        check_.clear();
                        publish(QStringLiteral(
                            "Draft transform loaded. Freeze a fresh frame, add Check points and Verify; then "
                            "explicitly Activate after reviewing measured errors."));
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
    if (shuttingDown_)
        return;
    shuttingDown_ = true;
    invalidate(true);
    profile_.reset();
    emit profileChanged();
    worker_.quit();
    emit changed();
}
} // namespace micro_robot
