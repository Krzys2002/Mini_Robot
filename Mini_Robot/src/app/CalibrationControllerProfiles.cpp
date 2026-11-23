#include "app/CalibrationController.hpp"
#include <QMetaObject>

#include <exception>
#include <stdexcept>

namespace micro_robot {
void CalibrationController::setProfileServices(std::unique_ptr<ICalibrationProfileStore> store,
    std::unique_ptr<ICalibrationUndistorter> undistorter)
{
    if (profileStore_ || undistorter_ || streaming_ || shuttingDown_)
        throw std::logic_error("Inject calibration profile services once before camera input.");
    profileStore_ = std::move(store);
    undistorter_ = std::move(undistorter);
    emit changed();
}
void CalibrationController::reloadProfile(const QString& file)
{
    if (!persistentIdentity_ || !profileStore_ || !latest_ || shuttingDown_ || busy_) return;
    if (profileIoBusy_) {
        if (file.isEmpty()) reloadPending_ = true;
        return;
    }
    profileLookupRequested_ = true;
    profileIoBusy_ = true;
    reloadPending_ = false;
    if (file.isEmpty()) {
        profile_.reset();
        undistortedPreview_ = false;
        ++previewEpoch_;
        presentPreview(previewFrame_, previewCorners_, previewAccepted_);
    }
    profileMessage_ = QStringLiteral("Loading the profile for this camera and delivered resolution…");
    emit changed();
    const auto store = profileStore_;
    const auto id = sourceId_.toStdString();
    const auto width = latest_->width;
    const auto height = latest_->height;
    const auto generation = profileGeneration_;
    QMetaObject::invokeMethod(workerContext_, [this, store, id, width, height, generation, file] {
        ProfileLoadOutcome outcome;
        try {
            outcome = file.isEmpty() ? store->load(id, width, height)
                : store->loadFile(file.toStdString(), id, width, height);
            if (outcome.state == ProfileLoadState::Loaded) {
                if (!outcome.calibration) outcome.error = "Profile store returned no model.";
                else {
                    outcome.error = validateIntrinsicCalibration(*outcome.calibration);
                    if (outcome.error.empty()) outcome.error = calibrationCompatibilityError(*outcome.calibration, id, width, height);
                }
            }
        } catch (const std::exception& error) { outcome.error = error.what(); }
        catch (...) { outcome.error = "Unknown profile loading failure."; }
        QMetaObject::invokeMethod(this, [this, generation, file, outcome = std::move(outcome)]() mutable {
            profileIoBusy_ = false;
            if (shuttingDown_) return;
            if (generation == profileGeneration_) {
                if (outcome.state == ProfileLoadState::Loaded && outcome.calibration && outcome.error.empty()) {
                    profile_ = std::move(outcome.calibration);
                    if (!file.isEmpty()) {
                        resetCapture();
                        undistortedPreview_ = false;
                        presentPreview(latest_, {}, false);
                    }
                    profileMessage_ = file.isEmpty() ? QStringLiteral("Saved profile loaded for this camera and resolution.")
                        : QStringLiteral("Camera profile loaded from %1").arg(file);
                    if (hasReprojectionWarning(*profile_, intrinsicConfig_))
                        profileMessage_ += QStringLiteral(" Warning: reprojection error exceeds threshold.");
                } else {
                    if (file.isEmpty()) profile_.reset();
                    if (outcome.state == ProfileLoadState::Missing && outcome.error.empty()) {
                        profileMessage_ = file.isEmpty()
                            ? QStringLiteral("No saved profile for this camera/resolution. Capture and save a calibration.")
                            : QStringLiteral("No profile file found at %1").arg(file);
                    } else {
                        profileMessage_ = QStringLiteral("Profile rejected: %1").arg(QString::fromStdString(outcome.error));
                    }
                }
                log_.info(profileMessage_.toStdString());
            }
            if (reloadPending_ || !profileLookupRequested_) reloadProfile();
            emit changed();
        }, Qt::QueuedConnection);
    }, Qt::QueuedConnection);
}
void CalibrationController::saveProfile(const QString& file)
{
    if (!canSaveProfile() || busy_) return;
    profileIoBusy_ = true;
    const auto calibration = *profile_;
    const auto generation = profileGeneration_;
    const auto store = profileStore_;
    profileMessage_ = QStringLiteral("Saving calibration profile…");
    emit changed();
    QMetaObject::invokeMethod(workerContext_, [this, store, calibration, generation, file] {
        std::string error;
        try { error = file.isEmpty() ? store->save(calibration) : store->saveFile(file.toStdString(), calibration); }
        catch (const std::exception& e) { error = e.what(); }
        catch (...) { error = "Unknown profile saving failure."; }
        QMetaObject::invokeMethod(this, [this, generation, file, error = std::move(error)] {
            profileIoBusy_ = false;
            if (shuttingDown_) return;
            if (generation == profileGeneration_) {
                if (error.empty()) {
                    profileMessage_ = file.isEmpty() ? QStringLiteral("Calibration saved for this camera and resolution.")
                        : QStringLiteral("Camera profile saved to %1").arg(file);
                } else {
                    profileMessage_ = QStringLiteral("Profile save failed: %1").arg(QString::fromStdString(error));
                }
                log_.info(profileMessage_.toStdString());
            }
            if (reloadPending_ || !profileLookupRequested_) reloadProfile();
            emit changed();
        }, Qt::QueuedConnection);
    }, Qt::QueuedConnection);
}
void CalibrationController::setUndistortedPreview(bool enabled)
{
    undistortedPreview_ = enabled && canUndistort();
    ++previewEpoch_;
    if (undistortedPreview_) processPreview();
    else { selectedFrame_ = previewFrame_; emitPreview(previewFrame_, previewCorners_, previewAccepted_, previewApproximate_); }
    emit changed();
}
void CalibrationController::presentPreview(FramePtr frame, const std::vector<ImageCorner>& corners, bool accepted, bool approximate)
{
    ++previewRevision_;
    previewFrame_ = std::move(frame);
    previewCorners_ = corners;
    previewAccepted_ = accepted;
    previewApproximate_ = approximate;
    if (undistortedPreview_ && previewFrame_ && previewCalibration()) processPreview();
    else { selectedFrame_ = previewFrame_; emitPreview(previewFrame_, previewCorners_, previewAccepted_, previewApproximate_); }
}
void CalibrationController::processPreview()
{
    if (previewBusy_ || trackingBusy_ || shuttingDown_ || !undistortedPreview_ || !previewFrame_ || !previewCalibration()) return;
    previewBusy_ = true;
    const auto frame = previewFrame_;
    const auto profile = *previewCalibration();
    const auto epoch = previewEpoch_;
    const auto revision = previewRevision_;
    const auto corners = previewCorners_;
    const auto accepted = previewAccepted_;
    const auto approximate = previewApproximate_;
    const auto undistorter = undistorter_;
    QMetaObject::invokeMethod(liveContext_, [this, frame, profile, epoch, undistorter, corners, accepted, approximate, revision] {
        UndistortedPreviewOutcome outcome;
        try { outcome = undistorter->undistortWithCorners(*frame, profile, corners); }
        catch (const std::exception& error) { outcome.error = error.what(); }
        catch (...) { outcome.error = "Unknown preview failure."; }
        if (outcome.frame && (!isValidFrame(*outcome.frame) || outcome.frame->width != frame->width
            || outcome.frame->height != frame->height || outcome.frame->sequence != frame->sequence
            || outcome.frame->capturedAt != frame->capturedAt || outcome.frame->pixelFormat != frame->pixelFormat))
            outcome.error = "Undistorted preview returned incompatible frame metadata.";
        QMetaObject::invokeMethod(this, [this, frame, epoch, accepted, approximate, revision, outcome = std::move(outcome)] {
            previewBusy_ = false;
            if (shuttingDown_) return;
            if (epoch == previewEpoch_ && undistortedPreview_) {
                if (outcome.frame && outcome.error.empty()) {
                    selectedFrame_ = frame;
                    emitPreview(outcome.frame, outcome.corners, accepted, approximate);
                }
                else {
                    undistortedPreview_ = false;
                    profileMessage_ = QStringLiteral("Undistorted preview failed: %1").arg(QString::fromStdString(outcome.error));
                    selectedFrame_ = previewFrame_;
                    emitPreview(previewFrame_, previewCorners_, previewAccepted_, previewApproximate_);
                    emit changed();
                }
            }
            // At most one active remap and one newest raw frame; never a frame queue.
            trackLive();
            if (epoch != previewEpoch_ || revision != previewRevision_) processPreview();
        }, Qt::QueuedConnection);
    }, Qt::QueuedConnection);
}
} // namespace micro_robot
