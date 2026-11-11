#include "app/CalibrationController.hpp"

#include <QMetaObject>

#include <chrono>
#include <algorithm>
#include <cmath>
#include <exception>
#include <stdexcept>
#include <utility>

namespace micro_robot {
CalibrationController::CalibrationController(std::unique_ptr<ICalibrationDetector> detector,
    std::unique_ptr<IIntrinsicCalibrationSolver> solver, LogChannel log,
    CalibrationCaptureConfig config, IntrinsicCalibrationConfig intrinsicConfig, QObject* parent)
    : QObject(parent), session_(config), log_(std::move(log)), detector_(std::move(detector)),
      solver_(std::move(solver)), intrinsicConfig_(intrinsicConfig)
{
    if (!detector_ || !solver_) throw std::invalid_argument("Calibration detector and solver are required");
    if (const auto error = validateIntrinsicConfig(intrinsicConfig_); !error.empty())
        throw std::invalid_argument(error);
    workerContext_ = new QObject;
    workerContext_->moveToThread(&worker_);
    connect(&worker_, &QThread::finished, workerContext_, &QObject::deleteLater);
    liveContext_ = new QObject;
    liveContext_->moveToThread(&liveWorker_);
    connect(&liveWorker_, &QThread::finished, liveContext_, &QObject::deleteLater);
    const auto drained = [this] {
        if (isStopped() && !stoppedEmitted_) { stoppedEmitted_ = true; emit stopped(); }
    };
    connect(&worker_, &QThread::finished, this, drained);
    connect(&liveWorker_, &QThread::finished, this, drained);
    worker_.start();
    liveWorker_.start();
}
CalibrationController::~CalibrationController()
{
    // Production shutdown keeps the GUI event loop alive until stopped().
    worker_.quit();
    liveWorker_.quit();
    worker_.wait();
    liveWorker_.wait();
}
void CalibrationController::publish(const QString& message)
{
    message_ = message;
    log_.info(message.toStdString());
    emit changed();
}
void CalibrationController::invalidate()
{
    ++generation_;
    ++liveEpoch_;
    resetLiveOverlay();
    detectedFrame_.reset();
    liveDetection_.reset();
    ++profileGeneration_;
    ++previewEpoch_;
    undistortedPreview_ = false;
    profile_.reset();
    profileLookupRequested_ = false;
    reloadPending_ = false;
    profileMessage_ = QStringLiteral("No compatible profile selected; waiting for a camera frame.");
    previewFrame_.reset();
    previewCorners_.clear();
    latest_.reset();
    session_.cancel();
    result_.reset();
    live_ = true;
    presentPreview({}, {}, false);
}
void CalibrationController::setInput(const QString& sourceId, bool streaming, bool persistentIdentity)
{
    if (shuttingDown_) return;
    if (sourceId != sourceId_ || persistentIdentity != persistentIdentity_ || (streaming_ && !streaming)) {
        invalidate();
        publish(QStringLiteral("Camera changed or stopped. Start a new capture session."));
    }
    sourceId_ = sourceId;
    streaming_ = streaming;
    persistentIdentity_ = persistentIdentity && !sourceId.isEmpty();
    if (!persistentIdentity_) profileMessage_ = QStringLiteral("Physical camera identity unavailable or ambiguous. Profile save/reload disabled; session calibration is available.");
    emit changed();
}
void CalibrationController::receiveFrame(FramePtr frame)
{
    if (!streaming_ || shuttingDown_ || !frame || !isValidFrame(*frame)) return;
    if (frame->width > 4096 || frame->height > 4096
        || static_cast<std::uint64_t>(frame->width) * frame->height > 4'000'000) {
        invalidate();
        publish(QStringLiteral("Camera resolution exceeds 4 megapixels. Lower it before capture."));
        return;
    }
    if (latest_ && (latest_->width != frame->width || latest_->height != frame->height)) {
        invalidate();
        publish(QStringLiteral("Camera resolution changed. Restart capture."));
    }
    if (latest_ && (frame->sequence <= latest_->sequence || frame->capturedAt <= latest_->capturedAt)) return;
    latest_ = std::move(frame);
    if (persistentIdentity_ && profileStore_ && !profileLookupRequested_) reloadProfile();
    if (live_) {
        if (previewMode_ == CalibrationLivePreviewMode::Track) {
            // Track before corrected remapping to avoid remapping each frame twice.
            trackLive();
            presentPreview(latest_, {}, false);
        } else if (previewMode_ == CalibrationLivePreviewMode::Hold) {
            presentPreview(latest_, liveCorners_, liveAccepted_,
                detectedFrame_ && latest_ != detectedFrame_ && !liveCorners_.empty());
        } else if (!livePreviewActive_ || (!detectedFrame_ && !liveDetectionBusy_)) {
            // Hold the initial sampled image while detection runs, then advance
            // image and corners together. The first result cannot rewind video.
            presentPreview(latest_, {}, false);
        }
        sampleLive();
    }
    emit changed();
}
bool CalibrationController::canCapture() const
{
    return !shuttingDown_ && !busy_ && !profileIoBusy_ && streaming_ && latest_
        && session_.state() == CalibrationCaptureState::Collecting;
}
void CalibrationController::restart()
{
    if (shuttingDown_ || !streaming_ || sourceId_.isEmpty()) return;
    ++generation_;
    ++liveEpoch_;
    resetLiveOverlay();
    detectedFrame_.reset();
    liveDetection_.reset();
    nextLiveSample_ = {};
    ++previewEpoch_;
    session_.restart(sourceId_.toStdString());
    result_.reset();
    live_ = true;
    if (latest_) presentPreview(latest_, {}, false);
    publish(QStringLiteral("Capture the full board in different image regions, distances and rotations."));
}
void CalibrationController::capture()
{
    if (!canCapture()) return;
    const auto frame = livePreviewActive_ && selectedFrame_ ? selectedFrame_ : latest_;
    const auto age = MonotonicClock::now() - frame->capturedAt;
    if (age < Duration::zero() || age > std::chrono::seconds{2}) {
        publish(QStringLiteral("Latest frame is stale. Wait for a fresh camera frame."));
        return;
    }
    ++liveEpoch_;
    resetLiveOverlay(); // A running live job cannot replace the selected capture.
    live_ = false;
    if (frame == detectedFrame_ && liveDetection_) {
        acceptCapture(frame, *liveDetection_);
        return;
    }
    busy_ = true;
    const auto generation = generation_;
    const auto target = session_.config().target;
    const auto detector = detector_;
    publish(QStringLiteral("Detecting chessboard corners…"));
    QMetaObject::invokeMethod(workerContext_, [this, detector, frame, target, generation] {
        CalibrationDetection result;
        try {
            result = detector->detect(*frame, target);
        } catch (const std::exception& error) {
            result.error = std::string{"Detection failed: "} + error.what();
        } catch (...) {
            result.error = "Detection failed with an unknown error.";
        }
        QMetaObject::invokeMethod(this, [this, frame, generation, result = std::move(result)]() mutable {
            busy_ = false;
            if (generation != generation_ || shuttingDown_) { emit changed(); return; }
            acceptCapture(frame, std::move(result));
        }, Qt::QueuedConnection);
    }, Qt::QueuedConnection);
}
void CalibrationController::acceptCapture(FramePtr frame, CalibrationDetection detection)
{
    const auto corners = detection.corners;
    const auto decision = session_.add(frame, std::move(detection));
    live_ = false;
    ++previewEpoch_;
    presentPreview(frame, corners, decision.accepted);
    publish(QString::fromStdString(decision.message));
}
void CalibrationController::setLivePreviewActive(bool active)
{
    if (shuttingDown_ || livePreviewActive_ == active) return;
    livePreviewActive_ = active;
    ++previewEpoch_;
    if (!active) undistortedPreview_ = false;
    ++liveEpoch_;
    resetLiveOverlay();
    detectedFrame_.reset();
    liveDetection_.reset();
    nextLiveSample_ = {};
    if (active && live_) {
        presentPreview(latest_, {}, false);
        sampleLive();
    }
}
void CalibrationController::sampleLive()
{
    const auto now = MonotonicClock::now();
    if (!livePreviewActive_ || !live_ || liveDetectionBusy_ || busy_ || profileIoBusy_
        || shuttingDown_ || !streaming_ || !latest_ || now < nextLiveSample_)
        return;
    if (now < latest_->capturedAt || now - latest_->capturedAt > std::chrono::seconds{2}) return;
    // Maximum 5 starts/s, further limited by detector duration. No pending
    // live jobs: receiveFrame retains just the newest acquisition for the next try.
    nextLiveSample_ = now + std::chrono::milliseconds{200};
    liveDetectionBusy_ = true;
    const auto epoch = liveEpoch_;
    const auto generation = generation_;
    const auto frame = latest_;
    const auto target = session_.config().target;
    const auto detector = detector_;
    QMetaObject::invokeMethod(workerContext_, [this, detector, frame, target, epoch, generation] {
        const auto start = MonotonicClock::now();
        CalibrationDetection detection;
        try { detection = detector->detect(*frame, target); }
        catch (const std::exception& e) { detection.error = e.what(); }
        catch (...) { detection.error = "Unknown live detection failure."; }
        const auto elapsed = MonotonicClock::now() - start;
        QMetaObject::invokeMethod(this, [this, frame, epoch, generation, elapsed,
                                        detection = std::move(detection)]() mutable {
            liveDetectionBusy_ = false;
            if (shuttingDown_ || generation != generation_ || epoch != liveEpoch_
                || !livePreviewActive_ || !live_) return;
            log_.debug("Live calibration frame=" + std::to_string(frame->sequence)
                + " detection_ms=" + std::to_string(std::chrono::duration<double, std::milli>(elapsed).count())
                + " queued_live=0");
            detectedFrame_ = frame;
            liveDetection_ = detection;
            // Evaluate existing quality/diversity rules on a disposable copy;
            // live feedback must never add an observation to the real session.
            auto probe = session_;
            if (probe.state() != CalibrationCaptureState::Collecting) probe.restart(sourceId_.toStdString());
            const auto decision = probe.add(frame, detection);
            liveCorners_ = detection.error.empty() ? detection.corners : std::vector<ImageCorner>{};
            liveAccepted_ = decision.accepted;
            resetTracking();
            if (previewMode_ == CalibrationLivePreviewMode::Track) {
                if (detection.error.empty() && detection.corners.size() == calibrationObjectPoints(session_.config().target).size()) {
                    trackingFrame_ = frame;
                    trackingCorners_ = detection.corners;
                    lastFullDetection_ = frame->capturedAt;
                }
                if (frame == latest_) presentPreview(frame, liveCorners_, liveAccepted_);
                trackLive();
            } else if (previewMode_ == CalibrationLivePreviewMode::Matched) {
                presentPreview(frame, liveCorners_, liveAccepted_);
            } else {
                // Held mode intentionally paints last detection coordinates over new pixels.
                presentPreview(latest_, liveCorners_, liveAccepted_, latest_ != frame && !liveCorners_.empty());
            }
        }, Qt::QueuedConnection);
    }, Qt::QueuedConnection);
}
void CalibrationController::resetLiveOverlay()
{
    liveCorners_.clear();
    liveAccepted_ = false;
    resetTracking();
}
void CalibrationController::setLiveTracker(std::unique_ptr<ICalibrationCornerTracker> tracker)
{
    if (!tracker || tracker_ || streaming_ || shuttingDown_)
        throw std::logic_error("Inject a live corner tracker once before camera input.");
    tracker_ = std::move(tracker);
}
void CalibrationController::resetTracking()
{
    ++trackingRevision_;
    trackingFrame_.reset();
    trackingCorners_.clear();
}
void CalibrationController::trackLive()
{
    if (previewMode_ != CalibrationLivePreviewMode::Track || !tracker_ || trackingBusy_ || previewBusy_ || busy_ || shuttingDown_
        || !live_ || !livePreviewActive_ || !latest_ || !trackingFrame_
        || latest_ == trackingFrame_) return;
    if (latest_->capturedAt < lastFullDetection_
        || latest_->capturedAt-lastFullDetection_ > std::chrono::milliseconds{500}) {
        resetTracking();
        return;
    }
    trackingBusy_ = true;
    const auto frame = latest_;
    const auto previous = trackingFrame_;
    const auto corners = trackingCorners_;
    const auto revision = trackingRevision_;
    const auto epoch = liveEpoch_;
    const auto tracker = tracker_;
    QMetaObject::invokeMethod(liveContext_, [this, frame, previous, corners, revision, epoch, tracker] {
        const auto start = MonotonicClock::now();
        std::vector<ImageCorner> tracked;
        try { tracked = tracker->track(*previous, *frame, corners); }
        catch (...) { tracked.clear(); }
        const auto elapsed = MonotonicClock::now()-start;
        QMetaObject::invokeMethod(this, [this, frame, revision, epoch, elapsed, expected = corners.size(),
                                        tracked = std::move(tracked)]() mutable {
            trackingBusy_ = false;
            if (shuttingDown_) return;
            if (epoch == liveEpoch_ && revision == trackingRevision_ && live_ && livePreviewActive_) {
                log_.debug("Calibration flow frame=" + std::to_string(frame->sequence)
                    + " tracking_ms=" + std::to_string(std::chrono::duration<double,std::milli>(elapsed).count()));
                if (tracked.size() != expected || !std::all_of(tracked.begin(), tracked.end(),
                    [&](const ImageCorner& p) { return std::isfinite(p.u) && std::isfinite(p.v)
                        && p.u >= 0 && p.v >= 0 && p.u < frame->width && p.v < frame->height; })) resetTracking();
                else {
                    trackingFrame_ = frame;
                    trackingCorners_ = tracked;
                    if (frame == latest_) presentPreview(frame, tracked, false, true);
                }
            }
            // One newest frame is retained, never an accumulating flow queue.
            trackLive();
            processPreview();
        }, Qt::QueuedConnection);
    }, Qt::QueuedConnection);
}
void CalibrationController::setLivePreviewMode(CalibrationLivePreviewMode mode)
{
    if (shuttingDown_ || previewMode_ == mode
        || (mode != CalibrationLivePreviewMode::Hold && mode != CalibrationLivePreviewMode::Track
            && mode != CalibrationLivePreviewMode::Matched)
        || (mode == CalibrationLivePreviewMode::Track && !tracker_)) return;
    previewMode_ = mode;
    ++liveEpoch_; // Discard old-mode detection and tracking completions.
    ++previewEpoch_;
    resetTracking();
    if (live_) {
        if (mode == CalibrationLivePreviewMode::Track) {
            if (detectedFrame_ && liveDetection_ && liveDetection_->error.empty()) {
                trackingFrame_ = detectedFrame_;
                trackingCorners_ = liveDetection_->corners;
                lastFullDetection_ = detectedFrame_->capturedAt;
            }
            presentPreview(latest_, {}, false);
            trackLive();
        } else if (mode == CalibrationLivePreviewMode::Matched && detectedFrame_) {
            presentPreview(detectedFrame_, liveCorners_, liveAccepted_);
        } else {
            presentPreview(latest_, liveCorners_, liveAccepted_,
                detectedFrame_ && latest_ != detectedFrame_ && !liveCorners_.empty());
        }
    } else {
        processPreview(); // A preference change preserves paused review and captured data.
    }
    emit changed();
}
void CalibrationController::removeView(int index)
{
    if (shuttingDown_ || busy_ || profileIoBusy_ || index < 0) return;
    if (session_.remove(static_cast<std::size_t>(index))) {
        ++generation_;
        ++profileGeneration_;
        result_.reset();
        profile_.reset();
        undistortedPreview_ = false;
        profileMessage_ = QStringLiteral("Capture changed. Recalculate before saving or using this calibration.");
        showLive();
        publish(QStringLiteral("View removed. Reposition the board and capture a replacement."));
    }
}
void CalibrationController::reviewView(int index)
{
    if (index < 0 || static_cast<std::size_t>(index) >= session_.views().size()) return;
    live_ = false;
    ++liveEpoch_;
    resetLiveOverlay();
    const auto& view = session_.views()[static_cast<std::size_t>(index)];
    ++previewEpoch_;
    presentPreview(view.frame, view.detection.corners, true);
}
void CalibrationController::showLive()
{
    if (shuttingDown_) return;
    ++liveEpoch_;
    resetLiveOverlay();
    detectedFrame_.reset();
    liveDetection_.reset();
    nextLiveSample_ = {};
    live_ = true;
    ++previewEpoch_;
    presentPreview(latest_, {}, false);
    sampleLive();
}
void CalibrationController::finish()
{
    if (shuttingDown_ || busy_) return;
    if (session_.state() != CalibrationCaptureState::Complete && !session_.finish()) {
        publish(QStringLiteral("More diverse views are required: vary location, distance and in-plane rotation."));
        return;
    }
    if (result_) return;
    if (profileIoBusy_) return;
    auto input = intrinsicCalibrationInput(session_);
    const auto config = intrinsicConfig_;
    const auto solver = solver_;
    const auto generation = generation_;
    busy_ = true;
    publish(QStringLiteral("Calculating camera intrinsics and reprojection errors…"));
    QMetaObject::invokeMethod(workerContext_, [this, solver, input = std::move(input), config, generation] {
        IntrinsicCalibrationOutcome outcome;
        try {
            outcome = solver->solve(input, config);
            if (outcome.result && outcome.error.empty()) {
                outcome.error = validateIntrinsicCalibration(*outcome.result, config);
                if (outcome.result->sourceId != input.sourceId || outcome.result->width != input.width
                    || outcome.result->height != input.height
                    || outcome.result->perViewErrors.size() != input.observations.size())
                    outcome.error = "Solver result does not match the captured camera/views.";
                if (outcome.error.empty()) {
                    for (std::size_t i = 0; i < input.observations.size(); ++i) {
                        const auto& view = outcome.result->perViewErrors[i];
                        if (view.frameSequence != input.observations[i].frameSequence
                            || view.pointCount != input.observations[i].corners.size()) {
                            outcome.error = "Solver error metadata does not match the captured views.";
                            break;
                        }
                    }
                }
            }
        } catch (const std::exception& error) {
            outcome.error = std::string{"Intrinsic calculation failed: "} + error.what();
        } catch (...) {
            outcome.error = "Intrinsic calculation failed with an unknown error.";
        }
        QMetaObject::invokeMethod(this, [this, generation, outcome = std::move(outcome)]() mutable {
            busy_ = false;
            if (generation != generation_ || shuttingDown_) { emit changed(); return; }
            if (!outcome.result || !outcome.error.empty()) {
                result_.reset();
                publish(QStringLiteral("Calibration rejected: %1 Restart capture with more tilted, diverse views.")
                    .arg(QString::fromStdString(outcome.error.empty() ? "No model returned." : outcome.error)));
                return;
            }
            result_ = std::move(outcome.result);
            profile_ = result_;
            profileMessage_ = persistentIdentity_
                ? QStringLiteral("Calculated profile — not saved. Save to keep it after restart.")
                : QStringLiteral("Session calibration only: physical identity unavailable or ambiguous; save/reload disabled.");
            ++previewEpoch_;
            if (undistortedPreview_) processPreview();
            publish(hasReprojectionWarning(*result_, intrinsicConfig_)
                ? QStringLiteral("Calibration calculated. Warning: reprojection error exceeds threshold; inspect views and recapture.")
                : QStringLiteral("Calibration calculated. Reprojection error is within the warning threshold."));
        }, Qt::QueuedConnection);
    }, Qt::QueuedConnection);
}
void CalibrationController::cancel()
{
    if (shuttingDown_) return;
    invalidate();
    publish(QStringLiteral("Capture cancelled. Observations cleared."));
}
void CalibrationController::shutdown()
{
    if (shuttingDown_) return;
    shuttingDown_ = true;
    invalidate();
    worker_.quit(); // In-flight work finishes without blocking the GUI.
    liveWorker_.quit();
    emit changed();
}
} // namespace micro_robot
