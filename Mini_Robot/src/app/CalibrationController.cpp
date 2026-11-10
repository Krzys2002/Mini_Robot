#include "app/CalibrationController.hpp"

#include <QMetaObject>

#include <chrono>
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
        // Keep the last full detection fixed while camera pixels advance.
        presentPreview(latest_, liveCorners_, liveAccepted_,
            detectedFrame_ && latest_ != detectedFrame_ && !liveCorners_.empty());
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
            // Update points over the newest pixels without rewinding video.
            presentPreview(latest_, liveCorners_, liveAccepted_, latest_ != frame && !liveCorners_.empty());
        }, Qt::QueuedConnection);
    }, Qt::QueuedConnection);
}
void CalibrationController::resetLiveOverlay()
{
    liveCorners_.clear();
    liveAccepted_ = false;
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
