#include "app/CalibrationController.hpp"

#include <QMetaObject>

#include <chrono>
#include <exception>
#include <stdexcept>
#include <utility>

namespace micro_robot {
CalibrationController::CalibrationController(std::unique_ptr<ICalibrationDetector> detector,
    LogChannel log, CalibrationCaptureConfig config, QObject* parent)
    : QObject(parent), session_(config), log_(std::move(log)), detector_(std::move(detector))
{
    if (!detector_) throw std::invalid_argument("Calibration detector is required");
    workerContext_ = new QObject;
    workerContext_->moveToThread(&worker_);
    connect(&worker_, &QThread::finished, workerContext_, &QObject::deleteLater);
    connect(&worker_, &QThread::finished, this, &CalibrationController::stopped);
    worker_.start();
}
CalibrationController::~CalibrationController()
{
    // Production shutdown keeps the GUI event loop alive until stopped().
    worker_.quit();
    worker_.wait();
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
    latest_.reset();
    session_.cancel();
    live_ = true;
    emit previewChanged({}, {}, false);
}
void CalibrationController::setInput(const QString& sourceId, bool streaming)
{
    if (shuttingDown_) return;
    if (sourceId != sourceId_ || (streaming_ && !streaming)) {
        invalidate();
        publish(QStringLiteral("Camera changed or stopped. Start a new capture session."));
    }
    sourceId_ = sourceId;
    streaming_ = streaming;
    emit changed();
}
void CalibrationController::receiveFrame(FramePtr frame)
{
    if (!streaming_ || shuttingDown_ || !frame || !isValidFrame(*frame)) return;
    if (frame->width > 4096 || frame->height > 4096
        || static_cast<std::uint64_t>(frame->width) * frame->height > 4'000'000) {
        publish(QStringLiteral("Camera resolution exceeds 4 megapixels. Lower it before capture."));
        latest_.reset();
        return;
    }
    if (latest_ && (latest_->width != frame->width || latest_->height != frame->height)) {
        invalidate();
        publish(QStringLiteral("Camera resolution changed. Restart capture."));
    }
    if (latest_ && (frame->sequence <= latest_->sequence || frame->capturedAt <= latest_->capturedAt)) return;
    latest_ = std::move(frame);
    if (live_) emit previewChanged(latest_, {}, false);
    emit changed();
}
bool CalibrationController::canCapture() const
{
    return !shuttingDown_ && !busy_ && streaming_ && latest_
        && session_.state() == CalibrationCaptureState::Collecting;
}
void CalibrationController::restart()
{
    if (shuttingDown_ || !streaming_ || sourceId_.isEmpty()) return;
    ++generation_;
    session_.restart(sourceId_.toStdString());
    live_ = true;
    if (latest_) emit previewChanged(latest_, {}, false);
    publish(QStringLiteral("Capture the full board in different image regions, distances and rotations."));
}
void CalibrationController::capture()
{
    if (!canCapture()) return;
    const auto age = MonotonicClock::now() - latest_->capturedAt;
    if (age < Duration::zero() || age > std::chrono::seconds{2}) {
        publish(QStringLiteral("Latest frame is stale. Wait for a fresh camera frame."));
        return;
    }
    busy_ = true;
    const auto generation = generation_;
    const auto frame = latest_;
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
            const auto corners = result.corners;
            const auto decision = session_.add(frame, std::move(result));
            live_ = false;
            emit previewChanged(frame, corners, decision.accepted);
            publish(QString::fromStdString(decision.message));
        }, Qt::QueuedConnection);
    }, Qt::QueuedConnection);
}
void CalibrationController::removeView(int index)
{
    if (shuttingDown_ || busy_ || index < 0) return;
    if (session_.remove(static_cast<std::size_t>(index))) {
        showLive();
        publish(QStringLiteral("View removed. Reposition the board and capture a replacement."));
    }
}
void CalibrationController::reviewView(int index)
{
    if (index < 0 || static_cast<std::size_t>(index) >= session_.views().size()) return;
    live_ = false;
    const auto& view = session_.views()[static_cast<std::size_t>(index)];
    emit previewChanged(view.frame, view.detection.corners, true);
}
void CalibrationController::showLive()
{
    live_ = true;
    emit previewChanged(latest_, {}, false);
}
void CalibrationController::finish()
{
    if (shuttingDown_ || busy_) return;
    if (session_.finish()) publish(QStringLiteral("Capture complete. Observations remain in this session. Calculating and saving calibration are not available yet."));
    else publish(QStringLiteral("More diverse views are required: vary location, distance and in-plane rotation."));
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
    worker_.quit(); // In-flight detection finishes without blocking the GUI.
    emit changed();
}
} // namespace micro_robot
