#include "app/WorkspaceCalibrationController.hpp"
#include <QMetaObject>
#include <exception>

namespace micro_robot
{
void WorkspaceCalibrationController::freezeMeasurement()
{
    if (!canFreezeMeasurement())
        return;
    const auto age = MonotonicClock::now() - latest_->capturedAt;
    if (age < Duration::zero() || age > MaximumFrozenFrameAge) {
        publish("Measurement frame is stale. Wait for a fresh frame.");
        return;
    }
    const auto frame = latest_;
    const auto optical = profile_->intrinsics;
    const auto token = ++measurementGeneration_;
    busy_ = true;
    measurementFrame_.reset();
    emit measurementFrameChanged();
    publish("Freezing a separate native-K undistorted measurement image; calibration references retained…");
    QMetaObject::invokeMethod(
        workerContext_,
        [this, frame, optical, token] {
            UndistortedPreviewOutcome result;
            try {
                result = undistorter_->undistort(*frame, optical);
            } catch (const std::exception& e) {
                result.error = e.what();
            } catch (...) {
                result.error = "Unknown measurement undistortion failure.";
            }
            if (result.frame &&
                (!isValidFrame(*result.frame) || result.frame->width != frame->width ||
                 result.frame->height != frame->height || result.frame->sequence != frame->sequence ||
                 result.frame->capturedAt != frame->capturedAt ||
                 result.frame->pixelFormat != frame->pixelFormat))
                result.error = "Measurement undistorter returned incompatible frame metadata.";
            QMetaObject::invokeMethod(
                this,
                [this, token, result = std::move(result)] {
                    busy_ = false;
                    if (shuttingDown_)
                        return;
                    if (token != measurementGeneration_) {
                        emit changed();
                        return;
                    }
                    if (result.frame && result.error.empty()) {
                        measurementFrame_ = result.frame;
                        emit measurementFrameChanged();
                        publish("Measurement frame frozen in native-K undistorted coordinates. Select "
                                "endpoints; camera/table must remain fixed.");
                    } else
                        publish(QStringLiteral("Measurement undistortion failed: %1")
                                    .arg(QString::fromStdString(result.error)));
                },
                Qt::QueuedConnection);
        },
        Qt::QueuedConnection);
}
} // namespace micro_robot
