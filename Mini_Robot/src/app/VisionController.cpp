#include "app/VisionController.hpp"

#include <QMetaObject>

#include <stdexcept>
#include <utility>

namespace micro_robot {
VisionController::VisionController(std::unique_ptr<IMarkerDetector> detector, LogChannel log, QObject* parent)
    : VisionController(std::move(detector), std::move(log), RobotMarkerMap{}, parent)
{
}
VisionController::VisionController(std::unique_ptr<IMarkerDetector> detector, LogChannel log,
                                   RobotMarkerMap markerMap, QObject* parent)
    : QObject(parent), log_(std::move(log)), markerMap_(std::move(markerMap)), detector_(std::move(detector))
{
    if (!detector_) throw std::invalid_argument("Marker detector is required");
    qRegisterMetaType<MarkerDetectionsPtr>();
    qRegisterMetaType<VisionDebugFramePtr>();
    qRegisterMetaType<MetricObservationBatchPtr>();
    workerContext_ = new QObject;
    workerContext_->moveToThread(&worker_);
    connect(&worker_, &QThread::finished, workerContext_, &QObject::deleteLater);
    connect(&worker_, &QThread::finished, this, &VisionController::stopped);
    worker_.start();
}
VisionController::~VisionController()
{
    worker_.quit();
    worker_.wait();
}
void VisionController::describe(const QString& text)
{
    description_ = text;
    emit descriptionChanged(description_);
}
void VisionController::invalidate()
{
    ++generation_;
    pending_.reset();
    outputRate_.reset();
    lastSequence_.reset();
    width_ = height_ = 0;
    emit observationsAvailable(sourceId_, {});
    emit metricObservationsAvailable(sourceId_, {});
    emit previewAvailable(sourceId_, {});
}
void VisionController::setInput(const QString& sourceId, bool streaming)
{
    if (shuttingDown_) return;
    if (sourceId == sourceId_ && streaming == streaming_) return;
    invalidate();
    sourceId_ = sourceId;
    streaming_ = streaming && !sourceId.isEmpty();
    describe(streaming_ ? QStringLiteral("AprilTag: waiting for camera frames")
                        : QStringLiteral("AprilTag: camera inactive"));
}
void VisionController::setMetricConverter(std::unique_ptr<IMetricObservationConverter> converter)
{
    if (shuttingDown_) return;
    invalidate();
    metricConverter_ = std::move(converter);
}
void VisionController::setWorkspaceCalibration(std::optional<WorkspaceCalibrationProfile> profile)
{
    if (shuttingDown_) return;
    invalidate();
    workspace_ = std::move(profile);
}
void VisionController::receiveFrame(FramePtr frame)
{
    if (shuttingDown_ || !streaming_ || !frame) return;
    if (!isValidFrame(*frame)) {
        log_.warning("AprilTag input rejected: invalid frame");
        describe(QStringLiteral("AprilTag: invalid input frame"));
        return;
    }
    if (width_ && (width_ != frame->width || height_ != frame->height)) invalidate();
    if (lastSequence_ && (frame->sequence <= *lastSequence_ || frame->capturedAt <= lastCapture_)) return;
    lastSequence_ = frame->sequence;
    lastCapture_ = frame->capturedAt;
    width_ = frame->width;
    height_ = frame->height;
    pending_ = std::move(frame);
    dispatch();
}
void VisionController::dispatch()
{
    if (busy_ || !pending_ || shuttingDown_ || !streaming_) return;
    busy_ = true;
    const auto frame = std::exchange(pending_, {});
    const auto generation = generation_;
    const auto detector = detector_;
    const auto markerMap = markerMap_;
    const auto converter = metricConverter_;
    const auto workspace = workspace_;
    const auto sourceName = sourceId_.toStdString();
    QMetaObject::invokeMethod(workerContext_, [this, detector, markerMap, converter, workspace, sourceName, frame, generation] {
        const auto start = MonotonicClock::now();
        auto result = std::make_shared<MarkerDetections>();
        try {
            *result = detector->detect(*frame);
        } catch (const std::exception& error) {
            result->error = std::string{"AprilTag detector failed: "} + error.what();
        } catch (...) {
            result->error = "AprilTag detector failed with an unknown error";
        }
        if (!result->error.empty()) result->markers.clear();
        result->frameSequence = frame->sequence;
        result->capturedAt = frame->capturedAt;
        result->width = frame->width;
        result->height = frame->height;
        result->processingTime = MonotonicClock::now() - start;
        auto preview = makeVisionDebugFrame(frame, result, markerMap);
        MetricObservationBatchPtr metric;
        if (converter) {
            try {
                metric = std::make_shared<MetricObservationBatch>(converter->convert(*result, sourceName, markerMap, workspace));
            } catch (const std::exception& error) {
                metric = std::make_shared<MetricObservationBatch>(MetricObservationBatch{MetricObservationStatus::InvalidGeometry, {}, error.what()});
            } catch (...) {
                metric = std::make_shared<MetricObservationBatch>(MetricObservationBatch{MetricObservationStatus::InvalidGeometry, {}, "Metric conversion failed"});
            }
        }
        QMetaObject::invokeMethod(this, [this, generation, result = std::move(result), preview = std::move(preview), metric = std::move(metric)] {
            busy_ = false;
            if (generation == generation_ && !shuttingDown_ && streaming_) {
                if (!result->error.empty()) log_.warning(result->error);
                log_.debug("AprilTag source=" + sourceId_.toStdString() + " frame=" + std::to_string(result->frameSequence)
                    + " markers=" + std::to_string(result->markers.size()) + " processing_us="
                    + std::to_string(std::chrono::duration_cast<std::chrono::microseconds>(result->processingTime).count()));
                describe(result->error.empty()
                    ? QStringLiteral("AprilTag frame %1: %2 tags, %3 ms").arg(result->frameSequence).arg(result->markers.size())
                        .arg(std::chrono::duration<double, std::milli>(result->processingTime).count(), 0, 'f', 2)
                    : QString::fromStdString(result->error));
                if (metric && generation == generation_ && !shuttingDown_) {
                    if (metric->status != MetricObservationStatus::Valid)
                        describe(description_ + QStringLiteral(" · metric unavailable: %1")
                            .arg(metric->error.empty() ? QStringLiteral("calibration required") : QString::fromStdString(metric->error)));
                    else {
                        const auto& r = metric->rejected;
                        describe(description_ + QStringLiteral(" · metric %1 · rejected: outside %2, geometry %3, duplicate %4, unassigned %5, family %6")
                            .arg(metric->measurements.size()).arg(r.outsideWorkspace).arg(r.invalidGeometry)
                            .arg(r.duplicateRobot).arg(r.unassignedId).arg(r.unsupportedFamily));
                    }
                }
                if (preview && generation == generation_ && !shuttingDown_) {
                    preview->inputGeneration = generation;
                    preview->outputFramesPerSecond = outputRate_.record(MonotonicClock::now());
                }
                const auto source = sourceId_; // Stable even if a direct observer changes input.
                // A synchronous description observer can invalidate the input.
                if (generation == generation_ && !shuttingDown_)
                    emit observationsAvailable(source, result);
                if (generation == generation_ && !shuttingDown_)
                    emit metricObservationsAvailable(source, metric);
                // Observation consumers can also invalidate synchronously.
                if (generation == generation_ && !shuttingDown_)
                    emit previewAvailable(source, preview);
            }
            dispatch();
        }, Qt::QueuedConnection);
    }, Qt::QueuedConnection);
}
void VisionController::shutdown()
{
    if (shuttingDown_) return;
    shuttingDown_ = true;
    streaming_ = false;
    invalidate();
    describe(QStringLiteral("AprilTag: stopped"));
    worker_.quit(); // Allows an in-flight detector call to finish off the GUI thread.
}
} // namespace micro_robot
