#include "app/DesktopController.hpp"

#include <QMetaObject>

#include <chrono>
#include <utility>

namespace micro_robot {
namespace {

QString cameraStateText(CameraState state)
{
    switch (state) {
    case CameraState::Stopped: return QStringLiteral("Stopped");
    case CameraState::Starting: return QStringLiteral("Starting");
    case CameraState::Streaming: return QStringLiteral("Streaming");
    case CameraState::Stopping: return QStringLiteral("Stopping");
    case CameraState::Disconnected: return QStringLiteral("Disconnected");
    case CameraState::Fault: return QStringLiteral("Fault");
    }
    return QStringLiteral("Unknown");
}

bool isCameraActive(CameraState state)
{
    return state == CameraState::Starting || state == CameraState::Streaming
        || state == CameraState::Stopping;
}

QString cameraDescription(const CameraInfo& info)
{
    return QString::fromStdString(info.displayName.empty() ? info.sourceId : info.displayName);
}

QString connectionStateText(ConnectionState state)
{
    switch (state) {
    case ConnectionState::Disconnected: return QStringLiteral("Disconnected");
    case ConnectionState::Connecting: return QStringLiteral("Connecting");
    case ConnectionState::Connected: return QStringLiteral("Connected");
    case ConnectionState::Disconnecting: return QStringLiteral("Disconnecting");
    case ConnectionState::Fault: return QStringLiteral("Fault");
    }
    return QStringLiteral("Unknown");
}

} // namespace

// Plain C++ camera/application sink owned by the controller. Weak camera and
// manager subscriptions cannot retain it. Detach before destruction; owning thread only.
class DesktopController::EventSink final : public ICameraEvents, public IRobotTransportEvents {
public:
    explicit EventSink(DesktopController& owner) : owner_(owner) {}
    void onFrame(FramePtr frame) override { owner_.receiveFrame(std::move(frame)); }
    void onCameraStateChanged(CameraState state, Timestamp) override
    {
        owner_.receiveCameraState(state);
    }
    void onCameraError(const CameraError& error) override
    {
        owner_.log_.error("Camera error: " + error.message);
        emit owner_.errorOccurred(QString::fromStdString(error.message));
    }
    void onConnectionStateChanged(RobotId id, ConnectionState state, Timestamp) override
    {
        owner_.receiveConnectionState(id, state);
    }
    void onTelemetry(const RobotTelemetry& telemetry) override { owner_.receiveTelemetry(telemetry); }
    void onTransportError(const TransportError& error) override
    {
        owner_.log_.error("Transport error: " + error.message, error.robotId);
        emit owner_.errorOccurred(QStringLiteral("Robot %1: %2")
            .arg(error.robotId.value()).arg(QString::fromStdString(error.message)));
    }
private:
    DesktopController& owner_;
};

DesktopController::DesktopController(RobotManager& robots, WorldModel& world,
                                     RobotId robotId, LogChannel log, QObject* parent)
    : QObject(parent), robots_(robots), world_(world), robotId_(robotId),
      log_(std::move(log)), events_(std::make_shared<EventSink>(*this))
{
    robots_.setEventSink(events_);
    frameExpiry_.setSingleShot(true);
    frameExpiry_.setTimerType(Qt::PreciseTimer);
    connect(&frameExpiry_, &QTimer::timeout, this, [this] {
        log_.warning("Camera frame expired; no current frame");
        emit frameDescriptionChanged(QStringLiteral("No current frame — waiting for camera"));
    });
}

DesktopController::~DesktopController()
{
    if (camera_) camera_->setEventSink({});
    robots_.setEventSink({});
}

void DesktopController::initialize(Timestamp now)
{
    if (initialized_ || shuttingDown_) {
        return;
    }
    initialized_ = true;
    startedAt_ = now;
    (void)robots_.addRobot(robotId_, now);
    log_.info("Controller initialized; robot registered", robotId_);
    emit applicationStatusChanged(QStringLiteral("Ready"));
    publishActiveCamera();
    receiveConnectionState(robotId_, robots_.connectionState(robotId_));
    publishWorld();
}

void DesktopController::setCameraSource(std::unique_ptr<ICameraSource> source)
{
    if (shuttingDown_) return;
    pendingCamera_ = std::move(source);
    replacementPending_ = true;
    frameExpiry_.stop();
    emit frameDescriptionChanged(QStringLiteral("No current frame — switching camera"));
    if (camera_ && (isCameraActive(camera_->state()) || cameraRequest_)) {
        log_.info("Stopping camera before switching source");
        requestCameraStop(); // Completed by receiveCameraState once quiescent.
        return;
    }
    applyCameraReplacement();
}

void DesktopController::applyCameraReplacement()
{
    if (!replacementPending_ || shuttingDown_) return;
    if (camera_ && (isCameraActive(camera_->state()) || cameraRequest_)) return;
    replacementPending_ = false;
    if (camera_) camera_->setEventSink({});
    camera_ = std::move(pendingCamera_); // Destroys the previous, quiescent source.
    if (camera_) camera_->setEventSink(events_);
    log_.info(camera_ ? "Active camera: " + cameraDescription(camera_->info()).toStdString()
                      : std::string{"Active camera cleared"});
    publishActiveCamera();
}

void DesktopController::publishActiveCamera()
{
    frameExpiry_.stop();
    emit activeCameraChanged(camera_ ? cameraDescription(camera_->info())
                                     : QStringLiteral("No camera selected"), camera_ != nullptr);
    emit frameDescriptionChanged(QStringLiteral("No frames received"));
    if (camera_) {
        receiveCameraState(camera_->state());
    } else {
        emit cameraStatusChanged(QStringLiteral("No camera selected"), false);
    }
    publishCameraActions();
}

void DesktopController::publishCameraActions()
{
    const auto info = camera_ ? camera_->info() : CameraInfo{};
    const bool streaming = camera_ && camera_->state() == CameraState::Streaming && !cameraRequest_
        && !replacementPending_ && !shuttingDown_;
    emit cameraInputChanged(QString::fromStdString(info.sourceId), streaming);
    emit calibrationInputChanged(QString::fromStdString(info.persistentCalibrationId.value_or(info.sourceId)),
        streaming, info.persistentCalibrationId && !info.persistentCalibrationId->empty());
    const bool ready = initialized_ && !shuttingDown_ && camera_ && !replacementPending_;
    const auto state = camera_ ? camera_->state() : CameraState::Stopped;
    const bool active = isCameraActive(state);
    const bool stopping = cameraRequest_ == CameraState::Stopping || state == CameraState::Stopping;
    emit cameraActionsChanged(ready && !cameraRequest_ && !active,
                              ready && !stopping && (active || cameraRequest_.has_value()));
}

void DesktopController::startCamera()
{
    if (!initialized_ || shuttingDown_ || !camera_ || replacementPending_
        || cameraRequest_ || isCameraActive(camera_->state())) return;
    cameraRequest_ = CameraState::Starting;
    log_.info("Camera start requested: " + cameraDescription(camera_->info()).toStdString());
    camera_->start(); // Contract: no inline backend callbacks.
    emit cameraStatusChanged(QStringLiteral("Starting"), true);
    publishCameraActions();
}

void DesktopController::requestCameraStop()
{
    if (!camera_ || cameraRequest_ == CameraState::Stopping) {
        publishCameraActions();
        return;
    }
    frameExpiry_.stop();
    emit frameDescriptionChanged(QStringLiteral("No current frame — camera stopping"));
    cameraRequest_ = CameraState::Stopping;
    log_.info("Camera stop requested: " + cameraDescription(camera_->info()).toStdString());
    camera_->stop();
    emit cameraStatusChanged(QStringLiteral("Stopping"), true);
    publishCameraActions();
}

void DesktopController::stopCamera()
{
    if (!initialized_ || shuttingDown_ || !camera_
        || camera_->state() == CameraState::Stopping || cameraRequest_ == CameraState::Stopping
        || (!cameraRequest_ && !isCameraActive(camera_->state()))) return;
    requestCameraStop();
}

void DesktopController::connectRobot()
{
    if (initialized_ && !shuttingDown_ && !robots_.connect(robotId_)) {
        const auto message = QStringLiteral("Robot %1: connection request refused while session is %2")
            .arg(robotId_.value()).arg(connectionStateText(robots_.connectionState(robotId_)));
        log_.warning(message.toStdString(), robotId_);
        emit errorOccurred(message);
    }
}

void DesktopController::disconnectRobot()
{
    if (initialized_ && !shuttingDown_) (void)robots_.disconnect(robotId_);
}

void DesktopController::shutdown()
{
    if (shuttingDown_) return;
    shuttingDown_ = true;
    log_.info("Shutdown requested; stopping camera and disconnecting robot");
    replacementPending_ = false;
    pendingCamera_.reset();
    if (camera_ && (cameraRequest_ || isCameraActive(camera_->state()))) requestCameraStop();
    publishCameraActions();
    robots_.shutdown();
    emit applicationStatusChanged(QStringLiteral("Shutting down"));
}

PoseUpdateResult DesktopController::applyPose(RobotId id, const Pose2D& pose, Timestamp at)
{
    const auto result = world_.updatePose(id, pose, at);
    if (result == PoseUpdateResult::Applied) publishWorld();
    return result;
}

double DesktopController::elapsedSeconds(Timestamp at) const
{
    return std::chrono::duration<double>(at - startedAt_).count();
}

void DesktopController::receiveFrame(FramePtr frame)
{
    if (!frame || !isValidFrame(*frame) || !camera_
        || camera_->state() != CameraState::Streaming || replacementPending_
        || shuttingDown_ || cameraRequest_ == CameraState::Stopping) return;
    frameExpiry_.start(CameraFrameTimeout);
    emit frameAvailable(frame);
    emit frameDescriptionChanged(QStringLiteral("Frame #%1 · %2 × %3 · capture +%4 s")
        .arg(static_cast<qulonglong>(frame->sequence)).arg(frame->width).arg(frame->height)
        .arg(elapsedSeconds(frame->capturedAt), 0, 'f', 2));
}

void DesktopController::receiveCameraState(CameraState state)
{
    if (!isCameraActive(state)
        || (state == CameraState::Streaming && cameraRequest_ == CameraState::Starting)) {
        cameraRequest_.reset();
    }
    const bool active = isCameraActive(state);
    if (state != CameraState::Streaming) {
        frameExpiry_.stop();
        emit frameDescriptionChanged(state == CameraState::Stopped
            ? QStringLiteral("No frames received")
            : QStringLiteral("No current frame — %1").arg(cameraStateText(state)));
    }
    const auto text = cameraStateText(state);
    if (state == CameraState::Fault) {
        log_.error("Camera state: " + text.toStdString());
    } else {
        log_.info("Camera state: " + text.toStdString());
    }
    emit cameraStatusChanged(cameraRequest_ == CameraState::Stopping
        ? QStringLiteral("Stopping") : text, active);
    publishCameraActions();
    if (replacementPending_ && !active && !replacementScheduled_) {
        // The source may be inside its own callback; destroy it only afterwards.
        replacementScheduled_ = true;
        QMetaObject::invokeMethod(this, [this] {
            replacementScheduled_ = false;
            applyCameraReplacement();
        }, Qt::QueuedConnection);
    }
}

void DesktopController::receiveConnectionState(RobotId id, ConnectionState state)
{
    if (shuttingDown_ && !robotSessionsStoppedEmitted_ && robots_.isStopped()) {
        robotSessionsStoppedEmitted_ = true;
        emit robotSessionsStopped();
    }
    if (id != robotId_) return;
    const bool active = state == ConnectionState::Connecting || state == ConnectionState::Connected
        || state == ConnectionState::Disconnecting;
    const auto text = connectionStateText(state);
    if (state == ConnectionState::Fault) {
        log_.error("Connection state: " + text.toStdString(), id);
    } else {
        log_.info("Connection state: " + text.toStdString(), id);
    }
    emit connectionStatusChanged(text, state == ConnectionState::Connected, active);
    if (state != ConnectionState::Connected) {
        emit telemetryDescriptionChanged(QStringLiteral("No active telemetry"));
    }
}

void DesktopController::receiveTelemetry(const RobotTelemetry& telemetry)
{
    if (telemetry.robotId != robotId_) return;
    const auto battery = telemetry.batteryVoltageVolts
        ? QStringLiteral("%1 V").arg(*telemetry.batteryVoltageVolts, 0, 'f', 2)
        : QStringLiteral("unknown battery");
    emit telemetryDescriptionChanged(QStringLiteral("Robot %1 · %2 · received +%3 s")
        .arg(robotId_.value()).arg(battery).arg(elapsedSeconds(telemetry.receivedAt), 0, 'f', 2));
}

void DesktopController::setWorkspaceBounds(std::optional<WorkspaceBounds> bounds)
{
    if (shuttingDown_) return;
    world_.setWorkspaceBounds(bounds);
    if (initialized_) publishWorld();
}

void DesktopController::publishWorld()
{
    const auto workspace = world_.workspaceBounds();
    emit workspaceDescriptionChanged(workspace
        ? QStringLiteral("Workspace: %1 m × %2 m · camera-to-world calibration not established")
            .arg(workspace->width().value, 0, 'g', 6).arg(workspace->height().value, 0, 'g', 6)
        : QStringLiteral("Workspace dimensions not configured · camera-to-world calibration not established"));
    const auto state = world_.robot(robotId_);
    if (!state) {
        emit worldDescriptionChanged(QStringLiteral("Robot %1 is not registered").arg(robotId_.value()));
        return;
    }
    const auto prefix = QStringLiteral("%1 registered robot(s) · Robot %2")
        .arg(static_cast<qulonglong>(world_.robotCount())).arg(robotId_.value());
    if (!state->pose) {
        emit worldDescriptionChanged(prefix + QStringLiteral("\nPose unknown"));
        return;
    }
    emit worldDescriptionChanged(prefix + QStringLiteral("\nx %1 m · y %2 m · heading %3 rad"
        "\nPose updated +%4 s")
        .arg(state->pose->x.value, 0, 'f', 2).arg(state->pose->y.value, 0, 'f', 2)
        .arg(state->pose->heading.value, 0, 'f', 2)
        .arg(elapsedSeconds(*state->poseUpdatedAt), 0, 'f', 2));
}

} // namespace micro_robot
