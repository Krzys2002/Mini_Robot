#include "app/DesktopController.hpp"

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

// Plain C++ sink owned by the controller. Weak backend subscriptions cannot retain
// the controller. Detach before destroying the sink; callbacks are thread-affine.
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

DesktopController::DesktopController(ICameraSource& camera, IRobotTransport& transport,
                                     WorldModel& world, RobotId robotId, LogChannel log,
                                     QObject* parent)
    : QObject(parent), camera_(camera), transport_(transport), world_(world), robotId_(robotId),
      log_(std::move(log)), events_(std::make_shared<EventSink>(*this))
{
    camera_.setEventSink(events_);
    transport_.setEventSink(events_);
}

DesktopController::~DesktopController()
{
    camera_.setEventSink({});
    transport_.setEventSink({});
}

void DesktopController::initialize(Timestamp now)
{
    if (initialized_ || shuttingDown_) {
        return;
    }
    initialized_ = true;
    startedAt_ = now;
    (void)world_.registerRobot(robotId_, now);
    log_.info("Controller initialized; robot registered", robotId_);
    emit applicationStatusChanged(QStringLiteral("Ready — Phase 0 simulation"));
    receiveCameraState(camera_.state());
    receiveConnectionState(robotId_, transport_.connectionState(robotId_));
    emit frameDescriptionChanged(QStringLiteral("No frames received"));
    publishWorld();
}

void DesktopController::startCamera()
{
    if (initialized_ && !shuttingDown_) camera_.start();
}

void DesktopController::stopCamera()
{
    if (initialized_ && !shuttingDown_) camera_.stop();
}

void DesktopController::connectRobot()
{
    if (initialized_ && !shuttingDown_) transport_.connect(robotId_);
}

void DesktopController::disconnectRobot()
{
    if (initialized_ && !shuttingDown_) transport_.disconnect(robotId_);
}

void DesktopController::shutdown()
{
    if (shuttingDown_) return;
    shuttingDown_ = true;
    log_.info("Shutdown requested; stopping camera and disconnecting robot");
    camera_.stop();
    transport_.disconnect(robotId_);
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
    if (!frame) return;
    emit frameDescriptionChanged(QStringLiteral("Frame #%1 · %2 × %3 · capture +%4 s")
        .arg(static_cast<qulonglong>(frame->sequence)).arg(frame->width).arg(frame->height)
        .arg(elapsedSeconds(frame->capturedAt), 0, 'f', 2));
}

void DesktopController::receiveCameraState(CameraState state)
{
    const bool active = state == CameraState::Starting || state == CameraState::Streaming
        || state == CameraState::Stopping;
    const auto text = cameraStateText(state);
    if (state == CameraState::Fault) {
        log_.error("Camera state: " + text.toStdString());
    } else {
        log_.info("Camera state: " + text.toStdString());
    }
    emit cameraStatusChanged(text, active);
}

void DesktopController::receiveConnectionState(RobotId id, ConnectionState state)
{
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

void DesktopController::publishWorld()
{
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
