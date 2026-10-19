#pragma once

#include "camera/ICameraSource.hpp"
#include "infrastructure/logging/Logger.hpp"
#include "transport/IRobotTransport.hpp"
#include "world/WorldModel.hpp"

#include <QObject>
#include <QString>

#include <memory>
#include <optional>

namespace micro_robot {

// Qt application adapter. Dependencies must outlive the controller; all calls and
// backend callbacks run on its owning thread. No concrete backend or widgets here.
// It owns the active camera source, if any; none is active until one is set.
// Lifecycle, camera/connection state changes, and backend errors are logged to `log`.
class DesktopController final : public QObject {
    Q_OBJECT

public:
    DesktopController(IRobotTransport& transport, WorldModel& world, RobotId robotId,
                      LogChannel log, QObject* parent = nullptr);
    ~DesktopController() override;

    void initialize(Timestamp now);
    [[nodiscard]] PoseUpdateResult applyPose(RobotId id, const Pose2D& pose, Timestamp at);

    // Replaces the active camera; nullptr clears it. A running source is asked to
    // stop first and is replaced once it reports Stopped, Disconnected, or Fault,
    // never inside its own callback (a queued call completes the swap). Camera
    // start is refused while a replacement is pending, and the new source stays
    // stopped. A later call supersedes a pending one. Ignored after shutdown().
    void setCameraSource(std::unique_ptr<ICameraSource> source);
    [[nodiscard]] bool hasCameraSource() const noexcept { return camera_ != nullptr; }

public slots:
    void startCamera();
    void stopCamera();
    void connectRobot();
    void disconnectRobot();
    // Composition code must drain backend stop/disconnect events before destruction.
    void shutdown();

signals:
    void applicationStatusChanged(const QString& status);
    // Present is false when no source is selected; description names the source.
    void activeCameraChanged(const QString& description, bool present);
    // Availability includes pending requests, source replacement, and shutdown.
    void cameraActionsChanged(bool canStart, bool canStop);
    void cameraStatusChanged(const QString& status, bool active);
    void frameDescriptionChanged(const QString& description);
    // Active includes connecting/disconnecting, so the UI can cancel a connection
    // attempt without treating a pending connection as connected.
    void connectionStatusChanged(const QString& status, bool connected, bool active);
    void telemetryDescriptionChanged(const QString& description);
    void worldDescriptionChanged(const QString& description);
    void errorOccurred(const QString& message);

private:
    class EventSink;
    void publishWorld();
    void publishActiveCamera();
    void publishCameraActions();
    void requestCameraStop();
    void applyCameraReplacement();
    void receiveFrame(FramePtr frame);
    void receiveCameraState(CameraState state);
    void receiveConnectionState(RobotId id, ConnectionState state);
    void receiveTelemetry(const RobotTelemetry& telemetry);
    [[nodiscard]] double elapsedSeconds(Timestamp at) const;

    IRobotTransport& transport_;
    WorldModel& world_;
    RobotId robotId_;
    LogChannel log_;
    std::shared_ptr<EventSink> events_;
    std::unique_ptr<ICameraSource> camera_;
    std::unique_ptr<ICameraSource> pendingCamera_;
    std::optional<CameraState> cameraRequest_;
    Timestamp startedAt_{};
    bool initialized_{};
    bool shuttingDown_{};
    bool replacementPending_{};
    bool replacementScheduled_{};
};

} // namespace micro_robot
