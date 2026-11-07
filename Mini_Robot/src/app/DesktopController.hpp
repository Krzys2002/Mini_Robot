#pragma once

#include "app/RobotManager.hpp"
#include "calibration/WorkspaceCalibration.hpp"
#include "camera/ICameraSource.hpp"
#include "infrastructure/configuration/ApplicationConfig.hpp"
#include "infrastructure/logging/Logger.hpp"
#include "transport/IRobotDiscovery.hpp"
#include "world/WorldModel.hpp"

#include <QList>
#include <QObject>
#include <QString>
#include <QTimer>

#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <vector>

namespace micro_robot {

struct RobotChoiceItem {
    RobotId id;
    QString label;
    bool ambiguous{};
};
// Qt application adapter. Dependencies must outlive the controller; all calls and
// backend callbacks run on its owning thread. No concrete backend or widgets here.
// It owns the active camera source, if any; none is active until one is set.
// Lifecycle, camera/connection state changes, and backend errors are logged to `log`.
// Frame metadata is cleared on loss/stop/replacement or three seconds of silence.
class DesktopController final : public QObject {
    Q_OBJECT

public:
    DesktopController(RobotManager& robots, WorldModel& world, RobotId robotId,
                      LogChannel log, QObject* parent = nullptr, ManualControlConfig manual = {});
    ~DesktopController() override;

    void initialize(Timestamp now);
    [[nodiscard]] PoseUpdateResult applyPose(RobotId id, const Pose2D& pose, Timestamp at);
    void refreshWorld();
    // Measured table dimensions only; this never implies a calibrated camera.
    void setWorkspaceBounds(std::optional<WorkspaceBounds> bounds);
    // Presentation/metric-bounds activation from the workspace calibration service.
    // No robot observation conversion here. Missing/invalid transforms deactivate.
    void setWorkspaceCalibration(const std::optional<WorkspaceCalibrationProfile>& profile);

    // Replaces the active camera; nullptr clears it. A running source is asked to
    // stop first and is replaced once it reports Stopped, Disconnected, or Fault,
    // never inside its own callback (a queued call completes the swap). Camera
    // start is refused while a replacement is pending, and the new source stays
    // stopped. A later call supersedes a pending one. Ignored after shutdown().
    void setCameraSource(std::unique_ptr<ICameraSource> source);
    [[nodiscard]] bool hasCameraSource() const noexcept { return camera_ != nullptr; }

    // Composition/discovery adapter supplies domain snapshots; no native peers
    // cross this boundary. Selection changes stop the old robot before switching.
    void setDiscoveredRobots(std::vector<DiscoveredRobot> robots);
    void setRobotScanState(bool scanning);
    [[nodiscard]] RobotId selectedRobot() const noexcept { return robotId_; }
    [[nodiscard]] ManualControlConfig manualControlConfig() const noexcept { return manualConfig_; }

public slots:
    void selectRobot(quint32 id);
    void scanRobots();
    void beginManualDrive(double linearMetersPerSecond, double angularRadiansPerSecond);
    void stopManualDrive(); // Explicit Stop also submits while idle.
    void cancelManualDrive(); // Input cancellation submits only while held.
    void emergencyStop();
    void clearEmergencyStop();
    void startCamera();
    void stopCamera();
    void connectRobot();
    void disconnectRobot();
    // Composition code must drain backend stop/disconnect events before destruction.
    void shutdown();

signals:
    void robotChoicesChanged(const QList<RobotChoiceItem>& robots, quint32 selectedId);
    void robotScanChanged(bool scanning);
    void robotScanRequested();
    void robotActionsChanged(bool canConnect, bool canDisconnect, bool canDrive,
                             bool canStop, bool canEmergencyStop);
    void manualStatusChanged(const QString& description);
    void emergencyClearAvailable(bool available);
    void applicationStatusChanged(const QString& status);
    // Present is false when no source is selected; description names the source.
    void activeCameraChanged(const QString& description, bool present);
    // Availability includes pending requests, source replacement, and shutdown.
    void cameraActionsChanged(bool canStart, bool canStop);
    void cameraStatusChanged(const QString& status, bool active);
    // Same-owner-thread consumers may retain immutable frames. No work queues here.
    void frameAvailable(FramePtr frame);
    void cameraInputChanged(const QString& sourceId, bool streaming);
    // Calibration uses physical identity independently from capture access paths.
    void calibrationInputChanged(const QString& identity, bool streaming, bool persistent);
    void frameDescriptionChanged(const QString& description);
    // Active includes connecting/disconnecting, so the UI can cancel a connection
    // attempt without treating a pending connection as connected.
    void connectionStatusChanged(const QString& status, bool connected, bool active);
    void telemetryDescriptionChanged(const QString& description);
    void worldDescriptionChanged(const QString& description);
    void workspaceDescriptionChanged(const QString& description);
    void robotSessionsStopped(); // All managed sessions drained during shutdown.
    void errorOccurred(const QString& message);

private:
    class EventSink;
    void publishWorld();
    void publishRobotChoices();
    void publishRobotActions();
    void refreshManualDrive();
    [[nodiscard]] bool canManualDrive() const;
    [[nodiscard]] bool selectedAmbiguous() const;
    void publishActiveCamera();
    void publishCameraActions();
    void requestCameraStop();
    void applyCameraReplacement();
    void receiveFrame(FramePtr frame);
    void receiveCameraState(CameraState state);
    void receiveConnectionState(RobotId id, ConnectionState state);
    void receiveTelemetry(const RobotTelemetry& telemetry);
    [[nodiscard]] double elapsedSeconds(Timestamp at) const;

    ManualControlConfig manualConfig_;
    QTimer manualRefresh_;
    std::optional<VelocityCommand> manualVelocity_;
    std::map<RobotId, DiscoveredRobot> discoveredRobots_;
    bool scanningRobots_{};
    bool manualFault_{};
    std::optional<RobotOperatingState> reportedOperatingState_;
    RobotManager& robots_;
    WorldModel& world_;
    RobotId robotId_;
    LogChannel log_;
    std::shared_ptr<EventSink> events_;
    std::unique_ptr<ICameraSource> camera_;
    std::unique_ptr<ICameraSource> pendingCamera_;
    std::optional<CameraState> cameraRequest_;
    QTimer frameExpiry_;
    Timestamp startedAt_{};
    std::uint64_t cameraPublicationGeneration_{};
    QString workspaceCalibrationDescription_{QStringLiteral("camera-to-world calibration not established")};
    bool initialized_{};
    bool shuttingDown_{};
    bool robotSessionsStoppedEmitted_{};
    bool replacementPending_{};
    bool replacementScheduled_{};
};

} // namespace micro_robot
