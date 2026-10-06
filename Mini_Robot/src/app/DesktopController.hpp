#pragma once

#include "camera/ICameraSource.hpp"
#include "transport/IRobotTransport.hpp"
#include "world/WorldModel.hpp"

#include <QObject>
#include <QString>

#include <memory>

namespace micro_robot {

// Qt application adapter. Dependencies must outlive the controller; all calls and
// backend callbacks run on its owning thread. No concrete backend or widgets here.
class DesktopController final : public QObject {
    Q_OBJECT

public:
    DesktopController(ICameraSource& camera, IRobotTransport& transport,
                      WorldModel& world, RobotId robotId, QObject* parent = nullptr);
    ~DesktopController() override;

    void initialize(Timestamp now);
    [[nodiscard]] PoseUpdateResult applyPose(RobotId id, const Pose2D& pose, Timestamp at);

public slots:
    void startCamera();
    void stopCamera();
    void connectRobot();
    void disconnectRobot();
    // Composition code must drain backend stop/disconnect events before destruction.
    void shutdown();

signals:
    void applicationStatusChanged(const QString& status);
    void cameraStatusChanged(const QString& status, bool active);
    void frameDescriptionChanged(const QString& description);
    void connectionStatusChanged(const QString& status, bool connected);
    void telemetryDescriptionChanged(const QString& description);
    void worldDescriptionChanged(const QString& description);
    void errorOccurred(const QString& message);

private:
    class EventSink;
    void publishWorld();
    void receiveFrame(FramePtr frame);
    void receiveCameraState(CameraState state);
    void receiveConnectionState(RobotId id, ConnectionState state);
    void receiveTelemetry(const RobotTelemetry& telemetry);
    [[nodiscard]] double elapsedSeconds(Timestamp at) const;

    ICameraSource& camera_;
    IRobotTransport& transport_;
    WorldModel& world_;
    RobotId robotId_;
    std::shared_ptr<EventSink> events_;
    Timestamp startedAt_{};
    bool initialized_{};
    bool shuttingDown_{};
};

} // namespace micro_robot
