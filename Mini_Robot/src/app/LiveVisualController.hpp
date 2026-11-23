#pragma once
#include "app/DesktopController.hpp"
#include "app/QtControlLoopScheduler.hpp"
#include "control/GoalPointController.hpp"
#include "vision/MetricObservation.hpp"
#include "world/Workspace.hpp"
#include <string_view>
#include <functional>
#include <QTimer>
#include <cstdint>
#include <filesystem>
#include <string_view>

namespace micro_robot {
class ControlTraceWriter;
struct LiveVisualSettings {
    static constexpr auto period = std::chrono::milliseconds{50};
    bool motionEnabled{}; // Composition root enables this only for simulation pending physical gates.
    ControlLoopConfig loop;
    GoalPointControllerConfig goal;
    // Software bring-up settings, not physical commissioning or actuator limits.
    static LiveVisualSettings fromApplication(const ApplicationConfig& config);
};

// Same-owner-thread application coordinator. Consumes accepted metric metadata,
// never images. WorldModel remains authoritative; explicit Start is always needed.
// Desktop/manager/world, optional trace callback and experiment writer outlive this object.
class LiveVisualController final : public QObject {
    Q_OBJECT
public:
    using Trace = std::function<void(std::string)>;
    using ImagePointUndistorter = std::function<std::optional<ImageCorner>(ImageCorner, const IntrinsicCalibration&)>;
    using ImagePointDistorter = std::function<std::optional<ImageCorner>(ImageCorner, const IntrinsicCalibration&)>;
    LiveVisualController(DesktopController& desktop, WorldModel& world, RobotManager& robots,
                         LiveVisualSettings settings, Trace trace = {},
                         ControlLoop::Clock clock = MonotonicClock::now, QObject* parent = nullptr,
                         ControlTraceWriter* experimentWriter = nullptr);
    ~LiveVisualController() override;
    void setInput(const QString& source, bool streaming);
    void setImagePointUndistorter(ImagePointUndistorter undistorter);
    void setImagePointDistorter(ImagePointDistorter distorter);
    // Invoke after RobotTracker ingestion. Metadata must match the authoritative
    // accepted timestamp; duplicate/old metadata cannot replace newer provenance.
    void receiveMeasurements(const MetricObservationBatchPtr& batch);
    void refresh();
    [[nodiscard]] bool running() const;
    [[nodiscard]] ControlOperationalState operationalState() const;
    [[nodiscard]] bool canAcknowledgeControllerFault() const;
    [[nodiscard]] QString description() const { return description_; }
    [[nodiscard]] std::optional<ControlCycle> lastCycle() const;
    [[nodiscard]] QString workspaceCalibrationRevision() const;
    [[nodiscard]] QString imageSource() const { return source_; }
    [[nodiscard]] RobotId selectedRobotId() const noexcept { return desktop_.selectedRobot(); }
    [[nodiscard]] std::optional<std::string> workspaceFrameId() const { return world_.workspaceFrameId(); }
    [[nodiscard]] std::optional<WorkspaceBounds> workspaceBounds() const { return world_.workspaceBounds(); }
    // Metadata-only preview gate: deliberately does not project any image point.
    [[nodiscard]] bool canMapImage(const QString& source, std::uint32_t width, std::uint32_t height,
                                   Timestamp capturedAt, const QString& workspaceRevision) const;
    [[nodiscard]] bool canEditPathFromImage(const QString& source, std::uint32_t width,
        std::uint32_t height, Timestamp capturedAt, const QString& workspaceRevision) const;
    // Pure coordinate conversions with live source/frame/revision and freshness
    // gates. They never set a target or issue motion.
    [[nodiscard]] std::optional<WorldPoint2D> mapImagePointToWorld(
        const QString& source, std::uint32_t width, std::uint32_t height, double u, double v,
        Timestamp capturedAt, const QString& workspaceRevision) const;
    [[nodiscard]] std::optional<ImageCorner> mapWorldPointToRawImage(WorldPoint2D point) const;
    [[nodiscard]] std::optional<Pose2D> targetPose() const { return target_; }
public slots:
    bool setTarget(double xMeters, double yMeters, double headingRadians);
    bool setTargetFromImage(const QString& source, std::uint32_t width, std::uint32_t height,
                            double u, double v, double headingRadians, Timestamp capturedAt,
                            const QString& workspaceRevision);
    bool start();
    void pause();
    void cancel();
    bool acknowledgeControllerFault();
    void stop();
signals:
    void descriptionChanged(const QString& text);
    void actionsChanged(bool canStart, bool canStop);
    void targetAvailabilityChanged(bool available);
    void controllerFaultAcknowledgementChanged(bool available);
private:
    void stopFor(std::string_view reason);
    void afterCycle();
    void record();
    bool startExperimentRun();
    void finishExperimentRun(std::string_view reason);
    void publish();
    bool ready() const;
    DesktopController& desktop_;
    WorldModel& world_;
    RobotManager& robots_;
    LiveVisualSettings settings_;
    GoalPointController policy_;
    Trace trace_;
    ControlTraceWriter* experimentWriter_{}; // Borrowed; must outlive this controller.
    ControlLoop::Clock clock_;
    RobotId selected_;
    std::optional<Pose2D> target_;
    std::optional<std::string> targetFrame_;
    QString targetCalibrationRevision_;
    std::optional<MetricPoseMeasurement> provenance_;
    std::optional<std::string> provenanceFrame_;
    std::unique_ptr<ControlLoop> loop_;
    std::unique_ptr<QtControlLoopScheduler> scheduler_;
    QTimer exportStatusTimer_;
    std::uint64_t nextSequence_{1};
    std::uint64_t nextRun_{1};
    std::uint64_t nextSample_{1};
    std::size_t droppedSamples_{};
    std::size_t samplesQueued_{};
    std::string runId_;
    std::string lastRecordedKey_;
    std::optional<std::filesystem::path> runExportPath_;
    bool experimentActive_{};
    QString source_, status_{QStringLiteral("Idle — apply a target and Start")}, description_;
    bool streaming_{};
    ImagePointUndistorter imagePointUndistorter_;
    ImagePointDistorter imagePointDistorter_;
};
} // namespace micro_robot
