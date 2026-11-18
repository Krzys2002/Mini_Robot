#pragma once
#include "app/DesktopController.hpp"
#include "app/QtControlLoopScheduler.hpp"
#include "control/GoalPointController.hpp"
#include "vision/MetricObservation.hpp"

namespace micro_robot {
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
// Desktop/manager/world and optional nonblocking trace callback outlive this object.
class LiveVisualController final : public QObject {
    Q_OBJECT
public:
    using Trace = std::function<void(std::string)>;
    LiveVisualController(DesktopController& desktop, WorldModel& world, RobotManager& robots,
                         LiveVisualSettings settings, Trace trace = {},
                         ControlLoop::Clock clock = MonotonicClock::now, QObject* parent = nullptr);
    ~LiveVisualController() override;
    void setInput(const QString& source, bool streaming);
    // Invoke after RobotTracker ingestion. Metadata must match the authoritative
    // accepted timestamp; duplicate/old metadata cannot replace newer provenance.
    void receiveMeasurements(const MetricObservationBatchPtr& batch);
    void refresh();
    [[nodiscard]] bool running() const;
    [[nodiscard]] ControlOperationalState operationalState() const;
    [[nodiscard]] bool canAcknowledgeControllerFault() const;
    [[nodiscard]] QString description() const { return description_; }
    [[nodiscard]] std::optional<ControlCycle> lastCycle() const;
public slots:
    bool setTarget(double xMeters, double yMeters, double headingRadians);
    bool start();
    bool acknowledgeControllerFault();
    void stop();
signals:
    void descriptionChanged(const QString& text);
    void actionsChanged(bool canStart, bool canStop);
    void controllerFaultAcknowledgementChanged(bool available);
private:
    void afterCycle();
    void record();
    void publish();
    bool ready() const;
    DesktopController& desktop_;
    WorldModel& world_;
    RobotManager& robots_;
    LiveVisualSettings settings_;
    GoalPointController policy_;
    Trace trace_;
    ControlLoop::Clock clock_;
    RobotId selected_;
    std::optional<Pose2D> target_;
    std::optional<std::string> targetFrame_;
    std::optional<MetricPoseMeasurement> provenance_;
    std::optional<std::string> provenanceFrame_;
    std::unique_ptr<ControlLoop> loop_;
    std::unique_ptr<QtControlLoopScheduler> scheduler_;
    std::uint64_t nextSequence_{1};
    QString source_, status_{QStringLiteral("Idle — apply a target and Start")}, description_;
    bool streaming_{};
};
} // namespace micro_robot
