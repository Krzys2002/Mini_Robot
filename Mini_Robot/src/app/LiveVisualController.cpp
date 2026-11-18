#include "app/LiveVisualController.hpp"
#include <QJsonDocument>
#include <QJsonObject>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace micro_robot {
LiveVisualSettings LiveVisualSettings::fromApplication(const ApplicationConfig& c)
{
    LiveVisualSettings settings;
    settings.loop = {std::min(period, c.manual.refreshInterval), c.manual.commandValidity, c.tracking.staleAfter,
        c.manual.maximumLinearMetersPerSecond, c.manual.maximumAngularRadiansPerSecond};
    settings.goal = {c.visualControl.positionGainPerSecond, c.visualControl.bearingGainPerSecond,
        c.visualControl.finalHeadingGainPerSecond, c.manual.maximumLinearMetersPerSecond,
        c.manual.maximumAngularRadiansPerSecond, Meters{c.visualControl.positionToleranceMeters},
        Radians{c.visualControl.headingToleranceRadians}, Radians{c.visualControl.turnInPlaceRadians},
        c.visualControl.alignFinalHeading, Meters{c.visualControl.positionExitToleranceMeters}};
    return settings;
}
LiveVisualController::LiveVisualController(DesktopController& desktop, WorldModel& world, RobotManager& robots,
    LiveVisualSettings settings, Trace trace, ControlLoop::Clock clock, QObject* parent)
    : QObject(parent), desktop_(desktop), world_(world), robots_(robots), settings_(settings), policy_(settings.goal),
      trace_(std::move(trace)), clock_(std::move(clock)), selected_(desktop.selectedRobot())
{
    if (!clock_) throw std::invalid_argument("Visual control requires a clock");
    ControlLoop validate{selected_, world_, robots_, policy_, settings_.loop, clock_};
    if (!settings_.motionEnabled)
        status_ = QStringLiteral("Live visual motion is disabled for this transport; physical acceptance is incomplete");
    connect(&desktop_, &DesktopController::motionInterruptRequested, this, &LiveVisualController::stop);
    connect(&desktop_, &DesktopController::cameraInputChanged, this, &LiveVisualController::setInput);
    connect(&desktop_, &DesktopController::robotChoicesChanged, this, [this](const auto&, quint32 id) {
        if (selected_ != RobotId{id}) {
            stop(); selected_ = RobotId{id}; target_.reset(); provenance_.reset();
        }
        if (running() && !desktop_.externalControlPermitted()) stop();
        publish();
    });
    publish();
}
LiveVisualController::~LiveVisualController() { stop(); }
bool LiveVisualController::running() const { return loop_ && loop_->state() == ControlLoopState::Running; }
std::optional<ControlCycle> LiveVisualController::lastCycle() const { return loop_ ? loop_->lastCycle() : std::nullopt; }
void LiveVisualController::setInput(const QString& source, bool streaming)
{
    if (source_ != source || !streaming) { stop(); provenance_.reset(); }
    source_ = source; streaming_ = streaming;
    publish();
}
void LiveVisualController::receiveMeasurements(const MetricObservationBatchPtr& batch)
{
    if (!batch || batch->status != MetricObservationStatus::Valid) {
        provenance_.reset(); stop();
    } else {
        for (const auto& m : batch->measurements) {
            const auto robot = world_.robot(selected_);
            if (m.robotId == selected_ && robot && robot->tracking && robot->poseUpdatedAt == m.capturedAt
                && (!provenance_ || provenanceFrame_ != world_.workspaceFrameId() || m.capturedAt > provenance_->capturedAt)) {
                provenance_ = m; provenanceFrame_ = world_.workspaceFrameId();
            }
        }
    }
    refresh();
}
bool LiveVisualController::setTarget(double x, double y, double heading)
{
    if (running()) return false;
    const auto bounds = world_.workspaceBounds();
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(heading)
        || !world_.workspaceFrameId() || !bounds || !bounds->contains({Meters{x}, Meters{y}})) {
        status_ = QStringLiteral("Target rejected — use finite coordinates inside the active calibrated workspace");
        publish(); return false;
    }
    target_ = Pose2D{Meters{x}, Meters{y}, Radians{heading}};
    targetFrame_ = world_.workspaceFrameId();
    status_ = QStringLiteral("Target applied — Start required"); publish(); return true;
}
bool LiveVisualController::ready() const
{
    const auto robot = world_.robot(selected_);
    return settings_.motionEnabled && streaming_ && target_ && targetFrame_ == world_.workspaceFrameId()
        && world_.workspaceFrameId() && provenanceFrame_ == world_.workspaceFrameId()
        && provenance_ && robot && robot->poseUpdatedAt == provenance_->capturedAt
        && world_.tracking(selected_, clock_()).status == TrackingStatus::Fresh
        && desktop_.externalControlPermitted();
}
bool LiveVisualController::start()
{
    if (running()) return true;
    if (!settings_.motionEnabled) {
        status_ = QStringLiteral("Live visual control disabled for this transport — physical acceptance is incomplete");
        publish(); return false;
    }
    if (!ready()) { status_ = QStringLiteral("Not ready — need a target, calibrated live measurement and ready robot session"); publish(); return false; }
    desktop_.cancelManualDrive();
    policy_.reset();
    scheduler_.reset(); loop_.reset();
    loop_ = std::make_unique<ControlLoop>(selected_, world_, robots_, policy_, settings_.loop, clock_,
        [this] { return nextSequence_++; });
    if (!loop_->setTarget(*target_) || !loop_->start()) { publish(); return false; }
    desktop_.setExternalControlActive(true);
    scheduler_ = std::make_unique<QtControlLoopScheduler>(*loop_);
    connect(scheduler_.get(), &QtControlLoopScheduler::cycleCompleted, this, &LiveVisualController::afterCycle);
    status_ = QStringLiteral("Running");
    if (!scheduler_->start()) {
        desktop_.setExternalControlActive(false);
        status_ = QStringLiteral("Paused — scheduler could not start; explicit Start required");
        publish(); return false;
    }
    if (running()) publish();
    return true;
}
void LiveVisualController::stop()
{
    const bool active = running();
    if (scheduler_) scheduler_->stop();
    if (active) { record(); status_ = QStringLiteral("Stopped — explicit Start required"); }
    desktop_.setExternalControlActive(false);
    publish();
}
void LiveVisualController::refresh()
{
    if (target_ && targetFrame_ != world_.workspaceFrameId()) { stop(); target_.reset(); }
    if (provenanceFrame_ != world_.workspaceFrameId()) provenance_.reset();
    if (running()) {
        if (!streaming_ || !desktop_.externalControlPermitted() || !provenance_) stop();
        else if (loop_->poll()) afterCycle();
    }
    publish();
}
void LiveVisualController::afterCycle()
{
    record();
    if (!running()) {
        if (scheduler_) scheduler_->stop();
        desktop_.setExternalControlActive(false);
        const auto result = loop_->lastCycle()->result;
        status_ = result == ControlCycleResult::TrackingUnavailable
            ? QStringLiteral("Paused — tracking stale or unavailable; explicit Start required")
            : result == ControlCycleResult::SessionUnavailable
                ? QStringLiteral("Paused — robot session not ready; explicit Start required")
                : result == ControlCycleResult::FrameChanged
                    ? QStringLiteral("Paused — workspace changed; apply a new target")
                    : QStringLiteral("Paused — control/timing unavailable; explicit Start required");
    } else if (policy_.controlState() == GoalControlState::Arrived) {
        stop(); status_ = QStringLiteral("Arrived — zero/Stop submitted");
    }
    publish();
}
void LiveVisualController::record()
{
    if (!trace_ || !loop_ || !loop_->lastCycle()) return;
    const auto c = *loop_->lastCycle();
    const auto stamp = [](Timestamp t) { return QString::number(std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count()); };
    QJsonObject o{{"robot_id", static_cast<int>(selected_.value())}, {"source", source_},
        {"camera_time_ns", c.observedAt ? QJsonValue(stamp(*c.observedAt)) : QJsonValue()},
        {"frame_sequence", provenance_ && c.observedAt == provenance_->capturedAt ? QJsonValue(QString::number(provenance_->frameSequence)) : QJsonValue()},
        {"control_time_ns", stamp(c.startedAt)}, {"submission_time_ns", stamp(c.finishedAt)},
        {"domain_sequence", c.commandSequence ? QJsonValue(QString::number(*c.commandSequence)) : QJsonValue()},
        {"stop_sequence", c.stopSequence ? QJsonValue(QString::number(*c.stopSequence)) : QJsonValue()},
        {"result", static_cast<int>(c.result)}, {"wire_sequence", QJsonValue()}};
    o["camera_timestamp_source"] = QStringLiteral("Frame.capturedAt; exposure time unverified");
    o["scheduled_time_ns"] = stamp(c.scheduledAt);
    o["lateness_ns"] = QString::number(std::chrono::duration_cast<std::chrono::nanoseconds>(c.lateness).count());
    o["command_submission"] = c.submission ? QJsonValue(static_cast<int>(*c.submission)) : QJsonValue();
    o["stop_submission"] = c.stopSubmission ? QJsonValue(static_cast<int>(*c.stopSubmission)) : QJsonValue();
    o["stop_command"] = c.stopSequence.has_value();
    o["command_type"] = c.bounded ? QStringLiteral("velocity") : QStringLiteral("stop");
    o["frame_id"] = QString::fromStdString(world_.workspaceFrameId().value_or(""));
    if (c.bounded) { o["v_m_s"] = c.bounded->linearMetersPerSecond; o["omega_rad_s"] = c.bounded->angularRadiansPerSecond; }
    if (c.error) { o["error_x_m"] = c.error->xMeters; o["error_y_m"] = c.error->yMeters; o["error_heading_rad"] = c.error->headingRadians; }
    trace_(QJsonDocument(o).toJson(QJsonDocument::Compact).toStdString());
}
void LiveVisualController::publish()
{
    const auto robot = world_.robot(selected_);
    description_ = QStringLiteral("Robot %1 · %2").arg(selected_.value()).arg(status_);
    if (robot && robot->pose) description_ += QStringLiteral("\nActual: x %1 m · y %2 m · heading %3 rad")
        .arg(robot->pose->x.value, 0, 'f', 3).arg(robot->pose->y.value, 0, 'f', 3).arg(robot->pose->heading.value, 0, 'f', 3);
    else description_ += QStringLiteral("\nActual pose unavailable");
    const auto tracking = world_.tracking(selected_, clock_());
    const auto trackingText = tracking.status == TrackingStatus::Fresh ? QStringLiteral("Fresh")
        : tracking.status == TrackingStatus::Stale ? QStringLiteral("Stale")
        : tracking.status == TrackingStatus::Lost ? QStringLiteral("Lost") : QStringLiteral("Unknown");
    description_ += QStringLiteral("\nTracking: %1").arg(trackingText);
    if (tracking.age) description_ += QStringLiteral(" · observation age %1 ms")
        .arg(std::chrono::duration<double, std::milli>(*tracking.age).count(), 0, 'f', 0);
    if (target_) description_ += QStringLiteral("\nTarget: x %1 m · y %2 m · heading %3 rad")
        .arg(target_->x.value, 0, 'f', 3).arg(target_->y.value, 0, 'f', 3).arg(target_->heading.value, 0, 'f', 3);
    emit descriptionChanged(description_);
    emit actionsChanged(!running() && ready(), running());
}
} // namespace micro_robot
