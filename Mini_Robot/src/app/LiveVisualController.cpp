#include "app/LiveVisualController.hpp"
#include "app/ControlTraceWriter.hpp"
#include "common/Angle.hpp"
#include "calibration/TabletopHomography.hpp"
#include <QJsonDocument>
#include <QJsonObject>
#include <QDateTime>
#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <string_view>

namespace micro_robot {
namespace {
QString operationalStateLabel(ControlOperationalState state)
{
    switch (state) {
    case ControlOperationalState::Ready: return QStringLiteral("READY");
    case ControlOperationalState::Running: return QStringLiteral("RUNNING");
    case ControlOperationalState::Paused: return QStringLiteral("PAUSED");
    case ControlOperationalState::LostTracking: return QStringLiteral("LOST_TRACKING");
    case ControlOperationalState::LostConnection: return QStringLiteral("LOST_CONNECTION");
    case ControlOperationalState::EStop: return QStringLiteral("ESTOP");
    case ControlOperationalState::Fault: return QStringLiteral("FAULT");
    }
    return QStringLiteral("FAULT");
}
QString cycleResultLabel(ControlCycleResult result)
{
    switch (result) {
    case ControlCycleResult::Submitted: return QStringLiteral("submitted");
    case ControlCycleResult::Stopped: return QStringLiteral("stopped");
    case ControlCycleResult::InvalidClock: return QStringLiteral("invalid_clock");
    case ControlCycleResult::TrackingUnavailable: return QStringLiteral("tracking_unavailable");
    case ControlCycleResult::SessionUnavailable: return QStringLiteral("session_unavailable");
    case ControlCycleResult::FrameChanged: return QStringLiteral("frame_changed");
    case ControlCycleResult::ControllerUnavailable: return QStringLiteral("controller_unavailable");
    case ControlCycleResult::ControllerFailure: return QStringLiteral("controller_failure");
    case ControlCycleResult::InvalidOutput: return QStringLiteral("invalid_output");
    case ControlCycleResult::SubmissionRejected: return QStringLiteral("submission_rejected");
    }
    return QStringLiteral("unknown");
}
QString submissionLabel(CommandSubmission submission)
{
    switch (submission) {
    case CommandSubmission::Submitted: return QStringLiteral("submitted");
    case CommandSubmission::UnknownRobot: return QStringLiteral("unknown_robot");
    case CommandSubmission::NotConnected: return QStringLiteral("not_connected");
    case CommandSubmission::ShuttingDown: return QStringLiteral("shutting_down");
    case CommandSubmission::EmergencyStopped: return QStringLiteral("emergency_stopped");
    case CommandSubmission::SafetyFault: return QStringLiteral("safety_fault");
    case CommandSubmission::StaleSession: return QStringLiteral("stale_session");
    }
    return QStringLiteral("unknown");
}
std::string_view motionInterruptReason(MotionInterruptReason reason)
{
    switch (reason) {
    case MotionInterruptReason::TransportError: return "safety_fault";
    case MotionInterruptReason::CameraChanged: return "camera_change";
    case MotionInterruptReason::CameraStopped: return "camera_stopped";
    case MotionInterruptReason::Disconnect: return "disconnect";
    case MotionInterruptReason::Shutdown: return "shutdown";
    case MotionInterruptReason::ConnectionLost: return "disconnect";
    case MotionInterruptReason::SafetyFault: return "safety_fault";
    case MotionInterruptReason::Scan: return "scan";
    case MotionInterruptReason::RobotSelection: return "robot_selection";
    case MotionInterruptReason::ManualTakeover: return "manual_takeover";
    case MotionInterruptReason::ManualStop: return "manual_stop";
    case MotionInterruptReason::EmergencyStop: return "emergency_stop";
    case MotionInterruptReason::EmergencyClear: return "emergency_clear";
    case MotionInterruptReason::WorkspaceChanged: return "workspace_change";
    }
    return "interrupted";
}
std::string_view cycleStopReason(const ControlCycle& cycle, ControlOperationalState state)
{
    if (state == ControlOperationalState::EStop) return "emergency_stop";
    switch (cycle.result) {
    case ControlCycleResult::Stopped:
        return state == ControlOperationalState::Fault ? "safety_fault" : "operator_stop";
    case ControlCycleResult::InvalidClock: return "clock_reversal";
    case ControlCycleResult::TrackingUnavailable: return "tracking_lost";
    case ControlCycleResult::SessionUnavailable:
        return state == ControlOperationalState::Fault ? "safety_fault"
            : state == ControlOperationalState::LostConnection ? "disconnect" : "session_unavailable";
    case ControlCycleResult::FrameChanged: return "workspace_change";
    case ControlCycleResult::ControllerUnavailable: return "controller_unavailable";
    case ControlCycleResult::ControllerFailure: return "controller_failure";
    case ControlCycleResult::InvalidOutput: return "invalid_output";
    case ControlCycleResult::SubmissionRejected:
        switch (cycle.submission.value_or(CommandSubmission::SafetyFault)) {
        case CommandSubmission::StaleSession: return "stale_session";
        case CommandSubmission::EmergencyStopped: return "emergency_stop";
        case CommandSubmission::SafetyFault: return "safety_fault";
        case CommandSubmission::NotConnected: return "disconnect";
        case CommandSubmission::ShuttingDown: return "shutdown";
        case CommandSubmission::UnknownRobot: return "robot_unavailable";
        case CommandSubmission::Submitted: return "submission_rejected";
        }
        return "submission_rejected";
    case ControlCycleResult::Submitted: return "completed";
    }
    return "control_inhibited";
}
}
LiveVisualSettings LiveVisualSettings::fromApplication(const ApplicationConfig& c)
{
    LiveVisualSettings settings;
    // ManualControlConfig is the single app-level source for both the policy's
    // request caps and the loop's final submission caps.
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
    LiveVisualSettings settings, Trace trace, ControlLoop::Clock clock, QObject* parent,
    ControlTraceWriter* experimentWriter)
    : QObject(parent), desktop_(desktop), world_(world), robots_(robots), settings_(settings), policy_(settings.goal),
      trace_(std::move(trace)), experimentWriter_(experimentWriter), clock_(std::move(clock)), selected_(desktop.selectedRobot())
{
    if (!clock_) throw std::invalid_argument("Visual control requires a clock");
    exportStatusTimer_.setInterval(static_cast<int>(LiveVisualSettings::period.count()));
    connect(&exportStatusTimer_, &QTimer::timeout, this, [this] {
        if (!experimentWriter_ || !runExportPath_) {
            exportStatusTimer_.stop();
            return;
        }
        const auto state = experimentWriter_->runExportState();
        publish();
        if (state == ControlRunExportState::Failed || state == ControlRunExportState::Complete)
            exportStatusTimer_.stop();
    });
    ControlLoop validate{selected_, world_, robots_, policy_, settings_.loop, clock_};
    if (!settings_.motionEnabled)
        status_ = QStringLiteral("Live visual motion is disabled for this transport; physical acceptance is incomplete");
    connect(&desktop_, &DesktopController::motionInterruptRequested, this,
        [this](MotionInterruptReason reason) { stopFor(motionInterruptReason(reason)); });
    connect(&desktop_, &DesktopController::cameraInputChanged, this, &LiveVisualController::setInput);
    connect(&desktop_, &DesktopController::robotChoicesChanged, this, [this](const auto&, quint32 id) {
        if (selected_ != RobotId{id}) {
            stop();
            scheduler_.reset();
            loop_.reset();
            policy_.reset();
            selected_ = RobotId{id}; target_.reset(); provenance_.reset();
            targetFrame_.reset(); targetCalibrationRevision_.clear(); provenanceFrame_.reset();
            status_ = QStringLiteral("Idle — apply a target and Start");
        }
        if (running() && !desktop_.externalControlPermitted()) stop();
        publish();
    });
    publish();
}
LiveVisualController::~LiveVisualController() { stop(); }
bool LiveVisualController::running() const { return loop_ && loop_->state() == ControlLoopState::Running; }
ControlOperationalState LiveVisualController::operationalState() const
{
    const auto sessionState = classifySessionOperationalState(robots_.session(selected_));
    if (sessionState != ControlOperationalState::Ready) return sessionState;
    if (loop_ && loop_->operationalState() == ControlOperationalState::Fault)
        return ControlOperationalState::Fault;
    const auto tracking = world_.tracking(selected_, clock_());
    const auto robot = world_.robot(selected_);
    if (!streaming_ || !provenance_ || tracking.status != TrackingStatus::Fresh || !tracking.age
        || *tracking.age >= settings_.loop.maximumObservationAge
        || !robot || robot->poseUpdatedAt != provenance_->capturedAt)
        return ControlOperationalState::LostTracking;
    return loop_ ? loop_->operationalState() : ControlOperationalState::Ready;
}
bool LiveVisualController::canAcknowledgeControllerFault() const
{
    return loop_ && loop_->hasLatchedControllerFault()
        && classifySessionOperationalState(robots_.session(selected_)) == ControlOperationalState::Ready
        && ready();
}
std::optional<ControlCycle> LiveVisualController::lastCycle() const { return loop_ ? loop_->lastCycle() : std::nullopt; }
QString LiveVisualController::workspaceCalibrationRevision() const
{
    return QString::fromStdString(desktop_.workspaceCalibrationRevision());
}
void LiveVisualController::setImagePointUndistorter(ImagePointUndistorter undistorter)
{
    imagePointUndistorter_ = std::move(undistorter);
}
void LiveVisualController::setInput(const QString& source, bool streaming)
{
    if (source_ != source || !streaming) { stopFor(streaming ? "camera_change" : "camera_stopped"); provenance_.reset(); }
    source_ = source; streaming_ = streaming;
    publish();
}
void LiveVisualController::receiveMeasurements(const MetricObservationBatchPtr& batch)
{
    if (!batch || batch->status != MetricObservationStatus::Valid) {
        provenance_.reset(); stopFor("invalid_measurement");
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
    targetCalibrationRevision_ = workspaceCalibrationRevision();
    status_ = QStringLiteral("Target applied — Start required"); publish(); return true;
}
bool LiveVisualController::setTargetFromImage(const QString& source, std::uint32_t width, std::uint32_t height,
    double u, double v, double heading, Timestamp capturedAt, const QString& workspaceRevision)
{
    if (running()) return false;
    const auto& profile = desktop_.workspaceCalibrationProfile();
    const auto activeRevision = workspaceCalibrationRevision();
    const auto activeFrame = world_.workspaceFrameId();
    const auto age = clock_() - capturedAt;
    if (!imagePointUndistorter_ || !profile || !profile->homography
        || !validateWorkspaceCalibrationProfile(*profile).empty()
        || activeRevision.isEmpty() || workspaceRevision != activeRevision
        || !activeFrame || *activeFrame != profile->frameId
        || source != source_ || source != QString::fromStdString(profile->intrinsics.sourceId)
        || !streaming_
        || width != profile->intrinsics.width || height != profile->intrinsics.height
        || !std::isfinite(u) || !std::isfinite(v) || !std::isfinite(heading)
        || u < 0 || v < 0 || u > width - 1 || v > height - 1
        || age < Duration::zero() || age >= settings_.loop.maximumObservationAge
        || (provenance_ && capturedAt < provenance_->capturedAt)) {
        status_ = QStringLiteral("Image target rejected — stale frame or camera, resolution, or calibration changed");
        publish();
        return false;
    }
    try {
        const auto undistorted = imagePointUndistorter_({u, v}, profile->intrinsics);
        if (!undistorted) throw std::runtime_error("Point undistortion failed");
        const auto world = projectWorkspacePoint(*profile->homography, *undistorted);
        if (!world) throw std::runtime_error("Workspace projection failed");
        return setTarget(world->x.value, world->y.value, heading);
    } catch (const std::exception&) {
        status_ = QStringLiteral("Image target rejected — calibration could not map this point");
        publish();
        return false;
    }
}
bool LiveVisualController::ready() const
{
    const auto robot = world_.robot(selected_);
    return settings_.motionEnabled && streaming_ && target_ && targetFrame_ == world_.workspaceFrameId()
        && targetCalibrationRevision_ == workspaceCalibrationRevision()
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
    if (!ready()) { status_ = QStringLiteral("%1 — explicit Start requires target, fresh measurement and safe session").arg(operationalStateLabel(operationalState())); publish(); return false; }
    desktop_.cancelManualDrive();
    policy_.reset();
    scheduler_.reset();
    if (!loop_) loop_ = std::make_unique<ControlLoop>(selected_, world_, robots_, policy_, settings_.loop, clock_,
        [this] { return nextSequence_++; });
    if (!loop_->setTarget(*target_) || !loop_->start()) { publish(); return false; }
    desktop_.setExternalControlActive(true);
    scheduler_ = std::make_unique<QtControlLoopScheduler>(*loop_);
    connect(scheduler_.get(), &QtControlLoopScheduler::cycleCompleted, this, &LiveVisualController::afterCycle);
    status_ = QStringLiteral("Running");
    if (!startExperimentRun()) status_ = QStringLiteral("Running — experiment export unavailable");
    if (!scheduler_->start()) {
        finishExperimentRun("scheduler_start_failed");
        desktop_.setExternalControlActive(false);
        status_ = QStringLiteral("Paused — scheduler could not start; explicit Start required");
        publish(); return false;
    }
    if (running()) publish();
    return true;
}
bool LiveVisualController::acknowledgeControllerFault()
{
    if (!canAcknowledgeControllerFault() || !loop_->acknowledgeControllerFault()) return false;
    policy_.reset();
    status_ = QStringLiteral("Controller fault acknowledged — explicit Start required");
    publish();
    return true;
}
void LiveVisualController::stop()
{
    stopFor("operator_stop");
}
void LiveVisualController::pause()
{
    if (!running()) return;
    stopFor("operator_stop");
    status_ = QStringLiteral("Paused — target retained; explicit Start required");
    publish();
}
void LiveVisualController::cancel()
{
    if (running()) stopFor("operator_stop");
    target_.reset();
    targetFrame_.reset();
    targetCalibrationRevision_.clear();
    status_ = QStringLiteral("Cancelled — target cleared");
    publish();
}
void LiveVisualController::stopFor(std::string_view reason)
{
    const bool active = running();
    const auto state = operationalState();
    if (state == ControlOperationalState::EStop) reason = "emergency_stop";
    else if (state == ControlOperationalState::Fault) reason = "safety_fault";
    else if (reason.empty()) reason = active ? "operator_stop" : "interrupted";
    if (scheduler_) scheduler_->stop();
    if (active) { record(); status_ = QStringLiteral("Stopped — explicit Start required"); }
    finishExperimentRun(reason);
    desktop_.setExternalControlActive(false);
    publish();
}
void LiveVisualController::refresh()
{
    if (target_ && (targetFrame_ != world_.workspaceFrameId()
        || targetCalibrationRevision_ != workspaceCalibrationRevision())) {
        stopFor("workspace_change"); target_.reset(); targetFrame_.reset(); targetCalibrationRevision_.clear();
    }
    if (provenanceFrame_ != world_.workspaceFrameId()) provenance_.reset();
    if (running()) {
        if (!streaming_) stopFor("camera_stopped");
        else if (!desktop_.externalControlPermitted()) {
            const auto state = operationalState();
            stopFor(state == ControlOperationalState::EStop ? "emergency_stop"
                : state == ControlOperationalState::Fault ? "safety_fault"
                : state == ControlOperationalState::LostConnection ? "disconnect" : "control_inhibited");
        } else if (!provenance_) stopFor("tracking_lost");
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
        const auto state = operationalState();
        const auto cycle = loop_->lastCycle();
        finishExperimentRun(cycle ? cycleStopReason(*cycle, state) : "control_inhibited");
    } else if (policy_.controlState() == GoalControlState::Arrived) {
        stopFor("arrival"); status_ = QStringLiteral("Arrived — zero/Stop submitted");
    }
    publish();
}
bool LiveVisualController::startExperimentRun()
{
    experimentActive_ = false;
    runExportPath_.reset();
    runId_.clear();
    nextSample_ = 1;
    droppedSamples_ = 0;
    samplesQueued_ = 0;
    lastRecordedKey_.clear();
    if (!experimentWriter_) return true;

    const auto now = clock_();
    runId_ = QStringLiteral("control-%1-robot-%2-%3")
        .arg(QDateTime::currentMSecsSinceEpoch()).arg(selected_.value()).arg(nextRun_++).toStdString();
    const auto& goal = policy_.config();
    QJsonObject header{
        {"type", QStringLiteral("run_start")}, {"schema_version", 1}, {"run_id", QString::fromStdString(runId_)},
        {"robot_id", static_cast<int>(selected_.value())}, {"source", source_},
        {"frame_id", QString::fromStdString(world_.workspaceFrameId().value_or(""))},
        {"started_monotonic_ns", QString::number(std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count())},
        {"clock", QStringLiteral("std::chrono::steady_clock")},
        {"coordinate_units", QStringLiteral("metres; radians")},
        {"controller", QStringLiteral("goal_point")},
        {"target_x_m", target_->x.value}, {"target_y_m", target_->y.value}, {"target_heading_rad", target_->heading.value},
        {"control_period_ms", static_cast<qint64>(settings_.loop.period.count())},
        {"command_validity_ms", static_cast<qint64>(settings_.loop.commandValidity.count())},
        {"maximum_observation_age_ns", QString::number(std::chrono::duration_cast<std::chrono::nanoseconds>(settings_.loop.maximumObservationAge).count())},
        {"maximum_linear_m_s", settings_.loop.maximumLinearMetersPerSecond},
        {"maximum_angular_rad_s", settings_.loop.maximumAngularRadiansPerSecond},
        {"position_gain_per_s", goal.positionGainPerSecond},
        {"bearing_gain_per_s", goal.bearingGainPerSecond},
        {"final_heading_gain_per_s", goal.finalHeadingGainPerSecond},
        {"position_tolerance_m", goal.positionTolerance.value},
        {"position_exit_tolerance_m", goal.positionExitTolerance.value},
        {"heading_tolerance_rad", goal.headingTolerance.value},
        {"turn_in_place_rad", goal.turnInPlaceAngle.value},
        {"align_final_heading", goal.alignFinalHeading}};
    const auto fileName = runId_ + ".jsonl";
    runExportPath_ = experimentWriter_->beginRun(fileName, QJsonDocument(header).toJson(QJsonDocument::Compact).toStdString());
    experimentActive_ = runExportPath_.has_value();
    if (experimentActive_) exportStatusTimer_.start();
    return experimentActive_;
}

void LiveVisualController::finishExperimentRun(std::string_view reason)
{
    if (!experimentActive_ || !experimentWriter_) return;
    const auto now = clock_();
    const QJsonObject footer{
        {"type", QStringLiteral("run_end")}, {"schema_version", 1}, {"run_id", QString::fromStdString(runId_)},
        {"ended_monotonic_ns", QString::number(std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count())},
        {"final_state", operationalStateLabel(operationalState())},
        {"reason", QString::fromUtf8(reason.data(), static_cast<qsizetype>(reason.size()))},
        {"samples_queued", QString::number(samplesQueued_)},
        {"dropped_samples", QString::number(droppedSamples_)},
        {"queue_complete", droppedSamples_ == 0}};
    (void)experimentWriter_->finishRun(QJsonDocument(footer).toJson(QJsonDocument::Compact).toStdString());
    experimentActive_ = false;
}

void LiveVisualController::record()
{
    if ((!trace_ && !experimentWriter_) || !loop_ || !loop_->lastCycle()) return;
    const auto c = *loop_->lastCycle();
    const auto stamp = [](Timestamp t) { return QString::number(std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count()); };
    const QString key = QStringLiteral("%1:%2:%3:%4")
        .arg(stamp(c.startedAt)).arg(static_cast<int>(c.result))
        .arg(c.commandSequence ? QString::number(*c.commandSequence) : QStringLiteral("-"))
        .arg(c.stopSequence ? QString::number(*c.stopSequence) : QStringLiteral("-"));
    if (key.toStdString() == lastRecordedKey_) return;
    lastRecordedKey_ = key.toStdString();

    QJsonObject o{{"type", QStringLiteral("sample")}, {"schema_version", 1},
        {"run_id", QString::fromStdString(runId_)}, {"sample_index", QString::number(nextSample_++)},
        {"robot_id", static_cast<int>(selected_.value())}, {"source", source_},
        {"camera_time_ns", c.observedAt ? QJsonValue(stamp(*c.observedAt)) : QJsonValue()},
        {"frame_sequence", provenance_ && c.observedAt == provenance_->capturedAt ? QJsonValue(QString::number(provenance_->frameSequence)) : QJsonValue()},
        {"control_time_ns", stamp(c.startedAt)}, {"submission_time_ns", stamp(c.finishedAt)},
        {"tracking_age_ns", c.observationAge ? QJsonValue(QString::number(std::chrono::duration_cast<std::chrono::nanoseconds>(*c.observationAge).count())) : QJsonValue()},
        {"domain_sequence", c.commandSequence ? QJsonValue(QString::number(*c.commandSequence)) : QJsonValue()},
        {"stop_sequence", c.stopSequence ? QJsonValue(QString::number(*c.stopSequence)) : QJsonValue()},
        {"result", cycleResultLabel(c.result)}, {"wire_sequence", QJsonValue()},
        {"operational_state", operationalStateLabel(c.operationalState)},
        {"frame_id", QString::fromStdString(c.frameId)},
        {"skipped_periods", QString::number(c.skippedPeriods)}};
    o["camera_timestamp_source"] = QStringLiteral("Frame.capturedAt; exposure time unverified");
    o["scheduled_time_ns"] = stamp(c.scheduledAt);
    o["lateness_ns"] = QString::number(std::chrono::duration_cast<std::chrono::nanoseconds>(c.lateness).count());
    o["command_submission"] = c.submission ? QJsonValue(submissionLabel(*c.submission)) : QJsonValue();
    o["stop_submission"] = c.stopSubmission ? QJsonValue(submissionLabel(*c.stopSubmission)) : QJsonValue();
    o["stop_command"] = c.stopSequence.has_value();
    o["command_type"] = c.bounded ? QStringLiteral("velocity") : QStringLiteral("stop");
    if (c.trackingConfidence) o["tracking_confidence"] = *c.trackingConfidence;
    if (c.measurementQuality) o["measurement_quality"] = *c.measurementQuality;
    if (c.rawPose) {
        o["raw_pose_x_m"] = c.rawPose->x.value;
        o["raw_pose_y_m"] = c.rawPose->y.value;
        o["raw_pose_heading_rad"] = c.rawPose->heading.value;
    }
    if (c.measuredPose) {
        o["pose_x_m"] = c.measuredPose->x.value;
        o["pose_y_m"] = c.measuredPose->y.value;
        o["pose_heading_rad"] = c.measuredPose->heading.value;
    }
    if (c.targetPose) {
        o["target_x_m"] = c.targetPose->x.value;
        o["target_y_m"] = c.targetPose->y.value;
        o["target_heading_rad"] = c.targetPose->heading.value;
    }
    if (c.requested) {
        o["requested_v_m_s"] = c.requested->linearMetersPerSecond;
        o["requested_omega_rad_s"] = c.requested->angularRadiansPerSecond;
    }
    if (c.bounded) {
        o["v_m_s"] = c.bounded->linearMetersPerSecond;
        o["omega_rad_s"] = c.bounded->angularRadiansPerSecond;
    }
    if (c.error) {
        o["error_x_m"] = c.error->xMeters;
        o["error_y_m"] = c.error->yMeters;
        o["error_heading_rad"] = c.error->headingRadians;
        o["error_distance_m"] = c.error->distanceMeters;
    }
    const auto line = QJsonDocument(o).toJson(QJsonDocument::Compact).toStdString();
    if (experimentWriter_) {
        if (experimentActive_) {
            if (experimentWriter_->enqueueSample(line)) ++samplesQueued_;
            else ++droppedSamples_;
        } else {
            (void)experimentWriter_->enqueue(line);
        }
    }
    if (trace_) trace_(line);
}
void LiveVisualController::publish()
{
    const auto robot = world_.robot(selected_);
    description_ = QStringLiteral("Robot %1 · %2 — %3").arg(selected_.value())
        .arg(operationalStateLabel(operationalState()), status_);
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
    if (target_ && robot && robot->pose) {
        const double dx = target_->x.value - robot->pose->x.value;
        const double dy = target_->y.value - robot->pose->y.value;
        const double headingError = normalizeAngle(Radians{
            target_->heading.value - robot->pose->heading.value}).value;
        description_ += QStringLiteral("\nPosition error: %1 mm · heading error: %2°")
            .arg(std::hypot(dx, dy) * 1000.0, 0, 'f', 1)
            .arg(headingError * 180.0 / std::numbers::pi, 0, 'f', 1);
    }
    if (runExportPath_ && experimentWriter_) {
        const auto utf8 = runExportPath_->u8string();
        const auto path = QString::fromUtf8(reinterpret_cast<const char*>(utf8.data()), static_cast<qsizetype>(utf8.size()));
        switch (experimentWriter_->runExportState()) {
        case ControlRunExportState::Opening:
            description_ += QStringLiteral("\nExperiment export opening: %1").arg(path); break;
        case ControlRunExportState::Open:
            description_ += QStringLiteral("\nExperiment export: %1").arg(path); break;
        case ControlRunExportState::Complete:
            description_ += QStringLiteral("\nExperiment export complete: %1").arg(path); break;
        case ControlRunExportState::Failed:
            description_ += QStringLiteral("\nExperiment export failed — see application log"); break;
        case ControlRunExportState::Idle:
            break;
        }
    }
    emit descriptionChanged(description_);
    emit actionsChanged(!running() && ready() && !(loop_ && loop_->hasLatchedControllerFault()), running());
    emit targetAvailabilityChanged(target_.has_value());
    emit controllerFaultAcknowledgementChanged(canAcknowledgeControllerFault());
}
} // namespace micro_robot
