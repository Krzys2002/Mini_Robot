#include "transport/ble/BleTransport.hpp"

#include <QMetaObject>

#include <locale>
#include <algorithm>
#include <sstream>
#include <utility>

namespace micro_robot {
namespace {
RobotOperatingState operatingState(BleRobotState state)
{
    switch (state) {
    case BleRobotState::Idle: return RobotOperatingState::Idle;
    case BleRobotState::Moving: return RobotOperatingState::Moving;
    case BleRobotState::EmergencyStop: return RobotOperatingState::EmergencyStop;
    case BleRobotState::Fault: return RobotOperatingState::Fault;
    }
    return RobotOperatingState::Unknown;
}
auto steadyMicroseconds(Timestamp at)
{
    return std::chrono::duration_cast<std::chrono::microseconds>(at.time_since_epoch()).count();
}
bool blockedVelocity(const RobotCommand& command, bool emergencyRequested, BleRobotState state)
{
    return std::holds_alternative<VelocityCommand>(command.payload)
        && (emergencyRequested || state == BleRobotState::EmergencyStop || state == BleRobotState::Fault);
}
} // namespace
void BleTransport::clearIo(Session& session)
{
    session.pending.reset();
    session.info.reset();
    session.writing.reset();
    session.emergencyWriting.reset();
    session.receiveSequence = {};
    session.transmitSequence = 0;
    session.lastVelocityAt.reset();
    session.pumpScheduled = session.nativeReady = session.emergencyConfirmed = false;
    session.robotState = BleRobotState::Idle;
    session.invalidNotifications = 0;
    session.sent.clear();
    if (session.pumpTimer) session.pumpTimer->stop();
    if (session.emergencyTimer) session.emergencyTimer->stop();
    if (session.emergencyWriteTimer) session.emergencyWriteTimer->stop();
    // emergencyRequested persists across connection generations; only a future
    // explicit safety-domain clear (#70) may release the adapter's intent latch.
}
void BleTransport::schedulePump(RobotId id)
{
    auto& session = sessions_.at(id);
    if (session.pumpScheduled || !session.pending || session.writing) return;
    session.pumpScheduled = true;
    const auto token = session.token;
    QMetaObject::invokeMethod(this, [this, id, token] {
        auto& current = sessions_.at(id);
        if (current.token != token) return;
        current.pumpScheduled = false;
        pump(id);
    }, Qt::QueuedConnection);
}
void BleTransport::pump(RobotId id)
{
    auto& session = sessions_.at(id);
    if (session.state != ConnectionState::Connected || !session.info || !session.pending || session.writing) return;
    const auto now = MonotonicClock::now();
    const bool velocity = std::holds_alternative<VelocityCommand>(session.pending->payload);
    if (blockedVelocity(*session.pending, session.emergencyRequested, session.robotState)) {
        session.pending.reset();
        error(id, TransportOperation::SendCommand, TransportErrorCode::InvalidCommand,
            "Queued velocity blocked by emergency-stop or robot fault state");
        return;
    }
    if (velocity && session.lastVelocityAt && now - *session.lastVelocityAt < config_.minimumVelocityInterval) {
        if (!session.pumpTimer) {
            session.pumpTimer = std::make_unique<QTimer>();
            session.pumpTimer->setSingleShot(true);
            session.pumpTimer->setTimerType(Qt::PreciseTimer);
            QObject::connect(session.pumpTimer.get(), &QTimer::timeout, this, [this, id] { schedulePump(id); });
        }
        const auto remaining = config_.minimumVelocityInterval - (now - *session.lastVelocityAt);
        session.pumpTimer->start(static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(remaining).count() + 1));
        return;
    }
    const auto command = std::exchange(session.pending, {});
    const auto sequence = session.transmitSequence;
    const auto encoded = encodeBleCommand(*command, *session.info, sequence, now);
    if (!encoded.frame) { error(id, TransportOperation::SendCommand, encoded.code, encoded.error); return; }
    const auto dispatched = backend_->write(session.token, {sequence, false, *command});
    if (!dispatched.dispatched) {
        error(id, TransportOperation::SendCommand, dispatched.code, dispatched.error);
        return;
    }
    ++session.transmitSequence; // Increment only for actual native handoffs.
    session.writing = sequence;
    if (velocity) session.lastVelocityAt = dispatched.dispatchedAt;
    rememberSent(id, sequence, velocity ? 2 : 1, dispatched.dispatchedAt, command->metadata.sequence);
    armTimeout(id, config_.phaseTimeout);
}
void BleTransport::rememberSent(RobotId id, std::uint16_t sequence, std::uint8_t type, Timestamp at,
                               std::optional<std::uint64_t> domainSequence)
{
    auto& session = sessions_.at(id);
    session.sent.push_back({sequence, type, at});
    while (session.sent.size() > config_.maximumTrackedCommands) session.sent.pop_front();
    if (log_) log_->debug("BLE tx token=" + std::to_string(session.token) + " wire_sequence=" + std::to_string(sequence)
        + " type=" + std::to_string(type) + " steady_us=" + std::to_string(steadyMicroseconds(at))
        + " domain_sequence=" + (domainSequence ? std::to_string(*domainSequence) : "unassigned"), id);
}
void BleTransport::cancelPendingMotion(RobotId id)
{
    const auto found = sessions_.find(id);
    if (found != sessions_.end() && found->second.pending
        && std::holds_alternative<VelocityCommand>(found->second.pending->payload))
        found->second.pending.reset();
}
void BleTransport::requestEmergencyStop(RobotId id)
{
    auto& session = sessions_[id];
    session.emergencyRequested = true;
    session.emergencyConfirmed = false;
    session.pending.reset(); // Takes priority over queued, not-yet-handed-off intent.
    QMetaObject::invokeMethod(this, [this, id] { sendEmergencyStop(id); }, Qt::QueuedConnection);
}
void BleTransport::sendEmergencyStop(RobotId id)
{
    auto& session = sessions_.at(id);
    if (!session.nativeReady || !session.info
        || (session.state != ConnectionState::Connected && session.state != ConnectionState::Connecting)) return;
    if (!session.emergencyRequested || session.emergencyConfirmed) return;
    if (!session.emergencyTimer) {
        session.emergencyTimer = std::make_unique<QTimer>();
        QObject::connect(session.emergencyTimer.get(), &QTimer::timeout, this, [this, id] { sendEmergencyStop(id); });
    }
    if (!session.emergencyTimer->isActive()) session.emergencyTimer->start(static_cast<int>(config_.emergencyStopRetry.count()));
    const auto sequence = session.transmitSequence;
    const auto outcome = backend_->write(session.token, {sequence, true, {}});
    if (!outcome.dispatched) {
        // Backpressure cannot accumulate an unbounded native E-stop queue.
        if (outcome.code != TransportErrorCode::Unavailable)
            fault(id, outcome.code, outcome.error, TransportOperation::SendCommand);
        return;
    }
    ++session.transmitSequence;
    session.emergencyWriting = sequence;
    rememberSent(id, sequence, 3, outcome.dispatchedAt);
    if (!session.emergencyWriteTimer) {
        session.emergencyWriteTimer = std::make_unique<QTimer>();
        session.emergencyWriteTimer->setSingleShot(true);
        QObject::connect(session.emergencyWriteTimer.get(), &QTimer::timeout, this, [this, id] {
            fault(id, TransportErrorCode::BackendFailure, "Emergency-stop ATT confirmation timed out", TransportOperation::SendCommand);
        });
    }
    session.emergencyWriteTimer->start(static_cast<int>(config_.phaseTimeout.count()));
}
void BleTransport::confirmEmergencyStop(RobotId id)
{
    auto& session = sessions_.at(id);
    session.emergencyConfirmed = true;
    session.emergencyWriting.reset();
    if (session.emergencyWriteTimer) session.emergencyWriteTimer->stop();
    if (session.emergencyTimer) session.emergencyTimer->stop();
    if (session.state == ConnectionState::Connecting && session.nativeReady) {
        session.timer->stop();
        changePhase(id, BlePhase::Ready);
        changeState(id, ConnectionState::Connected);
    }
}
void BleTransport::receive(std::uint64_t token, BleEndpoint endpoint, const QByteArray& bytes, Timestamp at)
{
    const auto id = findToken(token);
    if (!id) return;
    auto& session = sessions_.at(*id);
    if (!session.nativeReady || (session.state != ConnectionState::Connected && session.state != ConnectionState::Connecting)) return;
    const auto decoded = decodeBleNotification(endpoint,
        {reinterpret_cast<const std::uint8_t*>(bytes.constData()), static_cast<std::size_t>(bytes.size())});
    if (!decoded.frame) {
        session.robotState = BleRobotState::Fault; // Unknown input never enables velocity.
        if (session.pending && std::holds_alternative<VelocityCommand>(session.pending->payload)) session.pending.reset();
        if (++session.invalidNotifications >= config_.maximumInvalidNotifications)
            fault(*id, decoded.code, decoded.error, TransportOperation::ReceiveTelemetry);
        else error(*id, TransportOperation::ReceiveTelemetry, decoded.code, decoded.error);
        return;
    }
    session.invalidNotifications = 0;
    const auto accepted = session.receiveSequence.accept(decoded.frame->sequence);
    if (!accepted) {
        if (log_) log_->debug("BLE duplicate/old notification dropped token=" + std::to_string(token)
            + " wire_sequence=" + std::to_string(decoded.frame->sequence), *id);
        return;
    }
    if (log_) log_->debug("BLE rx token=" + std::to_string(token) + " wire_sequence=" + std::to_string(decoded.frame->sequence)
        + " unwrapped_sequence=" + std::to_string(accepted->unwrapped) + " gap=" + std::to_string(accepted->gap)
        + " steady_us=" + std::to_string(steadyMicroseconds(at)), *id);
    if (const auto* telemetry = std::get_if<BleTelemetryFrame>(&decoded.frame->payload)) {
        session.robotState = telemetry->state;
        if (session.emergencyRequested && telemetry->state == BleRobotState::EmergencyStop) confirmEmergencyStop(*id);
        if (log_) {
            std::ostringstream message;
            message.imbue(std::locale::classic());
            message << "BLE telemetry state=" << static_cast<unsigned>(telemetry->state)
                << " command_expired=" << telemetry->commandExpired << " battery_low=" << telemetry->batteryLow
                << " robot_uptime_ms=" << telemetry->uptimeMilliseconds << " last_accepted_sequence=";
            if (telemetry->lastAcceptedSequence) message << *telemetry->lastAcceptedSequence;
            else message << "unknown";
            log_->debug(message.str(), *id);
        }
        if (auto sink = sink_.lock()) sink->onTelemetry({*id, at, telemetry->batteryVolts,
            telemetry->linearMetersPerSecond, telemetry->angularRadiansPerSecond,
            {currentTransportModelVersion, accepted->unwrapped},
            operatingState(telemetry->state)});
    } else {
        const auto& result = std::get<BleCommandResultFrame>(decoded.frame->payload);
        session.robotState = result.state;
        const auto sent = std::find_if(session.sent.begin(), session.sent.end(), [&](const auto& entry) {
            return entry.sequence == result.commandSequence && entry.type == result.commandType;
        });
        if (log_) {
            std::string message = "BLE result command_sequence=" + std::to_string(result.commandSequence)
                + " command_type=" + std::to_string(result.commandType) + " result=" + bleResultName(result.result)
                + " state=" + std::to_string(static_cast<unsigned>(result.state));
            if (sent != session.sent.end() && at >= sent->at)
                message += " round_trip_us=" + std::to_string(std::chrono::duration_cast<std::chrono::microseconds>(at - sent->at).count());
            else message += " round_trip_us=unknown";
            log_->debug(message, *id);
        }
        if (session.emergencyRequested && result.commandType == 3 && result.result == 0 && sent != session.sent.end())
            confirmEmergencyStop(*id);
        if (result.result != 0) error(*id, TransportOperation::SendCommand, bleRejectionCode(result.result),
            std::string{"Robot rejected command wire_sequence="} + std::to_string(result.commandSequence) + ": " + bleResultName(result.result));
    }
}
} // namespace micro_robot
