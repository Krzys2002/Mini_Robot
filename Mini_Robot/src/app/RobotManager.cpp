#include "app/RobotManager.hpp"

#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace micro_robot {
namespace {
const char* connectionName(ConnectionState state)
{
    switch (state) {
    case ConnectionState::Disconnected: return "Disconnected";
    case ConnectionState::Connecting: return "Connecting";
    case ConnectionState::Connected: return "Connected";
    case ConnectionState::Disconnecting: return "Disconnecting";
    case ConnectionState::Fault: return "Fault";
    }
    return "Unknown";
}
bool terminal(ConnectionState state)
{
    return state == ConnectionState::Disconnected || state == ConnectionState::Fault;
}
}
class RobotManager::Events final : public IRobotTransportEvents {
public:
    explicit Events(RobotManager& service) : owner(&service) {}
    void onConnectionStateChanged(RobotId id, ConnectionState state, Timestamp at) override
    { if (owner) owner->receiveState(id, state, at); }
    void onTelemetry(const RobotTelemetry& sample) override
    { if (owner) owner->receiveTelemetry(sample); }
    void onTransportError(const TransportError& error) override
    { if (owner) owner->receiveError(error); }
    void onEmergencyClearResult(RobotId id, std::uint64_t request, bool accepted, Timestamp at) override
    { if (owner) owner->receiveEmergencyClearResult(id, request, accepted, at); }
    RobotManager* owner; // Invalidated before detach/destruction, even for a retained sink.
};
RobotManager::RobotManager(IRobotTransport& transport, WorldModel& world, std::optional<LogChannel> log)
    : transport_(transport), world_(world), log_(std::move(log)), events_(std::make_shared<Events>(*this))
{
    transport_.setEventSink(events_);
}
RobotManager::~RobotManager()
{
    shutdown();
    events_->owner = nullptr;
    transport_.setEventSink({});
}
void RobotManager::setEventSink(std::weak_ptr<IRobotTransportEvents> sink) { sink_ = std::move(sink); }
bool RobotManager::addRobot(RobotId id, Timestamp at)
{
    if (shuttingDown_ || sessions_.contains(id) || !terminal(transport_.connectionState(id))) return false;
    (void)world_.registerRobot(id, at);
    RobotSession state{id};
    if (emergencyLatched_.contains(id)) {
        state.emergencyStopRequested = true;
        state.safety = DesktopSafetyState::EmergencyStop;
    }
    state.connection = transport_.connectionState(id);
    sessions_.emplace(id, std::move(state));
    return true;
}
bool RobotManager::connect(RobotId id)
{
    const auto found = sessions_.find(id);
    if (shuttingDown_ || found == sessions_.end() || found->second.removing) return false;
    auto& state = found->second;
    if (state.connectRequested) return true;
    if (!terminal(state.connection)) return false; // Wait for disconnect completion.
    state.connectRequested = true;
    state.telemetry.reset();
    state.lastError.reset();
    transport_.connect(id);
    return true;
}
bool RobotManager::disconnect(RobotId id)
{
    const auto found = sessions_.find(id);
    if (found == sessions_.end()) return false;
    auto& state = found->second;
    const bool pending = state.connectRequested || !terminal(state.connection);
    state.connectRequested = false;
    state.telemetry.reset();
    state.clearRequest.reset();
    if (!state.emergencyStopRequested)
        state.safety = state.everConnected ? DesktopSafetyState::LostConnection : DesktopSafetyState::Disconnected;
    if (pending) transport_.disconnect(id);
    return true;
}
bool RobotManager::removeRobot(RobotId id)
{
    const auto found = sessions_.find(id);
    if (found == sessions_.end()) return false;
    if (!found->second.connectRequested && terminal(found->second.connection)) sessions_.erase(found);
    else {
        found->second.removing = true;
        (void)disconnect(id);
        // Cancellation before the adapter starts may have no state transition
        // (e.g. simulation connect+disconnect before advanceTo). No live link is
        // owned when both published states are already terminal.
        if (terminal(found->second.connection) && terminal(transport_.connectionState(id)))
            sessions_.erase(found);
    }
    return true;
}
std::optional<RobotSession> RobotManager::session(RobotId id) const
{
    const auto found = sessions_.find(id);
    return found == sessions_.end() ? std::nullopt : std::optional{found->second};
}
std::map<RobotId, RobotSession> RobotManager::sessions() const { return sessions_; }
ConnectionState RobotManager::connectionState(RobotId id) const
{
    const auto found = sessions_.find(id);
    return found == sessions_.end() ? ConnectionState::Disconnected : found->second.connection;
}
CommandSubmission RobotManager::sendCommand(RobotId id, const RobotCommand& command)
{
    if (shuttingDown_) return CommandSubmission::ShuttingDown;
    const auto found = sessions_.find(id);
    if (found == sessions_.end()) return CommandSubmission::UnknownRobot;
    const auto& state = found->second;
    if (!state.connectRequested || state.removing || state.connection != ConnectionState::Connected)
        return CommandSubmission::NotConnected;
    if (std::holds_alternative<VelocityCommand>(command.payload)) {
        if (state.emergencyStopRequested) return CommandSubmission::EmergencyStopped;
        if (state.safetyFault) return CommandSubmission::SafetyFault;
        if (state.motionAllowedAfter && (command.issuedAt < *state.motionAllowedAfter
            || (state.strictMotionBarrier && command.issuedAt == *state.motionAllowedAfter)))
            return CommandSubmission::StaleSession;
    }
    transport_.sendCommand(id, command);
    return CommandSubmission::Submitted;
}
CommandSubmission RobotManager::stop(RobotId id, Timestamp at, Duration validFor)
{
    return sendCommand(id, {StopCommand{}, at, validFor});
}
bool RobotManager::supportsEmergencyStop() const noexcept
{
    return transport_.supportsEmergencyStop();
}
bool RobotManager::requestEmergencyStop(RobotId id)
{
    const auto found = sessions_.find(id);
    if (shuttingDown_ || found == sessions_.end()) return false;
    emergencyLatched_.insert(id);
    found->second.emergencyStopRequested = true;
    found->second.clearRequest.reset();
    found->second.emergencyRequestedByOperator = true;
    found->second.safety = DesktopSafetyState::EmergencyStop;
    transport_.cancelPendingMotion(id);
    if (!supportsEmergencyStop()) return false; // Inhibit even without a capable adapter.
    transport_.requestEmergencyStop(id);
    if (log_) log_->warning("Emergency stop requested; physical execution unconfirmed", id);
    return true;
}
bool RobotManager::supportsEmergencyClear() const noexcept { return transport_.supportsEmergencyClear(); }
bool RobotManager::requestEmergencyClear(RobotId id)
{
    const auto found = sessions_.find(id);
    if (shuttingDown_ || found == sessions_.end() || !supportsEmergencyClear()) return false;
    auto& state = found->second;
    if (!state.emergencyStopRequested || state.clearRequest || state.safetyFault || !state.connectRequested
        || state.connection != ConnectionState::Connected || !state.telemetry
        || state.telemetry->operatingState != RobotOperatingState::EmergencyStop
        || nextClearRequest_ == std::numeric_limits<std::uint64_t>::max()) return false;
    const auto request = nextClearRequest_++;
    state.clearRequest = request;
    if (!transport_.requestEmergencyClear(id, request)) { state.clearRequest.reset(); return false; }
    return true;
}
void RobotManager::receiveEmergencyClearResult(RobotId id, std::uint64_t request, bool accepted, Timestamp at)
{
    const auto found = sessions_.find(id);
    if (found == sessions_.end()) return;
    auto& state = found->second;
    if (state.clearRequest != request || !state.connectRequested || state.connection != ConnectionState::Connected
        || (state.connectionChangedAt && at < *state.connectionChangedAt)) return;
    state.clearRequest.reset();
    if (accepted && !state.safetyFault) {
        state.emergencyStopRequested = false;
        state.emergencyRequestedByOperator = false;
        emergencyLatched_.erase(id);
        state.telemetry.reset();
        state.motionAllowedAfter = at;
        state.strictMotionBarrier = true;
        state.safety = DesktopSafetyState::Ready;
    }
    if (auto sink = sink_.lock()) sink->onEmergencyClearResult(id, request, accepted && !state.safetyFault, at);
}
void RobotManager::shutdown()
{
    if (shuttingDown_) return;
    shuttingDown_ = true;
    // Requests never invoke backend callbacks inline; keep iteration detached too.
    std::vector<RobotId> ids;
    for (const auto& [id, state] : sessions_) { (void)state; ids.push_back(id); }
    for (RobotId id : ids) (void)disconnect(id);
}
bool RobotManager::isStopped() const
{
    for (const auto& [id, state] : sessions_) {
        (void)id;
        if (state.connectRequested || !terminal(state.connection)) return false;
    }
    return true;
}
void RobotManager::receiveState(RobotId id, ConnectionState next, Timestamp at)
{
    const auto found = sessions_.find(id);
    if (found == sessions_.end() || transport_.connectionState(id) != next) return;
    auto& state = found->second;
    if (state.connectionChangedAt && at < *state.connectionChangedAt) return;
    const auto previous = state.connection;
    state.connection = next;
    state.connectionChangedAt = at;
    if (next == ConnectionState::Connected && previous != ConnectionState::Connected && state.connectRequested) {
        state.everConnected = true;
        state.motionAllowedAfter = at;
        state.strictMotionBarrier = false;
        state.safetyFault = false; // Explicit reconnect starts a new session.
    }
    if (state.emergencyStopRequested) state.safety = DesktopSafetyState::EmergencyStop;
    else if (next == ConnectionState::Fault || state.safetyFault) state.safety = DesktopSafetyState::Fault;
    else if (next == ConnectionState::Connected && state.connectRequested && !state.safetyFault)
        state.safety = DesktopSafetyState::Ready;
    else state.safety = state.everConnected ? DesktopSafetyState::LostConnection : DesktopSafetyState::Disconnected;
    if (next != ConnectionState::Connected) { state.telemetry.reset(); state.clearRequest.reset(); }
    if (terminal(next)) {
        state.connectRequested = false;
        if (state.removing) sessions_.erase(found);
    } else if (!state.connectRequested
               && (next == ConnectionState::Connecting || next == ConnectionState::Connected)) {
        // A late connect completion after cancellation must never enable commands.
        transport_.disconnect(id);
    }
    if (log_) log_->info(std::string{"Robot session connection state="} + connectionName(next), id);
    // No references into the map survive notification: observers may remove IDs.
    if (auto sink = sink_.lock()) sink->onConnectionStateChanged(id, next, at);
}
void RobotManager::receiveTelemetry(const RobotTelemetry& sample)
{
    const auto found = sessions_.find(sample.robotId);
    if (found == sessions_.end()) return;
    auto& state = found->second;
    if (shuttingDown_ || state.removing || !state.connectRequested || state.connection != ConnectionState::Connected
        || (state.connectionChangedAt && sample.receivedAt < *state.connectionChangedAt)
        || (state.telemetry && sample.receivedAt <= state.telemetry->receivedAt)) return;
    const auto finite = [](std::optional<double> value) { return !value || std::isfinite(*value); };
    if (!isSupportedTransportModelVersion(sample.metadata.modelVersion)) {
        receiveError({sample.robotId, TransportOperation::ReceiveTelemetry, TransportErrorCode::UnsupportedVersion,
                      "Unsupported telemetry domain model version", sample.receivedAt});
        return;
    }
    if (!finite(sample.batteryVoltageVolts) || !finite(sample.linearMetersPerSecond) || !finite(sample.angularRadiansPerSecond)) {
        receiveError({sample.robotId, TransportOperation::ReceiveTelemetry, TransportErrorCode::InvalidTelemetry,
                      "Invalid telemetry measurement", sample.receivedAt});
        return;
    }
    state.telemetry = sample;
    if (sample.operatingState == RobotOperatingState::EmergencyStop) {
        transport_.cancelPendingMotion(sample.robotId);
        emergencyLatched_.insert(sample.robotId);
        state.emergencyStopRequested = true;
        state.safety = DesktopSafetyState::EmergencyStop;
    } else if (sample.operatingState == RobotOperatingState::Fault) {
        if (state.clearRequest) {
            state.clearRequest.reset();
            transport_.requestEmergencyStop(sample.robotId);
        }
        transport_.cancelPendingMotion(sample.robotId);
        state.safetyFault = true;
        if (!state.emergencyStopRequested) state.safety = DesktopSafetyState::Fault;
    }
    if (log_) log_->debug("Robot telemetry received; sequence="
        + (sample.metadata.sequence ? std::to_string(*sample.metadata.sequence) : "unassigned"), sample.robotId);
    if (auto sink = sink_.lock()) sink->onTelemetry(sample);
}
void RobotManager::receiveError(const TransportError& error)
{
    const auto found = sessions_.find(error.robotId);
    if (found == sessions_.end()) return;
    if (found->second.connectionChangedAt && error.occurredAt < *found->second.connectionChangedAt) return;
    if (error.code == TransportErrorCode::EmergencyStopLatched) {
        emergencyLatched_.insert(error.robotId);
        found->second.emergencyStopRequested = true;
        found->second.safety = DesktopSafetyState::EmergencyStop;
        transport_.cancelPendingMotion(error.robotId);
    }
    // A rejected malformed command does not establish a transport failure.
    // Connection/receive failures are conservative motion-inhibition events.
    if (error.code != TransportErrorCode::EmergencyStopLatched
        && (error.operation != TransportOperation::SendCommand
        || (error.code != TransportErrorCode::InvalidCommand && error.code != TransportErrorCode::UnsupportedVersion))) {
        if (found->second.clearRequest) {
            found->second.clearRequest.reset();
            transport_.requestEmergencyStop(error.robotId);
        }
        transport_.cancelPendingMotion(error.robotId);
        found->second.safetyFault = true;
        if (!found->second.emergencyStopRequested) found->second.safety = DesktopSafetyState::Fault;
    }
    if (!found->second.lastError || error.occurredAt >= found->second.lastError->occurredAt)
        found->second.lastError = error;
    if (log_) log_->warning(error.message, error.robotId);
    if (auto sink = sink_.lock()) sink->onTransportError(error);
}
} // namespace micro_robot
