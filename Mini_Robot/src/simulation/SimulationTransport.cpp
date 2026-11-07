#include "simulation/SimulationTransport.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace micro_robot {
namespace {

bool fresh(const RobotCommand& command, Timestamp now)
{
    if (command.validFor <= Duration::zero() || command.issuedAt > now) {
        return false;
    }
    // A difference larger than Duration::max() is necessarily expired. Avoid
    // signed overflow when a malformed command spans the clock's extremes.
    const auto issued = command.issuedAt.time_since_epoch().count();
    const auto current = now.time_since_epoch().count();
    if (issued < 0 && current > std::numeric_limits<Duration::rep>::max() + issued) {
        return false;
    }
    return now - command.issuedAt < command.validFor;
}

// Requires from <= to. Like fresh(), avoids signed overflow across clock extremes.
bool elapsedAtLeast(Timestamp from, Timestamp to, Duration interval)
{
    const auto start = from.time_since_epoch().count();
    const auto end = to.time_since_epoch().count();
    if (start < 0 && end > std::numeric_limits<Duration::rep>::max() + start) {
        return true;
    }
    return to - from >= interval;
}

bool valid(const RobotCommand& command, Timestamp now, const SimulationTransportConfig& config)
{
    if (!fresh(command, now)) {
        return false;
    }
    if (std::chrono::duration_cast<std::chrono::milliseconds>(
            command.validFor - (now - command.issuedAt)).count() < 1) return false;
    if (const auto* velocity = std::get_if<VelocityCommand>(&command.payload)) {
        return std::abs(velocity->linearMetersPerSecond) <= config.maximumLinearMetersPerSecond
            && std::abs(velocity->angularRadiansPerSecond) <= config.maximumAngularRadiansPerSecond
            && std::isfinite(velocity->linearMetersPerSecond)
            && std::isfinite(velocity->angularRadiansPerSecond);
    }
    return true;
}

std::string_view toString(ConnectionState state)
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

std::string_view toString(TransportErrorCode code)
{
    switch (code) {
    case TransportErrorCode::Unavailable: return "Unavailable";
    case TransportErrorCode::NotConnected: return "NotConnected";
    case TransportErrorCode::InvalidCommand: return "InvalidCommand";
    case TransportErrorCode::BackendFailure: return "BackendFailure";
    case TransportErrorCode::UnsupportedVersion: return "UnsupportedVersion";
    case TransportErrorCode::InvalidTelemetry: return "InvalidTelemetry";
    case TransportErrorCode::RobotFault: return "RobotFault";
    case TransportErrorCode::EmergencyStopLatched: return "EmergencyStopLatched";
    }
    return "Unknown";
}

// Locale-independent fixed-point text, so log lines are identical on every host.
std::ostringstream logStream()
{
    std::ostringstream stream;
    stream.imbue(std::locale::classic());
    stream << std::fixed << std::setprecision(3);
    return stream;
}

std::string describe(const RobotCommand& command)
{
    auto text = logStream();
    if (const auto* velocity = std::get_if<VelocityCommand>(&command.payload)) {
        text << "velocity linear=" << velocity->linearMetersPerSecond
             << " m/s angular=" << velocity->angularRadiansPerSecond << " rad/s";
    } else {
        text << "stop";
    }
    text << " validFor=" << std::chrono::duration_cast<std::chrono::milliseconds>(
        command.validFor).count() << " ms";
    if (command.metadata.sequence) {
        text << " sequence=" << *command.metadata.sequence;
    }
    return text.str();
}

} // namespace

SimulationTransport::SimulationTransport(SimulationTransportConfig config,
                                         std::optional<LogChannel> log)
    : config_(config), log_(std::move(log))
{
    if (!std::isfinite(config_.maximumLinearMetersPerSecond)
        || config_.maximumLinearMetersPerSecond <= 0 || config_.maximumLinearMetersPerSecond > 32.767
        || !std::isfinite(config_.maximumAngularRadiansPerSecond)
        || config_.maximumAngularRadiansPerSecond <= 0 || config_.maximumAngularRadiansPerSecond > 32.767
        || config_.maximumCommandValidity.count() < 1 || config_.maximumCommandValidity.count() > 65535) {
        throw std::invalid_argument("Simulation command limits exceed the v1 protocol range");
    }
    if (config_.telemetryInterval < Duration::zero()) {
        throw std::invalid_argument("Simulation telemetry interval must not be negative");
    }
    if (!std::isfinite(config_.batteryVoltageVolts) || config_.batteryVoltageVolts <= 0.0) {
        throw std::invalid_argument("Simulation battery voltage must be finite and positive");
    }
}

void SimulationTransport::setEventSink(std::weak_ptr<IRobotTransportEvents> sink)
{
    sink_ = std::move(sink);
}

ConnectionState SimulationTransport::connectionState(RobotId id) const
{
    const auto found = sessions_.find(id);
    return found == sessions_.end() ? ConnectionState::Disconnected : found->second.state;
}

void SimulationTransport::connect(RobotId id)
{
    sessions_[id].desiredConnected = true;
    if (log_) log_->debug("Simulated connect requested", id);
}

void SimulationTransport::disconnect(RobotId id)
{
    auto& session = sessions_[id];
    if (log_) log_->debug("Simulated disconnect requested", id);
    session.desiredConnected = false;
    session.pendingCommand.reset();
    session.clearRequest.reset();
}

void SimulationTransport::sendCommand(RobotId id, const RobotCommand& command)
{
    auto& pending = sessions_[id].pendingCommand;
    // Match BLE: a queued Stop has priority over subsequent unsent motion.
    if (!pending || std::holds_alternative<StopCommand>(command.payload)
        || !std::holds_alternative<StopCommand>(pending->payload)) pending = command;
    if (log_) log_->debug("Simulated command submitted: " + describe(command), id);
}

void SimulationTransport::cancelPendingMotion(RobotId id)
{
    const auto found = sessions_.find(id);
    if (found != sessions_.end() && found->second.pendingCommand
        && std::holds_alternative<VelocityCommand>(found->second.pendingCommand->payload))
        found->second.pendingCommand.reset();
}

void SimulationTransport::requestEmergencyStop(RobotId id)
{
    auto& session = sessions_[id];
    session.emergencyStopRequested = true;
    session.clearRequest.reset();
    session.pendingCommand.reset();
    session.lastSentCommand.reset();
    session.motionStartedAt.reset();
    session.velocity = {};
    session.status.state = SimulationRobotState::Idle;
    session.statusChanged = true;
    if (log_) log_->warning("Simulated emergency-stop latch requested", id);
}

bool SimulationTransport::requestEmergencyClear(RobotId id, std::uint64_t request)
{
    const auto found = sessions_.find(id);
    if (found == sessions_.end() || found->second.state != ConnectionState::Connected
        || !found->second.desiredConnected || !found->second.emergencyStopRequested || found->second.clearRequest)
        return false;
    found->second.clearRequest = request;
    return true;
}

void SimulationTransport::simulateDisconnect(RobotId id)
{
    sessions_[id].pendingFailure = ConnectionState::Disconnected;
}

void SimulationTransport::simulateFault(RobotId id)
{
    sessions_[id].pendingFailure = ConnectionState::Fault;
}

std::optional<SimulationRobotStatus> SimulationTransport::robotStatus(RobotId id) const
{
    const auto found = sessions_.find(id);
    if (found == sessions_.end() || found->second.state != ConnectionState::Connected) return {};
    return found->second.status;
}

std::optional<RobotCommand> SimulationTransport::lastSentCommand(RobotId id) const
{
    const auto found = sessions_.find(id);
    return found == sessions_.end() ? std::nullopt : found->second.lastSentCommand;
}

void SimulationTransport::advanceTo(Timestamp now)
{
    if (advancing_) {
        throw std::logic_error("Recursive simulation transport advancement");
    }
    if (lastAdvance_ && now < *lastAdvance_) {
        throw std::invalid_argument("Simulation transport time cannot go backward");
    }
    struct AdvancementGuard {
        bool& active;
        explicit AdvancementGuard(bool& value) : active(value) { active = true; }
        ~AdvancementGuard() { active = false; }
    } guard{advancing_};
    lastAdvance_ = now;

    // Snapshot IDs so callback-created sessions wait until the next advance.
    std::vector<RobotId> ids;
    ids.reserve(sessions_.size());
    for (const auto& [id, session] : sessions_) {
        (void)session;
        ids.push_back(id);
    }
    for (const auto id : ids) {
        auto& session = sessions_.at(id);
        auto changeState = [&](ConnectionState state) {
            session.state = state;
            if (log_) {
                log_->info("Simulated connection state: " + std::string{toString(state)}, id);
            }
            if (auto sink = sink_.lock()) {
                sink->onConnectionStateChanged(id, state, now);
            }
        };
        auto resetSession = [&] {
            session.clearRequest.reset();
            session.lastSentCommand.reset();
                    session.lastTelemetryAt.reset();
            session.motionStartedAt.reset();
            session.velocity = {};
            session.status = {};
            session.transmitSequence = 0;
            session.telemetrySequence = 0;
            session.statusChanged = false;
        };
        if (const auto failure = std::exchange(session.pendingFailure, {})) {
            session.desiredConnected = false;
            session.pendingCommand.reset();
            resetSession();
            changeState(*failure);
            if (auto sink = sink_.lock()) {
                sink->onTransportError({id, TransportOperation::Disconnect,
                    TransportErrorCode::BackendFailure,
                    *failure == ConnectionState::Fault ? "Simulated backend fault" : "Simulated connection loss", now});
            }
            continue; // Recovery explicitly requested in a callback waits for next advance.
        }
        if (session.desiredConnected && (session.state == ConnectionState::Disconnected
            || session.state == ConnectionState::Fault)) {
            resetSession();
            changeState(ConnectionState::Connected);
        }
        // A connection callback may request immediate disconnection.
        if (!session.desiredConnected && session.state == ConnectionState::Connected) {
            resetSession();
            changeState(ConnectionState::Disconnected);
        }

        if (session.motionStartedAt && elapsedAtLeast(*session.motionStartedAt, now, session.motionValidity)) {
            session.motionStartedAt.reset();
            session.velocity = {};
            session.status.state = SimulationRobotState::Idle;
            session.status.commandExpired = true;
            session.statusChanged = true;
        }
        if (session.clearRequest && session.state == ConnectionState::Connected && session.desiredConnected) {
            const auto request = *std::exchange(session.clearRequest, {});
            session.emergencyStopRequested = false;
            session.pendingCommand.reset();
            session.motionStartedAt.reset();
            session.velocity = {};
            session.status.state = SimulationRobotState::Idle;
            session.statusChanged = true;
            if (auto sink = sink_.lock()) sink->onEmergencyClearResult(id, request, true, now);
        }
        const auto command = std::exchange(session.pendingCommand, std::nullopt);
        if (command) {
            std::optional<TransportErrorCode> error;
            if (session.state != ConnectionState::Connected || !session.desiredConnected) {
                error = TransportErrorCode::NotConnected;
            } else if (!isSupportedTransportModelVersion(command->metadata.modelVersion)) {
                error = TransportErrorCode::UnsupportedVersion;
            } else if (session.emergencyStopRequested
                       && std::holds_alternative<VelocityCommand>(command->payload)) {
                error = TransportErrorCode::InvalidCommand;
            } else if (!valid(*command, now, config_)) {
                error = TransportErrorCode::InvalidCommand;
            } else {
                session.lastSentCommand = *command;
                session.status.lastAcceptedSequence = session.transmitSequence++;
                session.status.commandExpired = false;
                const auto previousState = session.status.state;
                if (const auto* velocity = std::get_if<VelocityCommand>(&command->payload)) {
                    // Echo the scaled setpoint a v1 device would receive.
                    session.velocity = {std::round(velocity->linearMetersPerSecond * 1000) / 1000,
                                        std::round(velocity->angularRadiansPerSecond * 1000) / 1000};
                    session.motionStartedAt = now;
                    session.motionValidity = std::min(config_.maximumCommandValidity,
                        std::chrono::duration_cast<std::chrono::milliseconds>(command->validFor - (now - command->issuedAt)));
                    session.status.state = SimulationRobotState::Moving;
                } else {
                    session.velocity = {};
                    session.motionStartedAt.reset();
                    session.status.state = SimulationRobotState::Idle;
                }
                session.statusChanged = session.statusChanged || previousState != session.status.state;
                if (log_) log_->debug("Simulated command accepted: " + describe(*command), id);
            }
            if (error) {
                const std::string message = *error == TransportErrorCode::NotConnected
                    ? "Robot is disconnected"
                    : *error == TransportErrorCode::UnsupportedVersion
                        ? "Unsupported transport domain model version"
                        : "Command is invalid or stale";
                if (log_) {
                    log_->warning("Simulated command rejected (" + std::string{toString(*error)}
                        + "): " + message + "; " + describe(*command), id);
                }
                if (auto sink = sink_.lock()) {
                    sink->onTransportError(
                        {id, TransportOperation::SendCommand, *error, message, now});
                }
            }
        }

        if (session.state != ConnectionState::Connected || !session.desiredConnected
            || (!session.statusChanged && session.lastTelemetryAt && (*session.lastTelemetryAt == now
                || !elapsedAtLeast(*session.lastTelemetryAt, now, config_.telemetryInterval)))) {
            continue;
        }
        const auto velocity = session.velocity;
        session.lastTelemetryAt = now;
        session.statusChanged = false;
        const auto telemetrySequence = session.telemetrySequence++;
        if (log_) {
            auto text = logStream();
            text << "Simulated telemetry: battery=" << config_.batteryVoltageVolts
                 << " V linear=" << velocity.linearMetersPerSecond
                 << " m/s angular=" << velocity.angularRadiansPerSecond << " rad/s";
            log_->debug(text.str(), id);
        }
        if (auto sink = sink_.lock()) {
            sink->onTelemetry({id, now, config_.batteryVoltageVolts,
                velocity.linearMetersPerSecond, velocity.angularRadiansPerSecond,
                {currentTransportModelVersion, telemetrySequence},
                session.emergencyStopRequested ? RobotOperatingState::EmergencyStop
                    : (velocity.linearMetersPerSecond != 0 || velocity.angularRadiansPerSecond != 0)
                        ? RobotOperatingState::Moving : RobotOperatingState::Idle});
        }
    }
}

} // namespace micro_robot
