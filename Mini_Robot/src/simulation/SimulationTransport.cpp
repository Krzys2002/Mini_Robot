#include "simulation/SimulationTransport.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

namespace micro_robot {
namespace {

bool fresh(const RobotCommand& command, Timestamp now)
{
    return command.validFor > Duration::zero() && command.issuedAt <= now
        && now - command.issuedAt < command.validFor;
}

bool valid(const RobotCommand& command, Timestamp now)
{
    if (!fresh(command, now)) {
        return false;
    }
    if (const auto* velocity = std::get_if<VelocityCommand>(&command.payload)) {
        return std::isfinite(velocity->linearMetersPerSecond)
            && std::isfinite(velocity->angularRadiansPerSecond);
    }
    return true;
}

} // namespace

void SimulationTransport::setEventSink(std::weak_ptr<IRobotTransportEvents> sink)
{
    sink_ = std::move(sink);
}

ConnectionState SimulationTransport::connectionState(RobotId id) const
{
    const auto found = sessions_.find(id);
    return found == sessions_.end() ? ConnectionState::Disconnected : found->second.state;
}

void SimulationTransport::connect(RobotId id) { sessions_[id].desiredConnected = true; }

void SimulationTransport::disconnect(RobotId id)
{
    auto& session = sessions_[id];
    session.desiredConnected = false;
    session.pendingCommand.reset();
}

void SimulationTransport::sendCommand(RobotId id, const RobotCommand& command)
{
    sessions_[id].pendingCommand = command;
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
            if (auto sink = sink_.lock()) {
                sink->onConnectionStateChanged(id, state, now);
            }
        };
        if (session.desiredConnected && session.state == ConnectionState::Disconnected) {
            session.lastSentCommand.reset();
            session.lastTelemetryAt.reset();
            changeState(ConnectionState::Connected);
        }
        // A connection callback may request immediate disconnection.
        if (!session.desiredConnected && session.state == ConnectionState::Connected) {
            session.lastSentCommand.reset();
            session.lastTelemetryAt.reset();
            changeState(ConnectionState::Disconnected);
        }

        const auto command = std::exchange(session.pendingCommand, std::nullopt);
        if (command) {
            std::optional<TransportErrorCode> error;
            if (session.state != ConnectionState::Connected || !session.desiredConnected) {
                error = TransportErrorCode::NotConnected;
            } else if (!valid(*command, now)) {
                error = TransportErrorCode::InvalidCommand;
            } else {
                session.lastSentCommand = *command;
            }
            if (error) {
                if (auto sink = sink_.lock()) {
                    sink->onTransportError({id, TransportOperation::SendCommand, *error,
                        *error == TransportErrorCode::NotConnected
                            ? "Robot is disconnected" : "Command is invalid or stale", now});
                }
            }
        }

        if (session.state != ConnectionState::Connected || !session.desiredConnected
            || (session.lastTelemetryAt && *session.lastTelemetryAt == now)) {
            continue;
        }
        VelocityCommand velocity{};
        if (session.lastSentCommand && fresh(*session.lastSentCommand, now)) {
            if (const auto* sent = std::get_if<VelocityCommand>(&session.lastSentCommand->payload)) {
                velocity = *sent;
            }
        }
        session.lastTelemetryAt = now;
        if (auto sink = sink_.lock()) {
            sink->onTelemetry({id, now, 3.7,
                velocity.linearMetersPerSecond, velocity.angularRadiansPerSecond});
        }
    }
}

} // namespace micro_robot
