#pragma once

#include "transport/IRobotDiscovery.hpp"
#include "transport/IRobotTransport.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace micro_robot::testing {

// Test-only stand-in for a radio backend such as the future BLE adapter. One object
// implements discovery and transport, as a real adapter would, and maps RobotId to
// private peer addresses that never cross either interface. Requests only record
// intent; deliver(now) completes them, standing in for asynchronous radio
// completion on the owning thread. This is not a production backend.
class MockRadioLink final : public IRobotDiscovery, public IRobotTransport {
public:
    // Radio environment, controlled by the test. A peer in range is heard at the
    // start of every scan and whenever it advertises again during one.
    void advertise(const std::string& address, RobotId claimed, std::string name)
    {
        peers_.insert_or_assign(address, Peer{claimed, std::move(name), true, std::nullopt});
    }
    void withdraw(const std::string& address) { peers_.erase(address); }
    void setAdapterAvailable(bool available) { adapterAvailable_ = available; }
    void loseLink(const std::string& address) { lostLinks_.push_back(address); }
    void receiveTelemetry(const std::string& address, double batteryVoltageVolts)
    {
        notifications_.emplace_back(address, batteryVoltageVolts);
    }
    // What the radio would carry to a peer: the backend resolved the RobotId.
    [[nodiscard]] std::optional<RobotCommand> lastTransmitted(const std::string& address) const
    {
        const auto found = transmitted_.find(address);
        return found == transmitted_.end() ? std::nullopt : std::optional{found->second};
    }

    void deliver(Timestamp now)
    {
        deliverDiscovery(now);
        deliverSessions(now);
        deliverNotifications(now);
    }

    // IRobotDiscovery
    void setEventSink(std::weak_ptr<IRobotDiscoveryEvents> sink) override
    {
        discoverySink_ = std::move(sink);
    }
    RobotScanState scanState() const override { return scanState_; }
    std::vector<DiscoveredRobot> robots() const override { return inventory_; }
    void startScan() override { scanDesired_ = true; }
    void stopScan() override { scanDesired_ = false; }

    // IRobotTransport
    void setEventSink(std::weak_ptr<IRobotTransportEvents> sink) override
    {
        transportSink_ = std::move(sink);
    }
    ConnectionState connectionState(RobotId id) const override
    {
        const auto found = sessions_.find(id);
        return found == sessions_.end() ? ConnectionState::Disconnected : found->second.state;
    }
    void connect(RobotId id) override { sessions_[id].desired = true; }
    void disconnect(RobotId id) override
    {
        auto& session = sessions_[id];
        session.desired = false;
        session.pendingCommand.reset();
    }
    void sendCommand(RobotId id, const RobotCommand& command) override
    {
        sessions_[id].pendingCommand = command;
    }

private:
    struct Peer {
        RobotId claimed;
        std::string name;
        bool heard{};
        std::optional<Timestamp> lastHeardAt;
    };
    struct Session {
        bool desired{};
        ConnectionState state{ConnectionState::Disconnected};
        std::optional<std::string> address;
        std::optional<RobotCommand> pendingCommand;
    };

    void changeScanState(RobotScanState state, Timestamp now)
    {
        scanState_ = state;
        if (auto sink = discoverySink_.lock()) {
            sink->onScanStateChanged(state, now);
        }
    }
    void reportDiscoveryError(RobotDiscoveryErrorCode code, std::string message, Timestamp now)
    {
        if (auto sink = discoverySink_.lock()) {
            sink->onDiscoveryError({code, std::move(message), now});
        }
    }

    void deliverDiscovery(Timestamp now)
    {
        if (scanState_ == RobotScanState::Scanning && !adapterAvailable_) {
            scanDesired_ = false;
            changeScanState(RobotScanState::Idle, now);
            reportDiscoveryError(RobotDiscoveryErrorCode::Unavailable, "Radio adapter lost", now);
        }
        if (scanDesired_ && scanState_ == RobotScanState::Idle) {
            if (!adapterAvailable_) {
                scanDesired_ = false;
                reportDiscoveryError(RobotDiscoveryErrorCode::Unavailable,
                                     "Radio adapter unavailable", now);
                return;
            }
            for (auto& [address, peer] : peers_) {
                (void)address;
                peer.heard = true;
                peer.lastHeardAt.reset();
            }
            published_ = false;
            changeScanState(RobotScanState::Scanning, now);
        }
        if (!scanDesired_ && scanState_ == RobotScanState::Scanning) {
            changeScanState(RobotScanState::Idle, now);
        }
        if (scanState_ != RobotScanState::Scanning) {
            return;
        }

        // Group peers by claimed identity; the address stays inside this backend.
        std::map<RobotId, std::vector<std::string>> claims;
        std::vector<DiscoveredRobot> inventory;
        for (auto& [address, peer] : peers_) {
            if (std::exchange(peer.heard, false)) {
                peer.lastHeardAt = now;
            }
            if (!peer.lastHeardAt) {
                continue;
            }
            auto& addresses = claims[peer.claimed];
            addresses.push_back(address);
            if (addresses.size() == 1) {
                inventory.push_back({peer.claimed, peer.name, *peer.lastHeardAt});
            }
        }
        std::sort(inventory.begin(), inventory.end(),
                  [](const auto& left, const auto& right) { return left.robotId < right.robotId; });
        for (auto& robot : inventory) {
            const auto& addresses = claims.at(robot.robotId);
            robot.ambiguous = addresses.size() > 1;
            for (const auto& address : addresses) {
                if (*peers_.at(address).lastHeardAt > robot.lastSeenAt) {
                    robot.lastSeenAt = *peers_.at(address).lastHeardAt;
                }
            }
        }
        resolved_ = std::move(claims);
        if (published_ && sameInventory(inventory, inventory_)) {
            return;
        }
        published_ = true;
        inventory_ = inventory;
        if (auto sink = discoverySink_.lock()) {
            sink->onRobotsChanged(std::move(inventory));
        }
    }

    static bool sameInventory(const std::vector<DiscoveredRobot>& left,
                              const std::vector<DiscoveredRobot>& right)
    {
        return std::equal(left.begin(), left.end(), right.begin(), right.end(),
            [](const auto& a, const auto& b) {
                return a.robotId == b.robotId && a.displayName == b.displayName
                    && a.lastSeenAt == b.lastSeenAt && a.ambiguous == b.ambiguous;
            });
    }

    void deliverSessions(Timestamp now)
    {
        std::vector<RobotId> ids;
        for (const auto& [id, session] : sessions_) {
            (void)session;
            ids.push_back(id);
        }
        for (const auto id : ids) {
            auto& session = sessions_.at(id);
            auto changeState = [&](ConnectionState state) {
                session.state = state;
                if (auto sink = transportSink_.lock()) {
                    sink->onConnectionStateChanged(id, state, now);
                }
            };
            auto fail = [&](TransportErrorCode code, std::string message) {
                session.desired = false;
                session.address.reset();
                session.pendingCommand.reset();
                changeState(ConnectionState::Fault);
                reportTransportError(id, TransportOperation::Connect, code, std::move(message), now);
            };

            if (session.state == ConnectionState::Connected && session.address) {
                const auto lost = std::find(lostLinks_.begin(), lostLinks_.end(), *session.address);
                if (lost != lostLinks_.end()) {
                    fail(TransportErrorCode::BackendFailure, "Radio link lost");
                    continue;
                }
            }
            const bool idle = session.state == ConnectionState::Disconnected
                || session.state == ConnectionState::Fault;
            if (session.desired && idle) {
                const auto candidates = resolved_.find(id);
                if (candidates == resolved_.end() || candidates->second.size() != 1
                    || !peers_.contains(candidates->second.front())) {
                    const bool ambiguous = candidates != resolved_.end()
                        && candidates->second.size() > 1;
                    fail(TransportErrorCode::Unavailable,
                         ambiguous ? "Several peers claim this robot" : "Robot not discovered");
                    continue;
                }
                session.address = candidates->second.front();
                changeState(ConnectionState::Connecting);
                // A state callback may cancel the attempt before it completes.
                if (session.desired) {
                    changeState(ConnectionState::Connected);
                } else {
                    session.address.reset();
                    changeState(ConnectionState::Disconnected);
                }
            }
            if (!session.desired && session.state == ConnectionState::Connected) {
                changeState(ConnectionState::Disconnecting);
                session.address.reset();
                changeState(ConnectionState::Disconnected);
            }

            const auto command = std::exchange(session.pendingCommand, std::nullopt);
            if (!command) {
                continue;
            }
            if (session.state != ConnectionState::Connected || !session.desired) {
                reportTransportError(id, TransportOperation::SendCommand,
                    TransportErrorCode::NotConnected, "Robot is disconnected", now);
            } else if (!isSupportedTransportModelVersion(command->metadata.modelVersion)) {
                reportTransportError(id, TransportOperation::SendCommand,
                    TransportErrorCode::UnsupportedVersion, "Unsupported model version", now);
            } else if (!valid(*command, now)) {
                reportTransportError(id, TransportOperation::SendCommand,
                    TransportErrorCode::InvalidCommand, "Command is invalid or stale", now);
            } else {
                transmitted_.insert_or_assign(*session.address, *command);
            }
        }
        lostLinks_.clear();
    }

    void deliverNotifications(Timestamp now)
    {
        auto notifications = std::exchange(notifications_, {});
        for (const auto& [address, volts] : notifications) {
            // Notifications from peers without a session are dropped, as a radio
            // adapter would ignore unsolicited data.
            for (const auto& [id, session] : sessions_) {
                if (session.state == ConnectionState::Connected && session.address == address) {
                    if (auto sink = transportSink_.lock()) {
                        sink->onTelemetry({id, now, volts, std::nullopt, std::nullopt});
                    }
                    break;
                }
            }
        }
    }

    void reportTransportError(RobotId id, TransportOperation operation, TransportErrorCode code,
                              std::string message, Timestamp now)
    {
        if (auto sink = transportSink_.lock()) {
            sink->onTransportError({id, operation, code, std::move(message), now});
        }
    }

    static bool valid(const RobotCommand& command, Timestamp now)
    {
        if (command.validFor <= Duration::zero() || command.issuedAt > now
            || now - command.issuedAt >= command.validFor) {
            return false;
        }
        if (const auto* velocity = std::get_if<VelocityCommand>(&command.payload)) {
            return std::isfinite(velocity->linearMetersPerSecond)
                && std::isfinite(velocity->angularRadiansPerSecond);
        }
        return true;
    }

    std::map<std::string, Peer> peers_;
    bool adapterAvailable_{true};
    std::vector<std::string> lostLinks_;
    std::vector<std::pair<std::string, double>> notifications_;
    std::map<std::string, RobotCommand> transmitted_;

    std::weak_ptr<IRobotDiscoveryEvents> discoverySink_;
    bool scanDesired_{};
    RobotScanState scanState_{RobotScanState::Idle};
    std::vector<DiscoveredRobot> inventory_;
    bool published_{};
    std::map<RobotId, std::vector<std::string>> resolved_;

    std::weak_ptr<IRobotTransportEvents> transportSink_;
    std::map<RobotId, Session> sessions_;
};

} // namespace micro_robot::testing
