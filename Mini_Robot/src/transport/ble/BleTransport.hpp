#pragma once
#include "transport/IRobotTransport.hpp"
#include "transport/ble/IBleBackend.hpp"
#include "infrastructure/logging/Logger.hpp"

#include <QTimer>

#include <map>
#include <deque>
#include <memory>
#include <vector>

namespace micro_robot {
// BLE Central adapter with bounded command handoff and validated notifications.
// No autonomous motion, high-level safety policy or simulated data is generated.
// Owning Qt event-loop thread only. Requests/events are deferred. Observe domain
// connections via IRobotTransport; scan/phase signals are optional diagnostics.
// Disconnect active robots and await terminal states before destruction.
class BleTransport final : public QObject, public IRobotTransport {
    Q_OBJECT
public:
    explicit BleTransport(std::unique_ptr<IBleBackend> backend, BleTransportConfig config = {},
                          std::optional<LogChannel> log = std::nullopt);
    ~BleTransport() override;
    void setEventSink(std::weak_ptr<IRobotTransportEvents> sink) override;
    [[nodiscard]] ConnectionState connectionState(RobotId id) const override;
    void connect(RobotId id) override;
    void disconnect(RobotId id) override;
    void sendCommand(RobotId id, const RobotCommand& command) override;
    // Adapter-level wire primitive for protocol bring-up/future safety binding
    // (#70 owns the domain safety interface). Latches intent, rejects Velocity,
    // retries on the dedicated characteristic and reasserts before reconnect is
    // published Connected. Clearing the latch is deliberately not exposed here.
    void requestEmergencyStop(RobotId id);
    void startDiscovery();
    void cancelDiscovery(); // Cancels robots still awaiting this scan, not active links.
    [[nodiscard]] bool discovering() const noexcept { return scanning_; }
    [[nodiscard]] const std::vector<BleDevice>& devices() const noexcept { return devices_; }
    [[nodiscard]] BlePhase phase(RobotId id) const;
signals:
    void discoveryChanged();
    void discoveryError(QString message);
    void phaseChanged(micro_robot::RobotId id, micro_robot::BlePhase phase);
private:
    struct Session {
        std::uint64_t token{};
        ConnectionState state{ConnectionState::Disconnected};
        BlePhase phase{BlePhase::Disconnected};
        std::unique_ptr<QTimer> timer;
        std::optional<BleRobotInfo> info;
        BleReceiveSequence receiveSequence;
        std::uint16_t transmitSequence{};
        std::optional<RobotCommand> pending;
        std::optional<std::uint16_t> writing, emergencyWriting;
        std::optional<Timestamp> lastVelocityAt;
        bool pumpScheduled{}, nativeReady{}, emergencyRequested{}, emergencyConfirmed{};
        BleRobotState robotState{BleRobotState::Idle};
        std::size_t invalidNotifications{};
        std::unique_ptr<QTimer> pumpTimer, emergencyTimer, emergencyWriteTimer;
        struct Sent { std::uint16_t sequence{}; std::uint8_t type{}; Timestamp at{}; };
        std::deque<Sent> sent;
    };
    void beginScan();
    void finishScan(bool cancelled, const QString& error = {});
    void beginConnect(RobotId id);
    void beginDisconnect(RobotId id);
    void changePhase(RobotId id, BlePhase phase);
    void changeState(RobotId id, ConnectionState state);
    void error(RobotId id, TransportOperation operation, TransportErrorCode code, const std::string& message);
    void fault(RobotId id, TransportErrorCode code, const std::string& message,
               std::optional<TransportOperation> operation = {});
    [[nodiscard]] std::optional<RobotId> findToken(std::uint64_t token) const;
    void armTimeout(RobotId id, std::chrono::milliseconds duration);
    void clearIo(Session& session);
    void schedulePump(RobotId id);
    void pump(RobotId id);
    void sendEmergencyStop(RobotId id);
    void confirmEmergencyStop(RobotId id);
    void rememberSent(RobotId id, std::uint16_t sequence, std::uint8_t type, Timestamp at,
                      std::optional<std::uint64_t> domainSequence = {});
    void receive(std::uint64_t token, BleEndpoint endpoint, const QByteArray& bytes, Timestamp at);
    std::unique_ptr<IBleBackend> backend_;
    BleTransportConfig config_;
    std::optional<LogChannel> log_;
    std::weak_ptr<IRobotTransportEvents> sink_;
    std::map<RobotId, Session> sessions_;
    std::vector<BleDevice> devices_;
    QTimer scanTimer_;
    std::uint64_t nextToken_{};
    std::uint64_t scanToken_{};
    bool scanning_{};
};
} // namespace micro_robot
Q_DECLARE_METATYPE(micro_robot::RobotId)
