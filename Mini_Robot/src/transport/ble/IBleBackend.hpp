#pragma once
#include "transport/TransportTypes.hpp"
#include "transport/ble/BleTypes.hpp"
#include "transport/ble/BleProtocol.hpp"

#include <QObject>
#include <QString>
#include <QByteArray>

namespace micro_robot {
struct BleWriteRequest {
    std::uint16_t sequence{};
    bool emergencyStop{};
    RobotCommand command; // Owned intent; re-encode remaining validity at native dispatch.
};
struct BleWriteOutcome {
    bool dispatched{};
    Timestamp dispatchedAt{}; // Qt handoff, not a remote acknowledgement.
    TransportErrorCode code{TransportErrorCode::BackendFailure};
    std::string error;
};
// Internal asynchronous radio boundary. All calls and signals on the owning Qt
// event-loop thread. Tokens fence old scans/connections. No widgets/OS handles.
// release() cancels a token and invalidates its callbacks, without waiting.
class IBleBackend : public QObject {
    Q_OBJECT
public:
    using QObject::QObject;
    virtual void scan(std::uint64_t token, std::size_t maximumDevices) = 0;
    virtual void stopScan() = 0;
    virtual void open(std::uint64_t token, const std::string& deviceId, RobotId expected) = 0;
    virtual void close(std::uint64_t token) = 0;
    virtual void release(std::uint64_t token) = 0;
    // Prompt owning-thread handoff. No hidden command queue. The backend checks
    // readiness/limits/freshness immediately before calling the native write API.
    [[nodiscard]] virtual BleWriteOutcome write(std::uint64_t token, const BleWriteRequest& request) = 0;
signals:
    // Implementations filter BLE + project service UUID before publishing.
    void deviceFound(std::uint64_t token, micro_robot::BleDevice device);
    void scanFinished(std::uint64_t token);
    void phaseChanged(std::uint64_t token, micro_robot::BlePhase phase);
    // Ready means service/property/identity/CCCD validation has succeeded.
    void ready(std::uint64_t token, micro_robot::BleRobotInfo info);
    void notification(std::uint64_t token, micro_robot::BleEndpoint endpoint, QByteArray bytes,
                      micro_robot::Timestamp receivedAt);
    // For WithoutResponse this means native handoff; otherwise ATT confirmation.
    // Neither confirms command acceptance. Command Result is the application ACK.
    void written(std::uint64_t token, std::uint16_t sequence);
    void writeFailed(std::uint64_t token, QString message);
    void closed(std::uint64_t token);
    void failed(std::uint64_t token, micro_robot::TransportErrorCode code, QString message);
};
} // namespace micro_robot
Q_DECLARE_METATYPE(micro_robot::BleDevice)
Q_DECLARE_METATYPE(micro_robot::BlePhase)
Q_DECLARE_METATYPE(micro_robot::TransportErrorCode)
Q_DECLARE_METATYPE(micro_robot::BleRobotInfo)
Q_DECLARE_METATYPE(micro_robot::BleEndpoint)
Q_DECLARE_METATYPE(micro_robot::Timestamp)
