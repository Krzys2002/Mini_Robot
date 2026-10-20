#pragma once
#include "transport/TransportTypes.hpp"
#include "transport/ble/BleTypes.hpp"

#include <QObject>
#include <QString>

namespace micro_robot {
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
signals:
    // Implementations filter BLE + project service UUID before publishing.
    void deviceFound(std::uint64_t token, micro_robot::BleDevice device);
    void scanFinished(std::uint64_t token);
    void phaseChanged(std::uint64_t token, micro_robot::BlePhase phase);
    // Ready means service/property/identity/CCCD validation has succeeded.
    void ready(std::uint64_t token);
    void closed(std::uint64_t token);
    void failed(std::uint64_t token, micro_robot::TransportErrorCode code, QString message);
};
} // namespace micro_robot
Q_DECLARE_METATYPE(micro_robot::BleDevice)
Q_DECLARE_METATYPE(micro_robot::BlePhase)
Q_DECLARE_METATYPE(micro_robot::TransportErrorCode)
