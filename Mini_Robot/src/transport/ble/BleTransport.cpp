#include "transport/ble/BleTransport.hpp"

#include <QMetaObject>

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <utility>

namespace micro_robot {
BleTransport::BleTransport(std::unique_ptr<IBleBackend> backend, BleTransportConfig config,
                           std::optional<LogChannel> log)
    : backend_(std::move(backend)), config_(config), log_(std::move(log))
{
    const auto validTimeout = [](auto duration) {
        return duration.count() > 0 && duration.count() <= std::numeric_limits<int>::max();
    };
    if (!backend_ || backend_->parent() || backend_->thread() != thread()
        || !validTimeout(config.scanTimeout) || !validTimeout(config.phaseTimeout)
        || !validTimeout(config.disconnectTimeout) || !validTimeout(config.emergencyStopRetry)
        || !validTimeout(config.minimumVelocityInterval) || config.maximumDevices == 0
        || !config.maximumTrackedCommands || config.maximumTrackedCommands >= 32768
        || !config.maximumInvalidNotifications)
        throw std::invalid_argument("BLE requires an unparented same-thread backend and positive bounded timeouts/device limit");
    qRegisterMetaType<std::uint64_t>();
    qRegisterMetaType<BleDevice>();
    qRegisterMetaType<BlePhase>();
    qRegisterMetaType<RobotId>();
    qRegisterMetaType<TransportErrorCode>();
    qRegisterMetaType<BleRobotInfo>();
    qRegisterMetaType<BleEndpoint>();
    qRegisterMetaType<Timestamp>();
    scanTimer_.setSingleShot(true);
    QObject::connect(&scanTimer_, &QTimer::timeout, this, [this] { finishScan(false); });
    QObject::connect(backend_.get(), &IBleBackend::deviceFound, this,
        [this](std::uint64_t token, BleDevice device) {
            if (!scanning_ || token != scanToken_ || device.id.empty()) return;
            device.advertisedId = advertisedBleRobotId(device.name);
            auto found = std::find_if(devices_.begin(), devices_.end(), [&](const auto& d) { return d.id == device.id; });
            if (found != devices_.end()) *found = std::move(device);
            else if (devices_.size() < config_.maximumDevices) devices_.push_back(std::move(device));
            emit discoveryChanged();
        }, Qt::QueuedConnection);
    QObject::connect(backend_.get(), &IBleBackend::scanFinished, this, [this](std::uint64_t token) {
        if (scanning_ && token == scanToken_) finishScan(false);
    }, Qt::QueuedConnection);
    QObject::connect(backend_.get(), &IBleBackend::phaseChanged, this,
        [this](std::uint64_t token, BlePhase next) {
            const auto id = findToken(token);
            if (!id || connectionState(*id) != ConnectionState::Connecting) return;
            // Backends cannot extend timeouts by repeating the same state.
            if (next == BlePhase::DiscoveringServices || next == BlePhase::DiscoveringCharacteristics
                || next == BlePhase::ReadingIdentity || next == BlePhase::EnablingNotifications) {
                if (phase(*id) != next) {
                    changePhase(*id, next);
                    armTimeout(*id, config_.phaseTimeout);
                }
            }
        }, Qt::QueuedConnection);
    QObject::connect(backend_.get(), &IBleBackend::ready, this, [this](std::uint64_t token, BleRobotInfo info) {
        const auto id = findToken(token);
        if (!id || connectionState(*id) != ConnectionState::Connecting || phase(*id) == BlePhase::Scanning) return;
        auto& session = sessions_.at(*id);
        if (info.robotId != *id || !info.maximumLinearMillimetersPerSecond || info.maximumLinearMillimetersPerSecond > 32767
            || !info.maximumAngularMilliradiansPerSecond || info.maximumAngularMilliradiansPerSecond > 32767
            || !info.maximumValidityMilliseconds || !info.telemetryPeriodMilliseconds) {
            fault(*id, TransportErrorCode::BackendFailure, "Invalid validated Robot Info from BLE backend");
            return;
        }
        session.info = info;
        session.nativeReady = true;
        if (session.emergencyRequested) {
            armTimeout(*id, config_.phaseTimeout);
            sendEmergencyStop(*id);
        } else {
            session.timer->stop();
            changePhase(*id, BlePhase::Ready);
            changeState(*id, ConnectionState::Connected);
        }
    }, Qt::QueuedConnection);
    QObject::connect(backend_.get(), &IBleBackend::notification, this, &BleTransport::receive, Qt::QueuedConnection);
    QObject::connect(backend_.get(), &IBleBackend::written, this, [this](std::uint64_t token, std::uint16_t sequence) {
        const auto id = findToken(token);
        if (!id) return;
        auto& session = sessions_.at(*id);
        if (log_) log_->debug("BLE write handoff/ATT confirmation token=" + std::to_string(token)
            + " wire_sequence=" + std::to_string(sequence) + " steady_us="
            + std::to_string(std::chrono::duration_cast<std::chrono::microseconds>(MonotonicClock::now().time_since_epoch()).count()), *id);
        if (session.emergencyWriting == sequence) {
            session.emergencyWriting.reset();
            if (session.emergencyWriteTimer) session.emergencyWriteTimer->stop();
        }
        if (session.writing == sequence) {
            session.writing.reset();
            if (session.timer) session.timer->stop();
            schedulePump(*id);
        }
    }, Qt::QueuedConnection);
    QObject::connect(backend_.get(), &IBleBackend::writeFailed, this, [this](std::uint64_t token, const QString& message) {
        if (const auto id = findToken(token)) fault(*id, TransportErrorCode::BackendFailure,
            message.toStdString(), TransportOperation::SendCommand);
    }, Qt::QueuedConnection);
    QObject::connect(backend_.get(), &IBleBackend::closed, this, [this](std::uint64_t token) {
        const auto id = findToken(token);
        if (!id) return;
        if (connectionState(*id) != ConnectionState::Disconnecting) {
            fault(*id, TransportErrorCode::Unavailable, "BLE link disconnected unexpectedly");
            return;
        }
        auto& session = sessions_.at(*id);
        clearIo(session);
        session.timer->stop();
        backend_->release(token);
        session.token = 0;
        changePhase(*id, BlePhase::Disconnected);
        changeState(*id, ConnectionState::Disconnected);
    }, Qt::QueuedConnection);
    QObject::connect(backend_.get(), &IBleBackend::failed, this,
        [this](std::uint64_t token, TransportErrorCode code, const QString& message) {
            if (scanning_ && token == scanToken_) finishScan(false, message);
            else if (const auto id = findToken(token)) fault(*id, code, message.toStdString());
        }, Qt::QueuedConnection);
}
BleTransport::~BleTransport()
{
    // No callbacks into this adapter survive QObject destruction. Backend RAII
    // tears down native links/scanner, including a pending permission request.
    backend_->disconnect(this);
    backend_->stopScan();
    for (const auto& [id, session] : sessions_) {
        (void)id;
        if (session.token) backend_->release(session.token);
    }
}
void BleTransport::setEventSink(std::weak_ptr<IRobotTransportEvents> sink) { sink_ = std::move(sink); }
ConnectionState BleTransport::connectionState(RobotId id) const
{
    const auto found = sessions_.find(id);
    return found == sessions_.end() ? ConnectionState::Disconnected : found->second.state;
}
BlePhase BleTransport::phase(RobotId id) const
{
    const auto found = sessions_.find(id);
    return found == sessions_.end() ? BlePhase::Disconnected : found->second.phase;
}
std::optional<RobotId> BleTransport::findToken(std::uint64_t token) const
{
    if (token) for (const auto& [id, session] : sessions_) if (session.token == token) return id;
    return {};
}
void BleTransport::changePhase(RobotId id, BlePhase next)
{
    sessions_.at(id).phase = next;
    if (log_) log_->info(std::string{"BLE phase: "} + blePhaseName(next), id);
    emit phaseChanged(id, next);
}
void BleTransport::changeState(RobotId id, ConnectionState state)
{
    sessions_.at(id).state = state;
    if (auto sink = sink_.lock()) sink->onConnectionStateChanged(id, state, MonotonicClock::now());
}
void BleTransport::error(RobotId id, TransportOperation op, TransportErrorCode code, const std::string& message)
{
    if (log_) log_->warning(message, id);
    if (auto sink = sink_.lock()) sink->onTransportError({id, op, code, message, MonotonicClock::now()});
}
void BleTransport::fault(RobotId id, TransportErrorCode code, const std::string& message,
                         std::optional<TransportOperation> operationOverride)
{
    auto& session = sessions_.at(id);
    const auto operation = operationOverride ? *operationOverride : session.state == ConnectionState::Disconnecting
        ? TransportOperation::Disconnect : TransportOperation::Connect;
    clearIo(session);
    if (session.timer) session.timer->stop();
    const auto token = std::exchange(session.token, 0);
    backend_->release(token);
    changePhase(id, BlePhase::Fault);
    changeState(id, ConnectionState::Fault);
    error(id, operation, code, message);
}
void BleTransport::armTimeout(RobotId id, std::chrono::milliseconds duration)
{
    auto& session = sessions_.at(id);
    if (!session.timer) {
        session.timer = std::make_unique<QTimer>();
        session.timer->setSingleShot(true);
        session.timer->setTimerType(Qt::PreciseTimer);
        QObject::connect(session.timer.get(), &QTimer::timeout, this, [this, id] {
            const auto writing = sessions_.at(id).writing.has_value();
            fault(id, TransportErrorCode::BackendFailure, writing ? "BLE command write confirmation timed out"
                : std::string{"BLE timeout during "} + blePhaseName(phase(id)), writing
                    ? std::optional{TransportOperation::SendCommand} : std::nullopt);
        });
    }
    session.timer->start(static_cast<int>(duration.count()));
}
void BleTransport::connect(RobotId id)
{
    QMetaObject::invokeMethod(this, [this, id] { beginConnect(id); }, Qt::QueuedConnection);
}
void BleTransport::beginConnect(RobotId id)
{
    auto& session = sessions_[id];
    if (session.state == ConnectionState::Connected || session.state == ConnectionState::Connecting
        || session.state == ConnectionState::Disconnecting) return;
    session.token = ++nextToken_;
    clearIo(session);
    changePhase(id, BlePhase::Scanning);
    changeState(id, ConnectionState::Connecting);
    beginScan();
}
void BleTransport::startDiscovery()
{
    QMetaObject::invokeMethod(this, [this] { beginScan(); }, Qt::QueuedConnection);
}
void BleTransport::beginScan()
{
    if (scanning_) return;
    scanning_ = true;
    devices_.clear();
    scanToken_ = ++nextToken_;
    scanTimer_.start(static_cast<int>(config_.scanTimeout.count()));
    emit discoveryChanged();
    backend_->scan(scanToken_, config_.maximumDevices);
}
void BleTransport::cancelDiscovery()
{
    QMetaObject::invokeMethod(this, [this] { finishScan(true); }, Qt::QueuedConnection);
}
void BleTransport::finishScan(bool cancelled, const QString& scanError)
{
    if (!scanning_) return;
    scanning_ = false;
    scanToken_ = 0;
    scanTimer_.stop();
    backend_->stopScan();
    emit discoveryChanged();
    if (!scanError.isEmpty()) emit discoveryError(scanError);
    for (auto& [id, session] : sessions_) {
        if (session.phase != BlePhase::Scanning || session.state != ConnectionState::Connecting) continue;
        if (cancelled) { beginDisconnect(id); continue; }
        if (!scanError.isEmpty()) { fault(id, TransportErrorCode::Unavailable, scanError.toStdString()); continue; }
        std::vector<const BleDevice*> matches;
        for (const auto& device : devices_) if (device.advertisedId == id) matches.push_back(&device);
        if (matches.size() != 1) {
            fault(id, TransportErrorCode::Unavailable, matches.empty()
                ? "BLE robot not found before scan timeout; check power, advertisement and permissions"
                : "Ambiguous BLE robot identity: multiple peripherals advertise the requested RobotId");
            continue;
        }
        changePhase(id, BlePhase::Connecting);
        armTimeout(id, config_.phaseTimeout);
        backend_->open(session.token, matches.front()->id, id);
    }
}
void BleTransport::disconnect(RobotId id)
{
    QMetaObject::invokeMethod(this, [this, id] { beginDisconnect(id); }, Qt::QueuedConnection);
}
void BleTransport::beginDisconnect(RobotId id)
{
    if (connectionState(id) == ConnectionState::Disconnected || connectionState(id) == ConnectionState::Fault
        || connectionState(id) == ConnectionState::Disconnecting) return;
    auto& session = sessions_.at(id);
    const bool scanning = session.phase == BlePhase::Scanning;
    clearIo(session);
    changePhase(id, BlePhase::Disconnecting);
    changeState(id, ConnectionState::Disconnecting);
    if (scanning) {
        session.token = 0;
        changePhase(id, BlePhase::Disconnected);
        changeState(id, ConnectionState::Disconnected);
        return;
    }
    armTimeout(id, config_.disconnectTimeout);
    backend_->close(session.token);
}
void BleTransport::sendCommand(RobotId id, const RobotCommand& command)
{
    const auto reject = [this, id](TransportErrorCode code, const std::string& message) {
        QMetaObject::invokeMethod(this, [this, id, code, message] {
            error(id, TransportOperation::SendCommand, code, message);
        }, Qt::QueuedConnection);
    };
    if (!isSupportedTransportModelVersion(command.metadata.modelVersion)) {
        reject(TransportErrorCode::UnsupportedVersion, "Unsupported transport domain model version"); return;
    }
    if (connectionState(id) != ConnectionState::Connected || !sessions_.at(id).nativeReady || !sessions_.at(id).info) {
        reject(TransportErrorCode::NotConnected, "BLE robot is not connected"); return;
    }
    auto& session = sessions_.at(id);
    const auto encoded = encodeBleCommand(command, *session.info, session.transmitSequence, MonotonicClock::now());
    if (!encoded.frame) { reject(encoded.code, encoded.error); return; }
    if (std::holds_alternative<VelocityCommand>(command.payload)
        && (session.emergencyRequested || session.robotState == BleRobotState::EmergencyStop || session.robotState == BleRobotState::Fault)) {
        reject(TransportErrorCode::InvalidCommand, "Velocity blocked by emergency-stop or robot fault state"); return;
    }
    // Keep one newest intent. A pending Stop cannot be overwritten by Velocity.
    if (!session.pending || std::holds_alternative<StopCommand>(command.payload)
        || !std::holds_alternative<StopCommand>(session.pending->payload)) session.pending = command;
    schedulePump(id);
}
} // namespace micro_robot
