#include "transport/ble/QtBleBackend.hpp"
#include "transport/ble/BleWire.hpp"

#include <QBluetoothDeviceDiscoveryAgent>
#include <QBluetoothDeviceInfo>
#include <QBluetoothUuid>
#include <QCoreApplication>
#include <QLowEnergyCharacteristic>
#include <QLowEnergyController>
#include <QLowEnergyDescriptor>
#include <QLowEnergyService>
#if QT_VERSION >= QT_VERSION_CHECK(6, 5, 0)
#include <QPermissions>
#endif

#include <array>
#include <map>

namespace micro_robot {
namespace {
// Project UUID base (ble-gatt.md); %1 is the 16-bit ble_gatt suffix as four hex digits.
constexpr auto uuidTemplate = "4d52%1-1f2a-4032-8415-63a5fef8a0dc";
constexpr int uuidSuffixDigits = 4, uuidSuffixBase = 16;
QBluetoothUuid uuid(unsigned suffix)
{
    return QBluetoothUuid{QString::fromLatin1(uuidTemplate)
        .arg(suffix, uuidSuffixDigits, uuidSuffixBase, QLatin1Char('0'))};
}
// Qt treats a zero LE discovery timeout as "scan until stopped"; BleTransport owns the timeout.
constexpr int scanUntilStopped = 0;
// CCCD value 0x0001, little-endian: notifications on, indications off.
QByteArray notificationsEnabled() { return QByteArray::fromHex("0100"); }
using namespace ble_gatt;
static_assert(notificationCopyLimit == 21, "v1 backend notification copy bound is frozen");
class QtBleBackend final : public IBleBackend {
public:
    ~QtBleBackend() override
    {
        stopScan();
        while (!links_.empty()) release(links_.begin()->first);
    }
    void scan(std::uint64_t token, std::size_t maximumDevices) override
    {
        stopScan();
        scanToken_ = token;
        devices_.clear();
        maximumDevices_ = maximumDevices;
#if QT_VERSION >= QT_VERSION_CHECK(6, 5, 0)
        QBluetoothPermission permission;
        permission.setCommunicationModes(QBluetoothPermission::Access);
        const auto status = QCoreApplication::instance()->checkPermission(permission);
        if (status == Qt::PermissionStatus::Undetermined) {
            QCoreApplication::instance()->requestPermission(permission, this,
                [this, token](const QPermission& result) {
                    if (scanToken_ != token) return;
                    if (result.status() == Qt::PermissionStatus::Granted) beginScan(token);
                    else emit failed(token, TransportErrorCode::Unavailable,
                        QStringLiteral("Bluetooth permission denied; allow access in system settings"));
                });
            return;
        }
        if (status == Qt::PermissionStatus::Denied) {
            emit failed(token, TransportErrorCode::Unavailable, QStringLiteral("Bluetooth permission denied"));
            return;
        }
#endif
        beginScan(token);
    }
    void stopScan() override
    {
        scanToken_ = 0; // Also fences a pending permission callback.
        if (scanner_) {
            scanner_->disconnect(this);
            scanner_->stop();
            scanner_.reset();
        }
    }
    void open(std::uint64_t token, const std::string& deviceId, RobotId expected) override
    {
        const auto found = devices_.find(deviceId);
        if (found == devices_.end()) {
            emit failed(token, TransportErrorCode::Unavailable, QStringLiteral("BLE scan identity expired; scan again"));
            return;
        }
        auto* link = new Link(this, token, expected);
        links_.emplace(token, link);
        link->controller = QLowEnergyController::createCentral(found->second, link);
        if (!link->controller) { fail(link, "Cannot create Qt BLE Central controller"); return; }
        QObject::connect(link->controller, &QLowEnergyController::errorOccurred, link,
            [this, link](QLowEnergyController::Error) {
                if (link->established) lose(link);
                else fail(link, link->controller->errorString());
            });
        QObject::connect(link->controller, &QLowEnergyController::disconnected, link, [this, link] {
            if (!link->active) return;
            emit closed(link->token);
        });
        QObject::connect(link->controller, &QLowEnergyController::connected, link, [this, link] {
            if (!usable(link)) return;
            emit phaseChanged(link->token, BlePhase::DiscoveringServices);
            link->controller->discoverServices();
        });
        QObject::connect(link->controller, &QLowEnergyController::discoveryFinished, link, [this, link] {
            if (!usable(link)) return;
            if (!link->controller->services().contains(uuid(robotControlService))) { fail(link, "Robot Control service is missing"); return; }
            link->service = link->controller->createServiceObject(uuid(robotControlService), link);
            if (!link->service) { fail(link, "Cannot create Robot Control service"); return; }
            QObject::connect(link->service, &QLowEnergyService::errorOccurred, link,
                [this, link](QLowEnergyService::ServiceError error) {
                    if (link->established && usable(link)) {
                        link->failed = true;
                        emit writeFailed(link->token, QStringLiteral("BLE GATT write/channel operation failed (%1)").arg(static_cast<int>(error)));
                    } else fail(link, "BLE GATT service operation failed");
                });
            QObject::connect(link->service, &QLowEnergyService::stateChanged, link,
                [this, link](QLowEnergyService::ServiceState state) {
                    if (!usable(link)) return;
                    if (state == QLowEnergyService::RemoteServiceDiscovered) inspectService(link);
                    else if (state == QLowEnergyService::InvalidService) {
                        if (link->established) lose(link);
                        else fail(link, "Robot Control service became invalid");
                    }
                });
            QObject::connect(link->service, &QLowEnergyService::characteristicRead, link,
                [this, link](const QLowEnergyCharacteristic& characteristic, const QByteArray& value) {
                    if (!usable(link) || !link->readingInfo || characteristic.uuid() != uuid(robotInfoCharacteristic)) return;
                    link->readingInfo = false;
                    const auto result = validateBleRobotInfo(
                        {reinterpret_cast<const std::uint8_t*>(value.constData()), static_cast<std::size_t>(value.size())},
                        link->expected);
                    if (!result.info) {
                        fail(link, QString::fromStdString(result.error), result.unsupportedVersion
                            ? TransportErrorCode::UnsupportedVersion : TransportErrorCode::BackendFailure);
                        return;
                    }
                    link->info = *result.info;
                    emit phaseChanged(link->token, BlePhase::EnablingNotifications);
                    link->subscription = Subscription::CommandResult;
                    link->service->writeDescriptor(link->resultCccd, notificationsEnabled());
                });
            QObject::connect(link->service, &QLowEnergyService::descriptorWritten, link,
                [this, link](const QLowEnergyDescriptor& descriptor, const QByteArray& value) {
                    if (!usable(link)) return;
                    if ((link->subscription == Subscription::CommandResult && descriptor != link->resultCccd)
                        || (link->subscription == Subscription::Telemetry && descriptor != link->telemetryCccd)
                        || link->subscription == Subscription::Idle) return;
                    if (value != notificationsEnabled()) { fail(link, "BLE notification subscription was refused"); return; }
                    if (link->subscription == Subscription::CommandResult) {
                        link->subscription = Subscription::Telemetry;
                        link->service->writeDescriptor(link->telemetryCccd, notificationsEnabled());
                    } else {
                        link->subscription = Subscription::Idle;
                        link->established = true;
                        emit ready(link->token, link->info);
                    }
                });
            QObject::connect(link->service, &QLowEnergyService::characteristicChanged, link,
                [this, link](const QLowEnergyCharacteristic& characteristic, const QByteArray& bytes) {
                    if (!usable(link)) return;
                    const auto endpoint = characteristic.uuid() == uuid(telemetryCharacteristic)
                        ? BleEndpoint::Telemetry : BleEndpoint::CommandResult;
                    if (characteristic.uuid() != uuid(telemetryCharacteristic)
                        && characteristic.uuid() != uuid(commandResultCharacteristic)) return;
                    emit notification(link->token, endpoint, boundBleNotificationCopy(bytes), MonotonicClock::now());
                });
            QObject::connect(link->service, &QLowEnergyService::characteristicWritten, link,
                [this, link](const QLowEnergyCharacteristic& characteristic, const QByteArray& bytes) {
                    if (!usable(link)) return;
                    if (characteristic.uuid() != uuid(commandCharacteristic)
                        && characteristic.uuid() != uuid(emergencyStopCharacteristic)) return;
                    auto& pending = characteristic.uuid() == uuid(emergencyStopCharacteristic)
                        ? link->emergencyWrite : link->ordinaryWrite;
                    if (!pending || pending->bytes != bytes) return;
                    const auto sequence = pending->sequence;
                    pending.reset();
                    emit written(link->token, sequence);
                });
            emit phaseChanged(link->token, BlePhase::DiscoveringCharacteristics);
            // Qt 6.12 on macOS leaves CCCDs unavailable with SkipValueDiscovery.
            // Full discovery populates them before validation and subscription.
            // Robot Info is still explicitly read and validated below.
            link->service->discoverDetails(QLowEnergyService::FullDiscovery);
        });
        link->controller->connectToDevice();
    }
    void close(std::uint64_t token) override
    {
        const auto found = links_.find(token);
        if (found == links_.end()) { emit closed(token); return; }
        auto* link = found->second;
        link->closing = true;
        if (link->controller->state() == QLowEnergyController::UnconnectedState) emit closed(token);
        else link->controller->disconnectFromDevice();
    }
    BleWriteOutcome write(std::uint64_t token, const BleWriteRequest& request) override
    {
        const auto found = links_.find(token);
        if (found == links_.end() || !usable(found->second) || !found->second->established)
            return {false, {}, TransportErrorCode::NotConnected, "BLE GATT session is not ready"};
        auto* link = found->second;
        auto& pending = request.emergencyStop ? link->emergencyWrite : link->ordinaryWrite;
        if (pending) return {false, {}, TransportErrorCode::Unavailable, "BLE characteristic write is still pending"};
        const auto now = MonotonicClock::now();
        auto encoded = request.emergencyStop
            ? BleEncodeResult{encodeBleEmergencyStop(request.sequence), TransportErrorCode::InvalidCommand, {}}
            : request.emergencyClear
                ? BleEncodeResult{encodeBleEmergencyClear(request.sequence), TransportErrorCode::InvalidCommand, {}}
                : encodeBleCommand(request.command, link->info, request.sequence, now);
        if (!encoded.frame) return {false, {}, encoded.code, encoded.error};
        if (auto refused = applyBleOutgoingProtection(request, *encoded.frame))
            return {false, {}, TransportErrorCode::InvalidCommand, std::move(*refused)};
        const auto& frame = *encoded.frame;
        const QByteArray bytes(reinterpret_cast<const char*>(frame.bytes.data()), static_cast<qsizetype>(frame.bytes.size()));
        if (frame.withResponse) pending = PendingWrite{request.sequence, bytes};
        const auto target = request.emergencyStop ? emergencyStopCharacteristic : commandCharacteristic;
        link->service->writeCharacteristic(link->service->characteristic(uuid(target)), bytes,
            frame.withResponse ? QLowEnergyService::WriteWithResponse : QLowEnergyService::WriteWithoutResponse);
        if (!frame.withResponse) emit written(token, request.sequence);
        return {true, now, TransportErrorCode::BackendFailure, {}};
    }
    void release(std::uint64_t token) override
    {
        const auto found = links_.find(token);
        if (found == links_.end()) return;
        auto* link = found->second;
        links_.erase(found);
        link->active = false;
        if (link->controller) link->controller->disconnectFromDevice();
        // Transport processes our signals queued, so release is outside the
        // native signal callback. Deleting the context cancels all native slots.
        delete link;
    }
private:
    struct PendingWrite { std::uint16_t sequence{}; QByteArray bytes; };
    // CCCD subscription step: Command Result first, then Telemetry; Idle otherwise.
    enum class Subscription { Idle, CommandResult, Telemetry };
    struct Link : QObject {
        Link(QObject* parent, std::uint64_t value, RobotId id) : QObject(parent), token(value), expected(id) {}
        std::uint64_t token;
        RobotId expected;
        QLowEnergyController* controller{}; // QObject children of this link.
        QLowEnergyService* service{};
        QLowEnergyDescriptor resultCccd, telemetryCccd;
        bool active{true}, closing{}, failed{}, readingInfo{}, established{};
        Subscription subscription{Subscription::Idle};
        BleRobotInfo info;
        std::optional<PendingWrite> ordinaryWrite, emergencyWrite;
    };
    static bool usable(const Link* link) { return link->active && !link->closing && !link->failed; }
    void fail(Link* link, const QString& message, TransportErrorCode code = TransportErrorCode::BackendFailure)
    {
        if (!link->active || link->failed) return;
        link->failed = true;
        emit failed(link->token, code, message);
    }
    // After setup, a controller error or service invalidation is how the native stack
    // reports a vanished peer (often before `disconnected`); report it as link loss.
    // Setup-time errors remain failures because the link never became usable.
    void lose(Link* link)
    {
        if (!link->active || link->failed) return;
        link->failed = true;
        emit closed(link->token);
    }
    void beginScan(std::uint64_t token)
    {
        if (scanToken_ != token) return;
        scanner_ = std::make_unique<QBluetoothDeviceDiscoveryAgent>();
        // Adapter owns the actual timeout, including permission and cancellation.
        scanner_->setLowEnergyDiscoveryTimeout(scanUntilStopped);
        const auto discovered = [this, token](const QBluetoothDeviceInfo& info) {
            if (scanToken_ != token || !(info.coreConfigurations() & QBluetoothDeviceInfo::LowEnergyCoreConfiguration)
                || !info.serviceUuids().contains(uuid(robotControlService))) return;
            const auto key = !info.deviceUuid().isNull() ? info.deviceUuid().toString().toStdString()
                : info.address().toString().toStdString();
            if (key.empty() || (info.deviceUuid().isNull() && info.address().isNull())) return;
            // Match adapter's bounded inventory; updates to an existing key remain allowed.
            if (devices_.size() >= maximumDevices_ && !devices_.contains(key)) return;
            devices_.insert_or_assign(key, info);
            BleDevice device{key, info.name().toStdString(), {}, advertisedBleRobotId(info.name().toStdString())};
            if (!info.address().isNull()) device.address = info.address().toString().toStdString();
            emit deviceFound(token, std::move(device));
        };
        QObject::connect(scanner_.get(), &QBluetoothDeviceDiscoveryAgent::deviceDiscovered, this, discovered);
        QObject::connect(scanner_.get(), &QBluetoothDeviceDiscoveryAgent::deviceUpdated, this,
            [discovered](const QBluetoothDeviceInfo& info, QBluetoothDeviceInfo::Fields) { discovered(info); });
        QObject::connect(scanner_.get(), &QBluetoothDeviceDiscoveryAgent::finished, this,
            [this, token] { if (scanToken_ == token) emit scanFinished(token); });
        QObject::connect(scanner_.get(), &QBluetoothDeviceDiscoveryAgent::errorOccurred, this,
            [this, token](QBluetoothDeviceDiscoveryAgent::Error) {
                if (scanToken_ == token) emit failed(token, TransportErrorCode::Unavailable, scanner_->errorString());
            });
        scanner_->start(QBluetoothDeviceDiscoveryAgent::LowEnergyMethod);
    }
    void inspectService(Link* link)
    {
        if (link->readingInfo || link->subscription != Subscription::Idle || link->established) return;
        using C = QLowEnergyCharacteristic;
        const QBluetoothUuid cccd{QBluetoothUuid::DescriptorType::ClientCharacteristicConfiguration};
        std::array<BleCharacteristicCapabilities, characteristicCount> layout{};
        for (unsigned i = 0; i < layout.size(); ++i) {
            const auto characteristic = link->service->characteristic(uuid(robotInfoCharacteristic + i));
            const auto p = characteristic.properties();
            layout[i] = {characteristic.isValid(), p.testFlag(C::Read), p.testFlag(C::Write),
                p.testFlag(C::WriteNoResponse), p.testFlag(C::Notify), characteristic.descriptor(cccd).isValid()};
        }
        if (const auto error = validateBleGattLayout(layout); !error.empty()) {
            fail(link, QString::fromStdString(error));
            return;
        }
        link->resultCccd = link->service->characteristic(uuid(commandResultCharacteristic)).descriptor(cccd);
        link->telemetryCccd = link->service->characteristic(uuid(telemetryCharacteristic)).descriptor(cccd);
        if (!link->resultCccd.isValid() || !link->telemetryCccd.isValid()) { fail(link, "Missing notification CCCD"); return; }
        link->readingInfo = true;
        emit phaseChanged(link->token, BlePhase::ReadingIdentity);
        link->service->readCharacteristic(link->service->characteristic(uuid(robotInfoCharacteristic)));
    }
    std::unique_ptr<QBluetoothDeviceDiscoveryAgent> scanner_;
    std::uint64_t scanToken_{};
    std::size_t maximumDevices_{};
    std::map<std::string, QBluetoothDeviceInfo> devices_;
    std::map<std::uint64_t, Link*> links_; // Owned QObject contexts, explicitly released per token.
};
} // namespace
std::unique_ptr<IBleBackend> makeQtBleBackend() { return std::make_unique<QtBleBackend>(); }
QByteArray boundBleNotificationCopy(const QByteArray& bytes) { return bytes.left(ble_gatt::notificationCopyLimit); }
} // namespace micro_robot
