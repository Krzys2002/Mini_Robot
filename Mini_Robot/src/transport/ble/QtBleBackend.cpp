#include "transport/ble/QtBleBackend.hpp"

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
QBluetoothUuid uuid(unsigned suffix)
{
    return QBluetoothUuid{QStringLiteral("4d52%1-1f2a-4032-8415-63a5fef8a0dc")
        .arg(suffix, 4, 16, QLatin1Char('0'))};
}
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
            [this, link](QLowEnergyController::Error) { fail(link, link->controller->errorString()); });
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
            if (!link->controller->services().contains(uuid(1))) { fail(link, "Robot Control service is missing"); return; }
            link->service = link->controller->createServiceObject(uuid(1), link);
            if (!link->service) { fail(link, "Cannot create Robot Control service"); return; }
            QObject::connect(link->service, &QLowEnergyService::errorOccurred, link,
                [this, link](QLowEnergyService::ServiceError) { fail(link, "BLE GATT service operation failed"); });
            QObject::connect(link->service, &QLowEnergyService::stateChanged, link,
                [this, link](QLowEnergyService::ServiceState state) {
                    if (!usable(link)) return;
                    if (state == QLowEnergyService::RemoteServiceDiscovered) inspectService(link);
                    else if (state == QLowEnergyService::InvalidService) fail(link, "Robot Control service became invalid");
                });
            QObject::connect(link->service, &QLowEnergyService::characteristicRead, link,
                [this, link](const QLowEnergyCharacteristic& characteristic, const QByteArray& value) {
                    if (!usable(link) || !link->readingInfo || characteristic.uuid() != uuid(2)) return;
                    link->readingInfo = false;
                    const auto result = validateBleRobotInfo(
                        {reinterpret_cast<const std::uint8_t*>(value.constData()), static_cast<std::size_t>(value.size())},
                        link->expected);
                    if (!result.info) {
                        fail(link, QString::fromStdString(result.error), result.unsupportedVersion
                            ? TransportErrorCode::UnsupportedVersion : TransportErrorCode::BackendFailure);
                        return;
                    }
                    emit phaseChanged(link->token, BlePhase::EnablingNotifications);
                    link->subscription = 1;
                    link->service->writeDescriptor(link->resultCccd, QByteArray::fromHex("0100"));
                });
            QObject::connect(link->service, &QLowEnergyService::descriptorWritten, link,
                [this, link](const QLowEnergyDescriptor& descriptor, const QByteArray& value) {
                    if (!usable(link)) return;
                    if ((link->subscription == 1 && descriptor != link->resultCccd)
                        || (link->subscription == 2 && descriptor != link->telemetryCccd)
                        || link->subscription == 0) return;
                    if (value != QByteArray::fromHex("0100")) { fail(link, "BLE notification subscription was refused"); return; }
                    if (link->subscription == 1) {
                        link->subscription = 2;
                        link->service->writeDescriptor(link->telemetryCccd, QByteArray::fromHex("0100"));
                    } else {
                        link->subscription = 0;
                        link->established = true;
                        emit ready(link->token);
                    }
                });
            emit phaseChanged(link->token, BlePhase::DiscoveringCharacteristics);
            link->service->discoverDetails(QLowEnergyService::SkipValueDiscovery);
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
    struct Link : QObject {
        Link(QObject* parent, std::uint64_t value, RobotId id) : QObject(parent), token(value), expected(id) {}
        std::uint64_t token;
        RobotId expected;
        QLowEnergyController* controller{}; // QObject children of this link.
        QLowEnergyService* service{};
        QLowEnergyDescriptor resultCccd, telemetryCccd;
        bool active{true}, closing{}, failed{}, readingInfo{}, established{};
        int subscription{};
    };
    static bool usable(const Link* link) { return link->active && !link->closing && !link->failed; }
    void fail(Link* link, const QString& message, TransportErrorCode code = TransportErrorCode::BackendFailure)
    {
        if (!link->active || link->failed) return;
        link->failed = true;
        emit failed(link->token, code, message);
    }
    void beginScan(std::uint64_t token)
    {
        if (scanToken_ != token) return;
        scanner_ = std::make_unique<QBluetoothDeviceDiscoveryAgent>();
        // Adapter owns the actual timeout, including permission and cancellation.
        scanner_->setLowEnergyDiscoveryTimeout(0);
        const auto discovered = [this, token](const QBluetoothDeviceInfo& info) {
            if (scanToken_ != token || !(info.coreConfigurations() & QBluetoothDeviceInfo::LowEnergyCoreConfiguration)
                || !info.serviceUuids().contains(uuid(1))) return;
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
        if (link->readingInfo || link->subscription || link->established) return;
        using C = QLowEnergyCharacteristic;
        const QBluetoothUuid cccd{QBluetoothUuid::DescriptorType::ClientCharacteristicConfiguration};
        std::array<BleCharacteristicCapabilities, 5> layout{};
        for (unsigned i = 0; i < layout.size(); ++i) {
            const auto characteristic = link->service->characteristic(uuid(i + 2));
            const auto p = characteristic.properties();
            layout[i] = {characteristic.isValid(), p.testFlag(C::Read), p.testFlag(C::Write),
                p.testFlag(C::WriteNoResponse), p.testFlag(C::Notify), characteristic.descriptor(cccd).isValid()};
        }
        if (const auto error = validateBleGattLayout(layout); !error.empty()) {
            fail(link, QString::fromStdString(error));
            return;
        }
        link->resultCccd = link->service->characteristic(uuid(6)).descriptor(cccd);
        link->telemetryCccd = link->service->characteristic(uuid(5)).descriptor(cccd);
        if (!link->resultCccd.isValid() || !link->telemetryCccd.isValid()) { fail(link, "Missing notification CCCD"); return; }
        link->readingInfo = true;
        emit phaseChanged(link->token, BlePhase::ReadingIdentity);
        link->service->readCharacteristic(link->service->characteristic(uuid(2)));
    }
    std::unique_ptr<QBluetoothDeviceDiscoveryAgent> scanner_;
    std::uint64_t scanToken_{};
    std::size_t maximumDevices_{};
    std::map<std::string, QBluetoothDeviceInfo> devices_;
    std::map<std::uint64_t, Link*> links_; // Owned QObject contexts, explicitly released per token.
};
} // namespace
std::unique_ptr<IBleBackend> makeQtBleBackend() { return std::make_unique<QtBleBackend>(); }
} // namespace micro_robot
