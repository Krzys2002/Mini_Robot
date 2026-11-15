#pragma once
// NimBLE-Arduino 2.3.6 adapter of the v1 GATT peripheral (#163). Together with main.cpp this
// is the only production code that names NimBLE; every decision is made by the portable
// ble::PeripheralController, which this adapter feeds from the BLE host callbacks
// (nimble_host, priority 21). Nothing here logs, blocks or allocates after begin().
//
// Connection limit: the pinned Arduino-ESP32 3.3.7 libraries are prebuilt with an sdkconfig.h
// that fixes CONFIG_BT_NIMBLE_MAX_CONNECTIONS at 3 (and every NimBLE role on); NimBLE-Arduino
// takes its configuration from that header, so a -D build flag cannot lower it (it is
// redefined). One central per robot (#157 F1) is therefore enforced by the controller:
// advertising stops when the owner connects and any other connection is terminated unapplied.
#include "ble/GattProfile.hpp"
#include "ble/PeripheralController.hpp"
#include "protocol/V1Codec.hpp"
#include "provisioning/RobotIdentity.hpp"
#include <NimBLEDevice.h>
#include <array>
#include <cstddef>
#include <cstdint>
#include <new>
#include <span>

static_assert(CONFIG_BT_NIMBLE_MAX_CONNECTIONS >= 1, "the peripheral needs at least the owner's connection");

namespace codecell::esp {

// A written characteristic whose writes go straight from the host's write buffer to the
// controller. The base NimBLECharacteristic::writeEvent first copies the value into a heap
// buffer that grows on demand (an allocation on the callback path, against #156 B15) and only
// then calls onWrite(); this override skips that copy, so an Emergency Stop write reaches the
// latch before anything else touches it. NimBLEServer calls writeEvent() for every write
// that fits the characteristic's maximum length, which is the ATT maximum here.
class DirectWriteCharacteristic final : public NimBLECharacteristic {
public:
    using Handler = void (ble::PeripheralController::*)(ble::ConnectionHandle, std::span<const std::uint8_t>);

    DirectWriteCharacteristic(const ble::CharacteristicSpec& spec, std::uint16_t properties, NimBLEService& service,
                              ble::PeripheralController& controller, Handler handler)
        : NimBLECharacteristic(spec.uuid, properties, spec.maxValueLength, &service),
          controller_(controller),
          handler_(handler)
    {
    }

private:
    void writeEvent(const std::uint8_t* value, std::uint16_t length, NimBLEConnInfo& connection) override
    {
        (controller_.*handler_)(connection.getConnHandle(), std::span<const std::uint8_t>{value, length});
    }

    ble::PeripheralController& controller_;
    Handler handler_;
};

class NimBlePeripheral final : public ble::IPeripheralLink,
                               private NimBLEServerCallbacks,
                               private NimBLECharacteristicCallbacks {
public:
    // Initializes the host, registers the Robot Control service and sets the advertising data.
    // Runs once from setup(); all BLE heap is allocated here (B15). It does not advertise:
    // the controller starts that. False when any step failed; nothing is then advertised.
    [[nodiscard]] bool begin(provisioning::RobotId robotId, const protocol::v1::RobotInfoBytes& robotInfo,
                             ble::PeripheralController& controller) noexcept
    {
        if (server_) return false;
        controller_ = &controller;
        const auto name = provisioning::advertisedName(robotId);
        if (!NimBLEDevice::init(name.data())) return false;
        // F6: stated explicitly rather than inherited. No bonding, so no bond keys reach flash
        // (no NVS writes while motion is possible, #162), no MITM, no Secure Connections,
        // NoInputNoOutput. A central may still obtain unauthenticated Just Works encryption;
        // nothing depends on it.
        NimBLEDevice::setSecurityAuth(false, false, false);
        NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);

        server_ = NimBLEDevice::createServer();
        if (!server_) return false;
        server_->setCallbacks(this, false);
        server_->advertiseOnDisconnect(false);  // The controller restarts it after the session end.

        NimBLEService* service = server_->createService(ble::kServiceUuid);
        if (!service) return false;
        using C = ble::Characteristic;
        auto* info = createPlain(*service, C::RobotInfo);
        command_ = createWritten(*service, C::Command, &ble::PeripheralController::onCommandWrite);
        estop_ = createWritten(*service, C::EmergencyStop, &ble::PeripheralController::onEmergencyStopWrite);
        telemetry_ = createPlain(*service, C::Telemetry);
        result_ = createPlain(*service, C::CommandResult);
        if (!info || !command_ || !estop_ || !telemetry_ || !result_) return false;
        info->setValue(robotInfo.data(), robotInfo.size());
        // Telemetry stays empty until telemetry publication (#164) writes a frame: no frame is
        // invented here. CommandResult values are sent as notifications only.
        telemetry_->setCallbacks(this);
        result_->setCallbacks(this);
        if (!service->start()) return false;
        server_->start();  // Registers the GATT database now rather than at the first advertising start.

        const auto advertisingData = ble::advertisingData();
        const auto scan = ble::scanResponse(robotId);
        NimBLEAdvertisementData advertisement;
        NimBLEAdvertisementData response;
        if (!advertisement.addData(advertisingData.data(), advertisingData.size())
            || !response.addData(scan.view().data(), scan.view().size()))
            return false;
        advertising_ = NimBLEDevice::getAdvertising();
        if (!advertising_) return false;
        advertising_->enableScanResponse(true);
        return advertising_->setAdvertisementData(advertisement) && advertising_->setScanResponseData(response);
    }

    // ---- ble::IPeripheralLink ----
    bool notifyCommandResult(ble::ConnectionHandle connection, std::span<const std::uint8_t> frame) noexcept override
    {
        return result_ && result_->notify(frame.data(), frame.size(), connection);
    }
    bool terminate(ble::ConnectionHandle connection) noexcept override
    {
        return server_ && server_->disconnect(connection);
    }
    bool startAdvertising() noexcept override { return advertising_ && advertising_->start(); }
    bool stopAdvertising() noexcept override { return advertising_ && advertising_->stop(); }
    std::uint16_t attMtu(ble::ConnectionHandle connection) const noexcept override
    {
        return server_ ? server_->getPeerMTU(connection) : 0;
    }
    bool requestLinkParameters(ble::ConnectionHandle connection, const ble::LinkParameterRequest& request) noexcept override
    {
        // Called directly so that a refusal is visible (NimBLEServer::updateConnParams only logs it).
        ble_gap_upd_params params{};
        params.itvl_min = request.minIntervalUnits;
        params.itvl_max = request.maxIntervalUnits;
        params.latency = request.latency;
        params.supervision_timeout = request.supervisionTimeoutUnits;
        params.min_ce_len = BLE_GAP_INITIAL_CONN_MIN_CE_LEN;
        params.max_ce_len = BLE_GAP_INITIAL_CONN_MAX_CE_LEN;
        return ble_gap_update_params(connection, &params) == 0;
    }

private:
    NimBLECharacteristic* createPlain(NimBLEService& service, ble::Characteristic id) noexcept
    {
        const auto& spec = ble::specOf(id);
        return service.createCharacteristic(spec.uuid, properties(spec), spec.maxValueLength);
    }
    DirectWriteCharacteristic* createWritten(NimBLEService& service, ble::Characteristic id,
                                             DirectWriteCharacteristic::Handler handler) noexcept
    {
        const auto& spec = ble::specOf(id);
        auto* characteristic =
            new (std::nothrow) DirectWriteCharacteristic(spec, properties(spec), service, *controller_, handler);
        if (characteristic) service.addCharacteristic(characteristic);
        return characteristic;
    }
    static std::uint16_t properties(const ble::CharacteristicSpec& spec) noexcept
    {
        std::uint16_t p = 0;
        if (spec.read) p |= NIMBLE_PROPERTY::READ;
        if (spec.write) p |= NIMBLE_PROPERTY::WRITE;
        if (spec.writeWithoutResponse) p |= NIMBLE_PROPERTY::WRITE_NR;
        if (spec.notify) p |= NIMBLE_PROPERTY::NOTIFY;  // NimBLE adds the CCCD (0x2902).
        return p;
    }

    // ---- NimBLEServerCallbacks ----
    void onConnect(NimBLEServer*, NimBLEConnInfo& connection) override
    {
        controller_->onConnected(connection.getConnHandle());
    }
    void onDisconnect(NimBLEServer*, NimBLEConnInfo& connection, int reason) override
    {
        controller_->onDisconnected(connection.getConnHandle(), reason);
    }
    void onConnParamsUpdate(NimBLEConnInfo& connection) override
    {
        controller_->onLinkParametersUpdated(
            connection.getConnHandle(),
            {connection.getConnInterval(), connection.getConnLatency(), connection.getConnTimeout()});
    }
    void onMTUChange(std::uint16_t mtu, NimBLEConnInfo& connection) override
    {
        controller_->onMtuChanged(connection.getConnHandle(), mtu);
    }

    // ---- NimBLECharacteristicCallbacks (Telemetry and Command Result CCCDs) ----
    void onSubscribe(NimBLECharacteristic* characteristic, NimBLEConnInfo& connection, std::uint16_t value) override
    {
        constexpr std::uint16_t kNotificationBit = 0x0001;  // Bit 1 (indications) is not part of v1.
        if (characteristic != result_ && characteristic != telemetry_) return;
        const auto id = characteristic == result_ ? ble::Characteristic::CommandResult : ble::Characteristic::Telemetry;
        controller_->onSubscriptionChanged(connection.getConnHandle(), id, (value & kNotificationBit) != 0);
    }

    ble::PeripheralController* controller_{nullptr};
    NimBLEServer* server_{nullptr};
    NimBLEAdvertising* advertising_{nullptr};
    DirectWriteCharacteristic* command_{nullptr};
    DirectWriteCharacteristic* estop_{nullptr};
    NimBLECharacteristic* telemetry_{nullptr};
    NimBLECharacteristic* result_{nullptr};
};
} // namespace codecell::esp
