// Connection-only bench peripheral. No CodeCell, motor, GPIO or actuator APIs.
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include "FixtureData.hpp"

#include <array>
#include <atomic>
#include <cstring>

namespace {
constexpr const char* serviceUuid = "4d520001-1f2a-4032-8415-63a5fef8a0dc";
constexpr const char* infoUuid = "4d520002-1f2a-4032-8415-63a5fef8a0dc";
constexpr const char* commandUuid = "4d520003-1f2a-4032-8415-63a5fef8a0dc";
constexpr const char* estopUuid = "4d520004-1f2a-4032-8415-63a5fef8a0dc";
constexpr const char* telemetryUuid = "4d520005-1f2a-4032-8415-63a5fef8a0dc";
constexpr const char* resultUuid = "4d520006-1f2a-4032-8415-63a5fef8a0dc";
constexpr const char* profiles[] = {
    "normal", "wrong-id", "wrong-version", "missing-command", "bad-properties",
    "no-telemetry-notify", "slow-info", "wrong-service"
};
Preferences settings;
uint32_t robotId = 1;
String profile;
NimBLECharacteristic* telemetry = nullptr;
NimBLECharacteristic* result = nullptr;
std::atomic<bool> telemetrySubscribed{false};
std::atomic<bool> resultSubscribed{false};
uint16_t telemetrySequence = 0; // Arduino loop only.

void status() {
    Serial.printf("[%lu] fixture=0.1.0 id=%lu profile=%s subscriptions=result:%u telemetry:%u\n",
                  millis(), static_cast<unsigned long>(robotId), profile.c_str(),
                  unsigned(resultSubscribed.load()), unsigned(telemetrySubscribed.load()));
}
class ServerCallbacks final : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* server, NimBLEConnInfo& connection) override {
        Serial.printf("[%lu] link connected handle=%u\n", millis(), connection.getConnHandle());
        server->updateConnParams(connection.getConnHandle(), 6, 24, 0, 40);
    }
    void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int reason) override {
        resultSubscribed = false;
        telemetrySubscribed = false;
        Serial.printf("[%lu] link disconnected reason=%d; advertising resumes\n", millis(), reason);
    }
} serverCallbacks;
class CharacteristicCallbacks final : public NimBLECharacteristicCallbacks {
    void onRead(NimBLECharacteristic* characteristic, NimBLEConnInfo&) override {
        if (characteristic->getUUID() != NimBLEUUID(infoUuid)) return;
        Serial.printf("[%lu] Robot Info read profile=%s\n", millis(), profile.c_str());
        // Deliberately stalls this fixture's BLE host callback for cancellation testing.
        // Never use this profile in production; desktop phase timeout is 10 seconds.
        if (profile == "slow-info") delay(3000);
    }
    void onSubscribe(NimBLECharacteristic* characteristic, NimBLEConnInfo&, uint16_t value) override {
        if (characteristic == result) resultSubscribed = (value & 1) != 0;
        if (characteristic == telemetry) telemetrySubscribed = (value & 1) != 0;
        Serial.printf("[%lu] CCCD %s=%u ready=%u\n", millis(),
                      characteristic == result ? "result" : "telemetry", value,
                      unsigned(resultSubscribed.load() && telemetrySubscribed.load()));
    }
    void onWrite(NimBLECharacteristic* characteristic, NimBLEConnInfo&) override {
        Serial.printf("[%lu] WRITE IGNORED uuid=%s bytes=%u (connection-only fixture)\n",
                      millis(), characteristic->getUUID().toString().c_str(),
                      unsigned(characteristic->getValue().size()));
    }
} characteristicCallbacks;

void restart() {
    Serial.println("Saved; rebooting. Reconnect serial monitor if USB disappears.");
    Serial.flush();
    delay(100);
    ESP.restart();
}
void handleLine(const char* line) {
    if (strcmp(line, "status") == 0) { status(); return; }
    if (strncmp(line, "id ", 3) == 0) {
        uint32_t value = 0;
        if (ble_fixture::parseId(line + 3, value)) {
            if (settings.putUInt("id", value) != sizeof(uint32_t)) {
                Serial.println("ERROR: could not save id"); return;
            }
            restart(); return;
        }
    }
    if (strncmp(line, "profile ", 8) == 0) {
        for (const auto* name : profiles) {
            if (strcmp(line + 8, name) == 0) {
                if (settings.putString("profile", name) == 0) {
                    Serial.println("ERROR: could not save profile"); return;
                }
                restart(); return;
            }
        }
    }
    Serial.println("Commands (newline): status | id <uint32> | profile <name>");
    for (const auto* name : profiles) Serial.println(name);
}
} // namespace

void setup() {
    Serial.begin(115200); // Never wait for an attached USB host.
    if (!settings.begin("ble-fixture", false)) {
        Serial.println("ERROR: NVS unavailable; no BLE started");
        return;
    }
    robotId = settings.getUInt("id", 1);
    profile = settings.getString("profile", "normal");
    const String name = "MR-" + String(robotId);
    NimBLEDevice::init(name.c_str());
    auto* server = NimBLEDevice::createServer();
    server->setCallbacks(&serverCallbacks);
    server->advertiseOnDisconnect(true);
    const char* activeService = profile == "wrong-service"
        ? "4d5200ff-1f2a-4032-8415-63a5fef8a0dc" : serviceUuid;
    auto* service = server->createService(activeService);
    auto* info = service->createCharacteristic(infoUuid, NIMBLE_PROPERTY::READ);
    const auto record = ble_fixture::robotInfo(robotId, profile == "wrong-id", profile == "wrong-version");
    info->setValue(record.data(), record.size());
    info->setCallbacks(&characteristicCallbacks);
    if (profile != "missing-command") {
        auto* command = service->createCharacteristic(commandUuid,
            profile == "bad-properties" ? NIMBLE_PROPERTY::READ
                : NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
        command->setCallbacks(&characteristicCallbacks);
    }
    auto* estop = service->createCharacteristic(estopUuid,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    estop->setCallbacks(&characteristicCallbacks);
    telemetry = service->createCharacteristic(telemetryUuid,
        profile == "no-telemetry-notify" ? NIMBLE_PROPERTY::READ
            : NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    result = service->createCharacteristic(resultUuid, NIMBLE_PROPERTY::NOTIFY);
    telemetry->setCallbacks(&characteristicCallbacks);
    result->setCallbacks(&characteristicCallbacks);
    const auto initialTelemetry = ble_fixture::idleTelemetry(millis(), telemetrySequence++);
    telemetry->setValue(initialTelemetry.data(), initialTelemetry.size());
    // NimBLE supplies 0x2902 automatically for NOTIFY characteristics.
    service->start();
    NimBLEAdvertisementData advertisement;
    advertisement.setFlags(0x06);
    advertisement.addServiceUUID(activeService);
    NimBLEAdvertisementData scanResponse;
    scanResponse.setName(name.c_str());
    auto* advertising = NimBLEDevice::getAdvertising();
    advertising->setAdvertisementData(advertisement);
    advertising->setScanResponseData(scanResponse);
    advertising->enableScanResponse(true);
    advertising->start();
    status();
}

void loop() {
    // Fixed-size, nonblocking serial input; discard a whole overlong line.
    static char line[80];
    static size_t length = 0;
    static bool overflow = false;
    while (Serial.available()) {
        const char c = static_cast<char>(Serial.read());
        if (c == '\r') continue;
        if (c == '\n') {
            line[length] = '\0';
            if (!overflow) handleLine(line);
            else Serial.println("ERROR: command too long");
            length = 0;
            overflow = false;
        } else if (length + 1 < sizeof(line) && !overflow) line[length++] = c;
        else overflow = true;
    }
    static uint32_t lastTelemetry = 0;
    const uint32_t now = millis();
    if (telemetry && now - lastTelemetry >= 100) {
        lastTelemetry = now;
        const auto frame = ble_fixture::idleTelemetry(now, telemetrySequence++);
        telemetry->setValue(frame.data(), frame.size());
        if (resultSubscribed.load() && telemetrySubscribed.load()) telemetry->notify();
    }
    delay(1);
}
