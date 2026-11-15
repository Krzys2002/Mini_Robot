// CodeCell C6 production runtime (#158) with persistent identity and validated
// configuration (#159). Boots with the drive disabled into the local safety/actuator
// authority and its two periodic tasks; the runtime becomes ready only with a valid
// provisioned record. There is no Wi-Fi, access point, HTTP server, browser control or
// onboard target/distance execution in this image (enforced after linking by
// scripts/production_image_policy.py). Motion commands can only enter through
// FirmwareRuntime::commands(), which the BLE peripheral (#163) will call; until then
// nothing in this image commands motion, and the drive adapter refuses to enable (#160).
// USB serial carries status output and the maintenance console only, never motion.
#include <Arduino.h>
#include <esp_system.h>
#include <chrono>
#include <cstddef>
#include <string_view>
#include "EspProvisioningAdapters.hpp"
#include "EspRuntimeAdapters.hpp"
#include "provisioning/MaintenanceConsole.hpp"
#include "provisioning/ProvisionedConfiguration.hpp"
#include "runtime/FirmwareRuntime.hpp"

namespace {
using namespace codecell;

// Matches `monitor_speed` in platformio.ini.
constexpr unsigned long kSerialBaudRate = 115200;
// Status lines from loopTask (priority 1, the B13 log drain), never from timing paths.
// About 160 B every 5 s, well inside the 2048 B/s budget.
constexpr std::chrono::milliseconds kStatusReportPeriod{5000};
constexpr std::chrono::milliseconds kReporterPollPeriod{100};

esp::EspFirmwareClock firmwareClock;
esp::FreeRtosPlatform platform;
esp::InhibitedDriveOutput drive;
esp::NvsRecordStorage recordStorage;
provisioning::ConfigStore configStore{recordStorage};
provisioning::ProvisionedConfiguration configuration{configStore};
runtime::FirmwareRuntime firmwareRuntime{{firmwareClock, platform, drive, configuration, nullptr}};

class SerialMaintenanceOutput final : public provisioning::IMaintenanceOutput {
public:
    void line(const char* text) noexcept override { Serial.println(text); }
};

class RuntimeMaintenanceTarget final : public provisioning::IMaintenanceTarget {
public:
    bool enterMaintenance() noexcept override { return firmwareRuntime.enterMaintenance(); }
    void restart() noexcept override
    {
        Serial.flush();
        esp_restart();
    }
};

SerialMaintenanceOutput maintenanceOutput;
RuntimeMaintenanceTarget maintenanceTarget;
provisioning::MaintenanceConsole console{configStore, maintenanceTarget, maintenanceOutput};

runtime::StartResult startResult{runtime::StartResult::Faulted};
std::chrono::milliseconds lastReport{};
runtime::RuntimeStatus lastStatus{};
bool reported{false};

void reportStatus(const runtime::RuntimeStatus& status)
{
    const auto counters = firmwareRuntime.diagnostics();
    const auto& boot = configuration.bootState();
    char robotId[16] = "none";
    if (const auto info = configuration.robotInfo())
        snprintf(robotId, sizeof robotId, "%lu", static_cast<unsigned long>(info->robotId));
    Serial.printf("RUNTIME uptime_ms=%lu start=%s ready=%d state=%s output=%s fault=%s maintenance=%d "
                  "provisioning=%s robot_id=%s accepted=%lu rejected=%lu expired=%lu overwrites=%lu\n",
                  static_cast<unsigned long>(firmwareClock.now().count()),
                  startResult == runtime::StartResult::Running ? "running" : "faulted", status.ready ? 1 : 0,
                  runtime::toString(status.state), status.outputEnabled ? "enabled" : "disabled",
                  runtime::toString(status.fault), status.maintenance ? 1 : 0, provisioning::toString(boot.state),
                  robotId, static_cast<unsigned long>(counters[runtime::Counter::VelocityAccepted]),
                  static_cast<unsigned long>(counters[runtime::Counter::CommandRejected]),
                  static_cast<unsigned long>(counters[runtime::Counter::CommandExpired]),
                  static_cast<unsigned long>(counters[runtime::Counter::MailboxOverwrite]));
}

bool changed(const runtime::RuntimeStatus& a, const runtime::RuntimeStatus& b)
{
    return a.state != b.state || a.fault != b.fault || a.ready != b.ready || a.outputEnabled != b.outputEnabled
        || a.maintenance != b.maintenance;
}

// Fixed-size, non-blocking line reader; an overlong line is discarded whole.
void pollMaintenanceInput()
{
    static char line[provisioning::kMaintenanceLineMax + 1];
    static std::size_t length = 0;
    static bool overflow = false;
    while (Serial.available() > 0) {
        const char c = static_cast<char>(Serial.read());
        if (c == '\r') continue;
        if (c != '\n') {
            if (length < provisioning::kMaintenanceLineMax) line[length++] = c;
            else overflow = true;
            continue;
        }
        if (overflow) Serial.println("ERR line too long");
        else console.handleLine(std::string_view{line, length});
        length = 0;
        overflow = false;
    }
}
} // namespace

void setup()
{
    // The runtime disables the drive before loading the configuration; serial comes after.
    startResult = firmwareRuntime.start();
    Serial.begin(kSerialBaudRate);
    Serial.println("CodeCell C6 production runtime (#158/#159): BLE commands not yet available (#163); "
                   "drive inhibited (#160); type help for maintenance");
    const auto& boot = configuration.bootState();
    Serial.printf("PROVISIONING state=%s reason=%s generation=%lu\n", provisioning::toString(boot.state),
                  provisioning::toString(boot.reason), static_cast<unsigned long>(boot.generation.value_or(0)));
    if (const auto error = firmwareRuntime.configError())
        Serial.printf("RUNTIME config_error=%u\n", static_cast<unsigned>(*error));
}

void loop()
{
    pollMaintenanceInput();
    const auto now = firmwareClock.now();
    const auto status = firmwareRuntime.status();
    if (!reported || changed(status, lastStatus) || now - lastReport >= kStatusReportPeriod) {
        reportStatus(status);
        lastStatus = status;
        lastReport = now;
        reported = true;
    }
    delay(static_cast<std::uint32_t>(kReporterPollPeriod.count()));
}
