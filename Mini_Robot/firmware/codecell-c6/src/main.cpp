// CodeCell C6 production runtime (#158) with persistent identity and validated
// configuration (#159). Boots with the drive disabled into the local safety/actuator
// authority and its two periodic tasks; the runtime becomes ready only with a valid
// provisioned record. There is no Wi-Fi, access point, HTTP server, browser control or
// onboard target/distance execution in this image (enforced after linking by
// scripts/production_image_policy.py). Motion commands can only enter through
// FirmwareRuntime::commands(), which the BLE peripheral (#163) will call; until then
// nothing in this image commands motion. The drive is the bounded actuator contract
// (#160): it wakes the H-bridges only after a valid provisioned profile, and no powered
// motor acceptance exists yet (#238).
// Independent enforcement (#162): the safety supervisor is the only task on the 50 ms task
// watchdog. A watchdog expiry first sleeps both bridges from the ISR, then panics and resets;
// esp_restart() runs the runtime's shutdown first. Every boot starts Idle with outputs off
// and nothing restored; the boot and status lines report the reset reason.
// USB serial carries status output and the maintenance console only, never motion.
#include <Arduino.h>
#include <esp_attr.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <chrono>
#include <cstddef>
#include <string_view>
#include "EspProvisioningAdapters.hpp"
#include "CodeCellMotorDriver.hpp"
#include "EspRuntimeAdapters.hpp"
#include "drive/BoundedDriveOutput.hpp"
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
esp::CodeCellMotorDriver motors;
drive::BoundedDriveOutput driveOutput{motors};
bool motorsAttached{false};
esp::NvsRecordStorage recordStorage;
provisioning::ConfigStore configStore{recordStorage};
provisioning::ProvisionedConfiguration configuration{configStore};
runtime::FirmwareRuntime firmwareRuntime{{firmwareClock, platform, driveOutput, configuration, nullptr}};

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
const char* resetReason{"unknown"};
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
    const auto duty = driveOutput.lastDuty();
    const char* output = status.stopRamp ? "stop_ramp" : (status.outputEnabled ? "enabled" : "disabled");
    Serial.printf("RUNTIME uptime_ms=%lu start=%s ready=%d state=%s output=%s duty_m1=%d duty_m2=%d fault=%s "
                  "maintenance=%d session=%d expired_flag=%d reset=%s provisioning=%s robot_id=%s accepted=%lu "
                  "rejected=%lu expired=%lu overwrites=%lu\n",
                  static_cast<unsigned long>(firmwareClock.now().count()),
                  startResult == runtime::StartResult::Running ? "running" : "faulted", status.ready ? 1 : 0,
                  runtime::toString(status.state), output, static_cast<int>(duty.m1), static_cast<int>(duty.m2),
                  runtime::toString(status.fault), status.maintenance ? 1 : 0, status.sessionReady ? 1 : 0,
                  status.commandExpired ? 1 : 0, resetReason, provisioning::toString(boot.state), robotId,
                  static_cast<unsigned long>(counters[runtime::Counter::VelocityAccepted]),
                  static_cast<unsigned long>(counters[runtime::Counter::CommandRejected]),
                  static_cast<unsigned long>(counters[runtime::Counter::CommandExpired]),
                  static_cast<unsigned long>(counters[runtime::Counter::MailboxOverwrite]));
}

bool changed(const runtime::RuntimeStatus& a, const runtime::RuntimeStatus& b)
{
    return a.state != b.state || a.fault != b.fault || a.ready != b.ready || a.outputEnabled != b.outputEnabled
        || a.maintenance != b.maintenance || a.stopRamp != b.stopRamp || a.sessionReady != b.sessionReady
        || a.commandExpired != b.commandExpired;
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

// Task-watchdog ISR hook (ESP-IDF weak symbol, called before the TWDT panic): the outputs are
// otherwise left as they were for the ~0.41 s panic dump and the reset (#156 B17). Sleeping
// the bridges is one register write; nothing else is safe in this context.
extern "C" void IRAM_ATTR esp_task_wdt_isr_user_handler(void) { esp::CodeCellMotorDriver::sleepBridgesFromIsr(); }

void setup()
{
    resetReason = esp::resetReasonName(esp_reset_reason());
    // Bridge inputs are attached low first (bridges still asleep); the runtime then registers
    // the pre-restart shutdown hook (a checked start step: without it the runtime faults and
    // never becomes ready), disables the drive, loads the configuration and wakes the
    // bridges only with a valid profile. A failed attach makes the boot disable fail, so the
    // runtime faults. Serial comes after.
    motorsAttached = motors.begin();
    startResult = firmwareRuntime.start();
    Serial.begin(kSerialBaudRate);
    Serial.println("CodeCell C6 production runtime (#158-#162): BLE commands not yet available (#163); "
                   "bounded drive from the provisioned profile, powered checks blocked (#238); "
                   "type help for maintenance");
    Serial.printf("BOOT reset_reason=%s shutdown_handler=%s\n", resetReason,
                  firmwareRuntime.shutdownHookRegistered() ? "registered" : "failed");
    if (!motorsAttached) Serial.println("DRIVE attach=failed");
    const auto& boot = configuration.bootState();
    Serial.printf("PROVISIONING state=%s reason=%s generation=%lu pending=%s\n", provisioning::toString(boot.state),
                  provisioning::toString(boot.reason), static_cast<unsigned long>(boot.generation.value_or(0)),
                  provisioning::toString(boot.pending));
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
