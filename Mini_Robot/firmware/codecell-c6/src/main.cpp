// CodeCell C6 production runtime (#158). Boots with the drive disabled into the local
// safety/actuator authority and its two periodic tasks. There is no Wi-Fi, access point,
// HTTP server, browser control or onboard target/distance execution in this image
// (enforced after linking by scripts/production_image_policy.py). Motion commands can
// only enter through FirmwareRuntime::commands(), which the BLE peripheral (#163) will
// call; until then nothing in this image commands motion, and the drive adapter refuses
// to enable (#160).
#include <Arduino.h>
#include <chrono>
#include "EspRuntimeAdapters.hpp"
#include "runtime/FirmwareRuntime.hpp"

namespace {
using namespace codecell;

// Matches `monitor_speed` in platformio.ini.
constexpr unsigned long kSerialBaudRate = 115200;
// Status lines from loopTask (priority 1, the B13 log drain), never from timing paths.
// About 120 B every 5 s, well inside the 2048 B/s budget.
constexpr std::chrono::milliseconds kStatusReportPeriod{5000};
constexpr std::chrono::milliseconds kReporterPollPeriod{100};

esp::EspFirmwareClock firmwareClock;
esp::FreeRtosPlatform platform;
esp::InhibitedDriveOutput drive;
esp::CompiledConfiguration configuration;
runtime::FirmwareRuntime firmwareRuntime{{firmwareClock, platform, drive, configuration, nullptr}};

runtime::StartResult startResult{runtime::StartResult::Faulted};
std::chrono::milliseconds lastReport{};
runtime::RuntimeStatus lastStatus{};
bool reported{false};

void reportStatus(const runtime::RuntimeStatus& status)
{
    const auto counters = firmwareRuntime.diagnostics();
    Serial.printf("RUNTIME uptime_ms=%lu start=%s ready=%d state=%s output=%s fault=%s accepted=%lu "
                  "rejected=%lu expired=%lu overwrites=%lu\n",
                  static_cast<unsigned long>(firmwareClock.now().count()),
                  startResult == runtime::StartResult::Running ? "running" : "faulted", status.ready ? 1 : 0,
                  runtime::toString(status.state), status.outputEnabled ? "enabled" : "disabled",
                  runtime::toString(status.fault),
                  static_cast<unsigned long>(counters[runtime::Counter::VelocityAccepted]),
                  static_cast<unsigned long>(counters[runtime::Counter::CommandRejected]),
                  static_cast<unsigned long>(counters[runtime::Counter::CommandExpired]),
                  static_cast<unsigned long>(counters[runtime::Counter::MailboxOverwrite]));
}

bool changed(const runtime::RuntimeStatus& a, const runtime::RuntimeStatus& b)
{
    return a.state != b.state || a.fault != b.fault || a.ready != b.ready || a.outputEnabled != b.outputEnabled;
}
} // namespace

void setup()
{
    // The runtime disables the drive before anything else; serial comes after it.
    startResult = firmwareRuntime.start();
    Serial.begin(kSerialBaudRate);
    Serial.println("CodeCell C6 production runtime (#158): BLE commands not yet available (#163); "
                   "drive inhibited (#160)");
    if (const auto error = firmwareRuntime.configError())
        Serial.printf("RUNTIME config_error=%u\n", static_cast<unsigned>(*error));
}

void loop()
{
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
