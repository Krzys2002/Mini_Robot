// CodeCell C6 production runtime (#158) with persistent identity and validated
// configuration (#159). Boots with the drive disabled into the local safety/actuator
// authority and its two periodic tasks; the runtime becomes ready only with a valid
// provisioned record. There is no Wi-Fi, access point, HTTP server, browser control or
// onboard target/distance execution in this image (enforced after linking by
// scripts/production_image_policy.py). Motion commands can only enter through
// FirmwareRuntime::commands(), and only the v1 BLE peripheral (#163) calls it: it starts only
// with a validated identity, accepts one central, and is ready only while both CCCDs are
// enabled (restricted-lab v1 profile, #157; not authenticated). The drive is the bounded actuator contract
// (#160): it wakes the H-bridges only after a valid provisioned profile, and no powered
// motor acceptance exists yet (#238).
// Independent enforcement (#162): the safety supervisor is the only task on the 50 ms task
// watchdog. A watchdog expiry first sleeps both bridges from the ISR, then panics and resets;
// esp_restart() runs the runtime's shutdown first. Every boot starts Idle with outputs off
// and nothing restored; the boot and status lines report the reset reason.
// Telemetry (#164): a priority-5 telemetry task ticks every 5 ms and the BLE host decides
// and queues Telemetry frames on the shared robot counter. Battery and measured velocity are
// reported as unknown: no battery conversion is validated and no velocity sensor exists.
// USB serial carries status output and the maintenance console only, never motion.
#include <Arduino.h>
#include <esp_attr.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <chrono>
#include <cstddef>
#include <optional>
#include <string_view>
#include "EspProvisioningAdapters.hpp"
#include "CodeCellMotorDriver.hpp"
#include "EspRuntimeAdapters.hpp"
#include "NimBlePeripheral.hpp"
#include "ble/PeripheralController.hpp"
#include "drive/BoundedDriveOutput.hpp"
#include "protocol/V1Codec.hpp"
#include "provisioning/MaintenanceConsole.hpp"
#include "provisioning/ProvisionedConfiguration.hpp"
#include "runtime/FirmwareRuntime.hpp"
#include "telemetry/BatteryMonitor.hpp"
#include "telemetry/TelemetryInputs.hpp"

namespace {
using namespace codecell;

// Matches `monitor_speed` in platformio.ini.
constexpr unsigned long kSerialBaudRate = 115200;
// Status lines from loopTask (priority 1, the B13 log drain), never from timing paths.
// About 160 B every 5 s, well inside the 2048 B/s budget.
constexpr std::chrono::milliseconds kStatusReportPeriod{5000};
constexpr std::chrono::milliseconds kReporterPollPeriod{100};
// A BLE line (about 330 B) every status period and on a link change, at most once per
// second: at most 330 B/s, inside the 2048 B/s budget even while a central toggles CCCDs.
constexpr std::chrono::milliseconds kBleReportMinSpacing{1000};

esp::EspFirmwareClock firmwareClock;
esp::FreeRtosPlatform platform;
esp::CodeCellMotorDriver motors;
drive::BoundedDriveOutput driveOutput{motors};
bool motorsAttached{false};
esp::NvsRecordStorage recordStorage;
provisioning::ConfigStore configStore{recordStorage};
provisioning::ProvisionedConfiguration configuration{configStore};
runtime::FirmwareRuntime firmwareRuntime{{firmwareClock, platform, driveOutput, configuration, nullptr}};

class RuntimeStatusView final : public protocol::v1::IRuntimeStatusView {
public:
    runtime::RuntimeStatus status() const noexcept override { return firmwareRuntime.status(); }
};
RuntimeStatusView runtimeStatusView;

constexpr const char* kTelemetryTaskName = "mr_telemetry";

// ---- Telemetry inputs (#164) ----
// No battery sensor is wired and no evidence is approved: the CodeCell API conversion and the
// pack's thresholds are unverified (#154, #238 NOT RUN). Telemetry therefore sends 0xFFFF and
// never sets batteryLow; there is no low-battery protection in this image.
telemetry::BatteryMonitor battery{std::nullopt, nullptr};
telemetry::RobotTelemetryInputs telemetryInputs{battery};

// v1 BLE peripheral (#163). The controller is constructed in setup() only with a validated
// identity (static storage, no heap); every callback runs in nimble_host.
esp::NimBlePeripheral blePeripheral;
std::optional<ble::PeripheralController> bleController;
// Session security (#157 review): the explicitly selected restricted-lab pass-through. It
// permits everything and provides no authentication or encryption (ble/SessionSecurity.hpp).
ble::RestrictedLabPassThroughSecurity bleSecurity;
std::uint32_t warnedBypassedSessions{0};

enum class BleStart : std::uint8_t { Started, Unprovisioned, RobotInfoUnencodable, StackFailed, SecurityRefused };
BleStart bleStart{BleStart::Unprovisioned};

const char* toString(BleStart start)
{
    switch (start) {
    case BleStart::Started: return "started";
    case BleStart::Unprovisioned: return "unprovisioned";
    case BleStart::RobotInfoUnencodable: return "robot_info_unencodable";
    case BleStart::StackFailed: return "stack_failed";
    case BleStart::SecurityRefused: return "security_refused";
    }
    return "unknown";
}

// A robot without a validated record has no identity to publish and does not advertise
// (#159). A runtime that faulted for another reason still advertises: its authority refuses
// motion, and the central sees RobotFault.
BleStart startBle()
{
    const auto info = configuration.robotInfo();
    if (!info) return BleStart::Unprovisioned;
    const auto record = protocol::v1::encodeRobotInfo(*info, ble::kRobotInfoFirmwareVersion);
    if (!record) return BleStart::RobotInfoUnencodable;
    bleController.emplace(firmwareRuntime.commands(), runtimeStatusView, protocol::v1::commandLimits(*info),
                          blePeripheral, firmwareClock, telemetryInputs,
                          std::chrono::milliseconds{info->telemetryPeriodMilliseconds}, bleSecurity,
                          ble::kSecurityMode);
    if (!blePeripheral.begin(info->robotId, *record, *bleController)) return BleStart::StackFailed;
    bleController->start();
    if (bleController->state() == ble::LinkState::SecurityRefused) return BleStart::SecurityRefused;
    return BleStart::Started;
}

// The telemetry task's step (priority 5, every 5 ms): sample the battery port once per
// telemetry period (no sensor is wired, so nothing is read), then ask the host to evaluate
// the schedule. It never touches the session or the drive.
class TelemetryTick final : public runtime::IPeriodicStep {
public:
    void step() noexcept override
    {
        const auto now = firmwareClock.nowMicroseconds();
        if (battery.hasSensor() && (!lastSample_ || now - *lastSample_ >= runtime::Instant{timing::kTelemetryPeriod})) {
            battery.sample(now);
            lastSample_ = now;
        }
        blePeripheral.requestTelemetryTick();
    }

private:
    std::optional<runtime::Instant> lastSample_{};
};
TelemetryTick telemetryTick;
bool telemetryTaskRunning{false};

// If the task cannot start, no telemetry is published: the boot line says so, and the
// desktop's silence rule (3 periods) reports the link as stale. Motion safety never depends
// on telemetry (the watchdog, E-stop and faults are local).
bool startTelemetryTask()
{
    if (bleStart != BleStart::Started) return false;
    const runtime::TaskSpec spec{kTelemetryTaskName, timing::kTelemetryPriority, runtime::kDefaultTaskStackBytes,
                                 timing::kTelemetryTickPeriod, false};
    return platform.startPeriodicTask(spec, telemetryTick);
}

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
ble::LinkStatus lastBle{};
std::chrono::milliseconds lastBleReport{};
bool bleReported{false};

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

// One line with the link state, the negotiated values and failures, and the BLE counters.
void reportBle(const ble::LinkStatus& link)
{
    const auto c = bleController->diagnostics();
    char negotiated[64] = "none";
    if (link.negotiated)
        snprintf(negotiated, sizeof negotiated, "interval_us=%ld latency=%u timeout_ms=%ld in_request=%d",
                 static_cast<long>(ble::connectionInterval(*link.negotiated).count()),
                 static_cast<unsigned>(link.negotiated->latency),
                 static_cast<long>(ble::supervisionTimeout(*link.negotiated).count()),
                 ble::withinRequest(*link.negotiated, ble::kLinkParameterRequest) ? 1 : 0);
    char disconnect[48] = "none";
    if (link.lastDisconnectReason)
        snprintf(disconnect, sizeof disconnect, "%d since_write_ms=%ld", *link.lastDisconnectReason,
                 link.lastWriteBeforeDisconnect ? static_cast<long>(link.lastWriteBeforeDisconnect->count()) : -1L);
    using C = ble::BleCounter;
    Serial.printf("BLE state=%s cccd_result=%d cccd_telemetry=%d mtu=%u link_request=%s negotiated=%s "
                  "last_disconnect=%s connections=%lu rejected_centrals=%lu sessions=%lu results=%lu "
                  "result_failures=%lu unanswered=%lu estop_writes=%lu foreign=%lu early_cccd=%lu adv_failures=%lu "
                  "terminate_failures=%lu security=%s security_sessions=%lu security_refusals=%lu\n",
                  ble::toString(link.state), link.commandResultNotify ? 1 : 0, link.telemetryNotify ? 1 : 0,
                  static_cast<unsigned>(link.mtu), ble::toString(link.linkRequest), negotiated, disconnect,
                  static_cast<unsigned long>(c[C::Connections]), static_cast<unsigned long>(c[C::RejectedCentrals]),
                  static_cast<unsigned long>(c[C::SessionsReady]), static_cast<unsigned long>(c[C::ResultsSent]),
                  static_cast<unsigned long>(c[C::ResultNotifyFailures]),
                  static_cast<unsigned long>(c[C::UnansweredCommandWrites]),
                  static_cast<unsigned long>(c[C::EmergencyStopWrites]), static_cast<unsigned long>(c[C::ForeignEvents]),
                  static_cast<unsigned long>(c[C::EarlySubscriptions]),
                  static_cast<unsigned long>(c[C::AdvertisingFailures]),
                  static_cast<unsigned long>(c[C::TerminateFailures]),
                  !link.securitySession ? "none" : link.securityBypassed ? "bypassed" : "established",
                  static_cast<unsigned long>(c[C::SecuritySessions]), static_cast<unsigned long>(c[C::SecurityRefusals]));
}

bool changed(const ble::LinkStatus& a, const ble::LinkStatus& b)
{
    const auto interval = [](const ble::LinkStatus& s) { return s.negotiated ? s.negotiated->intervalUnits : 0; };
    return a.state != b.state || a.commandResultNotify != b.commandResultNotify || a.telemetryNotify != b.telemetryNotify
        || a.mtu != b.mtu || a.linkRequest != b.linkRequest || interval(a) != interval(b)
        || a.lastDisconnectReason != b.lastDisconnectReason;
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
    bleStart = startBle();
    telemetryTaskRunning = startTelemetryTask();
    Serial.begin(kSerialBaudRate);
    Serial.println("CodeCell C6 production runtime (#158-#164): v1 BLE peripheral and telemetry, restricted-lab profile "
                   "(unauthenticated); bounded drive from the provisioned profile, powered checks blocked (#238); "
                   "type help for maintenance");
    Serial.printf("BOOT reset_reason=%s shutdown_handler=%s\n", resetReason,
                  firmwareRuntime.shutdownHookRegistered() ? "registered" : "failed");
    const auto identity = configuration.robotInfo();
    const auto name = provisioning::advertisedName(identity ? identity->robotId : 0);
    Serial.printf("BLE start=%s profile=%s accepted_centrals=1 stack_max_connections=%d name=%s\n", toString(bleStart),
                  ble::toString(ble::kSecurityProfile), CONFIG_BT_NIMBLE_MAX_CONNECTIONS,
                  bleStart == BleStart::Started ? name.data() : "none");
    Serial.printf("SECURITY mode=%s policy=%s authenticated=no encrypted=no %s\n", ble::toString(ble::kSecurityMode),
                  bleSecurity.profile(), ble::kRestrictedLabSecurityWarning);
    Serial.printf("TELEMETRY start=%s tick_ms=%ld period_ms=%u battery=unknown battery_low=unsupported velocity=unknown\n",
                  telemetryTaskRunning ? "started" : (bleStart == BleStart::Started ? "failed" : "no_ble"),
                  static_cast<long>(timing::kTelemetryTickPeriod.count()),
                  identity ? static_cast<unsigned>(identity->telemetryPeriodMilliseconds) : 0U);
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
    if (bleController) {
        // One warning per new pass-through session, printed here and never from the BLE callback.
        const auto bypassed = bleController->diagnostics()[ble::BleCounter::BypassedSecuritySessions];
        if (bypassed != warnedBypassedSessions) {
            warnedBypassedSessions = bypassed;
            Serial.printf("SECURITY session=%lu %s\n", static_cast<unsigned long>(bypassed),
                          ble::kRestrictedLabSecurityWarning);
        }
        const auto link = bleController->status();
        const bool due = !bleReported || now - lastBleReport >= kStatusReportPeriod;
        if (due || (changed(link, lastBle) && now - lastBleReport >= kBleReportMinSpacing)) {
            reportBle(link);
            lastBle = link;
            lastBleReport = now;
            bleReported = true;
        }
    }
    delay(static_cast<std::uint32_t>(kReporterPollPeriod.count()));
}
