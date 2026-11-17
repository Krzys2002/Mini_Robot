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
// USB serial carries status output and the maintenance console only, never motion. Every line
// goes through a bounded ring drained without blocking at <= 2048 B/s (#156 B12/B13).
#include <Arduino.h>
#include <esp_app_desc.h>
#include <esp_attr.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdarg>
#include <cstddef>
#include <cstdio>
#include <optional>
#include <string_view>
#include "EspProvisioningAdapters.hpp"
#include "CodeCellMotorDriver.hpp"
#include "EspRuntimeAdapters.hpp"
#include "NimBlePeripheral.hpp"
#include "ble/PeripheralController.hpp"
#include "diagnostics/LogRing.hpp"
#include "diagnostics/StatusReport.hpp"
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
// One report round (IDENTITY, RUNTIME, BLE, TELEMETRY, TASK lines, RESOURCES), one line per
// loop pass and only once the ring has drained, so the ring never holds more than one report
// line and a maintenance reply always finds room. About 2.3 kB per round every 5 s, well
// inside the 2048 B/s budget. RUNTIME also prints at once on a status change, BLE on a link
// change at most once per second.
constexpr std::chrono::milliseconds kStatusReportPeriod{5000};
constexpr std::chrono::milliseconds kReporterPollPeriod{100};
constexpr std::chrono::milliseconds kBleReportMinSpacing{1000};
// USB CDC write timeout. Not 0: in Arduino-ESP32 3.3.7 HWCDC::write() counts its retries down
// from this value, so 0 underflows and a write that makes no progress would spin for ~49 days
// (found by reading the driver while chasing the console stall fixed in SerialLogSink). With
// 1 ms a stalled write gives up after one tick, so a drain pass blocks loopTask for at most a
// few milliseconds; the drain only offers what availableForWrite() reported anyway.
constexpr std::uint32_t kSerialTxTimeoutMs = 1;
// Before a deliberate restart the ring is written out with a blocking write of at most this
// per call, so that the maintenance confirmation reaches the host.
constexpr std::uint32_t kRestartFlushTimeoutMs = 100;
constexpr const char* kTelemetryTaskName = "mr_telemetry";
// The ELF SHA-256 prefix that names the running image (matches firmware-provenance.json).
constexpr std::size_t kBuildIdHexDigits = 16;

esp::EspFirmwareClock firmwareClock;
esp::FreeRtosPlatform platform;
esp::CodeCellMotorDriver motors;
drive::BoundedDriveOutput driveOutput{motors};
bool motorsAttached{false};
esp::NvsRecordStorage recordStorage;
provisioning::ConfigStore configStore{recordStorage};
provisioning::ProvisionedConfiguration configuration{configStore};
// The telemetry task's step (priority 5, every 5 ms): sample the battery port once per
// telemetry period (no sensor is wired, so nothing is read), then ask the host to evaluate
// the schedule. It never touches the session or the drive. Defined below, once the BLE
// peripheral exists; until the peripheral is running a tick does nothing.
class TelemetryTick final : public runtime::IPeriodicStep {
public:
    void step() noexcept override;

private:
    std::optional<runtime::Instant> lastSample_{};
    std::uint32_t checks_{0};
};
TelemetryTick telemetryTick;

// The telemetry task is part of the runtime's checked start-up (#164 review): if it cannot
// start, the runtime faults (TelemetryTaskStartFailed) and never accepts motion. Nothing
// relies on the desktop noticing the missing telemetry.
runtime::FirmwareRuntime firmwareRuntime{{firmwareClock, platform, driveOutput, configuration, nullptr,
                                          runtime::PeriodicTask{{kTelemetryTaskName, timing::kTelemetryPriority,
                                                                 runtime::kDefaultTaskStackBytes,
                                                                 timing::kTelemetryTickPeriod, false},
                                                                &telemetryTick}}};

class RuntimeStatusView final : public protocol::v1::IRuntimeStatusView {
public:
    runtime::RuntimeStatus status() const noexcept override { return firmwareRuntime.status(); }
};
RuntimeStatusView runtimeStatusView;

// ---- Bounded log (loopTask only) ----
class SerialLogSink final : public diagnostics::ILogSink {
public:
    std::size_t writable() noexcept override
    {
        // Without a host the CDC driver discards what it is given; keep the lines in the ring
        // instead (bounded, newest dropped and counted), so a monitor attached later still
        // sees the boot lines.
        if (!Serial) return 0;
        int free = Serial.availableForWrite();
        if (free <= 0) {
            // HWCDC (Arduino-ESP32 3.3.7) disables its TX interrupt when the connection flag
            // drops with bytes queued, and only write() or flush() enable it again. A drain that
            // offers bytes only when there is room would then wait forever (seen on the bench:
            // the console went silent while everything else ran). flush() re-arms it and waits
            // only while bytes move, plus kSerialTxTimeoutMs; without a reading host it discards
            // the driver's own queue (at most its 256 B), never this ring.
            Serial.flush();
            free = Serial.availableForWrite();
        }
        return free > 0 ? static_cast<std::size_t>(free) : 0;
    }
    std::size_t write(std::span<const char> bytes) noexcept override
    {
        return Serial.write(reinterpret_cast<const std::uint8_t*>(bytes.data()), bytes.size());
    }
};
diagnostics::LogRing logRing;
diagnostics::ByteBudget logBudget{timing::kLogBytesPerSecondMax, diagnostics::kLogBurstBytes};
SerialLogSink logSink;

void logLine(std::string_view text) { (void)logRing.push(text); }
void logLine(const diagnostics::ReportLine& line) { logLine(line.view()); }
#if defined(__GNUC__)
__attribute__((format(printf, 1, 2)))
#endif
void logf(const char* format, ...)
{
    diagnostics::ReportLine line;
    std::va_list args;
    va_start(args, format);
    const int written = std::vsnprintf(line.text.data(), line.text.size(), format, args);
    va_end(args);
    if (written < 0) return;
    line.length = std::min(static_cast<std::size_t>(written), line.text.size() - 1);
    logLine(line);
}
void drainLog() { (void)diagnostics::drainLog(logRing, logBudget, logSink, firmwareClock.nowMicroseconds()); }

// Restart path only: write out what the ring holds, then the USB buffer.
void flushLogBlocking()
{
    Serial.setTxTimeoutMs(kRestartFlushTimeoutMs);
    for (int pass = 0; pass < 2 && !logRing.empty(); ++pass) {
        const auto run = logRing.readable();
        logRing.consume(Serial.write(reinterpret_cast<const std::uint8_t*>(run.data()), run.size()));
    }
    Serial.flush();
}

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
// Published once the controller runs, for the telemetry task (which starts before BLE does).
std::atomic<const ble::PeripheralController*> bleControllerRunning{nullptr};
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
    bleControllerRunning.store(&*bleController, std::memory_order_release);
    if (bleController->state() == ble::LinkState::SecurityRefused) return BleStart::SecurityRefused;
    return BleStart::Started;
}

void TelemetryTick::step() noexcept
{
    const auto now = firmwareClock.nowMicroseconds();
    if (battery.hasSensor() && (!lastSample_ || now - *lastSample_ >= runtime::Instant{timing::kTelemetryPeriod})) {
        battery.sample(now);
        lastSample_ = now;
    }
    // Every millisecond: wake the host when a frame could be due (the controller decides
    // exactly), and every 5 ms regardless. Ticks are dropped until the peripheral is running.
    constexpr std::uint32_t kChecksPerRoutinePost =
        static_cast<std::uint32_t>(timing::kTelemetryRoutinePostPeriod / timing::kTelemetryTickPeriod);
    const bool routine = checks_++ % kChecksPerRoutinePost == 0;
    if (!routine) {
        const auto* controller = bleControllerRunning.load(std::memory_order_acquire);
        if (!controller) return;
        const auto status = firmwareRuntime.status();
        if (!controller->telemetryMayBeDue(now, status.state, status.commandExpired)) return;
    }
    blePeripheral.requestTelemetryTick();
}

class SerialMaintenanceOutput final : public provisioning::IMaintenanceOutput {
public:
    void line(const char* text) noexcept override { logLine(text); }
};

class RuntimeMaintenanceTarget final : public provisioning::IMaintenanceTarget {
public:
    bool enterMaintenance() noexcept override { return firmwareRuntime.enterMaintenance(); }
    void restart() noexcept override
    {
        flushLogBlocking();
        esp_restart();
    }
};

SerialMaintenanceOutput maintenanceOutput;
RuntimeMaintenanceTarget maintenanceTarget;
provisioning::MaintenanceConsole console{configStore, maintenanceTarget, maintenanceOutput};

runtime::StartResult startResult{runtime::StartResult::Faulted};
const char* resetReason{"unknown"};
std::array<char, kBuildIdHexDigits + 1> buildId{};

std::uint32_t uptimeMilliseconds() { return telemetry::uptimeMilliseconds(firmwareClock.nowMicroseconds()); }
std::optional<std::uint32_t> robotId()
{
    if (const auto info = configuration.robotInfo()) return info->robotId;
    return std::nullopt;
}
std::uint32_t sessionNumber() { return bleController ? bleController->diagnostics()[ble::BleCounter::SessionsReady] : 0; }

// The running image's ELF SHA-256 prefix (written into the app descriptor by the image tool),
// or "unavailable" if the descriptor carries none. Read from the descriptor bytes, because
// esp_app_get_elf_sha256() stops at CONFIG_APP_RETRIEVE_LEN_ELF_SHA (9) characters.
void readBuildId()
{
    const esp_app_desc_t* app = esp_app_get_description();
    bool allZero = true;
    for (std::size_t i = 0; app && i < kBuildIdHexDigits / 2; ++i) allZero = allZero && app->app_elf_sha256[i] == 0;
    if (!app || allZero) {
        std::snprintf(buildId.data(), buildId.size(), "unavailable");
        return;
    }
    for (std::size_t i = 0; i < kBuildIdHexDigits / 2; ++i)
        std::snprintf(buildId.data() + 2 * i, buildId.size() - 2 * i, "%02x", app->app_elf_sha256[i]);
}

diagnostics::ReportLine identityLine()
{
    return diagnostics::identityLine({robotId(), ble::kRobotInfoFirmwareVersion, std::string_view{buildId.data()},
                                      resetReason, sessionNumber(), uptimeMilliseconds()});
}

diagnostics::ReportLine runtimeLine(const runtime::RuntimeStatus& status)
{
    const auto duty = driveOutput.lastDuty();
    return diagnostics::runtimeLine({uptimeMilliseconds(), startResult == runtime::StartResult::Running, status,
                                     duty.m1, duty.m2, resetReason,
                                     provisioning::toString(configuration.bootState().state), robotId(),
                                     firmwareRuntime.diagnostics()});
}

diagnostics::ReportLine telemetryLine()
{
    diagnostics::TelemetryReport report;
    report.session = sessionNumber();
    report.taskRunning = firmwareRuntime.telemetryTaskStarted();
    report.ticksCoalesced = blePeripheral.ticksCoalesced();
    if (const auto info = configuration.robotInfo())
        report.period = std::chrono::milliseconds{info->telemetryPeriodMilliseconds};
    if (bleController) {
        report.counters = bleController->telemetryCounters();
        report.window = bleController->takeTelemetryWindow();
    }
    report.battery = battery.assess(firmwareClock.nowMicroseconds());
    report.batterySensor = battery.hasSensor();
    report.batteryRawMillivolts = battery.lastRawMillivolts();
    report.batteryReadFailures = battery.readFailures();
    return diagnostics::telemetryLine(report);
}

// Tasks outside the runtime's platform: the BLE host and this loop.
std::size_t stackFree(TaskHandle_t task) { return static_cast<std::size_t>(uxTaskGetStackHighWaterMark(task)); }

diagnostics::ReportLine resourcesLine()
{
    return diagnostics::resourcesLine({heap_caps_get_free_size(MALLOC_CAP_8BIT),
                                       heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT),
                                       heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)},
                                      logRing.stats());
}

std::chrono::milliseconds lastReport{};
runtime::RuntimeStatus lastStatus{};
bool reported{false};
ble::LinkStatus lastBle{};
std::chrono::milliseconds lastBleReport{};
bool bleReported{false};

void reportBle()
{
    lastBle = bleController->status();
    logLine(diagnostics::bleLine(lastBle, bleController->diagnostics()));
    lastBleReport = firmwareClock.now();
    bleReported = true;
}

// One report round, spread over loop passes: stage k prints line k.
class ReportRound {
public:
    void begin() noexcept { stage_ = 0; }
    [[nodiscard]] bool active() const noexcept { return stage_ < stageCount(); }
    void next() noexcept
    {
        if (!active()) return;
        emit(stage_++);
    }

private:
    static constexpr std::size_t kFixedStages = 4;  // IDENTITY, RUNTIME, BLE, TELEMETRY.
    static constexpr std::size_t kOtherTasks = 2;   // nimble_host, loopTask.
    [[nodiscard]] std::size_t stageCount() const noexcept
    {
        return kFixedStages + platform.startedTasks() + kOtherTasks + 1;  // + RESOURCES.
    }
    void emit(std::size_t stage) noexcept
    {
        if (stage == 0) return logLine(identityLine());
        if (stage == 1) return logLine(runtimeLine(firmwareRuntime.status()));
        if (stage == 2) {
            if (bleController) reportBle();
            return;
        }
        if (stage == 3) return logLine(telemetryLine());
        std::size_t task = stage - kFixedStages;
        if (task < platform.startedTasks()) {
            if (const auto r = platform.report(task))
                logLine(diagnostics::taskLine({r->name, r->priority, r->period, r->stackBytes, r->stackFreeBytes,
                                               r->timing}));
            return;
        }
        task -= platform.startedTasks();
        if (task == 0) {
            const TaskHandle_t host = xTaskGetHandle("nimble_host");
            logLine(diagnostics::taskLine({"nimble_host", timing::kNimbleHostPriority, {},
                                           static_cast<std::size_t>(CONFIG_BT_NIMBLE_HOST_TASK_STACK_SIZE),
                                           host ? std::optional<std::size_t>{stackFree(host)} : std::nullopt, {}}));
            return;
        }
        if (task == 1) {
            logLine(diagnostics::taskLine({"loopTask", static_cast<int>(uxTaskPriorityGet(nullptr)), kReporterPollPeriod,
                                           getArduinoLoopTaskStackSize(), stackFree(nullptr), {}}));
            return;
        }
        logLine(resourcesLine());
    }
    std::size_t stage_{static_cast<std::size_t>(-1)};
};
ReportRound reportRound;

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
        if (overflow) logLine("ERR line too long");
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
    readBuildId();
    Serial.begin(kSerialBaudRate);
    // Bounded writes: the drain offers only what the USB buffer has room for (B13).
    Serial.setTxTimeoutMs(kSerialTxTimeoutMs);
    logLine("CodeCell C6 production runtime (#158-#164): v1 BLE peripheral and telemetry, restricted-lab profile "
            "(unauthenticated); bounded drive from the provisioned profile, powered checks blocked (#238); "
            "type help for maintenance");
    logf("BOOT reset_reason=%s shutdown_handler=%s firmware=%u.%u.%u build=%s", resetReason,
         firmwareRuntime.shutdownHookRegistered() ? "registered" : "failed", ble::kRobotInfoFirmwareVersion.major,
         ble::kRobotInfoFirmwareVersion.minor, ble::kRobotInfoFirmwareVersion.patch, buildId.data());
    const auto identity = configuration.robotInfo();
    const auto name = provisioning::advertisedName(identity ? identity->robotId : 0);
    logf("BLE start=%s profile=%s accepted_centrals=1 stack_max_connections=%d name=%s", toString(bleStart),
         ble::toString(ble::kSecurityProfile), CONFIG_BT_NIMBLE_MAX_CONNECTIONS,
         bleStart == BleStart::Started ? name.data() : "none");
    logf("SECURITY mode=%s policy=%s authenticated=no encrypted=no %s", ble::toString(ble::kSecurityMode),
         bleSecurity.profile(), ble::kRestrictedLabSecurityWarning);
    logf("TELEMETRY start=%s tick_ms=%ld period_ms=%u battery=unknown battery_low=unsupported velocity=unknown",
         firmwareRuntime.telemetryTaskStarted() ? "started" : "failed",
         static_cast<long>(timing::kTelemetryTickPeriod.count()),
         identity ? static_cast<unsigned>(identity->telemetryPeriodMilliseconds) : 0U);
    if (!motorsAttached) logLine("DRIVE attach=failed");
    const auto& boot = configuration.bootState();
    logf("PROVISIONING state=%s reason=%s generation=%lu pending=%s", provisioning::toString(boot.state),
         provisioning::toString(boot.reason), static_cast<unsigned long>(boot.generation.value_or(0)),
         provisioning::toString(boot.pending));
    if (const auto error = firmwareRuntime.configError())
        logf("RUNTIME config_error=%u", static_cast<unsigned>(*error));
}

void loop()
{
    pollMaintenanceInput();
    const auto now = firmwareClock.now();
    const auto status = firmwareRuntime.status();
    if (!reported || now - lastReport >= kStatusReportPeriod) {
        reportRound.begin();
        lastReport = now;
        lastStatus = status;
        reported = true;
    } else if (changed(status, lastStatus)) {
        logLine(runtimeLine(status));
        lastStatus = status;
    }
    if (bleController) {
        // One warning per new pass-through session, logged here and never from the BLE callback.
        const auto bypassed = bleController->diagnostics()[ble::BleCounter::BypassedSecuritySessions];
        if (bypassed != warnedBypassedSessions) {
            warnedBypassedSessions = bypassed;
            logf("SECURITY session=%lu %s", static_cast<unsigned long>(bypassed), ble::kRestrictedLabSecurityWarning);
        }
    }
    if (bleController && !reportRound.active() && bleReported && changed(bleController->status(), lastBle)
        && now - lastBleReport >= kBleReportMinSpacing)
        reportBle();
    if (logRing.size() <= diagnostics::kReportBackpressureBytes) reportRound.next();
    drainLog();
    delay(static_cast<std::uint32_t>(kReporterPollPeriod.count()));
}
