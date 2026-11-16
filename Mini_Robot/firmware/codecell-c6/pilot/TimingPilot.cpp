// Timing pilot for #156: a measurement-only bench image, never robot firmware.
// It never calls CodeCell or DriveCell and configures no motor, sensor or GPIO pin;
// the "output gate" below is a timestamp stub. Its BLE service uses its own UUIDs,
// so no v1 desktop can mistake it for a robot. It does not parse protocol frames.
// Procedure and results: Mini_Robot/docs/testing/timing-pilot-156.md.
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <esp_chip_info.h>
#include <esp_heap_caps.h>
#include <esp_idf_version.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include "LatencyHistogram.hpp"

#include <array>
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {
using codecell::Microseconds;
using codecell::timing::LatencyHistogram;

constexpr unsigned long kSerialBaudRate = 115200;           // Matches monitor_speed.
constexpr std::uint32_t kDefaultTxTimeoutMs = 100;          // HWCDC default in Arduino-ESP32 3.3.7.
constexpr std::int64_t kMicrosecondsPerMillisecond = 1000;
constexpr std::int64_t kMicrosecondsPerSecond = 1000000;
constexpr std::size_t kLineCapacity = 160;
constexpr std::size_t kMaxTasks = 32;
constexpr std::uint32_t kTaskStackBytes = 4096;
constexpr std::size_t kMaxLogLineBytes = 256;
constexpr std::uint32_t kTwdtFeedPeriodMs = 10;
constexpr std::uint32_t kTwdtFeedDurationMs = 1000;
constexpr std::uint32_t kStopPollMs = 5;
constexpr std::uint32_t kHandoffSpacingTicks = 1;
constexpr std::size_t kTelemetryBytes = 18;                 // Same size as a v1 Telemetry frame.
constexpr std::size_t kResultBytes = 9;                     // Same size as a v1 CommandResult frame.
// Requested link parameters, as in ble-gatt.md: 7.5-30 ms interval, latency 0, 400 ms timeout.
constexpr std::uint16_t kMinIntervalUnits = 6, kMaxIntervalUnits = 24, kLatency = 0, kTimeoutUnits = 40;
constexpr double kIntervalUnitMs = 1.25, kTimeoutUnitMs = 10.0;
constexpr double kP50 = 0.50, kP99 = 0.99, kP999 = 0.999;

constexpr const char* kName = "MR-PILOT-156";
constexpr const char* kServiceUuid = "8d1e0001-5c3a-4b8e-9f6a-6f9c2b0d1560";
constexpr const char* kCommandUuid = "8d1e0002-5c3a-4b8e-9f6a-6f9c2b0d1560";
constexpr const char* kEstopUuid = "8d1e0003-5c3a-4b8e-9f6a-6f9c2b0d1560";
constexpr const char* kResultUuid = "8d1e0004-5c3a-4b8e-9f6a-6f9c2b0d1560";
constexpr const char* kTelemetryUuid = "8d1e0005-5c3a-4b8e-9f6a-6f9c2b0d1560";

std::int64_t nowUs() { return esp_timer_get_time(); }
Microseconds us(std::int64_t value) { return Microseconds{value}; }

void printHistogram(const char* test, const char* name, const LatencyHistogram& h)
{
    Serial.printf("PILOT %s %s n=%llu min=%lld p50=%lld p99=%lld p999=%lld max=%lld neg=%llu unit=us\n",
                  test, name, h.count(), h.min().count(), h.percentileUpperBound(kP50).count(),
                  h.percentileUpperBound(kP99).count(), h.percentileUpperBound(kP999).count(),
                  h.max().count(), h.negativeCount());
}

// ---- Task table and CPU share (FreeRTOS run-time stats use the esp_timer, in us). ----
struct TaskSnapshot {
    std::array<TaskStatus_t, kMaxTasks> tasks{};
    UBaseType_t count{};
    configRUN_TIME_COUNTER_TYPE total{};
    std::int64_t wallUs{};
};
void snapshot(TaskSnapshot& s)
{
    s.count = uxTaskGetSystemState(s.tasks.data(), s.tasks.size(), &s.total);
    s.wallUs = nowUs();
}
void printTasks(const char* test, const TaskSnapshot* before, const TaskSnapshot& after)
{
    const double wall = before ? static_cast<double>(after.wallUs - before->wallUs) : 0.0;
    for (UBaseType_t i = 0; i < after.count; ++i) {
        const auto& t = after.tasks[i];
        double share = -1.0;
        if (before && wall > 0.0) {
            for (UBaseType_t j = 0; j < before->count; ++j) {
                if (before->tasks[j].xHandle != t.xHandle) continue;
                // Unsigned subtraction is wrap-safe for windows shorter than the counter period.
                const configRUN_TIME_COUNTER_TYPE delta = t.ulRunTimeCounter - before->tasks[j].ulRunTimeCounter;
                share = 100.0 * static_cast<double>(delta) / wall;
            }
        }
        // ESP-IDF FreeRTOS reports the stack high-water mark in bytes.
        Serial.printf("PILOT %s task name=%s prio=%u stack_free_min_bytes=%u cpu_pct=%.2f\n", test,
                      t.pcTaskName, unsigned(t.uxCurrentPriority),
                      unsigned(t.usStackHighWaterMark), share);
    }
}
void printHeap(const char* test)
{
    Serial.printf("PILOT %s heap free=%u min_free=%u largest_block=%u unit=bytes\n", test,
                  unsigned(heap_caps_get_free_size(MALLOC_CAP_8BIT)),
                  unsigned(heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT)),
                  unsigned(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)));
}

// ---- Load generators used while another measurement runs. ----
std::atomic<bool> loadRunning{false};
std::atomic<int> loadTasksAlive{0};
struct CpuLoad { std::uint32_t busyMs; std::uint32_t idleMs; };
CpuLoad cpuLoad{};
std::uint32_t logLoadBytes = 0;
std::atomic<std::uint32_t> logLoadLines{0};
char logLoadLine[kMaxLogLineBytes];

void cpuLoadTask(void*)
{
    while (loadRunning.load()) {
        const auto start = nowUs();
        while (nowUs() - start < static_cast<std::int64_t>(cpuLoad.busyMs) * kMicrosecondsPerMillisecond) {}
        vTaskDelay(pdMS_TO_TICKS(cpuLoad.idleMs));
    }
    --loadTasksAlive;
    vTaskDelete(nullptr);
}
void logLoadTask(void*)
{
    while (loadRunning.load()) {
        Serial.write(reinterpret_cast<const std::uint8_t*>(logLoadLine), logLoadBytes);
        ++logLoadLines;
        taskYIELD();
    }
    --loadTasksAlive;
    vTaskDelete(nullptr);
}
bool startTask(TaskFunction_t fn, const char* name, UBaseType_t prio, TaskHandle_t* handle = nullptr)
{
    return xTaskCreate(fn, name, kTaskStackBytes, nullptr, prio, handle) == pdPASS;
}
void stopLoad()
{
    loadRunning = false;
    while (loadTasksAlive.load() > 0) vTaskDelay(pdMS_TO_TICKS(kStopPollMs));
}

// ---- Periodic wake-up lateness, measured from the scheduler tick edge. ----
// Measurement buffers are static below: the pilot must not shrink loopTask's stack.
struct Probe {
    std::uint32_t periodMs{};
    std::uint32_t durationMs{};
    TaskHandle_t waiter{};
    LatencyHistogram lateness;
    LatencyHistogram intervalDeviation; // |actual interval - period|
    std::uint32_t overruns{};
};
Probe probe;
void probeTask(void*)
{
    const std::int64_t tickUs = kMicrosecondsPerSecond / configTICK_RATE_HZ;
    // Locate a tick edge by spinning until the tick count changes (one tick at most).
    // Calibrate before any load task exists, then wait for the command to start loads.
    TickType_t edgeTick = xTaskGetTickCount();
    while (xTaskGetTickCount() == edgeTick) {}
    edgeTick = xTaskGetTickCount();
    const std::int64_t edgeUs = nowUs();
    xTaskNotifyGive(probe.waiter);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(probe.periodMs);
    std::int64_t previous = nowUs();
    const std::int64_t end = previous + static_cast<std::int64_t>(probe.durationMs) * kMicrosecondsPerMillisecond;
    for (;;) {
        if (xTaskDelayUntil(&last, period) == pdFALSE) ++probe.overruns;
        // `previous` starts at the go signal, so the first interval sample is not meaningful.
        const std::int64_t t = nowUs();
        // `last` is the tick the task should have woken at; ticks are wrap-safe (unsigned).
        const std::int64_t ideal = edgeUs + static_cast<std::int64_t>(TickType_t(last - edgeTick)) * tickUs;
        probe.lateness.record(us(t - ideal));
        const std::int64_t periodUs = static_cast<std::int64_t>(probe.periodMs) * kMicrosecondsPerMillisecond;
        probe.intervalDeviation.record(us(std::llabs(t - previous - periodUs)));
        previous = t;
        if (t >= end) break;
    }
    xTaskNotifyGive(probe.waiter);
    vTaskDelete(nullptr);
}

// ---- BLE measurement peripheral (pilot UUIDs; frames are echoed, never interpreted). ----
struct BleStats {
    LatencyHistogram interArrival, callback, notify, handoff, estopGate, telemetryNotify, telemetryLateness;
    std::uint32_t writes{}, estops{}, notifyFailures{}, telemetrySent{}, telemetryFailures{}, handoffs{};
    std::int64_t lastWriteUs{-1};
    bool interArrivalArmed{false}; // False after a reset: the first gap spans the reset.
};
BleStats ble;
// `ble` is written by the BLE host callbacks, the control consumer and the telemetry task,
// and reset or printed by the console in loopTask. LatencyHistogram has one writer and no
// locking, so every update, reset and snapshot of `ble` holds statsLock; the console
// copies one histogram at a time under it and prints the copy outside. The C6 is
// single-core: the critical section masks interrupts, so it also keeps the writer tasks
// from preempting a reset or copy. Each hold is one record or one histogram copy.
portMUX_TYPE statsLock = portMUX_INITIALIZER_UNLOCKED;
class StatsGuard {
public:
    StatsGuard() { portENTER_CRITICAL(&statsLock); }
    ~StatsGuard() { portEXIT_CRITICAL(&statsLock); }
    StatsGuard(const StatsGuard&) = delete;
    StatsGuard& operator=(const StatsGuard&) = delete;
};
void recordStat(LatencyHistogram& histogram, Microseconds value)
{
    StatsGuard guard;
    histogram.record(value);
}
NimBLEServer* server = nullptr;
NimBLECharacteristic* estopCharacteristic = nullptr;
NimBLECharacteristic* resultCharacteristic = nullptr;
NimBLECharacteristic* telemetryCharacteristic = nullptr;
std::atomic<bool> bleStarted{false};
std::atomic<bool> connected{false};
std::atomic<bool> resultSubscribed{false};
std::atomic<bool> telemetrySubscribed{false};
QueueHandle_t setpointMailbox = nullptr; // Depth 1, overwrite: newest wins.
std::uint16_t robotSequence = 0;         // Results and telemetry share it; guarded by sequenceLock.
portMUX_TYPE sequenceLock = portMUX_INITIALIZER_UNLOCKED;
volatile std::int64_t gateStubUs = 0;    // Written by the output-gate stub.

std::uint16_t nextSequence()
{
    portENTER_CRITICAL(&sequenceLock);
    const auto value = robotSequence++;
    portEXIT_CRITICAL(&sequenceLock);
    return value;
}
// Stand-in for "disable drive outputs". It only records the time; no pin is touched.
void IRAM_ATTR outputGateDisableStub() { gateStubUs = nowUs(); }

void notifyResult(const std::uint8_t* frame, std::size_t length, std::uint8_t type)
{
    if (!resultSubscribed.load()) return;
    const auto sequence = nextSequence();
    std::array<std::uint8_t, kResultBytes> result{1, 0x81, std::uint8_t(sequence & 0xff), std::uint8_t(sequence >> 8),
                                                  length >= 4 ? frame[2] : std::uint8_t{0},
                                                  length >= 4 ? frame[3] : std::uint8_t{0}, type, 0, 0};
    const auto start = nowUs();
    const bool ok = resultCharacteristic->notify(result.data(), result.size());
    const auto duration = us(nowUs() - start);
    StatsGuard guard;
    ble.notify.record(duration);
    if (!ok) ++ble.notifyFailures;
}
class ServerCallbacks final : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* s, NimBLEConnInfo& info) override
    {
        connected = true;
        Serial.printf("PILOT ble connect interval_ms=%.2f latency=%u timeout_ms=%.0f\n",
                      info.getConnInterval() * kIntervalUnitMs, info.getConnLatency(),
                      info.getConnTimeout() * kTimeoutUnitMs);
        s->updateConnParams(info.getConnHandle(), kMinIntervalUnits, kMaxIntervalUnits, kLatency, kTimeoutUnits);
    }
    void onConnParamsUpdate(NimBLEConnInfo& info) override
    {
        Serial.printf("PILOT ble params interval_ms=%.2f latency=%u timeout_ms=%.0f\n",
                      info.getConnInterval() * kIntervalUnitMs, info.getConnLatency(),
                      info.getConnTimeout() * kTimeoutUnitMs);
    }
    void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int reason) override
    {
        const auto t = nowUs();
        connected = false;
        resultSubscribed = false;
        telemetrySubscribed = false;
        // Robot-clock interval only; no desktop clock is involved.
        std::int64_t lastWriteUs = -1;
        {
            StatsGuard guard;
            lastWriteUs = ble.lastWriteUs;
        }
        Serial.printf("PILOT ble disconnect reason=%d since_last_write_us=%lld\n", reason,
                      lastWriteUs < 0 ? -1LL : static_cast<long long>(t - lastWriteUs));
    }
} serverCallbacks;
class CharacteristicCallbacks final : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* characteristic, NimBLEConnInfo&) override
    {
        const auto receipt = nowUs(); // Instrumentation point: receipt at callback entry.
        // Pointer comparison: parsing a UUID string here cost about 60 us per write.
        const bool estop = characteristic == estopCharacteristic;
        if (estop) {
            outputGateDisableStub();
            const auto gate = us(gateStubUs - receipt);
            StatsGuard guard;
            ble.estopGate.record(gate);
            ++ble.estops;
            ble.lastWriteUs = receipt;
        }
        const auto value = characteristic->getValue();
        if (!estop) {
            {
                StatsGuard guard;
                if (ble.interArrivalArmed) ble.interArrival.record(us(receipt - ble.lastWriteUs));
                ble.interArrivalArmed = true;
                ++ble.writes;
                ble.lastWriteUs = receipt;
            }
            if (setpointMailbox) xQueueOverwrite(setpointMailbox, &receipt);
        }
        notifyResult(value.data(), value.size(), estop ? 0x03 : (value.size() >= 2 ? value.data()[1] : 0));
        recordStat(ble.callback, us(nowUs() - receipt));
    }
    void onSubscribe(NimBLECharacteristic* characteristic, NimBLEConnInfo&, std::uint16_t value) override
    {
        const bool enabled = (value & 1) != 0;
        if (characteristic == resultCharacteristic) resultSubscribed = enabled;
        if (characteristic == telemetryCharacteristic) telemetrySubscribed = enabled;
    }
} characteristicCallbacks;

void controlConsumerTask(void*)
{
    for (;;) {
        std::int64_t receipt = 0;
        if (xQueueReceive(setpointMailbox, &receipt, portMAX_DELAY) == pdTRUE) {
            const auto handoff = us(nowUs() - receipt);
            StatsGuard guard;
            ble.handoff.record(handoff);
            ++ble.handoffs;
        }
    }
}
std::atomic<std::uint32_t> telemetryPeriodMs{0};
TaskHandle_t telemetryTask = nullptr;
void telemetryLoop(void*)
{
    TickType_t last = xTaskGetTickCount();
    std::int64_t previous = nowUs();
    for (;;) {
        const auto period = telemetryPeriodMs.load();
        if (period == 0) {
            vTaskDelay(pdMS_TO_TICKS(kStopPollMs));
            last = xTaskGetTickCount();
            previous = nowUs();
            continue;
        }
        xTaskDelayUntil(&last, pdMS_TO_TICKS(period));
        const auto t = nowUs();
        recordStat(ble.telemetryLateness,
                   us(std::llabs(t - previous - static_cast<std::int64_t>(period) * kMicrosecondsPerMillisecond)));
        previous = t;
        if (!(connected.load() && telemetrySubscribed.load())) continue;
        std::array<std::uint8_t, kTelemetryBytes> frame{1, 0x80};
        const auto sequence = nextSequence();
        frame[2] = sequence & 0xff;
        frame[3] = sequence >> 8;
        const auto start = nowUs();
        const bool ok = telemetryCharacteristic->notify(frame.data(), frame.size());
        const auto duration = us(nowUs() - start);
        StatsGuard guard;
        ble.telemetryNotify.record(duration);
        ok ? ++ble.telemetrySent : ++ble.telemetryFailures;
    }
}
bool startBle(UBaseType_t consumerPrio, UBaseType_t telemetryPrio)
{
    if (bleStarted.load()) return true;
    setpointMailbox = xQueueCreate(1, sizeof(std::int64_t));
    if (!setpointMailbox) return false;
    if (!NimBLEDevice::init(kName)) return false;
    server = NimBLEDevice::createServer();
    server->setCallbacks(&serverCallbacks);
    server->advertiseOnDisconnect(true);
    auto* service = server->createService(kServiceUuid);
    auto* command = service->createCharacteristic(kCommandUuid, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    estopCharacteristic = service->createCharacteristic(kEstopUuid, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    resultCharacteristic = service->createCharacteristic(kResultUuid, NIMBLE_PROPERTY::NOTIFY);
    telemetryCharacteristic = service->createCharacteristic(kTelemetryUuid, NIMBLE_PROPERTY::NOTIFY);
    for (auto* c : {command, estopCharacteristic, resultCharacteristic, telemetryCharacteristic}) {
        c->setCallbacks(&characteristicCallbacks);
    }
    service->start();
    NimBLEAdvertisementData advertisement;
    advertisement.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);
    advertisement.addServiceUUID(kServiceUuid);
    NimBLEAdvertisementData scanResponse;
    scanResponse.setName(kName);
    auto* advertising = NimBLEDevice::getAdvertising();
    advertising->setAdvertisementData(advertisement);
    advertising->setScanResponseData(scanResponse);
    advertising->enableScanResponse(true);
    if (!advertising->start()) return false;
    if (!startTask(controlConsumerTask, "pilot_ctrl", consumerPrio)) return false;
    if (!startTask(telemetryLoop, "pilot_telem", telemetryPrio, &telemetryTask)) return false;
    bleStarted = true;
    return true;
}
// Each histogram is copied under statsLock and printed from the copy (static: a histogram
// is several KB, too large for loopTask's stack seven times over). Histograms are taken one
// after another while the writers run, so two lines may cover slightly different moments;
// each line is internally consistent.
void printBleHistogram(const char* name, const LatencyHistogram& live)
{
    static LatencyHistogram copy;
    {
        StatsGuard guard;
        copy = live;
    }
    printHistogram("ble", name, copy);
}
void printBle()
{
    printBleHistogram("write_interarrival", ble.interArrival);
    printBleHistogram("write_callback_duration", ble.callback);
    printBleHistogram("result_notify_call", ble.notify);
    printBleHistogram("receipt_to_consumer", ble.handoff);
    printBleHistogram("estop_receipt_to_gate_stub", ble.estopGate);
    printBleHistogram("telemetry_notify_call", ble.telemetryNotify);
    printBleHistogram("telemetry_period_abs_deviation", ble.telemetryLateness);
    std::uint32_t writes{}, estops{}, handoffs{}, notifyFailures{}, telemetrySent{}, telemetryFailures{};
    {
        StatsGuard guard;
        writes = ble.writes;
        estops = ble.estops;
        handoffs = ble.handoffs;
        notifyFailures = ble.notifyFailures;
        telemetrySent = ble.telemetrySent;
        telemetryFailures = ble.telemetryFailures;
    }
    Serial.printf("PILOT ble counts writes=%lu estops=%lu handoffs=%lu result_notify_failures=%lu "
                  "telemetry_sent=%lu telemetry_failures=%lu connected=%u\n",
                  (unsigned long)writes, (unsigned long)estops, (unsigned long)handoffs,
                  (unsigned long)notifyFailures, (unsigned long)telemetrySent,
                  (unsigned long)telemetryFailures, unsigned(connected.load()));
}

// ---- Commands ----
void commandInfo()
{
    esp_chip_info_t chip{};
    esp_chip_info(&chip);
    Serial.printf("PILOT info chip_model=%d revision=%u cores=%u cpu_mhz=%lu tick_hz=%u idf=%s "
                  "reset_reason=%d uptime_us=%lld ble=%u\n",
                  int(chip.model), unsigned(chip.revision), unsigned(chip.cores),
                  (unsigned long)getCpuFrequencyMhz(), unsigned(configTICK_RATE_HZ), esp_get_idf_version(),
                  int(esp_reset_reason()), static_cast<long long>(nowUs()), unsigned(bleStarted.load()));
    printHeap("info");
    static TaskSnapshot s;
    snapshot(s);
    printTasks("info", nullptr, s);
}
void commandClock(std::uint32_t samples)
{
    static LatencyHistogram h;
    h.reset();
    auto previous = nowUs();
    for (std::uint32_t i = 0; i < samples; ++i) {
        const auto t = nowUs();
        h.record(us(t - previous));
        previous = t;
    }
    printHistogram("clock", "esp_timer_back_to_back", h);
}
void commandJitter(std::uint32_t periodMs, std::uint32_t seconds, UBaseType_t prio, char* rest)
{
    probe.lateness.reset(); // In place, for the same stack reason as BleStats.
    probe.intervalDeviation.reset();
    probe.overruns = 0;
    probe.periodMs = periodMs;
    probe.durationMs = seconds * 1000;
    probe.waiter = xTaskGetCurrentTaskHandle();
    TaskHandle_t probeHandle = nullptr;
    if (!startTask(probeTask, "pilot_probe", prio, &probeHandle)) {
        Serial.println("PILOT jitter error=task_create");
        return;
    }
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Probe has located a tick edge.
    loadRunning = true;
    // Optional loads: "cpu <prio> <busyMs> <idleMs>" and/or "log <prio> <bytes>".
    char* save = nullptr;
    for (char* word = strtok_r(rest, " ", &save); word; word = strtok_r(nullptr, " ", &save)) {
        if (strcmp(word, "cpu") == 0) {
            const auto p = std::strtoul(strtok_r(nullptr, " ", &save) ?: "1", nullptr, 10);
            cpuLoad.busyMs = std::strtoul(strtok_r(nullptr, " ", &save) ?: "1", nullptr, 10);
            cpuLoad.idleMs = std::strtoul(strtok_r(nullptr, " ", &save) ?: "1", nullptr, 10);
            ++loadTasksAlive;
            if (!startTask(cpuLoadTask, "pilot_cpu", p)) --loadTasksAlive;
        } else if (strcmp(word, "log") == 0) {
            const auto p = std::strtoul(strtok_r(nullptr, " ", &save) ?: "1", nullptr, 10);
            logLoadBytes = std::min<std::uint32_t>(std::strtoul(strtok_r(nullptr, " ", &save) ?: "80", nullptr, 10),
                                                   kMaxLogLineBytes);
            std::memset(logLoadLine, 'x', sizeof(logLoadLine));
            std::memcpy(logLoadLine, "LOADLOG ", 8);
            logLoadLine[logLoadBytes - 1] = '\n';
            logLoadLines = 0;
            ++loadTasksAlive;
            if (!startTask(logLoadTask, "pilot_log", p)) --loadTasksAlive;
        }
    }
    static TaskSnapshot before;
    snapshot(before);
    xTaskNotifyGive(probeHandle);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    static TaskSnapshot after;
    snapshot(after);
    stopLoad();
    Serial.printf("PILOT jitter config period_ms=%lu seconds=%lu prio=%u overruns=%lu log_lines=%lu\n",
                  (unsigned long)periodMs, (unsigned long)seconds, unsigned(prio),
                  (unsigned long)probe.overruns, (unsigned long)logLoadLines.load());
    printHistogram("jitter", "wake_lateness_from_tick", probe.lateness);
    printHistogram("jitter", "wake_interval_abs_deviation", probe.intervalDeviation);
    printTasks("jitter", &before, after);
    printHeap("jitter");
}
QueueHandle_t handoffQueue = nullptr;
volatile std::int64_t notifySentUs = 0;
LatencyHistogram queueHandoff, notifyHandoff;
TaskHandle_t handoffWaiter = nullptr;
void queueConsumer(void*)
{
    for (;;) {
        std::int64_t sent = 0;
        xQueueReceive(handoffQueue, &sent, portMAX_DELAY);
        if (sent < 0) break;
        queueHandoff.record(us(nowUs() - sent));
    }
    xTaskNotifyGive(handoffWaiter);
    vTaskDelete(nullptr);
}
void notifyConsumer(void*)
{
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (notifySentUs < 0) break;
        notifyHandoff.record(us(nowUs() - notifySentUs));
    }
    xTaskNotifyGive(handoffWaiter);
    vTaskDelete(nullptr);
}
void commandHandoff(std::uint32_t samples, UBaseType_t consumerPrio)
{
    queueHandoff.reset();
    notifyHandoff.reset();
    handoffWaiter = xTaskGetCurrentTaskHandle();
    handoffQueue = xQueueCreate(1, sizeof(std::int64_t));
    TaskHandle_t notifyTask = nullptr;
    if (!handoffQueue || !startTask(queueConsumer, "pilot_qcons", consumerPrio)
        || !startTask(notifyConsumer, "pilot_ncons", consumerPrio, &notifyTask)) {
        Serial.println("PILOT handoff error=create");
        return;
    }
    for (std::uint32_t i = 0; i < samples; ++i) {
        const auto t = nowUs();
        xQueueOverwrite(handoffQueue, &t);
        vTaskDelay(kHandoffSpacingTicks);
        notifySentUs = nowUs();
        xTaskNotifyGive(notifyTask);
        vTaskDelay(kHandoffSpacingTicks);
    }
    const std::int64_t stop = -1;
    xQueueOverwrite(handoffQueue, &stop);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    notifySentUs = -1;
    xTaskNotifyGive(notifyTask);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    vQueueDelete(handoffQueue);
    handoffQueue = nullptr;

    struct Snapshot { std::int64_t deadline; std::int32_t linear, angular; std::uint16_t sequence; };
    static Snapshot shared{}, copy{};
    portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
    SemaphoreHandle_t mutex = xSemaphoreCreateMutex();
    static LatencyHistogram critical, mutexPair;
    critical.reset();
    mutexPair.reset();
    for (std::uint32_t i = 0; i < samples; ++i) {
        auto t = nowUs();
        portENTER_CRITICAL(&lock);
        copy = shared;
        shared.sequence = static_cast<std::uint16_t>(i);
        portEXIT_CRITICAL(&lock);
        critical.record(us(nowUs() - t));
        t = nowUs();
        if (mutex && xSemaphoreTake(mutex, 0) == pdTRUE) xSemaphoreGive(mutex);
        mutexPair.record(us(nowUs() - t));
    }
    if (mutex) vSemaphoreDelete(mutex);
    Serial.printf("PILOT handoff config samples=%lu consumer_prio=%u producer_prio=%u\n",
                  (unsigned long)samples, unsigned(consumerPrio), unsigned(uxTaskPriorityGet(nullptr)));
    printHistogram("handoff", "queue_overwrite_to_receive", queueHandoff);
    printHistogram("handoff", "task_notify_to_wake", notifyHandoff);
    printHistogram("handoff", "critical_section_copy", critical);
    printHistogram("handoff", "mutex_take_give_uncontended", mutexPair);
}
void commandLog(std::uint32_t lines, std::uint32_t bytes, std::uint32_t timeoutMs,
                std::uint32_t delayS, std::uint32_t reportDelayS)
{
    bytes = std::clamp<std::uint32_t>(bytes, 2, kMaxLogLineBytes);
    char line[kMaxLogLineBytes];
    std::memset(line, 'y', sizeof(line));
    std::memcpy(line, "LOGTEST ", 8);
    line[bytes - 1] = '\n';
    if (delayS) vTaskDelay(pdMS_TO_TICKS(delayS * 1000));
    Serial.setTxTimeoutMs(timeoutMs);
    static LatencyHistogram h;
    h.reset();
    std::uint32_t shortWrites = 0;
    for (std::uint32_t i = 0; i < lines; ++i) {
        const auto t = nowUs();
        const auto written = Serial.write(reinterpret_cast<const std::uint8_t*>(line), bytes);
        h.record(us(nowUs() - t));
        if (written != bytes) ++shortWrites;
    }
    Serial.setTxTimeoutMs(kDefaultTxTimeoutMs);
    if (reportDelayS) vTaskDelay(pdMS_TO_TICKS(reportDelayS * 1000));
    Serial.printf("\nPILOT log config lines=%lu bytes=%lu tx_timeout_ms=%lu short_writes=%lu\n",
                  (unsigned long)lines, (unsigned long)bytes, (unsigned long)timeoutMs, (unsigned long)shortWrites);
    printHistogram("log", "serial_write_call", h);
}
void twdtStallTask(void*)
{
    esp_task_wdt_add(nullptr);
    const auto feeds = kTwdtFeedDurationMs / kTwdtFeedPeriodMs;
    for (std::uint32_t i = 0; i < feeds; ++i) {
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(kTwdtFeedPeriodMs));
    }
    esp_task_wdt_reset();
    Serial.printf("PILOT twdt stall uptime_us=%lld\n", static_cast<long long>(nowUs()));
    Serial.flush();
    for (;;) {} // Deliberate stall: the task watchdog must panic and reset the chip.
}
void commandTwdt(std::uint32_t timeoutMs)
{
    const esp_task_wdt_config_t config{timeoutMs, 0, true};
    const auto status = esp_task_wdt_reconfigure(&config);
    Serial.printf("PILOT twdt config timeout_ms=%lu reconfigure=%d\n", (unsigned long)timeoutMs, int(status));
    if (status == ESP_OK) startTask(twdtStallTask, "pilot_twdt", 1);
}

std::uint32_t arg(char*& save, std::uint32_t fallback)
{
    const char* word = strtok_r(nullptr, " ", &save);
    return word ? std::strtoul(word, nullptr, 10) : fallback;
}
void handleLine(char* line)
{
    char* save = nullptr;
    const char* command = strtok_r(line, " ", &save);
    if (!command) return;
    if (strcmp(command, "info") == 0) return commandInfo();
    if (strcmp(command, "clock") == 0) return commandClock(arg(save, 10000));
    if (strcmp(command, "jitter") == 0) {
        const auto period = arg(save, 5), seconds = arg(save, 10), prio = arg(save, 20);
        return commandJitter(period, seconds, prio, save ? save : const_cast<char*>(""));
    }
    if (strcmp(command, "handoff") == 0) {
        const auto samples = arg(save, 1000);
        return commandHandoff(samples, arg(save, 20));
    }
    if (strcmp(command, "log") == 0) {
        const auto lines = arg(save, 100), bytes = arg(save, 80), timeout = arg(save, kDefaultTxTimeoutMs);
        const auto delayS = arg(save, 0);
        return commandLog(lines, bytes, timeout, delayS, arg(save, 0));
    }
    if (strcmp(command, "ble") == 0) {
        const char* sub = strtok_r(nullptr, " ", &save);
        if (sub && strcmp(sub, "on") == 0) {
            const auto consumer = arg(save, 20);
            const bool ok = startBle(consumer, arg(save, 5));
            Serial.printf("PILOT ble started=%u name=%s\n", unsigned(ok), kName);
            printHeap("ble");
            return;
        }
        if (sub && strcmp(sub, "reset") == 0) {
            // Reset in place: a temporary BleStats (seven histograms) would overflow loopTask's stack.
            // One histogram per statsLock hold, so the writers wait at most one reset.
            for (auto* h : {&ble.interArrival, &ble.callback, &ble.notify, &ble.handoff, &ble.estopGate,
                            &ble.telemetryNotify, &ble.telemetryLateness}) {
                StatsGuard guard;
                h->reset();
            }
            {
                StatsGuard guard;
                ble.writes = ble.estops = ble.notifyFailures = ble.telemetrySent = ble.telemetryFailures = 0;
                ble.handoffs = 0;
                ble.interArrivalArmed = false;
            }
            Serial.println("PILOT ble reset");
            return;
        }
        return printBle();
    }
    if (strcmp(command, "telem") == 0) {
        telemetryPeriodMs = arg(save, 0);
        Serial.printf("PILOT telem period_ms=%lu\n", (unsigned long)telemetryPeriodMs.load());
        return;
    }
    if (strcmp(command, "twdt") == 0) return commandTwdt(arg(save, 100));
    Serial.println("PILOT help info | clock <n> | jitter <periodMs> <s> <prio> [cpu <prio> <busyMs> <idleMs>] "
                   "[log <prio> <bytes>] | handoff <n> <prio> | log <lines> <bytes> <timeoutMs> <delayS> "
                   "<reportDelayS> | ble on <consumerPrio> <telemetryPrio> | ble [stats|reset] | "
                   "telem <periodMs> | twdt <timeoutMs>");
}
} // namespace

void setup()
{
    Serial.begin(kSerialBaudRate); // Never waits for a USB host.
    Serial.printf("PILOT boot reset_reason=%d uptime_us=%lld (measurement image, no actuator/sensor access)\n",
                  int(esp_reset_reason()), static_cast<long long>(nowUs()));
}
void loop()
{
    static char line[kLineCapacity];
    static std::size_t length = 0;
    static bool overflow = false;
    while (Serial.available()) {
        const char c = static_cast<char>(Serial.read());
        if (c == '\r') continue;
        if (c == '\n') {
            line[length] = '\0';
            if (!overflow) handleLine(line);
            else Serial.println("PILOT error=line_too_long");
            length = 0;
            overflow = false;
            Serial.println("PILOT done");
        } else if (length + 1 < sizeof(line) && !overflow) line[length++] = c;
        else overflow = true;
    }
    vTaskDelay(1);
}
