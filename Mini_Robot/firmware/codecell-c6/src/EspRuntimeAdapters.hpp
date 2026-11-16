#pragma once
// ESP-IDF/FreeRTOS adapters for the #158 runtime ports. This file and main.cpp are the
// only production code that may name SDK, RTOS or board APIs; the portable runtime in
// include/runtime/ is host-tested with fakes.
#include "TimingBudget.hpp"
#include "runtime/RuntimePorts.hpp"
#include "runtime/TaskTiming.hpp"
#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <optional>

namespace codecell::esp {

class EspFirmwareClock final : public IFirmwareClock {
public:
    // esp_timer_get_time() is monotonic signed 64-bit microseconds since boot, never negative.
    Milliseconds now() const override
    {
        return monotonicMilliseconds(esp_timer_get_time()).value_or(Milliseconds::zero());
    }
    Microseconds nowMicroseconds() const override
    {
        return std::max(Microseconds{esp_timer_get_time()}, Microseconds::zero());
    }
};

// portMUX spinlock: statically initialized (no allocation, cannot fail). On the single-
// core C6 it masks interrupts and task switches for the few microseconds a holder needs.
class EspCriticalSection final : public runtime::ICriticalSection {
public:
    void enter() noexcept override { portENTER_CRITICAL(&lock_); }
    void exit() noexcept override { portEXIT_CRITICAL(&lock_); }

private:
    portMUX_TYPE lock_ = portMUX_INITIALIZER_UNLOCKED;
};

// The production image needs the supervisor, the control task and the telemetry tick (#164).
inline constexpr std::size_t kMaxPeriodicTasks = 3;

// One started periodic task, for the diagnostic report (#164).
struct PeriodicTaskReport {
    const char* name{""};
    int priority{0};
    std::chrono::milliseconds period{};
    std::size_t stackBytes{0};
    std::size_t stackFreeBytes{0};  // ESP-IDF FreeRTOS reports the high-water mark in bytes.
    runtime::TaskTimingSnapshot timing{};
};

// Task watchdog (#156 B17, #162): only the subscribed task (the supervisor) is watched. The
// idle task is not (the pinned sdkconfig does not watch it either), so a busy low-priority
// task cannot reset the robot, and the Arduino loopTask stays unsubscribed. Expiry panics;
// the ISR hook in main.cpp powers the bridges down first.
inline constexpr esp_task_wdt_config_t kTaskWatchdogConfig{
    static_cast<std::uint32_t>(timing::kTaskWatchdogTimeout.count()), 0, true};

// Name of an ESP-IDF reset reason, for the boot and status lines (#162). Diagnostic only:
// no reason restores or refuses anything, every boot starts Idle with outputs off.
[[nodiscard]] inline const char* resetReasonName(esp_reset_reason_t reason) noexcept
{
    switch (reason) {
    case ESP_RST_POWERON: return "power_on";
    case ESP_RST_EXT: return "external";
    case ESP_RST_SW: return "software";
    case ESP_RST_PANIC: return "panic";
    case ESP_RST_INT_WDT: return "interrupt_watchdog";
    case ESP_RST_TASK_WDT: return "task_watchdog";
    case ESP_RST_WDT: return "other_watchdog";
    case ESP_RST_DEEPSLEEP: return "deep_sleep";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_SDIO: return "sdio";
    case ESP_RST_USB: return "usb";
    case ESP_RST_JTAG: return "jtag";
    case ESP_RST_EFUSE: return "efuse";
    case ESP_RST_PWR_GLITCH: return "power_glitch";
    case ESP_RST_CPU_LOCKUP: return "cpu_lockup";
    case ESP_RST_UNKNOWN: break;
    }
    return "unknown";
}

class FreeRtosPlatform final : public runtime::IPlatform {
public:
    runtime::ICriticalSection* createCriticalSection() noexcept override
    {
        if (sectionCreated_) return nullptr;  // One handoff section per runtime.
        sectionCreated_ = true;
        return &section_;
    }

    bool startPeriodicTask(const runtime::TaskSpec& spec, runtime::IPeriodicStep& body) noexcept override
    {
        if (started_ >= tasks_.size() || spec.period <= std::chrono::milliseconds::zero()) return false;
        if (spec.watchdog && !configureWatchdog()) return false;
        auto& task = tasks_[started_];
        task.body = &body;
        task.periodTicks = static_cast<TickType_t>(timing::ticksAtLeast(spec.period));
        task.name = spec.name;
        task.priority = spec.priority;
        task.period = spec.period;
        task.stackBytes = spec.stackBytes;
        // The stack is allocated here, during initialization only (B15). Checked result.
        if (xTaskCreate(&FreeRtosPlatform::run, spec.name, static_cast<std::uint32_t>(spec.stackBytes), &task,
                        static_cast<UBaseType_t>(spec.priority), &task.handle)
            != pdPASS)
            return false;
        // The new task may already have run (it outranks setup()); it feeds only once
        // subscribed. A failed subscription deletes it: an unwatched supervisor never runs.
        if (spec.watchdog) {
            if (esp_task_wdt_add(task.handle) != ESP_OK) {
                vTaskDelete(task.handle);
                task.handle = nullptr;
                return false;
            }
            task.watched.store(true, std::memory_order_release);
        }
        ++started_;
        return true;
    }

    // One hook per image; esp_register_shutdown_handler() is checked, never assumed.
    bool registerShutdownHook(runtime::IShutdownHook& hook) noexcept override
    {
        runtime::IShutdownHook* expected = nullptr;
        if (!shutdownHook_.compare_exchange_strong(expected, &hook, std::memory_order_acq_rel))
            return expected == &hook;
        if (esp_register_shutdown_handler(&FreeRtosPlatform::runShutdownHook) != ESP_OK) {
            shutdownHook_.store(nullptr, std::memory_order_release);
            return false;
        }
        return true;
    }

    // Each watched task unsubscribes itself on its next step (no feed can race the removal);
    // this waits for that, at most one watchdog timeout, polling once per tick.
    void releaseTaskWatchdog() noexcept override
    {
        for (std::size_t i = 0; i < started_; ++i)
            if (tasks_[i].watched.load(std::memory_order_acquire))
                tasks_[i].releaseRequested.store(true, std::memory_order_release);
        const auto limit = timing::ticksAtLeast(timing::kTaskWatchdogTimeout);
        for (std::uint32_t waited = 0; waited < limit && anyWatched(); ++waited) vTaskDelay(1);
    }

    [[nodiscard]] std::size_t startedTasks() const noexcept { return started_; }

    // Reporter (loopTask only): timing window and stack high-water of the i-th started task.
    [[nodiscard]] std::optional<PeriodicTaskReport> report(std::size_t index) noexcept
    {
        if (index >= started_ || !tasks_[index].handle) return std::nullopt;
        auto& task = tasks_[index];
        return PeriodicTaskReport{task.name, task.priority, task.period, task.stackBytes,
                                  static_cast<std::size_t>(uxTaskGetStackHighWaterMark(task.handle)),
                                  task.timing.takeWindow()};
    }

private:
    // Runs inside esp_restart(), in the caller's task.
    static void runShutdownHook()
    {
        if (auto* hook = shutdownHook_.load(std::memory_order_acquire)) hook->beforeRestart();
    }
    inline static std::atomic<runtime::IShutdownHook*> shutdownHook_{nullptr};

    struct Task {
        runtime::IPeriodicStep* body{nullptr};
        TickType_t periodTicks{0};
        TaskHandle_t handle{nullptr};
        std::atomic<bool> watched{false};
        std::atomic<bool> releaseRequested{false};
        const char* name{""};
        int priority{0};
        std::chrono::milliseconds period{};
        std::size_t stackBytes{0};
        runtime::TaskTiming timing{};  // Recorded by the task itself, read by the reporter.
    };

    // The TWDT is initialized by ESP-IDF at boot (CONFIG_ESP_TASK_WDT_INIT, 5 s); the
    // supervisor's 50 ms replaces that. Initialize it here if a build ever disables that.
    static bool configureWatchdog() noexcept
    {
        esp_err_t result = esp_task_wdt_reconfigure(&kTaskWatchdogConfig);
        if (result == ESP_ERR_INVALID_STATE) result = esp_task_wdt_init(&kTaskWatchdogConfig);
        return result == ESP_OK;
    }

    [[nodiscard]] bool anyWatched() const noexcept
    {
        for (std::size_t i = 0; i < started_; ++i)
            if (tasks_[i].watched.load(std::memory_order_acquire)) return true;
        return false;
    }

    static void run(void* context)
    {
        constexpr std::int64_t kTickUs = 1'000'000 / configTICK_RATE_HZ;
        auto& task = *static_cast<Task*>(context);
        TickType_t lastWake = xTaskGetTickCount();
        task.timing.woke(esp_timer_get_time(), static_cast<std::uint32_t>(lastWake), kTickUs);
        for (;;) {
            // Diagnostics only (#164): two clock reads and a few atomic stores per step.
            const std::int64_t stepStart = esp_timer_get_time();
            task.body->step();
            task.timing.stepCompleted(runtime::Instant{esp_timer_get_time() - stepStart});
            // Fed only after a completed step: a step that never returns stops the feed.
            if (task.watched.load(std::memory_order_acquire)) {
                if (task.releaseRequested.load(std::memory_order_acquire)) {
                    (void)esp_task_wdt_delete(nullptr);
                    task.watched.store(false, std::memory_order_release);
                } else {
                    (void)esp_task_wdt_reset();
                }
            }
            // Absolute-time wake-ups; an overrun resynchronizes instead of bursting.
            if (xTaskDelayUntil(&lastWake, task.periodTicks) == pdFALSE) {
                lastWake = xTaskGetTickCount();
                task.timing.overrun();
            } else {
                task.timing.woke(esp_timer_get_time(), static_cast<std::uint32_t>(lastWake), kTickUs);
            }
        }
    }

    EspCriticalSection section_{};
    bool sectionCreated_{false};
    std::array<Task, kMaxPeriodicTasks> tasks_{};
    std::size_t started_{0};
};

// The drive output is drive::BoundedDriveOutput over CodeCellMotorDriver (#160).

// The configuration source is provisioning::ProvisionedConfiguration over
// EspProvisioningAdapters.hpp (#159).
} // namespace codecell::esp
