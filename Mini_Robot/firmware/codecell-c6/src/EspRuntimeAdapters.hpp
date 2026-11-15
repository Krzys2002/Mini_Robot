#pragma once
// ESP-IDF/FreeRTOS adapters for the #158 runtime ports. This file and main.cpp are the
// only production code that may name SDK, RTOS or board APIs; the portable runtime in
// include/runtime/ is host-tested with fakes.
#include "TimingBudget.hpp"
#include "runtime/RuntimePorts.hpp"
#include <algorithm>
#include <array>
#include <cstddef>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

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

// The production image needs exactly the supervisor and the control task.
inline constexpr std::size_t kMaxPeriodicTasks = 2;

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
        auto& task = tasks_[started_];
        task.body = &body;
        task.periodTicks = static_cast<TickType_t>(timing::ticksAtLeast(spec.period));
        // The stack is allocated here, during initialization only (B15). Checked result.
        if (xTaskCreate(&FreeRtosPlatform::run, spec.name, static_cast<std::uint32_t>(spec.stackBytes), &task,
                        static_cast<UBaseType_t>(spec.priority), &task.handle)
            != pdPASS)
            return false;
        ++started_;
        return true;
    }

private:
    struct Task {
        runtime::IPeriodicStep* body{nullptr};
        TickType_t periodTicks{0};
        TaskHandle_t handle{nullptr};
    };

    static void run(void* context)
    {
        auto& task = *static_cast<Task*>(context);
        TickType_t lastWake = xTaskGetTickCount();
        for (;;) {
            task.body->step();
            // Absolute-time wake-ups; an overrun resynchronizes instead of bursting.
            if (xTaskDelayUntil(&lastWake, task.periodTicks) == pdFALSE) lastWake = xTaskGetTickCount();
        }
    }

    EspCriticalSection section_{};
    bool sectionCreated_{false};
    std::array<Task, kMaxPeriodicTasks> tasks_{};
    std::size_t started_{0};
};

// Holds the drive disabled. The CodeCell/DriveCell actuator contract is not validated
// yet (#160, powered checks #238), so this image configures no drive pin and refuses
// every enable; a refused write faults the runtime. The electrical pin state during
// reset and boot is not established by this adapter (#238).
class InhibitedDriveOutput final : public runtime::IDriveOutput {
public:
    bool configure(const drive::DriveProfile&, std::chrono::milliseconds) noexcept override { return true; }
    bool disable() noexcept override { return true; }
    bool command(const runtime::BodyVelocity&, runtime::Instant) noexcept override { return false; }
    runtime::StopRampStep rampToStop(runtime::Instant) noexcept override { return runtime::StopRampStep::AtRest; }
};

// The configuration source is provisioning::ProvisionedConfiguration over
// EspProvisioningAdapters.hpp (#159).
} // namespace codecell::esp
