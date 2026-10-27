#pragma once
#include <cstdint>
#include <cstdlib>
#include <cmath>
#include <functional>
#include <memory>
#include <vector>

namespace fake {
inline std::uint32_t now{};
inline bool failAllocation{};
struct Semaphore { bool held{}; };
inline std::vector<std::unique_ptr<Semaphore>> semaphores;
inline std::function<void()> afterTake;
struct EndTask {};
}
using SemaphoreHandle_t = fake::Semaphore*;
inline constexpr int pdTRUE = 1;
inline constexpr int pdFALSE = 0;
inline constexpr int pdPASS = 1;
inline constexpr unsigned portMAX_DELAY = ~0U;
inline unsigned pdMS_TO_TICKS(unsigned ms) { return ms; }
inline std::uint32_t millis() { return fake::now; }
inline void vTaskDelay(unsigned) { throw fake::EndTask{}; }
inline SemaphoreHandle_t xSemaphoreCreateMutex() {
    if (fake::failAllocation) return nullptr;
    fake::semaphores.push_back(std::make_unique<fake::Semaphore>());
    return fake::semaphores.back().get();
}
inline int xSemaphoreTake(SemaphoreHandle_t s, unsigned wait) {
    // A timing-critical path must never block in these actual adapter tests.
    if (wait != 0) std::abort();
    if (!s || s->held) return pdFALSE;
    s->held = true;
    if (fake::afterTake) { auto callback = std::move(fake::afterTake); fake::afterTake = {}; callback(); }
    return pdTRUE;
}
inline void xSemaphoreGive(SemaphoreHandle_t s) { s->held = false; }
struct FakeSerial {
    template<class T> void print(const T&) {}
    template<class T> void println(const T&) {}
};
inline FakeSerial Serial;
#define constrain(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))
using std::abs;
