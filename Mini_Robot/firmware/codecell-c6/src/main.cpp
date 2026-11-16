// Build skeleton only: no motor initialization, command input, BLE or HTTP server.
#include <Arduino.h>
#include <esp_timer.h>
#include <chrono>
#include "FirmwareClock.hpp"

namespace {
// Matches `monitor_speed` in platformio.ini.
constexpr unsigned long kSerialBaudRate = 115200;
constexpr codecell::Milliseconds kIdleLoopPeriod{10};

// The only SDK-specific part of the clock: everything else is host-tested.
class EspFirmwareClock final : public codecell::IFirmwareClock {
public:
    codecell::Milliseconds now() const override
    {
        // esp_timer_get_time() is monotonic microseconds since boot and never negative.
        return codecell::monotonicMilliseconds(esp_timer_get_time())
            .value_or(codecell::Milliseconds::zero());
    }
};
EspFirmwareClock firmwareClock;
}
void setup()
{
    Serial.begin(kSerialBaudRate);
    Serial.println("CodeCell firmware build skeleton; control unavailable");
}
void loop()
{
    // Compile the hardware clock adapter without publishing synthetic telemetry.
    (void)firmwareClock.now();
    delay(static_cast<std::uint32_t>(kIdleLoopPeriod.count()));
}
