// Build skeleton only: no motor initialization, command input, BLE or HTTP server.
#include <Arduino.h>
#include <esp_timer.h>
#include "FirmwareClock.hpp"

namespace {
class EspFirmwareClock final : public codecell::IFirmwareClock {
public:
    codecell::Milliseconds now() const override
    {
        return std::chrono::duration_cast<codecell::Milliseconds>(
            std::chrono::microseconds{esp_timer_get_time()});
    }
};
EspFirmwareClock firmwareClock;
}
void setup()
{
    Serial.begin(115200);
    Serial.println("CodeCell firmware build skeleton; control unavailable");
}
void loop()
{
    // Compile the hardware clock adapter without publishing synthetic telemetry.
    (void)firmwareClock.now();
    delay(10);
}
