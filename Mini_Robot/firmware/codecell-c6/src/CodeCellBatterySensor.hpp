#pragma once
// CodeCell C6 battery voltage port (#309): the only production code that reads the battery
// ADC. The conversion and the provisional evidence are portable and host-tested
// (include/telemetry/CodeCellBattery.hpp).
//
// Board fact, from the pinned CodeCell 2.0.4 library (BatteryVoltageRead, commit 795616b):
// the battery node is divided 1:2 onto GPIO4 and read with analogReadMilliVolts, which
// applies the chip's eFuse ADC calibration. The firmware does not initialize the CodeCell
// library; this adapter reproduces only that one read. One raw sample per call, unaveraged,
// so a published value carries the ADC noise of a single conversion.
//
// Threading: called only from the telemetry task (BatteryMonitor::sample), the sole ADC user.
#include "telemetry/BatteryMonitor.hpp"
#include "telemetry/CodeCellBattery.hpp"
#include <Arduino.h>
#include <cstdint>
#include <optional>

namespace codecell::esp {

class CodeCellBatterySensor final : public telemetry::IBatterySensor {
public:
    static constexpr std::uint8_t kBatteryAdcPin = 4;  // GPIO4, the BAT divider tap.

    [[nodiscard]] std::optional<std::uint16_t> readMillivolts() noexcept override
    {
        return telemetry::codeCellBatteryMillivolts(analogReadMilliVolts(kBatteryAdcPin));
    }
};

} // namespace codecell::esp
