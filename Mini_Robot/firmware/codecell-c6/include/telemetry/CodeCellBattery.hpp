#pragma once
// CodeCell C6 battery voltage: the vendor conversion and the provisional bench evidence that
// lets telemetry publish it (#309). Portable C++20; the ADC read itself is the board adapter
// in src/CodeCellBatterySensor.hpp.
//
// Conversion (source fact, not a measurement): the pinned CodeCell 2.0.4 library
// (BatteryVoltageRead, commit 795616b) reads the battery node on GPIO4 through a 1:2 divider
// and reports `adc_mV * 2 + 65`. The vendor schematic shows the BAT net at the module's IO4.
//
// Evidence (provisional): the operator approved publishing the battery reading on the bench
// on 2026-11-20 before a reference-meter comparison exists. Only the `battery-bench`
// PlatformIO image (CODECELL_PROVISIONAL_BATTERY) uses it; the default image reports unknown
// and never reads the ADC (src/main.cpp). These values are therefore bench
// bring-up bounds, NOT the multimeter-verified evidence that BatteryMonitor.hpp describes;
// the paired API-versus-meter samples of the hardware record (#154/#238) are still open.
// - Low thresholds: CodeCell 2.0.4 `MIN_BATTERY_VOLTAGE` (3350 mV) to enter, with 100 mV
//   of hysteresis to leave. The flag is informational: neither the firmware nor the desktop
//   takes a protective action on it.
// - Plausible range: a single-cell LiPo between a 3.0 V discharge floor and 4.4 V, which
//   covers the 4.2 V charge voltage and the library's 4190 mV USB indication with margin.
//   Readings outside it stay diagnostic (`Implausible`) and telemetry sends "unknown".
// - Sample age: five telemetry periods, so one or two late samples do not blank the value.
#include "TimingBudget.hpp"
#include "telemetry/BatteryMonitor.hpp"
#include <cstdint>
#include <optional>

namespace codecell::telemetry {

inline constexpr std::uint32_t kCodeCellBatteryDividerRatio = 2;
inline constexpr std::uint32_t kCodeCellBatteryOffsetMillivolts = 65;

inline constexpr std::uint16_t kProvisionalBatteryMinPlausibleMillivolts = 3000;
inline constexpr std::uint16_t kProvisionalBatteryMaxPlausibleMillivolts = 4400;
inline constexpr std::uint16_t kProvisionalBatteryLowEnterMillivolts = 3350;  // CodeCell MIN_BATTERY_VOLTAGE.
inline constexpr std::uint16_t kProvisionalBatteryLowHysteresisMillivolts = 100;
inline constexpr std::uint32_t kProvisionalBatterySampleAgePeriods = 5;

inline constexpr BatteryEvidence kProvisionalCodeCellBatteryEvidence{
    kProvisionalBatteryMinPlausibleMillivolts,
    kProvisionalBatteryMaxPlausibleMillivolts,
    kProvisionalBatteryLowEnterMillivolts,
    static_cast<std::uint16_t>(kProvisionalBatteryLowEnterMillivolts + kProvisionalBatteryLowHysteresisMillivolts),
    timing::kTelemetryPeriod * kProvisionalBatterySampleAgePeriods,
};
static_assert(isUsable(kProvisionalCodeCellBatteryEvidence));

// Battery millivolts from the ADC pin millivolts, as CodeCell 2.0.4 computes them. A result
// that does not fit the wire is reported as the unrepresentable sentinel, which the monitor
// classifies as Implausible (never published) rather than as a failed read.
[[nodiscard]] constexpr std::uint16_t codeCellBatteryMillivolts(std::uint32_t adcMillivolts) noexcept
{
    constexpr std::uint32_t kMaxAdcMillivolts =
        (kUnrepresentableMillivolts - kCodeCellBatteryOffsetMillivolts) / kCodeCellBatteryDividerRatio;
    if (adcMillivolts > kMaxAdcMillivolts) return kUnrepresentableMillivolts;
    return static_cast<std::uint16_t>(adcMillivolts * kCodeCellBatteryDividerRatio + kCodeCellBatteryOffsetMillivolts);
}

// One reading from the value analogReadMilliVolts() returned. Arduino-ESP32 3.3.7 returns 0
// when ADC initialization, calibration or the conversion fails, so 0 is a failed read
// (nullopt, counted by BatteryMonitor::readFailures), never the 65 mV the formula would give.
// A genuine 0 mV pin reading cannot be told apart from a failure and is reported the same way.
[[nodiscard]] constexpr std::optional<std::uint16_t> codeCellBatteryReading(std::uint32_t adcMillivolts) noexcept
{
    if (adcMillivolts == 0) return std::nullopt;
    return codeCellBatteryMillivolts(adcMillivolts);
}

} // namespace codecell::telemetry
