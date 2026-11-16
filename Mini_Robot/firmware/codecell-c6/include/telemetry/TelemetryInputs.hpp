#pragma once
// The physical measurements a v1 Telemetry frame carries (#164), in wire units. Each one is
// either a real measurement with documented availability or unknown; nothing is inferred.
//
// | Field | Unit | Availability on the CodeCell C6 Drive |
// | --- | --- | --- |
// | batteryMillivolts | 1 mV | Unknown (0xFFFF): no approved conversion; see BatteryMonitor |
// | batteryLow flag | — | Never set: no verified threshold |
// | measuredLinear | 1 mm/s | Unknown (0x8000): no wheel encoders or speed sensors |
// | measuredAngular | 1 mrad/s | Unknown (0x8000): the IMU is not wired (#158) and its axes are unverified (#238) |
//
// A commanded setpoint, a PWM duty, an assumed wheel speed or an integrated IMU acceleration
// is not a measurement of body velocity, and is never reported as one. The desktop's
// WorldModel (external vision) stays the only source of measured motion.
#include "telemetry/BatteryMonitor.hpp"
#include <cstdint>
#include <optional>

namespace codecell::telemetry {

struct TelemetryMeasurements {
    std::optional<std::uint16_t> batteryMillivolts{};  // Empty: 0xFFFF.
    bool batteryLow{false};
    std::optional<std::int16_t> measuredLinearMillimetresPerSecond{};    // Empty: 0x8000.
    std::optional<std::int16_t> measuredAngularMilliradiansPerSecond{};  // Empty: 0x8000.
};

// Read in the BLE host context once per frame: must be non-blocking and allocation-free.
class ITelemetryInputs {
public:
    virtual ~ITelemetryInputs() = default;
    [[nodiscard]] virtual TelemetryMeasurements measurements(Instant now) const noexcept = 0;
};

// The production inputs: the battery policy, and no measured body velocity.
class RobotTelemetryInputs final : public ITelemetryInputs {
public:
    explicit RobotTelemetryInputs(const BatteryMonitor& battery) noexcept : battery_(battery) {}

    [[nodiscard]] TelemetryMeasurements measurements(Instant now) const noexcept override
    {
        const auto battery = battery_.assess(now);
        // No velocity source exists on this board (see the table above): always unknown.
        return {battery.telemetryMillivolts, battery.low, std::nullopt, std::nullopt};
    }

private:
    const BatteryMonitor& battery_;
};
} // namespace codecell::telemetry
