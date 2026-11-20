#pragma once
// Battery voltage for v1 telemetry (#164): which reading may be published, and when the
// batteryLow flag may be set. Portable C++20; the sensor is an injected port.
//
// A voltage is published only with approved evidence (BatteryEvidence), meaning a conversion
// compared against a reference meter on this board and pack, with its plausible range, low
// threshold and sample age. Without evidence every reading stays diagnostic: telemetry
// carries the v1 "unknown" sentinel (0xFFFF) and batteryLow is never set. The production
// image wires the CodeCell GPIO4 sensor with provisional bench evidence (#309,
// telemetry/CodeCellBattery.hpp): operator-approved bring-up bounds, not yet compared with a
// reference meter (hardware record #154, physical check #238).
//
// With evidence, a reading is published only while it is fresh (age <= maxSampleAge), inside
// the plausible range and representable (not the sentinel). Otherwise the quality says why
// and the value is unknown. batteryLow is set only from a valid reading at or below
// lowEnterMillivolts, and cleared at or above lowExitMillivolts (hysteresis; equal values
// mean none). An unknown reading never sets it, and clears it. No protective action is
// attached to the flag: the firmware has no verified low-battery protection.
//
// Threading: record() has one writer, the sampling context (the telemetry task). assess()
// may run in any context, including the BLE host, and reads two lock-free 32-bit atomics:
// the packed assessment, then its time. The writer stores the time last, so a torn read can
// only make a reading look older (and then unknown), never fresher.
#include "runtime/RuntimeTypes.hpp"
#include <atomic>
#include <chrono>
#include <cstdint>
#include <optional>

namespace codecell::telemetry {

using runtime::Instant;

// Approved battery evidence. Verified values must come from the reviewed hardware record
// (#238). The only instance, kProvisionalCodeCellBatteryEvidence, is a provisional bench
// approval (#309) that partly uses vendor values and is not that verified evidence.
struct BatteryEvidence {
    std::uint16_t minPlausibleMillivolts{0};
    std::uint16_t maxPlausibleMillivolts{0};
    std::uint16_t lowEnterMillivolts{0};
    std::uint16_t lowExitMillivolts{0};  // >= lowEnterMillivolts.
    std::chrono::milliseconds maxSampleAge{0};
};

[[nodiscard]] constexpr bool isUsable(const BatteryEvidence& e) noexcept
{
    return e.minPlausibleMillivolts <= e.maxPlausibleMillivolts && e.lowEnterMillivolts <= e.lowExitMillivolts
        && e.maxSampleAge > std::chrono::milliseconds::zero();
}

// A battery voltage port, read from the sampling context only. nullopt: the read failed.
class IBatterySensor {
public:
    virtual ~IBatterySensor() = default;
    [[nodiscard]] virtual std::optional<std::uint16_t> readMillivolts() noexcept = 0;
};

enum class BatteryQuality : std::uint8_t {
    NotValidated,  // No approved evidence: never published (the production state).
    NoSensor,      // Evidence exists but no sensor is wired.
    NoSample,      // No reading yet.
    ReadFailed,    // The latest read failed.
    Stale,         // The latest reading is older than maxSampleAge.
    Implausible,   // Outside the plausible range, or not representable on the wire.
    Valid,
};

struct BatteryAssessment {
    BatteryQuality quality{BatteryQuality::NotValidated};
    std::optional<std::uint16_t> telemetryMillivolts{};  // Empty: send 0xFFFF.
    bool low{false};
};

// The v1 "unknown" battery value; a reading equal to it cannot be published.
inline constexpr std::uint16_t kUnrepresentableMillivolts = 0xFFFF;

class BatteryMonitor {
public:
    // `evidence`: approved evidence, or nullopt (production). Unusable evidence counts as none.
    // `sensor`: optional; nullptr means no sensor is wired.
    BatteryMonitor(std::optional<BatteryEvidence> evidence, IBatterySensor* sensor) noexcept
        : evidence_(evidence && isUsable(*evidence) ? evidence : std::nullopt), sensor_(sensor)
    {
    }
    BatteryMonitor(const BatteryMonitor&) = delete;
    BatteryMonitor& operator=(const BatteryMonitor&) = delete;

    [[nodiscard]] bool hasSensor() const noexcept { return sensor_ != nullptr; }
    [[nodiscard]] bool validated() const noexcept { return evidence_.has_value(); }

    // Sampling context: read the sensor (if any) and publish the assessment.
    void sample(Instant now) noexcept
    {
        if (!sensor_) return;
        record(sensor_->readMillivolts(), now);
    }

    // Sampling context: one reading (nullopt: the read failed) at `now`.
    void record(std::optional<std::uint16_t> millivolts, Instant now) noexcept
    {
        samples_.fetch_add(1, std::memory_order_relaxed);
        if (!millivolts) readFailures_.fetch_add(1, std::memory_order_relaxed);
        else lastRaw_.store(*millivolts, std::memory_order_relaxed);
        const auto quality = classify(millivolts);
        low_ = quality == BatteryQuality::Valid ? nextLow(*millivolts) : false;
        packed_.store(pack(quality, quality == BatteryQuality::Valid ? *millivolts : 0, low_), std::memory_order_relaxed);
        at_.store(uptimeTag(now), std::memory_order_release);
        hasSample_.store(true, std::memory_order_release);
    }

    // Any context: what telemetry may carry at `now`.
    [[nodiscard]] BatteryAssessment assess(Instant now) const noexcept
    {
        if (!evidence_) return {BatteryQuality::NotValidated, std::nullopt, false};
        if (!sensor_) return {BatteryQuality::NoSensor, std::nullopt, false};
        if (!hasSample_.load(std::memory_order_acquire)) return {BatteryQuality::NoSample, std::nullopt, false};
        const std::uint32_t at = at_.load(std::memory_order_acquire);
        const std::uint32_t packed = packed_.load(std::memory_order_relaxed);
        const auto quality = static_cast<BatteryQuality>((packed >> kQualityShift) & kByteMask);
        if (quality != BatteryQuality::Valid) return {quality, std::nullopt, false};
        // Wrap-safe age in milliseconds; a reading "from the future" (never on one clock) is stale.
        const auto age = static_cast<std::uint32_t>(uptimeTag(now) - at);
        if (age > kMaxRepresentableAge || age > static_cast<std::uint32_t>(evidence_->maxSampleAge.count()))
            return {BatteryQuality::Stale, std::nullopt, false};
        return {BatteryQuality::Valid, static_cast<std::uint16_t>(packed & kMillivoltMask), (packed & kLowBit) != 0};
    }

    // Diagnostics (any context): the last successful raw reading, whatever its quality.
    [[nodiscard]] std::optional<std::uint16_t> lastRawMillivolts() const noexcept
    {
        if (samples_.load(std::memory_order_relaxed) == readFailures_.load(std::memory_order_relaxed)) return std::nullopt;
        return lastRaw_.load(std::memory_order_relaxed);
    }
    [[nodiscard]] std::uint32_t samples() const noexcept { return samples_.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint32_t readFailures() const noexcept { return readFailures_.load(std::memory_order_relaxed); }

private:
    static constexpr std::uint32_t kMillivoltMask = 0xFFFFU;
    static constexpr std::uint32_t kByteMask = 0xFFU;
    static constexpr unsigned kQualityShift = 16;
    static constexpr std::uint32_t kLowBit = 1U << 24;
    // Half the uint32 millisecond range: older (or "newer") tags are treated as stale.
    static constexpr std::uint32_t kMaxRepresentableAge = 0x7FFFFFFFU;

    [[nodiscard]] static constexpr std::uint32_t pack(BatteryQuality quality, std::uint16_t millivolts, bool low) noexcept
    {
        return static_cast<std::uint32_t>(millivolts) | (static_cast<std::uint32_t>(quality) << kQualityShift)
             | (low ? kLowBit : 0U);
    }
    [[nodiscard]] static constexpr std::uint32_t uptimeTag(Instant now) noexcept
    {
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
        return static_cast<std::uint32_t>(static_cast<std::uint64_t>(ms));
    }

    [[nodiscard]] BatteryQuality classify(std::optional<std::uint16_t> millivolts) const noexcept
    {
        if (!evidence_) return BatteryQuality::NotValidated;
        if (!millivolts) return BatteryQuality::ReadFailed;
        if (*millivolts == kUnrepresentableMillivolts || *millivolts < evidence_->minPlausibleMillivolts
            || *millivolts > evidence_->maxPlausibleMillivolts)
            return BatteryQuality::Implausible;
        return BatteryQuality::Valid;
    }

    // Writer context only. Hysteresis between the enter and exit thresholds.
    [[nodiscard]] bool nextLow(std::uint16_t millivolts) const noexcept
    {
        if (millivolts <= evidence_->lowEnterMillivolts) return true;
        if (millivolts >= evidence_->lowExitMillivolts) return false;
        return low_;
    }

    const std::optional<BatteryEvidence> evidence_;
    IBatterySensor* const sensor_;
    bool low_{false};  // Writer context only.
    std::atomic<std::uint32_t> packed_{pack(BatteryQuality::NoSample, 0, false)};
    std::atomic<std::uint32_t> at_{0};
    std::atomic<bool> hasSample_{false};
    std::atomic<std::uint16_t> lastRaw_{0};
    std::atomic<std::uint32_t> samples_{0};
    std::atomic<std::uint32_t> readFailures_{0};
};

[[nodiscard]] constexpr const char* toString(BatteryQuality quality) noexcept
{
    switch (quality) {
    case BatteryQuality::NotValidated: return "not_validated";
    case BatteryQuality::NoSensor: return "no_sensor";
    case BatteryQuality::NoSample: return "no_sample";
    case BatteryQuality::ReadFailed: return "read_failed";
    case BatteryQuality::Stale: return "stale";
    case BatteryQuality::Implausible: return "implausible";
    case BatteryQuality::Valid: return "valid";
    }
    return "unknown";
}
} // namespace codecell::telemetry
