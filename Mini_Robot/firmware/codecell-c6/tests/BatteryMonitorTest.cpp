// Host tests of the #164 battery policy: without approved evidence (the production state) no
// reading is ever published and batteryLow is never set; with evidence, only fresh, plausible,
// representable readings are, and the low flag follows its thresholds. The evidence values
// below are test fixtures, not CodeCell or battery data.
#include "telemetry/BatteryMonitor.hpp"
#include "telemetry/TelemetryInputs.hpp"
#include <gtest/gtest.h>
#include <cstdint>
#include <limits>
#include <optional>

using namespace codecell;
using namespace codecell::telemetry;
using namespace std::chrono_literals;

namespace {
class FakeBatterySensor final : public IBatterySensor {
public:
    std::optional<std::uint16_t> next{};
    int reads{0};
    std::optional<std::uint16_t> readMillivolts() noexcept override
    {
        ++reads;
        return next;
    }
};

constexpr BatteryEvidence kFixtureEvidence{1000, 5000, 2000, 2200, 500ms};
} // namespace

TEST(BatteryMonitorTest, ProductionStateWithoutEvidencePublishesNothingWhateverItReads)
{
    FakeBatterySensor sensor;
    BatteryMonitor monitor{std::nullopt, &sensor};
    for (const auto reading : {std::optional<std::uint16_t>{3900}, std::optional<std::uint16_t>{1500},
                               std::optional<std::uint16_t>{}}) {
        sensor.next = reading;
        monitor.sample(1s);
        const auto a = monitor.assess(1s);
        EXPECT_EQ(a.quality, BatteryQuality::NotValidated);
        EXPECT_FALSE(a.telemetryMillivolts);
        EXPECT_FALSE(a.low);
    }
    EXPECT_FALSE(monitor.validated());
    EXPECT_EQ(monitor.lastRawMillivolts(), 1500);  // Diagnostics still see the raw value.
    EXPECT_EQ(monitor.readFailures(), 1U);

    // The image as built: no evidence and no sensor; telemetry reports unknown and never low.
    BatteryMonitor production{std::nullopt, nullptr};
    production.sample(1s);
    const RobotTelemetryInputs inputs{production};
    const auto m = inputs.measurements(2s);
    EXPECT_FALSE(m.batteryMillivolts);
    EXPECT_FALSE(m.batteryLow);
    EXPECT_FALSE(m.measuredLinearMillimetresPerSecond);
    EXPECT_FALSE(m.measuredAngularMilliradiansPerSecond);
    EXPECT_EQ(production.samples(), 0U);
}

TEST(BatteryMonitorTest, UnusableEvidenceCountsAsNone)
{
    FakeBatterySensor sensor;
    sensor.next = 3000;
    for (const BatteryEvidence bad : {BatteryEvidence{5000, 1000, 2000, 2200, 500ms},
                                      BatteryEvidence{1000, 5000, 2200, 2000, 500ms},
                                      BatteryEvidence{1000, 5000, 2000, 2200, 0ms}}) {
        BatteryMonitor monitor{bad, &sensor};
        monitor.sample(1s);
        EXPECT_FALSE(monitor.validated());
        EXPECT_EQ(monitor.assess(1s).quality, BatteryQuality::NotValidated);
    }
}

TEST(BatteryMonitorTest, MissingSensorAndMissingOrFailedReadsAreExplicitAndUnknown)
{
    BatteryMonitor noSensor{kFixtureEvidence, nullptr};
    noSensor.sample(1s);
    EXPECT_EQ(noSensor.assess(1s).quality, BatteryQuality::NoSensor);

    FakeBatterySensor sensor;
    BatteryMonitor monitor{kFixtureEvidence, &sensor};
    EXPECT_EQ(monitor.assess(1s).quality, BatteryQuality::NoSample);
    sensor.next = std::nullopt;
    monitor.sample(1s);
    const auto failed = monitor.assess(1s);
    EXPECT_EQ(failed.quality, BatteryQuality::ReadFailed);
    EXPECT_FALSE(failed.telemetryMillivolts);
    EXPECT_FALSE(monitor.lastRawMillivolts());
    EXPECT_EQ(monitor.readFailures(), 1U);
}

TEST(BatteryMonitorTest, OnlyFreshPlausibleRepresentableReadingsArePublished)
{
    FakeBatterySensor sensor;
    BatteryMonitor monitor{kFixtureEvidence, &sensor};
    sensor.next = 3912;
    monitor.sample(1s);
    auto a = monitor.assess(1s + 500ms);  // Exactly at the age limit: still valid.
    EXPECT_EQ(a.quality, BatteryQuality::Valid);
    EXPECT_EQ(a.telemetryMillivolts, 3912);
    EXPECT_EQ(monitor.assess(1s + 501ms).quality, BatteryQuality::Stale);
    EXPECT_FALSE(monitor.assess(1s + 501ms).telemetryMillivolts);

    for (const std::uint16_t implausible : {std::uint16_t{999}, std::uint16_t{5001}, std::uint16_t{0}, kUnrepresentableMillivolts}) {
        sensor.next = implausible;
        monitor.sample(2s);
        a = monitor.assess(2s);
        EXPECT_EQ(a.quality, BatteryQuality::Implausible) << implausible;
        EXPECT_FALSE(a.telemetryMillivolts) << implausible;
    }
    // Even evidence whose range reaches 0xFFFF cannot publish the sentinel.
    BatteryMonitor wide{BatteryEvidence{0, 0xFFFF, 0, 0, 1s}, &sensor};
    sensor.next = kUnrepresentableMillivolts;
    wide.sample(1s);
    EXPECT_EQ(wide.assess(1s).quality, BatteryQuality::Implausible);
}

TEST(BatteryMonitorTest, LowFlagFollowsTheThresholdsWithHysteresisAndNeverFromAnUnknownReading)
{
    FakeBatterySensor sensor;
    BatteryMonitor monitor{kFixtureEvidence, &sensor};
    const auto lowAfter = [&](std::optional<std::uint16_t> mv, Instant at) {
        sensor.next = mv;
        monitor.sample(at);
        return monitor.assess(at).low;
    };
    EXPECT_FALSE(lowAfter(2100, 1s));  // Between the thresholds, not low before.
    EXPECT_TRUE(lowAfter(2000, 2s));   // At the enter threshold.
    EXPECT_TRUE(lowAfter(2100, 3s));   // Stays low inside the band.
    EXPECT_FALSE(lowAfter(2200, 4s));  // Cleared at the exit threshold.
    EXPECT_TRUE(lowAfter(1500, 5s));
    EXPECT_FALSE(lowAfter(std::nullopt, 6s));  // A failed read is unknown, so not low.
    EXPECT_FALSE(lowAfter(2100, 7s));          // And the band does not resurrect it.
    EXPECT_TRUE(lowAfter(1500, 8s));
    EXPECT_FALSE(monitor.assess(9s).low);      // Stale: unknown, not low.
}

TEST(BatteryMonitorTest, SampleAgeIsWrapSafeAcrossTheMillisecondCounter)
{
    FakeBatterySensor sensor;
    BatteryMonitor monitor{kFixtureEvidence, &sensor};
    const Instant beforeWrap{std::chrono::milliseconds{(std::int64_t{1} << 32) - 100}};
    sensor.next = 3000;
    monitor.sample(beforeWrap);
    EXPECT_EQ(monitor.assess(beforeWrap + 300ms).quality, BatteryQuality::Valid);  // Across the wrap.
    EXPECT_EQ(monitor.assess(beforeWrap + 600ms).quality, BatteryQuality::Stale);
}
