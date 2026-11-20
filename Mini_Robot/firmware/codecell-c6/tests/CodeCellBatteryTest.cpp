// Host tests of the CodeCell battery conversion and the provisional bench evidence (#309).
// The conversion is the pinned CodeCell 2.0.4 formula (adc_mV * 2 + 65); these tests pin that
// arithmetic and how the production BatteryMonitor configuration treats typical LiPo readings.
// They do not establish the accuracy of the reading on the physical board (#154/#238).
#include "telemetry/BatteryMonitor.hpp"
#include "telemetry/CodeCellBattery.hpp"
#include <gtest/gtest.h>
#include <cstdint>
#include <optional>

using namespace codecell;
using namespace codecell::telemetry;
using namespace std::chrono_literals;

namespace {
class FakeAdcSensor final : public IBatterySensor {
public:
    std::uint32_t adcMillivolts{0};
    std::optional<std::uint16_t> readMillivolts() noexcept override { return codeCellBatteryMillivolts(adcMillivolts); }
};
} // namespace

TEST(CodeCellBatteryTest, ConversionMatchesTheCodeCellLibraryFormula)
{
    EXPECT_EQ(codeCellBatteryMillivolts(0), 65);
    EXPECT_EQ(codeCellBatteryMillivolts(1000), 2065);
    EXPECT_EQ(codeCellBatteryMillivolts(1918), 3901);
    EXPECT_EQ(codeCellBatteryMillivolts(2067), 4199);
}

TEST(CodeCellBatteryTest, ConversionNeverWrapsAndOverflowIsUnrepresentable)
{
    constexpr std::uint32_t kLargestRepresentable = (kUnrepresentableMillivolts - kCodeCellBatteryOffsetMillivolts - 1)
        / kCodeCellBatteryDividerRatio;
    EXPECT_LT(codeCellBatteryMillivolts(kLargestRepresentable), kUnrepresentableMillivolts);
    EXPECT_EQ(codeCellBatteryMillivolts(kLargestRepresentable + 1), kUnrepresentableMillivolts);
    EXPECT_EQ(codeCellBatteryMillivolts(0xFFFFFFFFU), kUnrepresentableMillivolts);
}

TEST(CodeCellBatteryTest, ProvisionalEvidenceIsUsableAndKeepsItsDocumentedBounds)
{
    constexpr auto e = kProvisionalCodeCellBatteryEvidence;
    static_assert(isUsable(e));
    EXPECT_EQ(e.minPlausibleMillivolts, 3000);
    EXPECT_EQ(e.maxPlausibleMillivolts, 4400);
    EXPECT_EQ(e.lowEnterMillivolts, 3350);
    EXPECT_EQ(e.lowExitMillivolts, 3450);
    EXPECT_EQ(e.maxSampleAge, 500ms);
}

TEST(CodeCellBatteryTest, ProductionConfigurationPublishesAChargedCellWithoutTheLowFlag)
{
    FakeAdcSensor sensor;
    BatteryMonitor monitor{kProvisionalCodeCellBatteryEvidence, &sensor};
    ASSERT_TRUE(monitor.validated());
    sensor.adcMillivolts = 1918;  // 3901 mV.
    monitor.sample(1s);
    const auto a = monitor.assess(1s);
    EXPECT_EQ(a.quality, BatteryQuality::Valid);
    EXPECT_EQ(a.telemetryMillivolts, std::optional<std::uint16_t>{3901});
    EXPECT_FALSE(a.low);
}

TEST(CodeCellBatteryTest, ProductionConfigurationSetsAndClearsTheLowFlagWithHysteresis)
{
    FakeAdcSensor sensor;
    BatteryMonitor monitor{kProvisionalCodeCellBatteryEvidence, &sensor};
    sensor.adcMillivolts = 1640;  // 3345 mV: below 3350.
    monitor.sample(1s);
    EXPECT_TRUE(monitor.assess(1s).low);
    sensor.adcMillivolts = 1680;  // 3425 mV: inside the hysteresis band, stays low.
    monitor.sample(1100ms);
    EXPECT_TRUE(monitor.assess(1100ms).low);
    sensor.adcMillivolts = 1700;  // 3465 mV: above 3450, clears.
    monitor.sample(1200ms);
    EXPECT_FALSE(monitor.assess(1200ms).low);
}

TEST(CodeCellBatteryTest, ProductionConfigurationWithholdsImplausibleAndStaleReadings)
{
    FakeAdcSensor sensor;
    BatteryMonitor monitor{kProvisionalCodeCellBatteryEvidence, &sensor};
    sensor.adcMillivolts = 0;  // 65 mV: nothing on the divider.
    monitor.sample(1s);
    EXPECT_EQ(monitor.assess(1s).quality, BatteryQuality::Implausible);
    EXPECT_FALSE(monitor.assess(1s).telemetryMillivolts);
    EXPECT_EQ(monitor.lastRawMillivolts(), std::optional<std::uint16_t>{65});  // Still diagnostic.
    sensor.adcMillivolts = 2400;  // 4865 mV: above the plausible range.
    monitor.sample(2s);
    EXPECT_EQ(monitor.assess(2s).quality, BatteryQuality::Implausible);
    sensor.adcMillivolts = 1918;
    monitor.sample(3s);
    EXPECT_EQ(monitor.assess(3s + 500ms).quality, BatteryQuality::Valid);
    EXPECT_EQ(monitor.assess(3s + 501ms).quality, BatteryQuality::Stale);
}
