// Host tests of #164 local diagnostics: the bounded log ring and its byte budget against fake
// congested sinks, task wake-lateness estimation, and the report lines. Each line is formatted
// with worst-case values to prove it fits without truncation, and its keys are checked against
// an allowlist so no credential-like field can appear unnoticed.
#include "ProvisioningFakes.hpp"
#include "diagnostics/LogRing.hpp"
#include "diagnostics/StatusReport.hpp"
#include "provisioning/MaintenanceConsole.hpp"
#include "runtime/TaskTiming.hpp"
#include <gtest/gtest.h>
#include <algorithm>
#include <cstdint>
#include <limits>
#include <set>
#include <sstream>
#include <string>
#include <vector>

using namespace codecell;
using namespace codecell::diagnostics;
using namespace std::chrono_literals;

namespace {
constexpr std::uint32_t kMax32 = std::numeric_limits<std::uint32_t>::max();

class FakeSink final : public ILogSink {
public:
    std::size_t room{std::numeric_limits<std::size_t>::max()};  // writable() answer.
    std::size_t takeAtMost{std::numeric_limits<std::size_t>::max()};
    std::string received;
    int calls{0};
    std::size_t writable() noexcept override { return room; }
    std::size_t write(std::span<const char> bytes) noexcept override
    {
        ++calls;
        const auto n = std::min({bytes.size(), takeAtMost, room});
        if (room != std::numeric_limits<std::size_t>::max()) room -= n;
        received.append(bytes.data(), n);
        return n;
    }
};

std::set<std::string> keysOf(std::string_view line)
{
    std::set<std::string> keys;
    std::istringstream words{std::string{line}};
    std::string word;
    words >> word;  // The line kind.
    while (words >> word) {
        const auto eq = word.find('=');
        EXPECT_NE(eq, std::string::npos) << "every field is key=value: " << word;
        keys.insert(word.substr(0, eq));
    }
    return keys;
}

void expectBoundedWithKeys(const ReportLine& line, std::string_view kind, const std::set<std::string>& allowed)
{
    EXPECT_FALSE(line.truncated) << line.view();
    EXPECT_LE(line.length, kReportLineMax);
    EXPECT_EQ(line.view().substr(0, kind.size() + 1), std::string{kind} + " ") << line.view();
    for (const auto& key : keysOf(line.view())) EXPECT_TRUE(allowed.count(key)) << kind << " has unexpected key " << key;
    for (const char* forbidden : {"key", "password", "secret", "token", "psk", "ssid", "ltk", "irk"}) {
        std::string lower{line.view()};
        std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) { return std::tolower(c); });
        EXPECT_EQ(lower.find(forbidden), std::string::npos) << kind << " mentions " << forbidden;
    }
}
} // namespace

// ---- Log ring, budget and drain --------------------------------------------------------

TEST(LogRingTest, KeepsWholeLinesDropsTheNewestThatDoesNotFitAndCountsIt)
{
    LogRing ring;
    const std::string line(99, 'x');  // 100 bytes with the terminator.
    for (int i = 0; i < 20; ++i) EXPECT_TRUE(ring.push(line));
    EXPECT_EQ(ring.size(), 2000U);
    EXPECT_FALSE(ring.push(line));  // 48 bytes left: dropped whole, never cut.
    EXPECT_TRUE(ring.push(std::string(47, 'y')));
    EXPECT_EQ(ring.size(), LogRing::kCapacity);
    EXPECT_FALSE(ring.push(""));
    EXPECT_EQ(ring.stats().droppedLines, 2U);
    EXPECT_EQ(ring.stats().droppedBytes, 101U);
    EXPECT_EQ(ring.stats().highWater, LogRing::kCapacity);
    EXPECT_FALSE(ring.push(std::string(LogRing::kCapacity, 'z')));  // Longer than the ring.
}

TEST(LogRingTest, WrapsAroundAndDrainsInOrder)
{
    LogRing ring;
    ByteBudget budget{1'000'000, 1'000'000};
    FakeSink sink;
    std::string expected;
    for (int i = 0; i < 500; ++i) {
        const std::string line = "LINE " + std::to_string(i) + std::string(static_cast<std::size_t>(i % 37), '.');
        ASSERT_TRUE(ring.push(line));
        expected += line + "\n";
        sink.takeAtMost = static_cast<std::size_t>(1 + i % 60);  // Partial writes as a slow port takes them.
        (void)drainLog(ring, budget, sink, Instant{i * 1000});
    }
    sink.takeAtMost = std::numeric_limits<std::size_t>::max();
    while (!ring.empty()) (void)drainLog(ring, budget, sink, 1s);
    EXPECT_EQ(sink.received, expected);
    EXPECT_EQ(ring.stats().writtenBytes, expected.size());
}

TEST(LogRingTest, ACongestedOrAbsentHostNeverBlocksAndOnlyDropsCountedLines)
{
    LogRing ring;
    ByteBudget budget{timing::kLogBytesPerSecondMax, kLogBurstBytes};
    FakeSink sink;
    sink.room = 0;  // No USB host, or one that stopped reading.
    for (int i = 0; i < 1000; ++i) {
        (void)ring.push("RUNTIME uptime_ms=" + std::to_string(i) + " state=Idle");
        EXPECT_EQ(drainLog(ring, budget, sink, Instant{i * 100'000}), 0U);
    }
    EXPECT_EQ(sink.calls, 0);
    EXPECT_LE(ring.size(), LogRing::kCapacity);
    EXPECT_GT(ring.stats().droppedLines, 900U);
    EXPECT_EQ(ring.stats().lines + ring.stats().droppedLines, 1000U);
    sink.room = 64;  // The host comes back: at most what the port takes per pass.
    EXPECT_EQ(drainLog(ring, budget, sink, 200s), 64U);
}

TEST(LogRingTest, DrainedAverageStaysWithinTheB13Budget)
{
    LogRing ring;
    ByteBudget budget{timing::kLogBytesPerSecondMax, kLogBurstBytes};
    FakeSink sink;
    const std::string line(199, 'r');
    std::size_t written = 0;
    // A producer far above the budget for 20 s, drained every 100 ms as loopTask does.
    for (int pass = 0; pass <= 200; ++pass) {
        while (ring.push(line)) {}
        written += drainLog(ring, budget, sink, Instant{pass * 100'000});
    }
    // One second of burst plus 2048 B/s for 20 s.
    EXPECT_LE(written, kLogBurstBytes + 20U * timing::kLogBytesPerSecondMax);
    EXPECT_GE(written, 20U * timing::kLogBytesPerSecondMax);
}

TEST(LogRingTest, ByteBudgetCarriesFractionsAndDoesNotRefillOnAStillClock)
{
    ByteBudget budget{2048, 2048};
    EXPECT_EQ(budget.available(10s), 2048U);
    budget.spend(2048);
    EXPECT_EQ(budget.available(10s), 0U);
    std::size_t total = 0;
    for (int i = 1; i <= 1000; ++i) {  // 1000 steps of 1 ms: 2.048 bytes each.
        total += budget.available(10s + Instant{i * 1000});
        budget.spend(budget.available(10s + Instant{i * 1000}));
    }
    EXPECT_EQ(total, 2048U);
    EXPECT_EQ(budget.available(5s), 0U);  // A clock that goes back adds nothing.
}

// ---- Task timing -----------------------------------------------------------------------

TEST(TaskTimingTest, LatenessIsMeasuredFromTheEarliestPhaseOfTheTickGrid)
{
    runtime::TaskTiming timing;
    constexpr std::int64_t kTickUs = 1000;
    constexpr std::uint32_t kPeriodTicks = 5;
    timing.woke(1'000'300, 1000, kTickUs);  // Start: 300 us into tick 1000.
    timing.woke(1'005'100, 1000 + kPeriodTicks, kTickUs);  // Earlier phase found: lateness 0.
    timing.woke(1'010'900, 1000 + 2 * kPeriodTicks, kTickUs);  // 800 us late.
    timing.woke(1'015'150, 1000 + 3 * kPeriodTicks, kTickUs);  // 50 us late.
    timing.stepCompleted(Instant{42});
    timing.stepCompleted(Instant{7});
    timing.overrun();
    auto window = timing.takeWindow();
    EXPECT_EQ(window.latenessMaxUs, 800U);
    EXPECT_EQ(window.latenessWorstUs, 800U);
    EXPECT_EQ(window.stepMaxUs, 42U);
    EXPECT_EQ(window.steps, 2U);
    EXPECT_EQ(window.overruns, 1U);
    timing.woke(1'020'120, 1000 + 4 * kPeriodTicks, kTickUs);
    window = timing.takeWindow();
    EXPECT_EQ(window.latenessMaxUs, 20U);   // A fresh window...
    EXPECT_EQ(window.latenessWorstUs, 800U);  // ...but the worst since boot is kept.
    EXPECT_EQ(window.stepMaxUs, 0U);
}

TEST(TaskTimingTest, TickCounterWrapDoesNotDisturbTheGrid)
{
    runtime::TaskTiming timing;
    const std::uint32_t nearWrap = kMax32 - 2;
    timing.woke(5'000'000, nearWrap, 1000);
    timing.woke(5'005'010, nearWrap + 5, 1000);  // Wrapped tick count.
    EXPECT_EQ(timing.takeWindow().latenessMaxUs, 10U);
}

// ---- Report lines ----------------------------------------------------------------------

TEST(StatusReportTest, EveryLineFitsWithWorstCaseValuesAndCarriesOnlyItsKeys)
{
    runtime::DiagnosticsSnapshot runtimeCounters;
    runtimeCounters.counts.fill(kMax32);
    runtime::RuntimeStatus status{runtime::MotionState::EmergencyStop, runtime::FaultCode::DriveConfigurationRejected,
                                  true, true, true, true, true, true};
    const auto runtime = runtimeLine({kMax32, true, status, -255, -255, "interrupt_watchdog", "ReadUnavailable",
                                      kMax32, runtimeCounters});
    expectBoundedWithKeys(runtime, "RUNTIME",
                          {"uptime_ms", "start", "ready", "state", "output", "duty_m1", "duty_m2", "fault", "maintenance",
                           "session", "expired_flag", "reset", "provisioning", "robot_id", "accepted", "rejected",
                           "expired", "overwrites", "estops", "sessions_opened", "sessions_ended", "overloads",
                           "stale_blocked"});

    ble::LinkStatus link;
    link.state = ble::LinkState::AdvertisingFailed;
    link.mtu = 65535;
    link.linkRequest = ble::LinkRequestOutcome::Requested;
    link.negotiated = ble::LinkParameters{65535, 65535, 65535};
    link.lastDisconnectReason = std::numeric_limits<int>::min();
    link.lastWriteBeforeDisconnect = std::chrono::milliseconds{std::numeric_limits<std::int32_t>::max()};
    ble::BleDiagnosticsSnapshot bleCounters;
    bleCounters.counts.fill(kMax32);
    expectBoundedWithKeys(bleLine(link, bleCounters), "BLE",
                          {"state", "cccd_result", "cccd_telemetry", "mtu", "link_request", "negotiated", "latency",
                           "timeout_ms", "in_request", "last_disconnect", "since_write_ms", "connections",
                           "rejected_centrals", "sessions", "results", "result_failures", "unanswered", "estop_writes",
                           "foreign", "early_cccd", "adv_failures", "terminate_failures", "security",
                           "security_sessions", "security_refusals"});

    TelemetryReport telemetry;
    telemetry.session = kMax32;
    telemetry.period = std::chrono::milliseconds{65535};
    telemetry.ticksCoalesced = kMax32;
    telemetry.counters.counts.fill(kMax32);
    telemetry.window = {kMax32, kMax32, kMax32};
    telemetry.battery = {telemetry::BatteryQuality::NotValidated, 65534, true};
    telemetry.batteryRawMillivolts = 65535;
    telemetry.batteryReadFailures = kMax32;
    const std::set<std::string> telemetryKeys{
        "session",  "period_ms",          "task",          "ticks",           "not_ready",      "coalesced",
        "sent",     "session_start",      "state_change",  "periodic",        "notify_failures", "encode_failures",
        "gap_min_us", "gap_max_us",       "periodic_dev_max_us", "battery_mv", "battery_quality", "battery_low",
        "battery_sensor", "battery_raw_mv", "battery_read_failures", "velocity"};
    expectBoundedWithKeys(telemetryLine(telemetry), "TELEMETRY", telemetryKeys);
    expectBoundedWithKeys(telemetryLine(TelemetryReport{}), "TELEMETRY", telemetryKeys);

    TaskReport task{"nimble_host_with_a_long_name", 25, std::chrono::milliseconds{kMax32}, kMax32, kMax32,
                    runtime::TaskTimingSnapshot{kMax32, kMax32, kMax32, kMax32, kMax32, kMax32}};
    const std::set<std::string> taskKeys{"name", "prio", "period_ms", "steps", "overruns", "late_max_us", "late_worst_us",
                                         "step_max_us", "step_worst_us", "stack_free", "stack_size", "stack_free_pct"};
    expectBoundedWithKeys(taskLine(task), "TASK", taskKeys);
    expectBoundedWithKeys(taskLine(TaskReport{}), "TASK", taskKeys);

    LogStats log{kMax32, kMax32, kMax32, kMax32, LogRing::kCapacity};
    expectBoundedWithKeys(resourcesLine({0xFFFFFFFFU, 0xFFFFFFFFU, 0xFFFFFFFFU}, log), "RESOURCES",
                          {"heap_free", "heap_min_free", "heap_largest", "log_lines", "log_dropped_lines",
                           "log_dropped_bytes", "log_written", "log_high_water", "log_capacity"});

    const auto identity = identityLine({kMax32, {255, 255, 255}, "0123456789abcdef", "power_glitch", kMax32, kMax32});
    expectBoundedWithKeys(identity, "IDENTITY", {"robot_id", "firmware", "build", "profile", "reset", "session", "uptime_ms"});
    EXPECT_NE(identity.view().find("robot_id=4294967295 firmware=255.255.255 build=0123456789abcdef profile=restricted-lab-v1"),
              std::string::npos);
}

TEST(StatusReportTest, UnknownValuesAreSpelledOutNotInvented)
{
    const auto t = telemetryLine(TelemetryReport{});
    for (const char* field : {"battery_mv=unknown", "battery_quality=not_validated", "battery_low=0", "battery_sensor=none",
                              "battery_raw_mv=none", "velocity=unknown", "gap_max_us=none", "task=stopped"})
        EXPECT_NE(t.view().find(field), std::string::npos) << field;
    EXPECT_NE(identityLine({}).view().find("robot_id=none"), std::string::npos);
    EXPECT_NE(runtimeLine({}).view().find("robot_id=none"), std::string::npos);
    EXPECT_NE(taskLine({}).view().find("stack_free=unknown"), std::string::npos);
}

TEST(StatusReportTest, OverlongContentIsTruncatedSafelyNotOverrun)
{
    ReportLine line;
    const std::string huge(2 * kReportLineMax, 'h');
    appendf(line, "%s", huge.c_str());
    EXPECT_TRUE(line.truncated);
    EXPECT_EQ(line.length, kReportLineMax);
    EXPECT_EQ(line.text[kReportLineMax], '\0');
    appendf(line, "more");  // Ignored once truncated.
    EXPECT_EQ(line.length, kReportLineMax);
}

TEST(StatusReportTest, EveryMaintenanceReplyFitsTheLogRingBesideTheReporterLines)
{
    using namespace provisioning;
    using namespace provisioning::fakes;
    FakeRecordStorage storage;
    ConfigStore store{storage};
    FakeMaintenanceTarget target;
    target.storage = &storage;
    RecordingOutput output;
    MaintenanceConsole console{store, target, output};
    std::size_t largest = 0;
    const auto measure = [&](const std::string& command) {
        output.lines.clear();
        console.handleLine(command);
        std::size_t bytes = 0;
        for (const auto& l : output.lines) bytes += l.size() + 1;
        largest = std::max(largest, bytes);
    };
    for (const char* command : {"help", "status", "draft", "bogus", "maintenance", "draft"}) measure(command);
    for (const auto& info : kConfigFields) measure("set " + std::string{info.name} + " 1");
    measure("draft");
    measure("status");
    // The reporter adds a line only once the ring has drained (src/main.cpp), so the ring holds
    // at most that report line and one status-change line when a reply arrives.
    EXPECT_LE(kReportBackpressureBytes + 2 * (kReportLineMax + 1) + largest, LogRing::kCapacity) << largest;
}
