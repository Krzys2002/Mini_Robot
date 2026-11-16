#pragma once
#include "FirmwareClock.hpp"
#include <cstddef>
#include <cstdint>

// Timing, resource and stopping budgets for the CodeCell C6 runtime (#156).
// Status: PROPOSED for maintainer approval. The table, derivations, uncertainty
// and pass/fail methods are in Mini_Robot/docs/protocol/timing-budgets.md; the
// bench evidence is Mini_Robot/docs/testing/timing-pilot-156.md. This header
// names the values so #158/#162/#166 reuse them instead of copying literals.
// It contains no scheduler, watchdog or actuator code. Physical stopping
// (wheels stationary) is NOT bounded here: it is unmeasured (#238/#166).
//
// Clock rules: every deadline and interval uses the monotonic esp_timer clock in
// 64-bit microseconds since boot (no practical wrap). FreeRTOS ticks (1 ms) only
// schedule wake-ups. No desktop clock is used or assumed to be synchronized.
namespace codecell::timing {
using std::chrono::milliseconds;
using std::chrono::microseconds;

// ---- Platform facts of the pinned build (pioarduino 55.03.37, Arduino-ESP32 3.3.7) ----
inline constexpr milliseconds kSchedulerTick{1};          // CONFIG_FREERTOS_HZ = 1000
inline constexpr int kBleControllerPriority = 23;         // ble_ll_task, observed by the pilot
inline constexpr int kEspTimerTaskPriority = 22;          // esp_timer, observed by the pilot
inline constexpr int kNimbleHostPriority = 21;            // nimble_host (NimBLE-Arduino 2.3.6), observed
inline constexpr std::size_t kInboundAclBuffers = 24;     // CONFIG_BT_NIMBLE_TRANSPORT_ACL_FROM_LL_COUNT

// ---- Desktop needs (owned and configured by the desktop; mirrored for derivation) ----
inline constexpr milliseconds kDesktopVelocityRefresh{100};  // ManualControlConfig::refreshInterval
inline constexpr int kDesktopTelemetrySilencePeriods = 3;    // robot-protocol.md, freshness rules
// Upper end of the connection interval the robot requests (ble-gatt.md, link parameters).
inline constexpr milliseconds kRequestedMaxConnectionInterval{30};

// ---- Command validity (advertised as Robot Info maxCommandValidityMs) ----
inline constexpr milliseconds kCommandValidityCeiling{250};

// ---- Independent safety supervisor ----
inline constexpr milliseconds kSafetyCheckPeriod{5};
// Wake-up lateness allowance: archived pilot worst 0.801 ms at priority 20 (write flood),
// 0.108 ms at priority 22; a development run reached 1.403 ms at priority 20.
inline constexpr microseconds kSchedulingAllowance{2000};
// Drive-disable write allowance. UNMEASURED: no actuator was touched (#238 gate).
inline constexpr microseconds kOutputDisableAllowance{1000};

// ---- Normal control path ----
inline constexpr milliseconds kControlPeriod{10};
inline constexpr milliseconds kControlStallLimit{3 * kControlPeriod};
// Commanded output ramp to zero for Stop/expiry/disconnect. Physical deceleration is unknown.
inline constexpr milliseconds kStopRampMax{100};
// Stop decision -> commanded output must be zero, otherwise the supervisor disables the drive.
inline constexpr milliseconds kStopEnforcementLimit{kStopRampMax + 2 * kControlPeriod};

// ---- Emergency stop (callback entry is the receipt instrumentation point) ----
inline constexpr microseconds kEstopCallbackToDisableBound{1000};
// Robot-side wait ahead of the E-stop callback: every inbound ACL buffer full of
// ordinary writes, each handled within an allowance above the pilot's worst callback (0.583 ms).
inline constexpr microseconds kWriteCallbackAllowance{800};
inline constexpr microseconds kRobotStackQueueAllowance{static_cast<long long>(kInboundAclBuffers) * kWriteCallbackAllowance};

// ---- Telemetry, results and logging ----
inline constexpr milliseconds kTelemetryPeriod{100};       // advertised telemetryPeriodMs
inline constexpr milliseconds kStateTelemetryMinSpacing{20};
inline constexpr int kTelemetryNotificationsPerSecondMax =
    static_cast<int>(std::chrono::seconds{1} / kTelemetryPeriod + std::chrono::seconds{1} / kStateTelemetryMinSpacing);
inline constexpr int kCommandResultsPerReceivedFrame = 1;  // While the session is ready (v1).
inline constexpr std::size_t kSetpointMailboxDepth = 1;    // Newest accepted setpoint wins.
inline constexpr std::size_t kLogRingBytes = 2048;
inline constexpr std::size_t kLogBytesPerSecondMax = 2048;

// ---- Resources and supervision ----
inline constexpr milliseconds kTaskWatchdogTimeout{50};    // Supervisor subscribed; panic + reset.
inline constexpr int kCpuUtilizationPercentMax = 50;       // Non-idle share under supported stress.
inline constexpr std::size_t kMinFreeHeapBytes = 64 * 1024;
inline constexpr int kMinStackFreePercent = 25;
inline constexpr int kSafetyPriority = 22;
inline constexpr int kControlPriority = 20;
inline constexpr int kTelemetryPriority = 5;
inline constexpr int kLogDrainPriority = 1;

// ---- Composed bounds ----
// A deadline can fall just after a check, so detection waits one period plus lateness.
[[nodiscard]] constexpr microseconds detectionBound(milliseconds period, microseconds lateness)
{
    return period + lateness;
}
inline constexpr microseconds kExpiryDetectionBound = detectionBound(kSafetyCheckPeriod, kSchedulingAllowance);
// Receipt of the last valid Velocity -> drive output zero or electrically disabled, with or
// without the control task: expiry, enforcement window, one more check, then the disable write.
[[nodiscard]] constexpr microseconds lastValidCommandToOutputOff(milliseconds validity)
{
    return validity + kExpiryDetectionBound + kStopEnforcementLimit + kExpiryDetectionBound
        + kOutputDisableAllowance;
}
inline constexpr milliseconds kLastValidCommandToOutputOffBound{400};
inline constexpr milliseconds kStalledControlToOutputOffBound{40};
inline constexpr milliseconds kEstopStackReceiptToDisableBound{25};
inline constexpr milliseconds kReceiptToSetpointBound{2};   // Validated and in the mailbox.
inline constexpr milliseconds kReceiptToActuatorWriteBound{15};

// Whole ticks for a wait of at least `duration`. pdMS_TO_TICKS truncates instead,
// which can shorten a timeout; budgets therefore round waits up.
[[nodiscard]] constexpr std::uint32_t ticksAtLeast(microseconds duration, milliseconds tick = kSchedulerTick)
{
    if (duration <= microseconds::zero()) return 0;
    const auto tickUs = microseconds{tick}.count();
    return static_cast<std::uint32_t>((duration.count() + tickUs - 1) / tickUs);
}

// ---- Consistency of the table (compile-time; pass/fail methods are in the docs) ----
static_assert(kCommandValidityCeiling >= 2 * kDesktopVelocityRefresh + kRequestedMaxConnectionInterval,
              "one lost refresh plus a connection interval must not expire a refreshed command");
static_assert(lastValidCommandToOutputOff(kCommandValidityCeiling) <= kLastValidCommandToOutputOffBound);
static_assert(kControlStallLimit + kExpiryDetectionBound + kOutputDisableAllowance <= kStalledControlToOutputOffBound);
static_assert(kRobotStackQueueAllowance + kEstopCallbackToDisableBound <= kEstopStackReceiptToDisableBound);
static_assert(kReceiptToSetpointBound + kControlPeriod + kSchedulingAllowance <= kReceiptToActuatorWriteBound);
static_assert(kSafetyCheckPeriod % kSchedulerTick == milliseconds::zero(), "periods are whole ticks");
static_assert(kControlPeriod % kSchedulerTick == milliseconds::zero(), "periods are whole ticks");
static_assert(kSchedulingAllowance < microseconds{kSafetyCheckPeriod}, "a check must not overrun its period");
static_assert(kTaskWatchdogTimeout >= 4 * kExpiryDetectionBound, "watchdog must tolerate allowed lateness");
static_assert(kTaskWatchdogTimeout < kCommandValidityCeiling, "a stalled supervisor resets before a ceiling");
static_assert(kTelemetryPeriod * kDesktopTelemetrySilencePeriods > kTelemetryPeriod + 2 * kRequestedMaxConnectionInterval,
              "nominal telemetry jitter must not trip the desktop silence threshold");
static_assert(kBleControllerPriority > kSafetyPriority && kSafetyPriority > kNimbleHostPriority
              && kNimbleHostPriority > kControlPriority && kControlPriority > kTelemetryPriority
              && kTelemetryPriority > kLogDrainPriority, "priority order of the budget");
static_assert(kSafetyPriority == kEspTimerTaskPriority,
              "shares a priority with esp_timer: no application esp_timer callbacks on critical paths");
} // namespace codecell::timing
