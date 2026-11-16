#pragma once
// Runtime scheduling configuration (#158). Defaults come from the #156 budget table in
// TimingBudget.hpp; validation rejects any value that would break those bounds. A
// configuration that fails validation never starts a task and never arms the drive.
// Persistent provisioning (#159) supplies commandValidityCeiling and the drive profile
// (#160) from the validated record (provisioning/ProvisionedConfiguration.hpp); the other
// values stay budget defaults. The default drive profile is invalid on purpose, so a
// default RuntimeConfig schedules correctly but can never arm the outputs.
#include "TimingBudget.hpp"
#include "drive/DriveProfile.hpp"
#include <cstddef>
#include <cstdint>

namespace codecell::runtime {
// Pilot task stack (pilot/TimingPilot.cpp). Production high-water marks are
// unmeasured; B16 (>= 25 % free) is checked on the bench, not assumed.
inline constexpr std::size_t kDefaultTaskStackBytes = 4096;

struct TaskConfig {
    int priority{0};
    std::size_t stackBytes{kDefaultTaskStackBytes};
};

struct RuntimeConfig {
    std::chrono::milliseconds safetyCheckPeriod{timing::kSafetyCheckPeriod};
    std::chrono::milliseconds controlPeriod{timing::kControlPeriod};
    std::chrono::milliseconds controlStallLimit{timing::kControlStallLimit};
    std::chrono::milliseconds commandValidityCeiling{timing::kCommandValidityCeiling};
    TaskConfig safetyTask{timing::kSafetyPriority, kDefaultTaskStackBytes};
    TaskConfig controlTask{timing::kControlPriority, kDefaultTaskStackBytes};
    drive::DriveProfile driveProfile{};  // Geometry, actuator map, output bound and body limits (#160).
};

enum class ConfigError : std::uint8_t {
    None = 0,
    PeriodNotPositive,
    PeriodNotWholeTicks,
    PeriodAboveBudget,          // Slower than the #156 period, so B4/B7 no longer compose.
    StallLimitOutOfRange,       // Must be within [control period, budget stall limit].
    ValidityCeilingOutOfRange,  // Must be within [1 ms, budget ceiling].
    PriorityOrderViolated,      // B18: BLE controller > safety > nimble_host > control > log drain.
    StackSizeZero,
    DriveProfileInvalid,        // drive::validate() rejected the profile (#160).
};

// The scheduling rules alone (#158): periods, stall limit, ceiling, priorities, stacks.
[[nodiscard]] constexpr ConfigError validateScheduling(const RuntimeConfig& config) noexcept
{
    using std::chrono::milliseconds;
    for (const auto period : {config.safetyCheckPeriod, config.controlPeriod}) {
        if (period <= milliseconds::zero()) return ConfigError::PeriodNotPositive;
        if (period % timing::kSchedulerTick != milliseconds::zero()) return ConfigError::PeriodNotWholeTicks;
    }
    if (config.safetyCheckPeriod > timing::kSafetyCheckPeriod || config.controlPeriod > timing::kControlPeriod)
        return ConfigError::PeriodAboveBudget;
    if (config.controlStallLimit < config.controlPeriod || config.controlStallLimit > timing::kControlStallLimit)
        return ConfigError::StallLimitOutOfRange;
    if (config.commandValidityCeiling < milliseconds{1}
        || config.commandValidityCeiling > timing::kCommandValidityCeiling)
        return ConfigError::ValidityCeilingOutOfRange;
    const int safety = config.safetyTask.priority;
    const int control = config.controlTask.priority;
    if (!(timing::kBleControllerPriority > safety && safety > timing::kNimbleHostPriority
          && timing::kNimbleHostPriority > control && control > timing::kLogDrainPriority))
        return ConfigError::PriorityOrderViolated;
    if (config.safetyTask.stackBytes == 0 || config.controlTask.stackBytes == 0) return ConfigError::StackSizeZero;
    return ConfigError::None;
}

// Everything the runtime needs before it may become ready.
[[nodiscard]] constexpr ConfigError validate(const RuntimeConfig& config) noexcept
{
    if (const auto error = validateScheduling(config); error != ConfigError::None) return error;
    if (drive::validate(config.driveProfile) != drive::DriveProfileError::None) return ConfigError::DriveProfileInvalid;
    return ConfigError::None;
}

static_assert(validateScheduling(RuntimeConfig{}) == ConfigError::None, "the budget defaults must schedule correctly");
static_assert(validate(RuntimeConfig{}) == ConfigError::DriveProfileInvalid,
              "no drive profile is invented: a default configuration cannot arm the outputs");
} // namespace codecell::runtime
