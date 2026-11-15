#pragma once
// Production runtime orchestration (#158). It wires the injected clock, platform, drive,
// configuration and optional sensor to one ActuatorGate (local safety and actuator
// authority), one CommandProcessor (the only motion-command authority), the setpoint
// mailbox and the two periodic tasks.
//
// start() order, each step checked; any failure leaves the runtime in Fault with the
// drive disabled, reports the first FaultCode and never starts normal motion:
//   1. disable the drive;          2. load and validate the configuration;
//   3. create the critical section; 4. start the safety supervisor, then the control task;
//   5. mark ready (commands are rejected with RejectedNotReady before this).
// A supervisor that already runs when a later step fails keeps the outputs off.
#include "runtime/ActuatorGate.hpp"
#include "runtime/CommandProcessor.hpp"
#include "runtime/RuntimeConfig.hpp"
#include "runtime/RuntimeDiagnostics.hpp"
#include "runtime/RuntimePorts.hpp"
#include "runtime/RuntimeTasks.hpp"
#include "runtime/SetpointMailbox.hpp"
#include <optional>

namespace codecell::runtime {

inline constexpr const char* kSafetyTaskName = "mr_safety";
inline constexpr const char* kControlTaskName = "mr_control";

struct RuntimeDependencies {
    const IFirmwareClock& clock;
    IPlatform& platform;
    IDriveOutput& drive;
    IConfigurationSource& configuration;
    IInertialSensor* inertialSensor{nullptr};  // Optional; nullptr means no sensor.
};

enum class StartResult : std::uint8_t { Running, Faulted };

class FirmwareRuntime {
public:
    explicit FirmwareRuntime(const RuntimeDependencies& dependencies) noexcept
        : clock_(dependencies.clock),
          platform_(dependencies.platform),
          configuration_(dependencies.configuration),
          gate_(dependencies.drive, diagnostics_),
          commands_(gate_, mailbox_, diagnostics_),
          supervisor_(gate_, dependencies.clock),
          control_(gate_, mailbox_, dependencies.clock, dependencies.inertialSensor, diagnostics_)
    {
    }
    FirmwareRuntime(const FirmwareRuntime&) = delete;
    FirmwareRuntime& operator=(const FirmwareRuntime&) = delete;

    // Call once from setup(). Later calls return the first result and do nothing.
    StartResult start() noexcept
    {
        if (result_) return *result_;
        result_ = startOnce();
        return *result_;
    }

    // The single motion-command authority, for the BLE adapter (#163).
    [[nodiscard]] IMotionCommandSink& commands() noexcept { return commands_; }
    [[nodiscard]] RuntimeStatus status() const noexcept { return gate_.status(); }
    // Local maintenance path (#159): stop, clear intent, disable the drive and refuse
    // motion until reset. Callable in any state, including Fault and before start().
    [[nodiscard]] bool enterMaintenance() noexcept { return gate_.enterMaintenance(); }
    [[nodiscard]] DiagnosticsSnapshot diagnostics() const noexcept { return diagnostics_.snapshot(); }
    [[nodiscard]] const std::optional<RuntimeConfig>& config() const noexcept { return config_; }
    [[nodiscard]] std::optional<ConfigError> configError() const noexcept { return configError_; }

    // The steps the platform schedules; exposed so host tests can drive them directly.
    [[nodiscard]] IPeriodicStep& safetyStep() noexcept { return supervisor_; }
    [[nodiscard]] IPeriodicStep& controlStep() noexcept { return control_; }

private:
    StartResult fail(FaultCode code) noexcept
    {
        gate_.fault(code);
        return StartResult::Faulted;
    }

    StartResult startOnce() noexcept
    {
        if (!gate_.disableAtBoot()) return fail(FaultCode::DriveDisableFailed);

        const auto loaded = configuration_.load();
        if (!loaded) return fail(FaultCode::ConfigurationUnavailable);
        if (const auto error = validate(*loaded); error != ConfigError::None) {
            configError_ = error;
            return fail(FaultCode::ConfigurationInvalid);
        }
        config_ = loaded;
        commands_.setValidityCeiling(config_->commandValidityCeiling);
        supervisor_.setStallLimit(config_->controlStallLimit);

        ICriticalSection* section = platform_.createCriticalSection();
        if (!section) return fail(FaultCode::SynchronizationUnavailable);
        gate_.bind(*section);
        mailbox_.bind(*section);

        const TaskSpec safety{kSafetyTaskName, config_->safetyTask.priority, config_->safetyTask.stackBytes,
                              config_->safetyCheckPeriod};
        if (!platform_.startPeriodicTask(safety, supervisor_)) return fail(FaultCode::SupervisorTaskStartFailed);
        const TaskSpec control{kControlTaskName, config_->controlTask.priority, config_->controlTask.stackBytes,
                               config_->controlPeriod};
        if (!platform_.startPeriodicTask(control, control_)) return fail(FaultCode::ControlTaskStartFailed);

        gate_.markReady(clock_.nowMicroseconds());
        return gate_.status().ready ? StartResult::Running : StartResult::Faulted;
    }

    const IFirmwareClock& clock_;
    IPlatform& platform_;
    IConfigurationSource& configuration_;
    RuntimeDiagnostics diagnostics_{};
    SetpointMailbox mailbox_{};
    ActuatorGate gate_;
    CommandProcessor commands_;
    SafetySupervisor supervisor_;
    ControlLoop control_;
    std::optional<RuntimeConfig> config_{};
    std::optional<ConfigError> configError_{};
    std::optional<StartResult> result_{};
};
} // namespace codecell::runtime
