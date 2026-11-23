#pragma once
// Production runtime orchestration (#158). It wires the injected clock, platform, drive,
// configuration and optional sensor to one ActuatorGate (local safety and actuator
// authority), one CommandProcessor (the only motion-command authority), the setpoint
// mailbox and the two periodic tasks.
//
// start() order, each step checked; any failure leaves the runtime in Fault with the
// drive disabled, reports the first FaultCode and never starts normal motion:
//   First bind synchronization; if unavailable, no task (including reporting) starts.
//   1. register the pre-restart shutdown hook (shutdown() inside esp_restart(), #162), so
//      that from here on a deliberate restart powers the drive down first, even when a
//      later step fails;
//   2. disable the drive, then require the shutdown hook and critical section;
//   3. load and validate the configuration;
//   4. configure the drive with the validated profile (#160);
//   5. start the safety supervisor (subscribed to the task
//      watchdog, B17), then the control task, then the telemetry task when one is given
//      (#164 review: mandatory telemetry is part of the checked start-up, so a robot whose
//      telemetry cannot run never becomes motion-ready);
//   6. mark ready (commands are rejected with RejectedNotReady before this).
// When an earlier step fails, start() still starts the telemetry task, so a faulted robot
// keeps reporting Fault to a connected central. That task never touches the drive.
// A supervisor that already runs when a later step fails keeps the outputs off.
//
// Recovery (#162): faults are sticky and keep the drive disabled and powered down; only a
// reset clears them. Every boot, including after a task-watchdog reset or a brownout, starts
// here with nothing restored: Idle, outputs off, no session, no setpoint, E-stop latch
// cleared (it is RAM-only in v1).
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

// A periodic task the runtime starts as part of its checked start-up.
struct PeriodicTask {
    TaskSpec spec;
    IPeriodicStep* step{nullptr};
};

struct RuntimeDependencies {
    const IFirmwareClock& clock;
    IPlatform& platform;
    IDriveOutput& drive;
    IConfigurationSource& configuration;
    IInertialSensor* inertialSensor{nullptr};  // Optional; nullptr means no sensor.
    // Production: the #164 telemetry task. When given, failing to start it is a start-up
    // fault (TelemetryTaskStartFailed).
    std::optional<PeriodicTask> telemetryTask{};
};

enum class StartResult : std::uint8_t { Running, Faulted };

class FirmwareRuntime {
public:
    explicit FirmwareRuntime(const RuntimeDependencies& dependencies) noexcept
        : clock_(dependencies.clock),
          platform_(dependencies.platform),
          configuration_(dependencies.configuration),
          telemetryTask_(dependencies.telemetryTask),
          gate_(dependencies.drive, diagnostics_, dependencies.clock),
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
        if (*result_ == StartResult::Faulted) startTelemetryForReporting();
        return *result_;
    }

    // The single motion-command authority, for the BLE adapter (#163).
    [[nodiscard]] IMotionCommandSink& commands() noexcept { return commands_; }
    [[nodiscard]] RuntimeStatus status() const noexcept { return gate_.status(); }
    // Local maintenance path (#159): stop, clear intent, disable the drive and refuse
    // motion until reset. Callable in any state, including Fault and before start(). Once
    // the drive is confirmed off, the task watchdog is released (IPlatform), because the
    // maintenance flash writes stall the chip and there is no motion left to supervise.
    [[nodiscard]] bool enterMaintenance() noexcept
    {
        if (!gate_.enterMaintenance()) return false;
        platform_.releaseTaskWatchdog();
        return true;
    }
    // Before a deliberate restart (production: the ESP-IDF shutdown handler run by
    // esp_restart()): end motion and the session, disable and power down the drive.
    void shutdown() noexcept { gate_.shutdown(); }
    [[nodiscard]] DiagnosticsSnapshot diagnostics() const noexcept { return diagnostics_.snapshot(); }
    [[nodiscard]] const std::optional<RuntimeConfig>& config() const noexcept { return config_; }
    [[nodiscard]] std::optional<ConfigError> configError() const noexcept { return configError_; }
    // Whether start() registered the pre-restart shutdown hook (boot report).
    [[nodiscard]] bool shutdownHookRegistered() const noexcept { return shutdownHookRegistered_; }
    // Whether start() started the telemetry task (boot and TELEMETRY report).
    [[nodiscard]] bool telemetryTaskStarted() const noexcept { return telemetryTaskStarted_; }

    // The steps the platform schedules; exposed so host tests can drive them directly.
    [[nodiscard]] IPeriodicStep& safetyStep() noexcept { return supervisor_; }
    [[nodiscard]] IPeriodicStep& controlStep() noexcept { return control_; }

private:
    // After a start-up fault: telemetry only reports the state (Fault), so it may run; its own
    // failure changes nothing, because the runtime is already faulted.
    void startTelemetryForReporting() noexcept
    {
        if (!synchronizationReady_ || !telemetryTask_ || telemetryTaskStarted_ || !telemetryTask_->step) return;
        if (gate_.status().fault == FaultCode::TelemetryTaskStartFailed) return;
        telemetryTaskStarted_ = platform_.startPeriodicTask(telemetryTask_->spec, *telemetryTask_->step);
    }

    StartResult fail(FaultCode code) noexcept
    {
        gate_.fault(code);
        return StartResult::Faulted;
    }

    StartResult startOnce() noexcept
    {
        // Bind before any fallible initialization can start fault-reporting telemetry.
        ICriticalSection* section = platform_.createCriticalSection();
        if (section) {
            gate_.bind(*section);
            mailbox_.bind(*section);
            synchronizationReady_ = true;
        }
        shutdownHookRegistered_ = platform_.registerShutdownHook(shutdownHook_);
        if (!gate_.disableAtBoot()) return fail(FaultCode::DriveDisableFailed);
        if (!shutdownHookRegistered_) return fail(FaultCode::ShutdownHookUnavailable);
        if (!section) return fail(FaultCode::SynchronizationUnavailable);

        const auto loaded = configuration_.load();
        if (!loaded) return fail(FaultCode::ConfigurationUnavailable);
        if (const auto error = validate(*loaded); error != ConfigError::None) {
            configError_ = error;
            return fail(FaultCode::ConfigurationInvalid);
        }
        config_ = loaded;
        if (!gate_.configureDrive(config_->driveProfile, config_->controlPeriod))
            return fail(FaultCode::DriveConfigurationRejected);
        commands_.setValidityCeiling(config_->commandValidityCeiling);
        commands_.setVelocityLimits(config_->driveProfile.maxLinearMetresPerSecond,
                                    config_->driveProfile.maxAngularRadiansPerSecond);
        supervisor_.setStallLimit(config_->controlStallLimit);

        const TaskSpec safety{kSafetyTaskName, config_->safetyTask.priority, config_->safetyTask.stackBytes,
                              config_->safetyCheckPeriod, true};
        if (!platform_.startPeriodicTask(safety, supervisor_)) return fail(FaultCode::SupervisorTaskStartFailed);
        const TaskSpec control{kControlTaskName, config_->controlTask.priority, config_->controlTask.stackBytes,
                               config_->controlPeriod};
        if (!platform_.startPeriodicTask(control, control_)) return fail(FaultCode::ControlTaskStartFailed);
        if (telemetryTask_) {
            if (!telemetryTask_->step || !platform_.startPeriodicTask(telemetryTask_->spec, *telemetryTask_->step))
                return fail(FaultCode::TelemetryTaskStartFailed);
            telemetryTaskStarted_ = true;
        }

        gate_.markReady(clock_.nowMicroseconds());
        return gate_.status().ready ? StartResult::Running : StartResult::Faulted;
    }

    class ShutdownHook final : public IShutdownHook {
    public:
        explicit ShutdownHook(FirmwareRuntime& runtime) noexcept : runtime_(runtime) {}
        void beforeRestart() noexcept override { runtime_.shutdown(); }

    private:
        FirmwareRuntime& runtime_;
    };

    const IFirmwareClock& clock_;
    IPlatform& platform_;
    IConfigurationSource& configuration_;
    std::optional<PeriodicTask> telemetryTask_;
    RuntimeDiagnostics diagnostics_{};
    SetpointMailbox mailbox_{};
    ActuatorGate gate_;
    CommandProcessor commands_;
    SafetySupervisor supervisor_;
    ControlLoop control_;
    ShutdownHook shutdownHook_{*this};
    bool shutdownHookRegistered_{false};
    bool telemetryTaskStarted_{false};
    bool synchronizationReady_{false};
    std::optional<RuntimeConfig> config_{};
    std::optional<ConfigError> configError_{};
    std::optional<StartResult> result_{};
};
} // namespace codecell::runtime
