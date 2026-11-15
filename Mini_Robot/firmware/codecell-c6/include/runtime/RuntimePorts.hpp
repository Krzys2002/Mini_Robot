#pragma once
// Injectable hardware/platform boundaries of the production runtime (#158). Production
// adapters live in src/ and are the only code allowed to call Arduino, ESP-IDF,
// FreeRTOS, CodeCell/DriveCell or radio APIs. Host tests substitute fakes.
#include "FirmwareClock.hpp"
#include "drive/DriveProfile.hpp"
#include "runtime/RuntimeConfig.hpp"
#include "runtime/RuntimeTypes.hpp"
#include <cstddef>
#include <optional>

namespace codecell::runtime {

// Progress of one normal stop-ramp step (#160, #156 B6).
enum class StopRampStep : std::uint8_t {
    Ramping,  // The commanded output moved toward zero and is not zero yet.
    AtRest,   // The commanded output is zero on every wheel.
    Refused,  // The write failed; the runtime faults and disables.
};

// Drive outputs. Only ActuatorGate holds this reference, and apart from configure() it
// calls every function inside its critical section, so implementations must be short
// non-blocking writes: no allocation, lock, wait, logging or radio call. The bounded
// CodeCell implementation (#160) is drive::BoundedDriveOutput: body-to-wheel conversion,
// saturation, rate and reversal limits and the actuator map live behind this boundary.
class IDriveOutput {
public:
    virtual ~IDriveOutput() = default;
    // Applies the validated drive profile once during start-up, before any task runs and
    // before the runtime is ready. False means the adapter refused it; the runtime faults.
    [[nodiscard]] virtual bool configure(const drive::DriveProfile& profile,
                                         std::chrono::milliseconds controlPeriod) noexcept = 0;
    // Switch the drive outputs off immediately (no ramp) and forget any ramp state.
    // Idempotent. False means the adapter could not confirm the disable; the runtime then
    // reports a fault. E-stop, faults, maintenance and boot use only this path.
    [[nodiscard]] virtual bool disable() noexcept = 0;
    // One bounded, rate-limited step toward the given body velocity at `now`. False means
    // the write was refused.
    [[nodiscard]] virtual bool command(const BodyVelocity& velocity, Instant now) noexcept = 0;
    // One step of the normal stop ramp toward zero at `now` (Stop, expiry, session end).
    [[nodiscard]] virtual StopRampStep rampToStop(Instant now) noexcept = 0;
};

// Optional local inertial measurement (for example the CodeCell IMU). It is read by the
// control task only, stays on the robot and is never a pose: the desktop WorldModel
// remains the authoritative world pose. A missing or failing sensor never blocks motion
// commands or safety; a blocked read is caught by the control stall limit (B7).
struct InertialSample {
    Instant at{};
    double yawRateRadiansPerSecond{0.0};
};
class IInertialSensor {
public:
    virtual ~IInertialSensor() = default;
    [[nodiscard]] virtual std::optional<InertialSample> read() noexcept = 0;
};

// Short, bounded mutual exclusion for the handoffs (critical-section copy measured at
// <= 9 us by the #156 pilot). It never waits indefinitely on a lock held by a blocked
// task: holders only copy a few words and perform non-blocking drive writes.
class ICriticalSection {
public:
    virtual ~ICriticalSection() = default;
    virtual void enter() noexcept = 0;
    virtual void exit() noexcept = 0;
};
// Scoped enter/exit. A null section is allowed only while the runtime is still
// single-threaded (before FirmwareRuntime::start() has bound one and started any task).
class CriticalSectionGuard {
public:
    explicit CriticalSectionGuard(ICriticalSection* section) noexcept : section_(section)
    {
        if (section_) section_->enter();
    }
    ~CriticalSectionGuard()
    {
        if (section_) section_->exit();
    }
    CriticalSectionGuard(const CriticalSectionGuard&) = delete;
    CriticalSectionGuard& operator=(const CriticalSectionGuard&) = delete;

private:
    ICriticalSection* section_;
};

// One periodic activity. The platform calls step() once per period from its own task.
class IPeriodicStep {
public:
    virtual ~IPeriodicStep() = default;
    virtual void step() noexcept = 0;
};

struct TaskSpec {
    const char* name{""};
    int priority{0};
    std::size_t stackBytes{0};
    std::chrono::milliseconds period{};
};

// Scheduler/RTOS boundary. Every creation is checked: a failure is returned, never
// ignored or retried silently, and the runtime then stays in Fault with the drive off.
class IPlatform {
public:
    virtual ~IPlatform() = default;
    // The platform owns the returned object for the program lifetime; nullptr on failure.
    [[nodiscard]] virtual ICriticalSection* createCriticalSection() noexcept = 0;
    // Starts a task that calls `body.step()` every `spec.period` (absolute-time wake-ups,
    // waits rounded up to whole ticks). `body` must outlive the task. False on failure.
    [[nodiscard]] virtual bool startPeriodicTask(const TaskSpec& spec, IPeriodicStep& body) noexcept = 0;
};

// Configuration source: the validated provisioned record (#159). nullopt means no usable
// configuration; the runtime then stays in Fault with the drive off.
class IConfigurationSource {
public:
    virtual ~IConfigurationSource() = default;
    [[nodiscard]] virtual std::optional<RuntimeConfig> load() noexcept = 0;
};

// BLE adapter boundary: the single motion-command authority. The BLE peripheral (#163)
// passes each write through protocol::v1::CommandSession (#161), stamps receipt at
// callback entry on the robot clock and calls at most one of these per frame. No other code
// may command motion.
//
// Session lifecycle (#162): openSession() when the session becomes ready (both CCCDs
// enabled), sessionEnded() when it stops being ready. Velocity and ClearEmergencyStop
// carry the token; any token other than the current one is refused with
// RejectedSessionNotReady, so a frame from an ended or replaced session never moves the
// robot. Stop and EmergencyStop need no token: they take effect before readiness too (v1).
class IMotionCommandSink {
public:
    virtual ~IMotionCommandSink() = default;
    // The session is ready. Ends any previous session as for Stop (replacement) and returns
    // the token of the new one. The E-stop latch and faults are kept.
    [[nodiscard]] virtual SessionToken openSession() noexcept = 0;
    [[nodiscard]] virtual CommandOutcome velocity(SessionToken session, const BodyVelocity& velocity,
                                                  std::chrono::milliseconds validFor, Instant receipt) noexcept = 0;
    [[nodiscard]] virtual CommandOutcome stop() noexcept = 0;
    // Disables the drive before returning. Never rejected (v1).
    [[nodiscard]] virtual CommandOutcome emergencyStop() noexcept = 0;
    [[nodiscard]] virtual CommandOutcome clearEmergencyStop(SessionToken session) noexcept = 0;
    // Disconnect, notification disable or result overload of that session: stop as for Stop,
    // keep the latch. A token that is no longer current is ignored.
    virtual void sessionEnded(SessionToken session, SessionEndReason reason) noexcept = 0;
};
} // namespace codecell::runtime
