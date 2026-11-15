#pragma once
// Domain values shared by the production runtime boundaries (#158). Portable C++20:
// no Arduino, ESP-IDF, FreeRTOS, CodeCell or radio types may appear here.
#include <chrono>
#include <cmath>
#include <cstdint>

namespace codecell::runtime {
// Monotonic time since boot on the robot clock (esp_timer in production). Never desktop time.
using Instant = std::chrono::microseconds;
using std::chrono::microseconds;
using std::chrono::milliseconds;

// Supervisory body-frame setpoint in SI units: metres per second and radians per
// second (positive = counter-clockwise seen from above). Range limits belong to the
// codec (#161) and the actuator contract (#160); this type only carries the value.
struct BodyVelocity {
    double linearMetresPerSecond{0.0};
    double angularRadiansPerSecond{0.0};

    [[nodiscard]] bool isFinite() const noexcept
    {
        return std::isfinite(linearMetresPerSecond) && std::isfinite(angularRadiansPerSecond);
    }
};

// Values match the v1 Robot states (docs/protocol/robot-protocol.md, "Robot states").
enum class MotionState : std::uint8_t { Idle = 0, Moving = 1, EmergencyStop = 2, Fault = 3 };

// First fault that moved the runtime to Fault. Every fault leaves the drive disabled.
enum class FaultCode : std::uint8_t {
    None = 0,
    DriveDisableFailed,          // The drive adapter could not confirm a disable.
    ConfigurationUnavailable,    // The configuration source returned nothing.
    ConfigurationInvalid,        // The configuration failed RuntimeConfig validation.
    SynchronizationUnavailable,  // The platform could not create the handoff critical section.
    SupervisorTaskStartFailed,
    ControlTaskStartFailed,
    DriveWriteFailed,            // The drive adapter refused a permitted write.
    ControlStalled,              // No control update within the stall limit while the drive was on.
};

// Answer of the single motion-command authority. Mapping to v1 result codes is #161/#163.
enum class CommandOutcome : std::uint8_t {
    Accepted,
    RejectedNotReady,              // Startup has not completed (or never will: see FaultCode).
    RejectedEmergencyStopLatched,
    RejectedFault,
    RejectedInvalid,               // Non-finite velocity or a non-positive validity window.
};

// Why a BLE session ended; every reason stops motion as for Stop and keeps the E-stop latch.
enum class SessionEndReason : std::uint8_t {
    Disconnected,
    NotificationsDisabled,
    ResultQueueExhausted,  // #156 overload policy: a CommandResult could not be queued.
};

[[nodiscard]] constexpr const char* toString(MotionState state) noexcept
{
    switch (state) {
    case MotionState::Idle: return "Idle";
    case MotionState::Moving: return "Moving";
    case MotionState::EmergencyStop: return "EmergencyStop";
    case MotionState::Fault: return "Fault";
    }
    return "Unknown";
}

[[nodiscard]] constexpr const char* toString(FaultCode fault) noexcept
{
    switch (fault) {
    case FaultCode::None: return "None";
    case FaultCode::DriveDisableFailed: return "DriveDisableFailed";
    case FaultCode::ConfigurationUnavailable: return "ConfigurationUnavailable";
    case FaultCode::ConfigurationInvalid: return "ConfigurationInvalid";
    case FaultCode::SynchronizationUnavailable: return "SynchronizationUnavailable";
    case FaultCode::SupervisorTaskStartFailed: return "SupervisorTaskStartFailed";
    case FaultCode::ControlTaskStartFailed: return "ControlTaskStartFailed";
    case FaultCode::DriveWriteFailed: return "DriveWriteFailed";
    case FaultCode::ControlStalled: return "ControlStalled";
    }
    return "Unknown";
}
} // namespace codecell::runtime
