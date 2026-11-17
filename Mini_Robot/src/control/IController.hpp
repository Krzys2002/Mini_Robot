#pragma once

#include "transport/TransportTypes.hpp"
#include "world/RobotState.hpp"

namespace micro_robot {

struct ControlInput {
    RobotState robot; // Latest measured/filtered pose, never retimestamped or predicted.
    TrackingInfo tracking;
    Pose2D target;
    Timestamp cycleAt;
};

// Replaceable supervisory policy. Called on the service's owning thread: prompt,
// bounded, no blocking I/O, GUI access, transport calls or service reentrancy.
// Return no value when a valid setpoint cannot be computed. Exceptions inhibit
// the loop. Goal/path control policies are separate issues.
class IController {
public:
    virtual ~IController() = default;
    [[nodiscard]] virtual std::optional<VelocityCommand> calculate(const ControlInput& input) = 0;
};

} // namespace micro_robot
