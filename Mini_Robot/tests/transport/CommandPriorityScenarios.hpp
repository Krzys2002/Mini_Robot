#pragma once
#include "transport/TransportTypes.hpp"
#include <vector>

namespace micro_robot::test {
struct CommandPriorityScenario {
    const char* name;
    std::vector<RobotCommand> pending;
    bool stop;
    double linear{};
};
// Run these same bursts against simulation and the production BLE adapter,
// before either backend is allowed to dispatch its bounded pending slot.
inline std::vector<CommandPriorityScenario> commandPriorityScenarios(Timestamp at)
{
    using namespace std::chrono_literals;
    const RobotCommand first{VelocityCommand{0.1, 0}, at, 1s};
    const RobotCommand second{VelocityCommand{0.2, 0}, at, 1s};
    const RobotCommand stop{StopCommand{}, at, 1s};
    return {{"latest velocity", {first, second}, false, 0.2},
            {"stop supersedes velocity", {first, stop}, true},
            {"velocity cannot replace stop", {stop, second}, true},
            {"stop retained through burst", {first, stop, second, first}, true},
            {"repeated stop", {stop, stop, second}, true}};
}
} // namespace micro_robot::test
