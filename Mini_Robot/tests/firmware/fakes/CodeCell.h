#pragma once
#include <cstdint>
#include <vector>
struct FakeDrive {
    struct Output { bool forward; std::uint8_t speed; };
    std::vector<Output> outputs;
    void Drive(bool direction, std::uint8_t speed) { outputs.push_back({direction, speed}); }
};
struct CodeCell {
    FakeDrive Drive1, Drive2;
    void Run(int) {}
    void Motion_RotationRead(float& roll, float& pitch, float& yaw) { roll = pitch = yaw = 0; }
    void Motion_LinearAccRead(float& x, float& y, float& z) { x = y = z = 0; }
    std::uint16_t BatteryVoltageRead() { return 3700; }
    std::uint16_t BatteryLevelRead() { return 75; }
};
