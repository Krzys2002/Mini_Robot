#pragma once
// Deterministic fakes for the #159 provisioning ports. The storage fake can cut power
// after a chosen number of mutating operations and tear the interrupted write at any
// byte, which models the worst case (a partial value persisted) for every step.
#include "provisioning/RobotConfigRecord.hpp"
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace codecell::provisioning::fakes {

// Test values that satisfy validate(). They are NOT CodeCell calibration data.
inline RobotConfig validConfig(RobotId id = 7)
{
    RobotConfig c;
    c.robotId = id;
    c.boardProfile = static_cast<std::uint8_t>(BoardProfile::CodeCellC6Drive);
    c.requiredCapabilities = capability::kDifferentialDrive;
    c.drivePolarity = polarity::kRightReversed;
    c.maxDriveOutputPercent = 40;
    c.wheelDiameterMicrometres = 17'000;
    c.trackWidthMicrometres = 60'000;
    c.maxWheelSpeedMillimetresPerSecond = 200;
    c.maxLinearMillimetresPerSecond = 150;
    c.maxAngularMilliradiansPerSecond = 3'000;
    c.maxCommandValidityMilliseconds = 200;
    c.telemetryPeriodMilliseconds = static_cast<std::uint16_t>(timing::kTelemetryPeriod.count());
    return c;
}

} // namespace codecell::provisioning::fakes
