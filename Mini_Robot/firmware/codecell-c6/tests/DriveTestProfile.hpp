#pragma once
// A valid drive profile shared by the #160 host tests. Test values only: they are not
// measurements of the robot and are not approved limits (#238).
#include "drive/DriveProfile.hpp"

namespace codecell::drive::fixtures {
inline constexpr DriveProfile kTestDriveProfile{
    .trackWidthMetres = 0.06,
    .maxWheelSpeedMetresPerSecond = 0.2,
    .maxOutputFraction = 0.4,
    .maxLinearMetresPerSecond = 0.15,
    .maxAngularRadiansPerSecond = 3.0,
    .leftReversed = false,
    .rightReversed = true,
};
} // namespace codecell::drive::fixtures
