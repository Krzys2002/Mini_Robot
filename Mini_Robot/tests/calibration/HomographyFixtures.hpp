#pragma once
#include "calibration/TabletopHomography.hpp"
#include "IntrinsicFixtures.hpp"
namespace micro_robot::test {
inline WorkspaceHomographyInput homographyInput()
{
    return {{1, "table-A", WorkspaceBounds{Meters{1}, Meters{0.6}}, validIntrinsicCalibration()},
        {{{100, 700}, {Meters{0}, Meters{0}}}, {{900, 700}, {Meters{1}, Meters{0}}},
         {{900, 100}, {Meters{1}, Meters{0.6}}}, {{100, 100}, {Meters{0}, Meters{0.6}}}},
        {{{500, 400}, {Meters{0.5}, Meters{0.3}}}}};
}
inline WorkspaceHomography knownHomography() { return {{0.00125, 0, -0.125, 0, -0.001, 0.7, 0, 0, 1}}; }
}
