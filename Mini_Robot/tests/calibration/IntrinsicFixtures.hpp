#pragma once
#include "calibration/IntrinsicCalibration.hpp"

namespace micro_robot::test {
inline IntrinsicCalibration validIntrinsicCalibration()
{
    return {"camera-a", 1000, 800, {800, 0, 500, 0, 810, 400, 0, 0, 1},
            {-0.1, 0.02, 0.001, -0.001, 0.003}, 0.5,
            {{1, 54, 0.5}, {2, 54, 0.5}, {3, 54, 0.5}}};
}
} // namespace micro_robot::test
