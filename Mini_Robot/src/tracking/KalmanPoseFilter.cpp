#include "tracking/KalmanPoseFilter.hpp"
#include "common/Angle.hpp"
#include <cmath>
#include <stdexcept>

namespace micro_robot {
KalmanPoseFilter::KalmanPoseFilter(KalmanNoise noise) : noise_(noise)
{
    if (!std::isfinite(noise.positionProcess) || noise.positionProcess < 0
        || !std::isfinite(noise.headingProcess) || noise.headingProcess < 0
        || !std::isfinite(noise.positionMeasurement) || noise.positionMeasurement <= 0
        || !std::isfinite(noise.headingMeasurement) || noise.headingMeasurement <= 0)
        throw std::invalid_argument("Invalid Kalman process/measurement variance");
}
bool KalmanPoseFilter::update(const Pose2D& measurement, double dt)
{
    std::array<double, 3> z{measurement.x.value, measurement.y.value, normalizeAngle(measurement.heading).value};
    for (double value : z) if (!std::isfinite(value)) return false;
    if (!axes_) {
        axes_ = std::array<Axis, 3>{Axis{z[0], 0, noise_.positionMeasurement, 0, 1},
            Axis{z[1], 0, noise_.positionMeasurement, 0, 1},
            Axis{z[2], 0, noise_.headingMeasurement, 0, 1}};
        return true;
    }
    if (!std::isfinite(dt) || dt <= 0 || dt > 10) return false;
    auto next = *axes_;
    for (std::size_t i = 0; i < next.size(); ++i) {
        auto& a = next[i];
        const double q = i == 2 ? noise_.headingProcess : noise_.positionProcess;
        const double r = i == 2 ? noise_.headingMeasurement : noise_.positionMeasurement;
        const double predicted = a.position + dt * a.velocity;
        const double pp = a.pp + 2 * dt * a.pv + dt * dt * a.vv + q * dt * dt * dt / 3;
        const double pv = a.pv + dt * a.vv + q * dt * dt / 2;
        const double vv = a.vv + q * dt;
        const double innovation = i == 2 ? normalizeAngle(Radians{z[i] - normalizeAngle(Radians{predicted}).value}).value
                                         : z[i] - predicted;
        const double denominator = pp + r;
        const double kp = pp / denominator, kv = pv / denominator;
        a.position = predicted + kp * innovation;
        if (i == 2) a.position = normalizeAngle(Radians{a.position}).value;
        a.velocity += kv * innovation;
        // Joseph covariance update, preserving symmetry and nonnegative diagonal.
        a.pp = (1-kp)*(1-kp)*pp + kp*kp*r;
        a.pv = (1-kp)*(pv-kv*pp) + kp*kv*r;
        a.vv = vv - 2*kv*pv + kv*kv*pp + kv*kv*r;
        if (!std::isfinite(a.position) || !std::isfinite(a.velocity) || !std::isfinite(a.pp)
            || !std::isfinite(a.pv) || !std::isfinite(a.vv) || a.pp < 0 || a.vv < 0) return false;
    }
    axes_ = next;
    return true;
}
std::optional<Pose2D> KalmanPoseFilter::pose() const
{
    if (!axes_) return {};
    return Pose2D{Meters{(*axes_)[0].position}, Meters{(*axes_)[1].position}, Radians{(*axes_)[2].position}};
}
} // namespace micro_robot
