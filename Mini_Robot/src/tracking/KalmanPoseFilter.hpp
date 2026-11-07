#pragma once
#include "common/Pose2D.hpp"
#include <array>
#include <optional>

namespace micro_robot {
struct KalmanNoise {
    // White acceleration spectral densities: m²/s³ and rad²/s³.
    double positionProcess{.05}, headingProcess{.2};
    // Measurement variances: m² and rad² (strictly positive).
    double positionMeasurement{.0001}, headingMeasurement{.0004};
};
// Owner-thread estimator. State [x,y,theta,vx,vy,omega], measurement
// [x,y,theta]. Independent constant-velocity axes (no cross-axis covariance).
// No clock reads/prediction between observations; caller supplies capture dt.
class KalmanPoseFilter final {
public:
    explicit KalmanPoseFilter(KalmanNoise noise = {});
    // First sample initializes exactly. Later dt must be finite in (0,10] s.
    // Invalid/numerically overflowing updates leave all state unchanged.
    [[nodiscard]] bool update(const Pose2D& measurement, double dtSeconds);
    [[nodiscard]] std::optional<Pose2D> pose() const;
private:
    struct Axis { double position{}, velocity{}, pp{}, pv{}, vv{1}; };
    KalmanNoise noise_;
    std::optional<std::array<Axis, 3>> axes_;
};
} // namespace micro_robot
