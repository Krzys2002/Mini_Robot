#include "calibration/WorkspaceCalibration.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace micro_robot {
std::array<WorldPoint2D, WorkspaceCornerCount> workspaceCorners(const WorkspaceBounds& bounds)
{
    const auto lo = bounds.minimum(), hi = bounds.maximum();
    return {lo, WorldPoint2D{hi.x, lo.y}, hi, WorldPoint2D{lo.x, hi.y}};
}
std::string validateWorkspaceBoundary(const WorkspaceBoundary& b, std::uint32_t width, std::uint32_t height)
{
    if (width < MinimumWorkspaceImageDimensionPixels || height < MinimumWorkspaceImageDimensionPixels) return "Boundary requires a valid native image size.";
    for (const auto& p : b)
        if (!std::isfinite(p.u) || !std::isfinite(p.v) || p.u < 0 || p.v < 0
            || p.u > width - 1 || p.v > height - 1)
            return "Corners must be finite and inside the undistorted native image.";
    double sign = 0;
    for (std::size_t i = 0; i < b.size(); ++i) {
        const auto& a = b[i]; const auto& p = b[(i + 1) % b.size()];
        const auto& c = b[(i + 2) % b.size()];
        const double cross = ((p.u - a.u) / width) * ((c.v - p.v) / height)
                           - ((p.v - a.v) / height) * ((c.u - p.u) / width);
        if (std::abs(cross) <= WorkspaceBoundaryCrossTolerance || (sign && std::signbit(cross) != std::signbit(sign)))
            return "Reselect corners: duplicate, crossed, concave or collinear boundary. Use base, +X, opposite, +Y order.";
        sign = cross;
    }
    return {};
}
std::string validateWorkspaceCalibrationProfile(const WorkspaceCalibrationProfile& profile)
{
    if (profile.version != WorkspaceCalibrationProfileVersion) return "Unsupported workspace profile version.";
    if (!profile.bounds.contains({})) return "Workspace origin must lie inside the measured rectangle.";
    if (profile.acquisitionId.size() > MaximumWorkspaceFrameIdBytes) return "Oversized workspace acquisition identity.";
    if (profile.frameId.empty() || profile.frameId.size() > MaximumWorkspaceFrameIdBytes)
        return "Workspace frame identity is missing or oversized.";
    if (profile.boundary) {
        if (auto error = validateWorkspaceBoundary(*profile.boundary, profile.intrinsics.width,
                                                   profile.intrinsics.height); !error.empty()) return error;
    }
    if (profile.workspaceId.empty()) return "Workspace identity is required.";
    if (profile.imageCoordinates != WorkspaceImageCoordinates::UndistortedNativeCameraMatrix)
        return "Unsupported workspace image coordinate convention.";
    if (const auto error = validateIntrinsicCalibration(profile.intrinsics); !error.empty()) return error;
    if (profile.homography) {
        auto h = profile.homography->imageToWorld;
        double scale = 0;
        for (const double value : h) {
            if (!std::isfinite(value)) return "Workspace homography contains non-finite values.";
            scale = std::max(scale, std::abs(value));
        }
        if (scale == 0) return "Workspace homography is singular.";
        // Homogeneous scale has no meaning. Normalize before the determinant to
        // avoid overflow for equivalent finite matrix representations.
        for (double& value : h) value /= scale;
        const double terms[]{h[0] * h[4] * h[8], h[1] * h[5] * h[6], h[2] * h[3] * h[7],
                             -h[2] * h[4] * h[6], -h[1] * h[3] * h[8], -h[0] * h[5] * h[7]};
        double determinant = 0, magnitude = 0;
        for (double term : terms) {
            determinant += term;
            magnitude += std::abs(term);
        }
        // Cancellation can leave a rounded nonzero determinant for dependent
        // rows. Use a relative error bound, without rejecting small SI scales.
        if (std::abs(determinant) <= 16 * std::numeric_limits<double>::epsilon() * magnitude)
            return "Workspace homography is singular or numerically unresolved.";
    }
    return {};
}
std::string workspaceCalibrationCompatibilityError(const WorkspaceCalibrationProfile& profile,
                                                    const IntrinsicCalibration& active)
{
    if (const auto error = validateWorkspaceCalibrationProfile(profile); !error.empty()) return error;
    if (const auto error = validateIntrinsicCalibration(active); !error.empty()) return error;
    const auto& bound = profile.intrinsics;
    if (active.sourceId != bound.sourceId || active.width != bound.width || active.height != bound.height)
        return "Workspace calibration belongs to another camera or resolution.";
    if (active.cameraMatrix != bound.cameraMatrix || active.distortion != bound.distortion)
        return "Workspace calibration belongs to another intrinsic optical model.";
    return {};
}
} // namespace micro_robot
