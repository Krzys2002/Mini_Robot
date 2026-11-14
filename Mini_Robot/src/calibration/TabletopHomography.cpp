#include "calibration/TabletopHomography.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace micro_robot {
namespace {
constexpr std::size_t maximumPoints = 64;

struct Point {
    double x, y;
};

double cross(Point a, Point b, Point c)
{
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

bool spansPlane(std::vector<Point> points)
{
    std::sort(points.begin(), points.end(),
              [](Point a, Point b) { return a.x < b.x || (a.x == b.x && a.y < b.y); });
    std::vector<Point> hull;
    for (const auto point : points) {
        while (hull.size() >= 2 && cross(hull[hull.size() - 2], hull.back(), point) <= 1e-8)
            hull.pop_back();
        hull.push_back(point);
    }
    const auto lowerSize = hull.size();
    for (auto it = points.rbegin() + 1; it != points.rend(); ++it) {
        while (hull.size() > lowerSize && cross(hull[hull.size() - 2], hull.back(), *it) <= 1e-8)
            hull.pop_back();
        hull.push_back(*it);
    }
    return hull.size() >= 4; // Closed hull with at least three non-collinear vertices.
}

bool duplicate(const WorkspaceCorrespondence& a, const WorkspaceCorrespondence& b,
               const WorkspaceCalibrationProfile& profile)
{
    return std::hypot((a.image.u - b.image.u) / profile.intrinsics.width,
                      (a.image.v - b.image.v) / profile.intrinsics.height) <= 1e-8 ||
           std::hypot((a.world.x.value - b.world.x.value) / profile.bounds.width().value,
                      (a.world.y.value - b.world.y.value) / profile.bounds.height().value) <= 1e-8;
}

bool validPoint(const WorkspaceCorrespondence& p, const WorkspaceCalibrationProfile& profile)
{
    return std::isfinite(p.image.u) && std::isfinite(p.image.v) && p.image.u >= 0 && p.image.v >= 0 &&
           p.image.u <= profile.intrinsics.width - 1 && p.image.v <= profile.intrinsics.height - 1 &&
           profile.bounds.contains(p.world);
}

std::optional<WorkspaceMappingErrors> errors(const WorkspaceHomography& h,
                                             const std::vector<WorkspaceCorrespondence>& points)
{
    WorkspaceMappingErrors result;
    double squares = 0;
    for (const auto& p : points) {
        const auto projected = projectWorkspacePoint(h, p.image);
        if (!projected)
            return {};
        const double d =
            std::hypot(projected->x.value - p.world.x.value, projected->y.value - p.world.y.value);
        if (!std::isfinite(d))
            return {};
        result.distancesMeters.push_back(d);
        squares += d * d;
        result.maximumMeters = std::max(result.maximumMeters, d);
    }
    result.rmsMeters = std::sqrt(squares / static_cast<double>(points.size()));
    if (!std::isfinite(result.rmsMeters))
        return {};
    return result;
}
} // namespace

std::optional<WorldPoint2D> projectWorkspacePoint(const WorkspaceHomography& transform, ImageCorner p)
{
    if (!std::isfinite(p.u) || !std::isfinite(p.v))
        return {};
    auto h = transform.imageToWorld;
    double scale = 0;
    for (const auto v : h) {
        if (!std::isfinite(v))
            return {};
        scale = std::max(scale, std::abs(v));
    }
    if (scale == 0)
        return {};
    for (auto& v : h)
        v /= scale;
    const double terms[]{h[0] * h[4] * h[8],  h[1] * h[5] * h[6],  h[2] * h[3] * h[7],
                         -h[2] * h[4] * h[6], -h[1] * h[3] * h[8], -h[0] * h[5] * h[7]};
    double determinant = 0, determinantMagnitude = 0;
    for (double term : terms) {
        determinant += term;
        determinantMagnitude += std::abs(term);
    }
    if (std::abs(determinant) <= 16 * std::numeric_limits<double>::epsilon() * determinantMagnitude)
        return {};
    const double w = h[6] * p.u + h[7] * p.v + h[8];
    const double magnitude = std::abs(h[6] * p.u) + std::abs(h[7] * p.v) + std::abs(h[8]);
    if (!std::isfinite(w) || std::abs(w) <= 64 * std::numeric_limits<double>::epsilon() * magnitude)
        return {};
    const double x = (h[0] * p.u + h[1] * p.v + h[2]) / w;
    const double y = (h[3] * p.u + h[4] * p.v + h[5]) / w;
    if (!std::isfinite(x) || !std::isfinite(y))
        return {};
    return WorldPoint2D{Meters{x}, Meters{y}};
}

std::optional<ImageCorner> unprojectWorkspacePoint(const WorkspaceHomography& transform, WorldPoint2D point)
{
    auto h = transform.imageToWorld;
    double scale = 0;
    for (auto value : h) {
        if (!std::isfinite(value)) return {};
        scale = std::max(scale, std::abs(value));
    }
    if (scale == 0) return {};
    for (auto& value : h) value /= scale;
    // Adjugate is an inverse up to homogeneous scale; projection checks singularity/horizon.
    const WorkspaceHomography inverse{{h[4]*h[8]-h[5]*h[7], h[2]*h[7]-h[1]*h[8], h[1]*h[5]-h[2]*h[4],
        h[5]*h[6]-h[3]*h[8], h[0]*h[8]-h[2]*h[6], h[2]*h[3]-h[0]*h[5],
        h[3]*h[7]-h[4]*h[6], h[1]*h[6]-h[0]*h[7], h[0]*h[4]-h[1]*h[3]}};
    const auto projected = projectWorkspacePoint(inverse, {point.x.value, point.y.value});
    if (!projected) return {};
    return ImageCorner{projected->x.value, projected->y.value};
}
std::string validateWorkspaceHomographyInput(const WorkspaceHomographyInput& input)
{
    auto profile = input.profile;
    profile.homography.reset(); // An old estimate does not constrain a new fit.
    if (auto error = validateWorkspaceCalibrationProfile(profile); !error.empty())
        return error;
    if (input.fitPoints.size() < 4 || input.fitPoints.size() > maximumPoints ||
        input.checkPoints.size() > maximumPoints)
        return "Use 4–64 fitting points and at most 64 independent check points.";
    const auto& k = profile.intrinsics;
    std::vector<Point> image, world;
    std::vector<WorkspaceCorrespondence> seen;
    for (const auto& p : input.fitPoints) {
        for (const auto& previous : seen)
            if (duplicate(p, previous, profile))
                return "Remove duplicate fitting image/world points.";
        seen.push_back(p);
        if (!validPoint(p, profile))
            return "Each paired point must be finite and inside the native image and measured workspace.";
        image.push_back({p.image.u / k.width, p.image.v / k.height});
        world.push_back({(p.world.x.value - profile.bounds.minimum().x.value) / profile.bounds.width().value,
                         (p.world.y.value - profile.bounds.minimum().y.value) / profile.bounds.height().value});
    }
    if (!spansPlane(image) || !spansPlane(world))
        return "Fitting image and world points must span a non-degenerate two-dimensional region.";
    std::vector<WorkspaceCorrespondence> all = input.fitPoints;
    for (const auto& p : input.checkPoints) {
        if (!validPoint(p, profile))
            return "Check points must be finite and inside the native image and workspace.";
        for (const auto& previous : all) {
            if (duplicate(p, previous, profile))
                return "Check points must be independent of the fitting points and each other.";
        }
        all.push_back(p);
    }
    return {};
}

std::optional<WorkspaceMappingErrors>
evaluateWorkspaceChecks(const WorkspaceCalibrationProfile& profile,
                        const std::vector<WorkspaceCorrespondence>& points)
{
    if (!profile.homography || !validateWorkspaceCalibrationProfile(profile).empty() || points.empty() ||
        points.size() > maximumPoints)
        return {};
    for (std::size_t i = 0; i < points.size(); ++i) {
        if (!validPoint(points[i], profile))
            return {};
        for (std::size_t j = 0; j < i; ++j)
            if (duplicate(points[i], points[j], profile))
                return {};
    }
    return errors(*profile.homography, points);
}

WorkspaceHomographyOutcome evaluateWorkspaceHomography(const WorkspaceHomographyInput& input,
                                                       const WorkspaceHomography& transform)
{
    const auto fail = [](std::string error) { return WorkspaceHomographyOutcome{{}, std::move(error)}; };
    if (auto error = validateWorkspaceHomographyInput(input); !error.empty())
        return fail(error);
    auto profile = input.profile;
    profile.homography = transform;
    if (auto error = validateWorkspaceCalibrationProfile(profile); !error.empty())
        return fail(error);
    // Same strict denominator sign at every sample guarantees no horizon inside
    // their convex hull, regardless of input ordering or interior/edge points.
    double sign = 0;
    for (const auto& p : input.fitPoints) {
        auto h = transform.imageToWorld;
        double scale = 0;
        for (auto value : h)
            scale = std::max(scale, std::abs(value));
        for (auto& value : h)
            value /= scale;
        const double w = h[6] * p.image.u + h[7] * p.image.v + h[8];
        if (!projectWorkspacePoint(transform, p.image) || !std::isfinite(w) || w == 0 ||
            (sign != 0 && std::signbit(w) != std::signbit(sign)))
            return fail("Homography horizon crosses the fitting region.");
        sign = w;
    }
    const auto fit = errors(transform, input.fitPoints);
    if (!fit)
        return fail("Cannot evaluate finite fitting residuals.");
    WorkspaceHomographyResult result{std::move(profile), *fit, {}};
    if (!input.checkPoints.empty()) {
        result.check = errors(transform, input.checkPoints);
        if (!result.check)
            return fail("Cannot evaluate finite independent check errors.");
    }
    return {std::move(result), {}};
}
} // namespace micro_robot
