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

std::vector<Point> convexHull(std::vector<Point> points)
{
    std::sort(points.begin(), points.end(),
              [](Point a, Point b) { return a.x < b.x || (a.x == b.x && a.y < b.y); });
    std::vector<Point> hull;
    for (const auto point : points) {
        while (hull.size() >= 2 && cross(hull[hull.size() - 2], hull.back(), point) <= WorkspaceHullCrossTolerance)
            hull.pop_back();
        hull.push_back(point);
    }
    const auto lowerSize = hull.size();
    for (auto it = points.rbegin() + 1; it != points.rend(); ++it) {
        while (hull.size() > lowerSize && cross(hull[hull.size() - 2], hull.back(), *it) <= WorkspaceHullCrossTolerance)
            hull.pop_back();
        hull.push_back(*it);
    }
    return hull; // Closed hull; at least three distinct vertices are needed to span a plane.
}

bool spansPlane(const std::vector<Point>& points)
{
    return convexHull(points).size() >= 4;
}

double hullArea(const std::vector<Point>& hull)
{
    double area = 0;
    for (std::size_t i = 1; i < hull.size(); ++i)
        area += hull[i-1].x * hull[i].y - hull[i].x * hull[i-1].y;
    return std::abs(area) / 2;
}

bool insideHull(const std::vector<Point>& hull, Point point)
{
    for (std::size_t i = 1; i < hull.size(); ++i)
        if (cross(hull[i-1],hull[i],point) < -WorkspaceHullCrossTolerance) return false;
    return true;
}

WorkspaceFitCoverage fitCoverage(const WorkspaceHomographyInput& input)
{
    const auto& p = input.profile;
    std::vector<Point> image, world;
    for (const auto& pair : input.fitPoints) {
        image.push_back({pair.image.u / p.intrinsics.width, pair.image.v / p.intrinsics.height});
        world.push_back({pair.world.x.value / p.bounds.width().value, pair.world.y.value / p.bounds.height().value});
    }
    const auto imageHull = convexHull(image), worldHull = convexHull(world);
    WorkspaceFitCoverage coverage{hullArea(imageHull), hullArea(worldHull), 0};
    for (const auto& pair : input.checkPoints)
        if (!insideHull(imageHull,{pair.image.u / p.intrinsics.width,pair.image.v / p.intrinsics.height})
            || !insideHull(worldHull,{pair.world.x.value / p.bounds.width().value,pair.world.y.value / p.bounds.height().value}))
            ++coverage.extrapolatedCheckCount;
    return coverage;
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
        world.push_back({p.world.x.value / profile.bounds.width().value,
                         p.world.y.value / profile.bounds.height().value});
    }
    if (!spansPlane(image) || !spansPlane(world))
        return "Fitting points are degenerate or nearly collinear. Spread visible references across near/far edges "
               "and both table axes; raise an almost edge-on camera. Do not guess occluded points.";
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
            return fail("Homography horizon crosses the fitting region. Check paired labels/plane, raise the camera "
                        "and reselect visible references on the same side of the horizon.");
        sign = w;
    }
    const auto fit = errors(transform, input.fitPoints);
    if (!fit)
        return fail("Cannot evaluate finite fitting residuals.");
    WorkspaceHomographyResult result{std::move(profile), *fit, {}, fitCoverage(input)};
    if (!input.checkPoints.empty()) {
        result.check = errors(transform, input.checkPoints);
        if (!result.check)
            return fail("Cannot evaluate finite independent check errors.");
    }
    return {std::move(result), {}};
}
} // namespace micro_robot
