#include "calibration/TabletopHomography.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace micro_robot {
namespace {
constexpr std::size_t maximumPoints = 64;
struct Point { double x, y; };
bool convex(const std::vector<Point>& p)
{
    double sign = 0;
    for (std::size_t i = 0; i < p.size(); ++i) {
        const auto a = p[i], b = p[(i + 1) % p.size()];
        for (std::size_t j = 0; j < p.size(); ++j) {
            if (j == i || j == (i + 1) % p.size()) continue;
            const auto c = p[j];
            const double cross = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
            if (!std::isfinite(cross) || std::abs(cross) <= 1e-8) return false;
            if (sign == 0) sign = cross;
            if (sign * cross <= 0) return false;
        }
    }
    return true;
}
std::optional<WorkspaceMappingErrors> errors(const WorkspaceHomography& h,
    const std::vector<WorkspaceCorrespondence>& points)
{
    WorkspaceMappingErrors result;
    double squares = 0;
    for (const auto& p : points) {
        const auto projected = projectWorkspacePoint(h, p.image);
        if (!projected) return {};
        const double d = std::hypot(projected->x.value - p.world.x.value, projected->y.value - p.world.y.value);
        if (!std::isfinite(d)) return {};
        result.distancesMeters.push_back(d);
        squares += d * d;
        result.maximumMeters = std::max(result.maximumMeters, d);
    }
    result.rmsMeters = std::sqrt(squares / static_cast<double>(points.size()));
    if (!std::isfinite(result.rmsMeters)) return {};
    return result;
}
}
std::optional<WorldPoint2D> projectWorkspacePoint(const WorkspaceHomography& transform, ImageCorner p)
{
    if (!std::isfinite(p.u) || !std::isfinite(p.v)) return {};
    auto h = transform.imageToWorld;
    double scale = 0;
    for (const auto v : h) { if (!std::isfinite(v)) return {}; scale = std::max(scale, std::abs(v)); }
    if (scale == 0) return {};
    for (auto& v : h) v /= scale;
    const double terms[]{h[0]*h[4]*h[8], h[1]*h[5]*h[6], h[2]*h[3]*h[7],
        -h[2]*h[4]*h[6], -h[1]*h[3]*h[8], -h[0]*h[5]*h[7]};
    double determinant = 0, determinantMagnitude = 0;
    for (double term : terms) { determinant += term; determinantMagnitude += std::abs(term); }
    if (std::abs(determinant) <= 16 * std::numeric_limits<double>::epsilon() * determinantMagnitude) return {};
    const double w = h[6] * p.u + h[7] * p.v + h[8];
    const double magnitude = std::abs(h[6] * p.u) + std::abs(h[7] * p.v) + std::abs(h[8]);
    if (!std::isfinite(w) || std::abs(w) <= 64 * std::numeric_limits<double>::epsilon() * magnitude) return {};
    const double x = (h[0] * p.u + h[1] * p.v + h[2]) / w;
    const double y = (h[3] * p.u + h[4] * p.v + h[5]) / w;
    if (!std::isfinite(x) || !std::isfinite(y)) return {};
    return WorldPoint2D{Meters{x}, Meters{y}};
}
std::string validateWorkspaceHomographyInput(const WorkspaceHomographyInput& input)
{
    auto profile = input.profile;
    profile.homography.reset(); // An old estimate does not constrain a new fit.
    if (auto error = validateWorkspaceCalibrationProfile(profile); !error.empty()) return error;
    if (input.fitPoints.size() < 4 || input.fitPoints.size() > maximumPoints || input.checkPoints.size() > maximumPoints)
        return "Use 4–64 fitting points and at most 64 independent check points.";
    const auto& k = profile.intrinsics;
    std::vector<Point> image, world;
    const auto valid = [&](const WorkspaceCorrespondence& p) {
        return std::isfinite(p.image.u) && std::isfinite(p.image.v)
            && p.image.u >= 0 && p.image.v >= 0 && p.image.u < k.width && p.image.v < k.height
            && profile.bounds.contains(p.world);
    };
    for (const auto& p : input.fitPoints) {
        if (!valid(p)) return "Each paired point must be finite and inside the native image and measured workspace.";
        image.push_back({p.image.u / k.width, p.image.v / k.height});
        world.push_back({p.world.x.value / profile.bounds.width().value, p.world.y.value / profile.bounds.height().value});
    }
    if (!convex(image) || !convex(world))
        return "Fitting points must form a non-degenerate convex perimeter in matching order; remove duplicates, crossings and collinear points.";
    std::vector<WorkspaceCorrespondence> all = input.fitPoints;
    for (const auto& p : input.checkPoints) {
        if (!valid(p)) return "Check points must be finite and inside the native image and workspace.";
        for (const auto& previous : all) {
            if (std::hypot((p.image.u - previous.image.u) / k.width, (p.image.v - previous.image.v) / k.height) <= 1e-8
                || std::hypot((p.world.x.value - previous.world.x.value) / profile.bounds.width().value,
                    (p.world.y.value - previous.world.y.value) / profile.bounds.height().value) <= 1e-8)
                return "Check points must be independent of the fitting points and each other.";
        }
        all.push_back(p);
    }
    return {};
}
WorkspaceHomographyOutcome evaluateWorkspaceHomography(const WorkspaceHomographyInput& input,
    const WorkspaceHomography& transform)
{
    const auto fail = [](std::string error) { return WorkspaceHomographyOutcome{{}, std::move(error)}; };
    if (auto error = validateWorkspaceHomographyInput(input); !error.empty()) return fail(error);
    auto profile = input.profile; profile.homography = transform;
    if (auto error = validateWorkspaceCalibrationProfile(profile); !error.empty()) return fail(error);
    // A linear denominator cannot cross zero anywhere inside a convex fit polygon
    // if all vertices have the same strict sign. Do not allow a projective fold.
    double sign = 0;
    for (const auto& p : input.fitPoints) {
        auto h = transform.imageToWorld;
        double scale = 0;
        for (auto value : h) scale = std::max(scale, std::abs(value));
        for (auto& value : h) value /= scale;
        const double w = h[6] * p.image.u + h[7] * p.image.v + h[8];
        if (!projectWorkspacePoint(transform, p.image) || !std::isfinite(w) || w == 0
            || (sign != 0 && std::signbit(w) != std::signbit(sign))) return fail("Homography horizon crosses the fitting region.");
        sign = w;
    }
    const auto fit = errors(transform, input.fitPoints);
    if (!fit) return fail("Cannot evaluate finite fitting residuals.");
    WorkspaceHomographyResult result{std::move(profile), *fit, {}};
    if (!input.checkPoints.empty()) {
        result.check = errors(transform, input.checkPoints);
        if (!result.check) return fail("Cannot evaluate finite independent check errors.");
    }
    return {std::move(result), {}};
}
} // namespace micro_robot
