#include "calibration/WorkspaceDistance.hpp"
#include "calibration/WorkspaceProfile.hpp"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <numeric>

namespace micro_robot
{
namespace
{
using Matrix = std::array<double, 9>;
std::optional<Matrix> normalized(const WorkspaceHomography& transform)
{
    auto h = transform.imageToWorld;
    double scale = 0;
    for (auto v : h) {
        if (!std::isfinite(v))
            return {};
        scale = std::max(scale, std::abs(v));
    }
    if (scale == 0)
        return {};
    for (auto& v : h)
        v /= scale;
    return h;
}
Matrix adjugate(const Matrix& h)
{
    return {h[4] * h[8] - h[5] * h[7], h[2] * h[7] - h[1] * h[8], h[1] * h[5] - h[2] * h[4],
            h[5] * h[6] - h[3] * h[8], h[0] * h[8] - h[2] * h[6], h[2] * h[3] - h[0] * h[5],
            h[3] * h[7] - h[4] * h[6], h[1] * h[6] - h[0] * h[7], h[0] * h[4] - h[1] * h[3]};
}
bool halfPlane(double a, double b, double c, WorldPoint2D q, double radius)
{
    const double scale = std::max({std::abs(a), std::abs(b), std::abs(c)});
    if (!std::isfinite(scale) || scale == 0)
        return false;
    a /= scale;
    b /= scale;
    c /= scale;
    const double value = a * q.x.value + b * q.y.value + c, margin = radius * std::hypot(a, b);
    const double tolerance = WorkspaceDistanceSupportRoundoffUlps * std::numeric_limits<double>::epsilon() *
                             (std::abs(a * q.x.value) + std::abs(b * q.y.value) + std::abs(c) + margin);
    return std::isfinite(value) && std::isfinite(margin) && std::isfinite(tolerance) &&
           value + tolerance >= margin;
}
bool worldHull(const WorkspaceDiagnostics& d, const std::vector<WorkspaceViewPoint>& hull, WorldPoint2D q,
               double radius)
{
    if (hull.size() < 3)
        return false;
    const auto& b = d.profile.bounds;
    for (std::size_t i = 0; i < hull.size(); ++i) {
        const auto p = hull[i], next = hull[(i + 1) % hull.size()];
        const double dx = next.u - p.u, dy = next.v - p.v;
        const double a = -dy / b.width().value, c = dx / b.height().value;
        const double constant = dy * p.u - dx * p.v - a * b.minimum().x.value - c * b.minimum().y.value;
        if (!halfPlane(a, c, constant, q, radius))
            return false;
    }
    return true;
}
std::optional<double> selectionRadius(const WorkspaceDiagnostics& d,
                                      const WorkspaceDistanceEndpoint& endpoint)
{
    const auto radius = endpoint.selection.maximumRadius;
    if (std::none_of(endpoint.selection.justification.begin(), endpoint.selection.justification.end(),
                     [](unsigned char c) { return !std::isspace(c); }) ||
        endpoint.selection.justification.size() > MaximumWorkspaceSelectionMethodBytes || !radius ||
        !std::isfinite(*radius) || *radius < 0)
        return {};
    if (endpoint.selection.space == WorkspaceSelectionSpace::WorldMeters)
        return radius;
    if (endpoint.selection.space != WorkspaceSelectionSpace::NativeUndistortedPixels)
        return {};
    const auto image = unprojectWorkspacePoint(*d.profile.homography, endpoint.world);
    const auto matrix = normalized(*d.profile.homography);
    if (!image || !matrix)
        return {};
    const auto& h = *matrix;
    const auto q = projectWorkspacePoint(*d.profile.homography, *image);
    if (!q)
        return {};
    const double denominator =
        std::abs(h[6] * image->u + h[7] * image->v + h[8]) - *radius * std::hypot(h[6], h[7]);
    const double tolerance = WorkspaceDistanceSupportRoundoffUlps * std::numeric_limits<double>::epsilon() *
                             (std::abs(h[6] * image->u) + std::abs(h[7] * image->v) + std::abs(h[8]));
    if (!std::isfinite(denominator) || denominator <= tolerance)
        return {};
    // f(p+delta)-f(p) = (A-f(p)c')delta/(d+c'delta).
    // The spectral norm and denominator margin bound the entire native-pixel disk.
    std::array<double, 4> b{h[0] - q->x.value * h[6], h[1] - q->x.value * h[7], h[3] - q->y.value * h[6],
                            h[4] - q->y.value * h[7]};
    double scale = 0;
    for (auto v : b)
        scale = std::max(scale, std::abs(v));
    if (!std::isfinite(scale))
        return {};
    if (scale == 0)
        return 0;
    for (auto& v : b)
        v /= scale;
    const double aa = b[0] * b[0] + b[2] * b[2], bb = b[0] * b[1] + b[2] * b[3],
                 dd = b[1] * b[1] + b[3] * b[3];
    const double spectral = scale * std::sqrt((aa + dd + std::hypot(aa - dd, 2 * bb)) / 2);
    const double bound = *radius * spectral / denominator;
    return std::isfinite(bound) ? std::optional<double>{bound} : std::nullopt;
}
std::optional<double> predictionDiskRadius(const WorkspaceDiagnostics& d, WorldPoint2D q, double radius)
{
    if (!d.prediction || !workspaceDistanceDiskSupported(d, q, radius, true))
        return {};
    const auto& m = *d.prediction;
    const auto& b = d.profile.bounds;
    const std::array<double, 3> phi{1, (q.x.value - b.minimum().x.value) / b.width().value,
                                    (q.y.value - b.minimum().y.value) / b.height().value};
    const auto& n = m.inverseNormal;
    const double h = phi[0] * (n[0] * phi[0] + n[1] * phi[1] + n[2] * phi[2]) +
                     phi[1] * (n[3] * phi[0] + n[4] * phi[1] + n[5] * phi[2]) +
                     phi[2] * (n[6] * phi[0] + n[7] * phi[1] + n[8] * phi[2]);
    const double aa = n[4] / b.width().value / b.width().value,
                 bb = n[5] / b.width().value / b.height().value,
                 dd = n[8] / b.height().value / b.height().value;
    const double eigen = (aa + dd + std::hypot(aa - dd, 2 * bb)) / 2;
    if (!std::isfinite(h) || h < 0 || !std::isfinite(eigen) || eigen < 0)
        return {};
    const double leverageRoot = std::sqrt(h) + radius * std::sqrt(eigen);
    const double factor = m.endpointCritical * std::hypot(1, leverageRoot);
    auto axis = [&](const auto& beta, double variance) {
        const double mean = std::inner_product(phi.begin(), phi.end(), beta.begin(), 0.0);
        const double slope = std::hypot(beta[1] / b.width().value, beta[2] / b.height().value);
        return std::abs(mean) + radius * slope + factor * std::sqrt(variance);
    };
    const double result = std::hypot(axis(m.betaX, m.varianceX), axis(m.betaY, m.varianceY));
    return std::isfinite(result) ? std::optional<double>{result} : std::nullopt;
}
} // namespace
std::optional<WorkspaceCameraViewport> workspaceCameraViewport(std::uint32_t width, std::uint32_t height,
                                                               double viewWidth, double viewHeight)
{
    if (width < MinimumWorkspaceImageDimensionPixels || height < MinimumWorkspaceImageDimensionPixels ||
        !std::isfinite(viewWidth) || !std::isfinite(viewHeight) || viewWidth <= 0 || viewHeight <= 0)
        return {};
    const double scale = std::min(viewWidth / width, viewHeight / height), w = width * scale,
                 h = height * scale;
    if (!std::isfinite(w) || !std::isfinite(h) || w <= 0 || h <= 0)
        return {};
    return WorkspaceCameraViewport{(viewWidth - w) / 2, (viewHeight - h) / 2, w, h, width, height};
}
std::optional<ImageCorner> workspaceCameraPick(const WorkspaceCameraViewport& viewport, double x, double y)
{
    if (viewport.nativeWidth < MinimumWorkspaceImageDimensionPixels ||
        viewport.nativeHeight < MinimumWorkspaceImageDimensionPixels || !std::isfinite(viewport.left) ||
        !std::isfinite(viewport.top) || !std::isfinite(viewport.width) || !std::isfinite(viewport.height) ||
        !std::isfinite(x) || !std::isfinite(y) || viewport.width <= 0 || viewport.height <= 0)
        return {};
    ImageCorner p{(x - viewport.left) * viewport.nativeWidth / viewport.width - 0.5,
                  (y - viewport.top) * viewport.nativeHeight / viewport.height - 0.5};
    const double tolerance = WorkspaceDistanceSupportRoundoffUlps * std::numeric_limits<double>::epsilon() *
                             std::max(viewport.nativeWidth, viewport.nativeHeight);
    if (!std::isfinite(p.u) || !std::isfinite(p.v) || p.u < -tolerance || p.v < -tolerance ||
        p.u > viewport.nativeWidth - 1 + tolerance || p.v > viewport.nativeHeight - 1 + tolerance)
        return {};
    p.u = std::clamp(p.u, 0.0, static_cast<double>(viewport.nativeWidth - 1));
    p.v = std::clamp(p.v, 0.0, static_cast<double>(viewport.nativeHeight - 1));
    return p;
}
std::optional<WorkspaceViewPoint> workspaceCameraDisplay(const WorkspaceCameraViewport& viewport,
                                                         ImageCorner p)
{
    if (!std::isfinite(viewport.left) || !std::isfinite(viewport.top) || !std::isfinite(p.u) ||
        !std::isfinite(p.v) || p.u < 0 || p.v < 0 || p.u > viewport.nativeWidth - 1 ||
        p.v > viewport.nativeHeight - 1 || viewport.nativeWidth < MinimumWorkspaceImageDimensionPixels ||
        viewport.nativeHeight < MinimumWorkspaceImageDimensionPixels || !std::isfinite(viewport.width) ||
        !std::isfinite(viewport.height) || viewport.width <= 0 || viewport.height <= 0)
        return {};
    return WorkspaceViewPoint{viewport.left + (p.u + 0.5) * viewport.width / viewport.nativeWidth,
                              viewport.top + (p.v + 0.5) * viewport.height / viewport.nativeHeight};
}
bool workspaceDistanceDiskSupported(const WorkspaceDiagnostics& d, WorldPoint2D q, double radius,
                                    bool requirePrediction)
{
    if (!d.profile.homography || !d.profile.bounds.contains(q) || !std::isfinite(radius) || radius < 0)
        return false;
    const auto& b = d.profile.bounds;
    if (!halfPlane(1, 0, -b.minimum().x.value, q, radius) ||
        !halfPlane(-1, 0, b.maximum().x.value, q, radius) ||
        !halfPlane(0, 1, -b.minimum().y.value, q, radius) ||
        !halfPlane(0, -1, b.maximum().y.value, q, radius))
        return false;
    const auto matrix = normalized(*d.profile.homography);
    if (!matrix)
        return false;
    const auto g = adjugate(*matrix);
    const double denominator = g[6] * q.x.value + g[7] * q.y.value + g[8];
    const double margin = std::abs(denominator) - radius * std::hypot(g[6], g[7]);
    const double tolerance = WorkspaceDistanceSupportRoundoffUlps * std::numeric_limits<double>::epsilon() *
                             (std::abs(g[6] * q.x.value) + std::abs(g[7] * q.y.value) + std::abs(g[8]));
    if (!std::isfinite(margin) || margin <= tolerance)
        return false;
    const double sign = std::signbit(denominator) ? -1 : 1;
    auto imageConstraint = [&](double u, double v, double constant) {
        return halfPlane(sign * (u * g[0] + v * g[3] + constant * g[6]),
                         sign * (u * g[1] + v * g[4] + constant * g[7]),
                         sign * (u * g[2] + v * g[5] + constant * g[8]), q, radius);
    };
    if (!imageConstraint(1, 0, 0) || !imageConstraint(0, 1, 0) ||
        !imageConstraint(-1, 0, d.profile.intrinsics.width - 1) ||
        !imageConstraint(0, -1, d.profile.intrinsics.height - 1))
        return false;
    if (d.acquisition) {
        if (!worldHull(d, d.fitHull, q, radius) || d.imageFitHull.size() < 3)
            return false;
        for (std::size_t i = 0; i < d.imageFitHull.size(); ++i) {
            const auto a = d.imageFitHull[i], next = d.imageFitHull[(i + 1) % d.imageFitHull.size()];
            const double dx = next.u - a.u, dy = next.v - a.v;
            if (!imageConstraint(-dy / d.profile.intrinsics.width, dx / d.profile.intrinsics.height,
                                 dy * a.u - dx * a.v))
                return false;
        }
    }
    return !requirePrediction || worldHull(d, d.predictionHull, q, radius);
}
std::optional<WorldPoint2D> workspaceDistancePointFromImage(const WorkspaceDiagnostics& d, ImageCorner image)
{
    if (!d.profile.homography || !std::isfinite(image.u) || !std::isfinite(image.v) || image.u < 0 ||
        image.v < 0 || image.u > d.profile.intrinsics.width - 1 || image.v > d.profile.intrinsics.height - 1)
        return {};
    auto q = projectWorkspacePoint(*d.profile.homography, image);
    return q && workspaceDistanceDiskSupported(d, *q, 0) ? q : std::nullopt;
}
WorkspaceDistanceOutcome measureWorkspaceDistance(
    const WorkspaceDiagnostics& d,
    const std::array<WorkspaceDistanceEndpoint, WorkspaceDistanceEndpointCount>& endpoints)
{
    for (const auto& endpoint : endpoints)
        if (!workspaceDistanceDiskSupported(d, endpoint.world, 0))
            return {{}, "Endpoint centre is outside supported geometry or has an invalid mapping."};
    WorkspaceDistanceResult result;
    result.frameId = d.profile.frameId;
    result.calibrationRevision = serializeWorkspaceProfile(d.profile);
    result.distanceMeters = std::hypot(endpoints[1].world.x.value - endpoints[0].world.x.value,
                                       endpoints[1].world.y.value - endpoints[0].world.y.value);
    if (!std::isfinite(result.distanceMeters))
        return {{}, "Distance is not finite."};
    auto unavailable = [&](std::string reason) {
        result.uncertaintyUnavailable = std::move(reason);
        return WorkspaceDistanceOutcome{result, {}};
    };
    for (std::size_t i = 0; i < endpoints.size(); ++i) {
        result.selectionRadiiMeters[i] = selectionRadius(d, endpoints[i]);
        if (!result.selectionRadiiMeters[i])
            return unavailable(
                "Endpoint " + std::to_string(i + 1) +
                " selection bound/justification is missing or invalid, or crosses a projective horizon.");
    }
    if (!d.prediction)
        return unavailable(d.uncertaintyUnavailable.empty()
                               ? "Calibration prediction evidence is unavailable."
                               : d.uncertaintyUnavailable);
    for (std::size_t i = 0; i < endpoints.size(); ++i) {
        const double radius = *result.selectionRadiiMeters[i];
        result.calibrationRadiiMeters[i] = predictionDiskRadius(d, endpoints[i].world, radius);
        if (!result.calibrationRadiiMeters[i])
            return unavailable("Endpoint " + std::to_string(i + 1) +
                               " selection footprint leaves supported geometry/survey coverage, or its "
                               "prediction bound is not finite.");
    }
    const double bound = *result.selectionRadiiMeters[0] + *result.selectionRadiiMeters[1] +
                         *result.calibrationRadiiMeters[0] + *result.calibrationRadiiMeters[1];
    if (!std::isfinite(bound) || !std::isfinite(result.distanceMeters + bound))
        return unavailable("Distance uncertainty interval is not finite.");
    result.plusMinusMeters = bound;
    result.lowerMeters = std::max(0.0, result.distanceMeters - bound);
    result.upperMeters = result.distanceMeters + bound;
    return {result, {}};
}
} // namespace micro_robot
