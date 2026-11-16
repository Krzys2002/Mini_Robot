#include "calibration/WorkspaceDiagnostics.hpp"
#include "calibration/WorkspaceProfile.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <set>

namespace micro_robot
{
namespace
{
using P = WorkspaceViewPoint;
double cross(P a, P b, P c) { return (b.u - a.u) * (c.v - a.v) - (b.v - a.v) * (c.u - a.u); }
std::vector<P> hull(std::vector<P> p)
{
    std::sort(p.begin(), p.end(), [](P a, P b) { return a.u < b.u || (a.u == b.u && a.v < b.v); });
    p.erase(std::unique(p.begin(), p.end(), [](P a, P b) { return a.u == b.u && a.v == b.v; }), p.end());
    if (p.size() < 3)
        return {};
    std::vector<P> h;
    for (auto a : p) {
        while (h.size() > 1 && cross(h[h.size() - 2], h.back(), a) <= 0)
            h.pop_back();
        h.push_back(a);
    }
    const auto lower = h.size();
    for (auto it = p.rbegin() + 1; it != p.rend(); ++it) {
        while (h.size() > lower && cross(h[h.size() - 2], h.back(), *it) <= 0)
            h.pop_back();
        h.push_back(*it);
    }
    h.pop_back();
    return h.size() >= 3 ? h : std::vector<P>{};
}
bool inside(const std::vector<P>& h, P p)
{
    if (h.size() < 3)
        return false;
    for (std::size_t i = 0; i < h.size(); ++i)
        if (cross(h[i], h[(i + 1) % h.size()], p) < -WorkspaceMapHullTolerance)
            return false;
    return true;
}
P normalized(const WorkspaceBounds& b, WorldPoint2D p)
{
    return {(p.x.value - b.minimum().x.value) / b.width().value,
            (p.y.value - b.minimum().y.value) / b.height().value};
}
std::array<double, 3> design(const WorkspaceBounds& b, WorldPoint2D p)
{
    const auto q = normalized(b, p);
    return {1, q.u, q.v};
}
double dot(const std::array<double, 3>& a, const std::array<double, 3>& b)
{
    return std::inner_product(a.begin(), a.end(), b.begin(), 0.0);
}
std::array<double, 3> multiply(const std::array<double, 9>& a, const std::array<double, 3>& b)
{
    return {a[0] * b[0] + a[1] * b[1] + a[2] * b[2], a[3] * b[0] + a[4] * b[1] + a[5] * b[2],
            a[6] * b[0] + a[7] * b[1] + a[8] * b[2]};
}
std::array<double, 9> adjugate(const std::array<double, 9>& a)
{
    return {a[4] * a[8] - a[5] * a[7], a[2] * a[7] - a[1] * a[8], a[1] * a[5] - a[2] * a[4],
            a[5] * a[6] - a[3] * a[8], a[0] * a[8] - a[2] * a[6], a[2] * a[3] - a[0] * a[5],
            a[3] * a[7] - a[4] * a[6], a[1] * a[6] - a[0] * a[7], a[0] * a[4] - a[1] * a[3]};
}
std::optional<std::array<double, 9>> inverse(std::array<double, 9> a)
{
    double scale = 0;
    for (double x : a)
        scale = std::max(scale, std::abs(x));
    if (!std::isfinite(scale) || scale == 0)
        return {};
    for (auto& x : a)
        x /= scale;
    auto inv = adjugate(a);
    const double det = a[0] * inv[0] + a[1] * inv[3] + a[2] * inv[6];
    if (!std::isfinite(det) || det == 0)
        return {};
    for (auto& x : inv) {
        x = (x / det) / scale;
        if (!std::isfinite(x))
            return {};
    }
    return inv;
}
bool geometricSupport(const WorkspaceDiagnostics& d, WorldPoint2D p)
{
    if (!d.profile.homography || !d.profile.bounds.contains(p))
        return false;
    const auto image = unprojectWorkspacePoint(*d.profile.homography, p);
    if (!image || image->u < 0 || image->v < 0 || image->u > d.profile.intrinsics.width - 1 ||
        image->v > d.profile.intrinsics.height - 1)
        return false;
    if (d.acquisition && (!inside(d.fitHull, normalized(d.profile.bounds, p)) ||
                          !inside(d.imageFitHull, {image->u / d.profile.intrinsics.width,
                                                   image->v / d.profile.intrinsics.height})))
        return false;
    return true;
}
std::optional<double> betaFraction(double a, double b, double x)
{
    const double qab = a + b, qap = a + 1, qam = a - 1;
    auto nonzero = [](double v) {
        return std::abs(v) < WorkspaceBetaTiny ? std::copysign(WorkspaceBetaTiny, v) : v;
    };
    double c = 1, d = 1 / nonzero(1 - qab * x / qap), h = d;
    for (int m = 1; m <= WorkspaceBetaIterations; ++m) {
        double aa = m * (b - m) * x / ((qam + 2 * m) * (a + 2 * m));
        d = 1 / nonzero(1 + aa * d);
        c = nonzero(1 + aa / c);
        h *= d * c;
        aa = -(a + m) * (qab + m) * x / ((a + 2 * m) * (qap + 2 * m));
        d = 1 / nonzero(1 + aa * d);
        c = nonzero(1 + aa / c);
        const double delta = d * c;
        h *= delta;
        if (std::abs(delta - 1) < WorkspaceBetaTolerance)
            return h;
    }
    return {};
}
std::optional<double> beta(double a, double b, double x)
{
    if (x <= 0)
        return 0;
    if (x >= 1)
        return 1;
    const double factor =
        std::exp(std::lgamma(a + b) - std::lgamma(a) - std::lgamma(b) + a * std::log(x) + b * std::log1p(-x));
    const bool direct = x < (a + 1) / (a + b + 2);
    auto fraction = direct ? betaFraction(a, b, x) : betaFraction(b, a, 1 - x);
    if (!fraction)
        return {};
    return direct ? factor * *fraction / a : 1 - factor * *fraction / b;
}
void preparePrediction(WorkspaceDiagnostics& d)
{
    auto fail = [&](const char* s) { d.uncertaintyUnavailable = s; };
    if (!d.evidence.independentSurveyConfirmed)
        return fail("Independent survey evidence has not been confirmed.");
    if (!d.evidence.gaussianAffineModelConfirmed)
        return fail("Gaussian affine residual-model assumptions have not been justified.");
    const double floor = d.evidence.additionalStandardUncertaintyMeters;
    if (!std::isfinite(floor) || floor <= 0 || floor > MaximumWorkspaceSurveyStandardUncertaintyMeters ||
        !std::isfinite(floor * floor) || floor * floor == 0 || d.evidence.method.empty())
        return fail("Record a positive finite per-axis standard uncertainty and survey/model method.");
    std::vector<const WorkspaceErrorObservation*> refs;
    for (const auto& o : d.observations)
        if (o.surveyed)
            refs.push_back(&o);
    if (refs.size() < MinimumWorkspaceSurveyChecks)
        return fail("At least six independently surveyed Check references are required.");
    double area = 0;
    for (std::size_t i = 0; i < d.predictionHull.size(); ++i) {
        auto a = d.predictionHull[i], b = d.predictionHull[(i + 1) % d.predictionHull.size()];
        area += a.u * b.v - a.v * b.u;
    }
    if (area / 2 < MinimumWorkspaceSurveyAreaFraction)
        return fail("Survey coverage is too small or collinear.");
    WorkspacePredictionModel m;
    std::array<double, 9> normal{};
    std::array<double, 3> tx{}, ty{};
    for (auto o : refs) {
        auto phi = design(d.profile.bounds, o->mappedWorld);
        for (std::size_t i = 0; i < 3; ++i) {
            tx[i] += phi[i] * o->errorXMeters;
            ty[i] += phi[i] * o->errorYMeters;
            for (std::size_t j = 0; j < 3; ++j)
                normal[3 * i + j] += phi[i] * phi[j];
        }
    }
    const auto inv = inverse(normal);
    if (!inv)
        return fail("Survey residual design is singular.");
    const double condition = std::sqrt(std::inner_product(normal.begin(), normal.end(), normal.begin(), 0.0) *
                                       std::inner_product(inv->begin(), inv->end(), inv->begin(), 0.0));
    if (!std::isfinite(condition) || condition > MaximumWorkspaceRegressionCondition)
        return fail("Survey residual design is poorly constrained.");
    m.inverseNormal = *inv;
    m.betaX = multiply(*inv, tx);
    m.betaY = multiply(*inv, ty);
    m.degreesOfFreedom = refs.size() - 3;
    for (auto o : refs) {
        const auto phi = design(d.profile.bounds, o->mappedWorld);
        const double x = o->errorXMeters - dot(phi, m.betaX), y = o->errorYMeters - dot(phi, m.betaY);
        m.varianceX += x * x;
        m.varianceY += y * y;
    }
    m.varianceX = m.varianceX / static_cast<double>(m.degreesOfFreedom) + floor * floor;
    m.varianceY = m.varianceY / static_cast<double>(m.degreesOfFreedom) + floor * floor;
    // Two axes, two equal tails each: alpha/4 per tail. Endpoints use alpha=.025 each.
    auto point = workspaceStudentCritical(1 - (1 - WorkspacePointCoverage) / 4, m.degreesOfFreedom);
    auto endpoint = workspaceStudentCritical(1 - (1 - WorkspaceEndpointCoverage) / 4, m.degreesOfFreedom);
    if (!point || !endpoint || !std::isfinite(m.varianceX) || !std::isfinite(m.varianceY))
        return fail("Prediction model is not finite or quantile computation failed.");
    m.pointCritical = *point;
    m.endpointCritical = *endpoint;
    d.prediction = m;
}
} // namespace
std::optional<double> workspaceStudentCritical(double p, std::size_t df)
{
    if (!std::isfinite(p) || p <= 0.5 || p >= 1 || df == 0 || df > MaximumWorkspaceCheckPoints)
        return {};
    double lo = 0, hi = MaximumWorkspaceStudentQuantile;
    auto cdf = [df](double t) -> std::optional<double> {
        const double n = static_cast<double>(df);
        auto b = beta(n / 2, 0.5, n / (n + t * t));
        if (!b)
            return {};
        return 1 - *b / 2;
    };
    auto max = cdf(hi);
    if (!max || *max < p)
        return {};
    for (int i = 0; i < WorkspaceQuantileIterations; ++i) {
        const double mid = (lo + hi) / 2;
        auto c = cdf(mid);
        if (!c)
            return {};
        if (*c < p)
            lo = mid;
        else
            hi = mid;
    }
    return (lo + hi) / 2;
}
std::optional<double> workspaceRelativeErrorPercent(double e, const WorkspaceBounds& b)
{
    const double denominator = std::hypot(b.width().value, b.height().value);
    if (!std::isfinite(e) || e < 0 || !std::isfinite(denominator) ||
        denominator <= MinimumWorkspaceRelativeDenominatorMeters)
        return {};
    const double value = 100 * (e / denominator);
    return std::isfinite(value) ? std::optional<double>{value} : std::nullopt;
}
std::optional<double> workspacePredictionRadius(const WorkspaceDiagnostics& d, WorldPoint2D p,
                                                double coverage)
{
    if (!d.prediction || (coverage != WorkspacePointCoverage && coverage != WorkspaceEndpointCoverage) ||
        !geometricSupport(d, p) || !inside(d.predictionHull, normalized(d.profile.bounds, p)))
        return {};
    const auto& m = *d.prediction;
    const auto phi = design(d.profile.bounds, p);
    const double leverage = dot(phi, multiply(m.inverseNormal, phi));
    if (!std::isfinite(leverage) || leverage < 0)
        return {};
    const double t = coverage == WorkspacePointCoverage ? m.pointCritical : m.endpointCritical;
    const double rx = std::abs(dot(phi, m.betaX)) + t * std::sqrt((1 + leverage) * m.varianceX);
    const double ry = std::abs(dot(phi, m.betaY)) + t * std::sqrt((1 + leverage) * m.varianceY);
    const double radius = std::hypot(rx, ry);
    return std::isfinite(radius) ? std::optional<double>{radius} : std::nullopt;
}
WorkspaceMapSample sampleWorkspaceDiagnostics(const WorkspaceDiagnostics& d, WorldPoint2D p)
{
    WorkspaceMapSample s;
    s.world = p;
    s.fitCoverageKnown = bool(d.acquisition);
    s.geometrySupported = geometricSupport(d, p);
    if (!s.geometrySupported)
        return s;
    auto h = d.profile.homography->imageToWorld;
    double scale = 0;
    for (auto v : h)
        scale = std::max(scale, std::abs(v));
    if (scale == 0 || !std::isfinite(scale))
        return s;
    for (auto& v : h)
        v /= scale;
    // A projective inverse needs only the adjugate: preserve homogeneous-scale invariance.
    const auto g = adjugate(h);
    const auto image = unprojectWorkspacePoint(*d.profile.homography, p);
    if (!image)
        return s;
    const double w = g[6] * p.x.value + g[7] * p.y.value + g[8];
    const double dx = std::hypot(g[0] - image->u * g[6], g[3] - image->v * g[6]) / std::abs(w) /
                      WorkspaceMapCentimetersPerMeter;
    const double dy = std::hypot(g[1] - image->u * g[7], g[4] - image->v * g[7]) / std::abs(w) /
                      WorkspaceMapCentimetersPerMeter;
    if (std::isfinite(dx) && std::isfinite(dy)) {
        s.xPixelsPerCentimeter = dx;
        s.yPixelsPerCentimeter = dy;
    }
    if (inside(d.surveyHull, normalized(d.profile.bounds, p))) {
        // Normalized inverse-distance-squared interpolation, weighted stably by nearest distance.
        double nearest = std::numeric_limits<double>::infinity();
        const auto q = normalized(d.profile.bounds, p);
        for (const auto& o : d.observations)
            if (o.surveyed) {
                auto r = normalized(d.profile.bounds, o.point.world);
                nearest = std::min(nearest, std::hypot(q.u - r.u, q.v - r.v));
            }
        double weights = 0, mean = 0;
        for (const auto& o : d.observations)
            if (o.surveyed) {
                auto r = normalized(d.profile.bounds, o.point.world);
                const double distance = std::hypot(q.u - r.u, q.v - r.v);
                if (distance == 0) {
                    mean = o.absoluteMeters;
                    weights = 1;
                    break;
                }
                const double weight = (nearest / distance) * (nearest / distance);
                mean += (weight / (weights + weight)) * (o.absoluteMeters - mean);
                weights += weight;
            }
        if (weights > 0) {
            s.interpolatedAbsoluteMeters = mean;
            s.interpolatedRelativePercent =
                workspaceRelativeErrorPercent(*s.interpolatedAbsoluteMeters, d.profile.bounds);
        }
    }
    s.predictionRadiusMeters = workspacePredictionRadius(d, p);
    return s;
}
WorkspaceDiagnosticsOutcome
buildWorkspaceDiagnostics(WorkspaceCalibrationProfile profile,
                          std::shared_ptr<const WorkspaceAcquisitionRecord> acquisition,
                          WorkspaceSurveyEvidence evidence, std::size_t columns, std::size_t rows)
{
    auto fail = [](std::string s) { return WorkspaceDiagnosticsOutcome{{}, std::move(s)}; };
    if (auto e = validateWorkspaceCalibrationProfile(profile); !e.empty())
        return fail(e);
    if (!profile.homography)
        return fail("Activate a compatible workspace transform first.");
    if (columns < MinimumWorkspaceMapDimension || rows < MinimumWorkspaceMapDimension ||
        columns > MaximumWorkspaceMapDimension || rows > MaximumWorkspaceMapDimension)
        return fail("Sampling dimensions must be between 2 and 65; no truncation.");
    if (evidence.referenceIds.size() > MaximumWorkspaceCheckPoints ||
        evidence.method.size() > MaximumWorkspaceSurveyMethodBytes)
        return fail("Survey evidence exceeds bounded capacity.");
    auto d = std::make_shared<WorkspaceDiagnostics>(WorkspaceDiagnostics{
        std::move(profile), acquisition, std::move(evidence), columns, rows, {}, {}, {}, {}, {}, {}, {}, {}});
    std::set<std::uint64_t> selected(d->evidence.referenceIds.begin(), d->evidence.referenceIds.end());
    if (selected.size() != d->evidence.referenceIds.size())
        return fail("Survey reference IDs must be unique.");
    if (acquisition) {
        if (auto e = validateWorkspaceAcquisition(*acquisition); !e.empty())
            return fail(e);
        if (serializeWorkspaceProfile(acquisition->profile) != serializeWorkspaceProfile(d->profile))
            return fail("Acquisition does not match the complete active calibration.");
        std::vector<P> fit, image, survey, prediction;
        for (const auto& r : acquisition->references) {
            const auto mapped = projectWorkspacePoint(*d->profile.homography, r.point.image);
            if (!mapped)
                return fail("Cannot map a recorded reference.");
            const bool surveyed = selected.erase(r.id) > 0;
            if (surveyed && !r.check)
                return fail("Only held-out Check references may be surveyed evidence.");
            const double x = mapped->x.value - r.point.world.x.value,
                         y = mapped->y.value - r.point.world.y.value, e = std::hypot(x, y);
            if (!std::isfinite(e))
                return fail("Recorded reference error is not finite.");
            d->observations.push_back({r.id, r.point, r.check,
                                       surveyed && d->evidence.independentSurveyConfirmed, *mapped, x, y, e,
                                       workspaceRelativeErrorPercent(e, d->profile.bounds)});
            if (!r.check) {
                fit.push_back(normalized(d->profile.bounds, r.point.world));
                image.push_back({r.point.image.u / d->profile.intrinsics.width,
                                 r.point.image.v / d->profile.intrinsics.height});
            } else if (surveyed && d->evidence.independentSurveyConfirmed) {
                survey.push_back(normalized(d->profile.bounds, r.point.world));
                prediction.push_back(normalized(d->profile.bounds, *mapped));
            }
        }
        d->fitHull = hull(std::move(fit));
        d->imageFitHull = hull(std::move(image));
        d->surveyHull = hull(std::move(survey));
        d->predictionHull = hull(std::move(prediction));
    }
    if (!selected.empty())
        return fail("Survey IDs are unavailable in this active acquisition.");
    preparePrediction(*d);
    for (std::size_t row = 0; row < rows; ++row)
        for (std::size_t col = 0; col < columns; ++col) {
            const auto& b = d->profile.bounds;
            const WorldPoint2D p{Meters{b.minimum().x.value + b.width().value * static_cast<double>(col) /
                                                                  static_cast<double>(columns - 1)},
                                 Meters{b.minimum().y.value + b.height().value * static_cast<double>(row) /
                                                                  static_cast<double>(rows - 1)}};
            d->samples.push_back(sampleWorkspaceDiagnostics(*d, p));
        }
    return {std::move(d), {}};
}
} // namespace micro_robot
