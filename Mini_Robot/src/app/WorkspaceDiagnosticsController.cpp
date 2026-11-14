#include "app/WorkspaceDiagnosticsController.hpp"
#include "calibration/WorkspaceProfile.hpp"
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMetaObject>
#include <QSaveFile>
#include <exception>

namespace micro_robot
{
namespace
{
QJsonValue number(std::optional<double> v) { return v ? QJsonValue{*v} : QJsonValue{QJsonValue::Null}; }
QString id(std::uint64_t n) { return QString::number(n); } // String preserves full 64-bit IDs.
QJsonArray point(WorldPoint2D p) { return {p.x.value, p.y.value}; }
} // namespace
std::string exportWorkspaceDiagnostics(const WorkspaceDiagnostics& d, const QString& path)
{
    try {
        QJsonObject root{
            {"schema", "mini-robot-workspace-diagnostics-v1"},
            {"diagnosticPlane", QString::fromStdString(d.diagnosticPlane)},
            {"worldFrameId", QString::fromStdString(d.profile.frameId)},
            {"calibrationRevision", QString::fromStdString(serializeWorkspaceProfile(d.profile))},
            {"observedPerAxisToleranceMeters", WorkspaceObservedAxisToleranceMeters},
            {"toleranceMeaning", "Observed signed X/Y components only; interpolation, radial radii and conditional 95% coverage do not certify maximum robot error"},
            {"profileCanonical", QString::fromStdString(serializeWorkspaceProfile(d.profile))},
            {"acquisitionCanonical",
             d.acquisition ? QJsonValue{QString::fromStdString(serializeWorkspaceAcquisition(*d.acquisition))}
                           : QJsonValue{QJsonValue::Null}},
            {"coordinates", "world metres; native-K undistorted pixel centres; world +Y up"},
            {"relativeDenominator",
             "workspace diagonal in metres; percentage = 100 * absolute error / diagonal"},
            {"interpolation", "inverse squared distance in normalized world extents; confirmed survey hull "
                              "only; not an uncertainty bound"},
            {"predictionMethod", "affine per-axis OLS residual prediction against normalized H-mapped "
                                 "coordinates; SSE/(n-3) + positive additional standard uncertainty squared; "
                                 "Student-t with axis Bonferroni; unchanged H coordinates"},
            {"coverageMeaning",
             "pointwise conditional >=95% two-axis prediction, not simultaneous map coverage; requires "
             "independent Gaussian homoscedastic affine residual observations and fixed H-mapped pixel "
             "predictors/model; shared camera/target systematics beyond model are not covered"},
            {"endpointContract",
             "97.5% per-endpoint two-axis bounds; sum radii gives >=95% pair bound by union inequality "
             "without endpoint independence; endpoint selection bounds must be added separately"},
            {"units", QJsonObject{{"scale", "px/cm"},
                                  {"absolute", "m"},
                                  {"relative", "% of workspace diagonal"},
                                  {"predictionRadius", "m"}}},
            {"columns", static_cast<int>(d.columns)},
            {"rows", static_cast<int>(d.rows)},
            {"rowOrder", "minimum to maximum world Y; columns minimum to maximum world X"},
            {"uncertaintyUnavailable", QString::fromStdString(d.uncertaintyUnavailable)}};
        QJsonArray ids;
        for (auto n : d.evidence.referenceIds)
            ids.append(id(n));
        root["surveyEvidence"] = QJsonObject{
            {"referenceIds", ids},
            {"independentSurveyConfirmed", d.evidence.independentSurveyConfirmed},
            {"gaussianAffineModelConfirmed", d.evidence.gaussianAffineModelConfirmed},
            {"additionalStandardUncertaintyMeters", number(d.evidence.additionalStandardUncertaintyMeters)},
            {"method", QString::fromStdString(d.evidence.method)}};
        QJsonArray refs;
        for (const auto& o : d.observations)
            refs.append(QJsonObject{{"id", id(o.referenceId)},
                                    {"image", QJsonArray{o.point.image.u, o.point.image.v}},
                                    {"world", point(o.point.world)},
                                    {"mappedWorld", point(o.mappedWorld)},
                                    {"check", o.check},
                                    {"surveyed", o.surveyed},
                                    {"errorXMeters", o.errorXMeters},
                                    {"errorYMeters", o.errorYMeters},
                                    {"absoluteMeters", o.absoluteMeters},
                                    {"relativePercent", number(o.relativePercent)}});
        root["observations"] = refs;
        QJsonArray samples;
        for (const auto& s : d.samples)
            samples.append(QJsonObject{{"world", point(s.world)},
                                       {"geometrySupported", s.geometrySupported},
                                       {"fitCoverageKnown", s.fitCoverageKnown},
                                       {"xPixelsPerCentimeter", number(s.xPixelsPerCentimeter)},
                                       {"yPixelsPerCentimeter", number(s.yPixelsPerCentimeter)},
                                       {"interpolatedAbsoluteMeters", number(s.interpolatedAbsoluteMeters)},
                                       {"interpolatedRelativePercent", number(s.interpolatedRelativePercent)},
                                       {"predictionRadiusMeters", number(s.predictionRadiusMeters)}});
        root["samples"] = samples;
        auto hull = [](const auto& points) {
            QJsonArray a;
            for (auto p : points)
                a.append(QJsonArray{p.u, p.v});
            return a;
        };
        root["fitHullNormalizedWorld"] = hull(d.fitHull);
        root["fitHullNormalizedImage"] = hull(d.imageFitHull);
        root["surveyHullNormalizedWorld"] = hull(d.surveyHull);
        root["predictionHullNormalizedMappedWorld"] = hull(d.predictionHull);
        if (d.prediction) {
            auto array = [](const auto& v) {
                QJsonArray a;
                for (auto n : v)
                    a.append(n);
                return a;
            };
            const auto& m = *d.prediction;
            root["prediction"] = QJsonObject{{"inverseNormal", array(m.inverseNormal)},
                                             {"betaX", array(m.betaX)},
                                             {"betaY", array(m.betaY)},
                                             {"varianceX", m.varianceX},
                                             {"varianceY", m.varianceY},
                                             {"degreesOfFreedom", static_cast<int>(m.degreesOfFreedom)},
                                             {"pointCritical", m.pointCritical},
                                             {"endpointCritical", m.endpointCritical},
                                             {"pointCoverage", WorkspacePointCoverage},
                                             {"endpointCoverage", WorkspaceEndpointCoverage}};
        }
        const auto bytes = QJsonDocument(root).toJson(QJsonDocument::Indented);
        if (static_cast<std::size_t>(bytes.size()) > MaximumWorkspaceDiagnosticsExportBytes)
            return "Diagnostics export exceeds the byte limit.";
        QSaveFile file(path);
        file.setDirectWriteFallback(false);
        if (!file.open(QIODevice::WriteOnly) || file.write(bytes) != bytes.size() || !file.commit())
            return file.errorString().toStdString();
    } catch (const std::exception& e) {
        return e.what();
    } catch (...) {
        return "Unknown diagnostics export failure.";
    }
    return {};
}
WorkspaceDiagnosticsController::WorkspaceDiagnosticsController(QObject* parent, Builder builder)
    : QObject(parent), builder_(std::move(builder))
{
    context_ = new QObject;
    context_->moveToThread(&worker_);
    connect(&worker_, &QThread::finished, context_, &QObject::deleteLater);
    connect(&worker_, &QThread::finished, this, &WorkspaceDiagnosticsController::stopped);
    worker_.start();
}
WorkspaceDiagnosticsController::~WorkspaceDiagnosticsController()
{
    worker_.quit();
    worker_.wait();
}
void WorkspaceDiagnosticsController::setCalibration(std::optional<WorkspaceCalibrationProfile> p,
                                                    std::shared_ptr<const WorkspaceAcquisitionRecord> a)
{
    if (shuttingDown_)
        return;
    ++generation_;
    pending_.reset();
    snapshot_.reset();
    profile_ = std::move(p);
    acquisition_ = std::move(a);
    evidence_ = {};
    message_ = profile_ ? QStringLiteral("Preparing current calibration maps…")
                        : QStringLiteral("Active calibration unavailable; maps and survey settings cleared.");
    emit calibrationChanged();
    emit changed();
    if (profile_)
        request();
}
void WorkspaceDiagnosticsController::request(WorkspaceSurveyEvidence e, std::size_t columns, std::size_t rows)
{
    if (shuttingDown_ || !profile_)
        return;
    ++generation_;
    snapshot_.reset();
    evidence_ = std::move(e);
    // Reject oversized requests before copying them into worker buffering.
    if (evidence_.referenceIds.size() > MaximumWorkspaceCheckPoints ||
        evidence_.method.size() > MaximumWorkspaceSurveyMethodBytes ||
        columns < MinimumWorkspaceMapDimension || rows < MinimumWorkspaceMapDimension ||
        columns > MaximumWorkspaceMapDimension || rows > MaximumWorkspaceMapDimension) {
        pending_.reset();
        message_ = "Map request exceeds bounded capacity.";
        emit changed();
        return;
    }
    Task task{*profile_, acquisition_, evidence_, columns, rows, generation_};
    if (running_) {
        pending_ = std::move(task);
        message_ = "Latest map request pending; obsolete results will be discarded.";
        emit changed();
    } else
        launch(std::move(task));
}
void WorkspaceDiagnosticsController::launch(Task task)
{
    running_ = true;
    message_ = "Computing bounded scale, surveyed error and conditional prediction maps…";
    emit changed();
    QMetaObject::invokeMethod(
        context_,
        [this, task = std::move(task)]() mutable {
            WorkspaceDiagnosticsOutcome outcome;
            try {
                outcome = builder_(std::move(task.profile), std::move(task.acquisition),
                                   std::move(task.evidence), task.columns, task.rows);
            } catch (const std::exception& e) {
                outcome.error = e.what();
            } catch (...) {
                outcome.error = "Unknown map computation failure.";
            }
            QMetaObject::invokeMethod(
                this,
                [this, generation = task.generation, outcome = std::move(outcome)] {
                    if (!shuttingDown_ && generation == generation_) {
                        snapshot_ = outcome.result;
                        message_ =
                            snapshot_
                                ? (snapshot_->prediction
                                       ? QStringLiteral(
                                             "Conditional prediction model ready; pointwise coverage only. "
                                             "Physical validation remains separate.")
                                       : QStringLiteral(
                                             "Scale/residual maps ready. Uncertainty unavailable: %1")
                                             .arg(QString::fromStdString(snapshot_->uncertaintyUnavailable)))
                                : QString::fromStdString(outcome.error);
                    }
                    finished();
                },
                Qt::QueuedConnection);
        },
        Qt::QueuedConnection);
}
void WorkspaceDiagnosticsController::finished()
{
    running_ = false;
    if (pending_ && !shuttingDown_) {
        auto task = std::move(*pending_);
        pending_.reset();
        launch(std::move(task));
    } else
        emit changed();
}
void WorkspaceDiagnosticsController::exportSnapshot(const QString& path)
{
    if (shuttingDown_ || running_ || !snapshot_ || path.isEmpty())
        return;
    const auto snapshot = snapshot_;
    const auto generation = generation_;
    running_ = true;
    message_ = "Exporting immutable diagnostic snapshot…";
    emit changed();
    QMetaObject::invokeMethod(
        context_,
        [this, snapshot, path, generation] {
            auto error = exportWorkspaceDiagnostics(*snapshot, path);
            QMetaObject::invokeMethod(
                this,
                [this, error, generation] {
                    if (!shuttingDown_ && generation == generation_)
                        message_ =
                            error.empty()
                                ? QStringLiteral("Diagnostic snapshot exported atomically.")
                                : QStringLiteral("Export failed: %1").arg(QString::fromStdString(error));
                    finished();
                },
                Qt::QueuedConnection);
        },
        Qt::QueuedConnection);
}
void WorkspaceDiagnosticsController::shutdown()
{
    if (shuttingDown_)
        return;
    shuttingDown_ = true;
    ++generation_;
    pending_.reset();
    snapshot_.reset();
    profile_.reset();
    acquisition_.reset();
    evidence_ = {};
    worker_.quit();
    emit calibrationChanged();
    emit changed();
}
} // namespace micro_robot
