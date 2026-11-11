#include "ui/WorkspaceDiagnosticsDialog.hpp"
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMouseEvent>
#include <QPainter>
#include <QPushButton>
#include <QScrollArea>
#include <QTableView>
#include <QToolTip>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>

namespace micro_robot
{
namespace
{
QString shown(std::optional<double> value, const char* unit, double factor = 1)
{
    return value ? QStringLiteral("%1 %2").arg(*value * factor, 0, 'g', 6).arg(unit)
                 : QStringLiteral("unavailable");
}
std::optional<double> millimeters(std::optional<double> value)
{
    if (!value || !std::isfinite(*value * 1000))
        return {};
    return *value * 1000;
}
QColor color(double value, double minimum, double maximum)
{
    const double fraction =
        maximum > minimum ? std::clamp((value - minimum) / (maximum - minimum), 0.0, 1.0) : 0.5;
    return QColor::fromHsvF((1 - fraction) * 0.66, 0.85, 0.95);
}
} // namespace
class WorkspaceMapView final : public QWidget
{
public:
    explicit WorkspaceMapView(QWidget* parent) : QWidget(parent)
    {
        setMouseTracking(true);
        setMinimumSize(420, 300);
    }
    std::shared_ptr<const WorkspaceDiagnostics> snapshot;
    int mode{};
    QRectF plot() const
    {
        return {60, 30, static_cast<double>(width() - 85), static_cast<double>(height() - 125)};
    }
    std::optional<double> value(const WorkspaceMapSample& s) const
    {
        switch (mode) {
        case 0:
            return s.xPixelsPerCentimeter;
        case 1:
            return s.yPixelsPerCentimeter;
        case 2:
            return millimeters(s.interpolatedAbsoluteMeters);
        case 3:
            return s.interpolatedRelativePercent;
        default:
            return millimeters(s.predictionRadiusMeters);
        }
    }
    QString units() const { return mode < 2 ? "px/cm" : mode == 3 ? "% of workspace diagonal" : "mm"; }

protected:
    void paintEvent(QPaintEvent*) override
    {
        QPainter painter(this);
        painter.fillRect(rect(), QColor{25, 28, 33});
        painter.setPen(Qt::white);
        if (!snapshot) {
            painter.drawText(rect(), Qt::AlignCenter, "Current maps unavailable / computing");
            return;
        }
        const auto& d = *snapshot;
        const auto r = plot();
        double minimum = std::numeric_limits<double>::infinity(), maximum = -minimum;
        std::size_t available = 0;
        for (const auto& s : d.samples)
            if (auto v = value(s)) {
                minimum = std::min(minimum, *v);
                maximum = std::max(maximum, *v);
                ++available;
            }
        QImage raster(static_cast<int>(d.columns), static_cast<int>(d.rows), QImage::Format_RGB32);
        for (std::size_t y = 0; y < d.rows; ++y)
            for (std::size_t x = 0; x < d.columns; ++x) {
                const auto v = value(d.samples[y * d.columns + x]);
                raster.setPixelColor(static_cast<int>(x), static_cast<int>(d.rows - 1 - y),
                                     v ? color(*v, minimum, maximum)
                                       : ((x + y) % 2 ? QColor{70, 70, 70} : QColor{95, 95, 95}));
            }
        // Each sample owns its nearest-node cell; half-sized edge cells preserve exact world coordinates.
        const double cellX = r.width() / static_cast<double>(d.columns - 1),
                     cellY = r.height() / static_cast<double>(d.rows - 1);
        painter.save();
        painter.setClipRect(r);
        painter.drawImage(r.adjusted(-cellX / 2, -cellY / 2, cellX / 2, cellY / 2), raster);
        painter.restore();
        painter.setPen(Qt::white);
        painter.drawRect(r);
        const auto& b = d.profile.bounds;
        painter.drawText(QRectF{r.left(), r.bottom() + 4, r.width(), 22}, Qt::AlignLeft,
                         QStringLiteral("%1 m").arg(b.minimum().x.value));
        painter.drawText(QRectF{r.left(), r.bottom() + 4, r.width(), 22}, Qt::AlignRight,
                         QStringLiteral("%1 m  +X").arg(b.maximum().x.value));
        painter.drawText(QPointF{3, r.top() + 10}, QStringLiteral("%1 m").arg(b.maximum().y.value));
        painter.drawText(QPointF{3, r.top() - 5}, "+Y");
        painter.drawText(QPointF{3, r.bottom()}, QStringLiteral("%1 m").arg(b.minimum().y.value));
        painter.save();
        painter.setClipRect(r.adjusted(-5, -5, 5, 5));
        std::size_t labelled = 0;
        for (const auto& o : d.observations) {
            const auto q = workspaceToView(b, o.point.world);
            if (!q)
                continue;
            const QPointF at{r.left() + q->u * r.width(), r.top() + q->v * r.height()};
            painter.setPen(QPen(o.surveyed ? Qt::white : o.check ? Qt::yellow : Qt::green, 1.5));
            if (o.surveyed) {
                painter.drawLine(at + QPointF{-4, -4}, at + QPointF{4, 4});
                painter.drawLine(at + QPointF{-4, 4}, at + QPointF{4, -4});
            } else if (o.check)
                painter.drawEllipse(at, 3, 3);
            else
                painter.drawRect(QRectF{at - QPointF{2, 2}, QSizeF{4, 4}});
            if (labelled++ < MaximumWorkspaceMapReferenceLabels)
                painter.drawText(at + QPointF{5, -5}, QString::number(o.referenceId));
        }
        painter.restore();
        painter.setPen(Qt::white);
        const QRectF legend{r.left(), r.bottom() + 34, r.width(), 14};
        if (available) {
            QLinearGradient gradient(legend.topLeft(), legend.topRight());
            for (int i = 0; i <= WorkspaceMapLegendStops; ++i) {
                const double fraction = static_cast<double>(i) / WorkspaceMapLegendStops;
                gradient.setColorAt(fraction,
                                    color(minimum + fraction * (maximum - minimum), minimum, maximum));
            }
            painter.fillRect(legend, gradient);
            painter.drawText(QRectF{legend.left(), legend.bottom() + 2, legend.width(), 20}, Qt::AlignLeft,
                             QStringLiteral("%1 %2").arg(minimum, 0, 'g', 5).arg(units()));
            painter.drawText(QRectF{legend.left(), legend.bottom() + 2, legend.width(), 20}, Qt::AlignRight,
                             QStringLiteral("%1 %2").arg(maximum, 0, 'g', 5).arg(units()));
        } else
            painter.drawText(legend, Qt::AlignCenter, "No supported samples for this field");
        painter.drawText(QRectF{r.left(), height() - 24.0, r.width(), 24}, Qt::AlignLeft,
                         QStringLiteral("%1/%2 samples · grey: unavailable · hover for sample values")
                             .arg(available)
                             .arg(d.samples.size()));
    }
    void mouseMoveEvent(QMouseEvent* event) override
    {
        if (!snapshot || !plot().contains(event->position()))
            return;
        const auto r = plot();
        const auto& d = *snapshot;
        const auto x = static_cast<std::size_t>(
            std::lround((event->position().x() - r.left()) / r.width() * static_cast<double>(d.columns - 1)));
        const auto y = static_cast<std::size_t>(
            std::lround((r.bottom() - event->position().y()) / r.height() * static_cast<double>(d.rows - 1)));
        const auto& s = d.samples[y * d.columns + x];
        QToolTip::showText(
            event->globalPosition().toPoint(),
            QStringLiteral("Sample world (%1, %2) m\nX %3; Y %4\nInterpolated survey error %5; "
                           "%6\nConditional pointwise 95% radius %7\nGeometry %8; fit coverage %9")
                .arg(s.world.x.value, 0, 'g', 8)
                .arg(s.world.y.value, 0, 'g', 8)
                .arg(shown(s.xPixelsPerCentimeter, "px/cm"))
                .arg(shown(s.yPixelsPerCentimeter, "px/cm"))
                .arg(shown(s.interpolatedAbsoluteMeters, "mm", 1000))
                .arg(shown(s.interpolatedRelativePercent, "% diagonal"))
                .arg(shown(s.predictionRadiusMeters, "mm", 1000))
                .arg(s.geometrySupported ? "supported" : "unavailable")
                .arg(s.fitCoverageKnown ? "known" : "unavailable"),
            this);
    }
};
void WorkspaceSurveyModel::setAcquisition(std::shared_ptr<const WorkspaceAcquisitionRecord> a,
                                          const std::vector<std::uint64_t>& selected)
{
    beginResetModel();
    acquisition_ = std::move(a);
    selected_ = {selected.begin(), selected.end()};
    endResetModel();
}
std::vector<std::uint64_t> WorkspaceSurveyModel::selectedIds() const
{
    return {selected_.begin(), selected_.end()};
}
int WorkspaceSurveyModel::rowCount(const QModelIndex& p) const
{
    return p.isValid() || !acquisition_ ? 0 : static_cast<int>(acquisition_->references.size());
}
int WorkspaceSurveyModel::columnCount(const QModelIndex& p) const { return p.isValid() ? 0 : 7; }
QVariant WorkspaceSurveyModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (role != Qt::DisplayRole)
        return {};
    if (orientation == Qt::Vertical)
        return section + 1;
    const QStringList labels{"Surveyed Check",         "ID / role",  "World X (m)", "World Y (m)",
                             "Absolute residual (mm)", "% diagonal", "Source"};
    return section >= 0 && section < labels.size() ? QVariant{labels[section]} : QVariant{};
}
QVariant WorkspaceSurveyModel::data(const QModelIndex& index, int role) const
{
    if (!acquisition_ || !index.isValid() || index.row() < 0 || index.row() >= rowCount() ||
        index.column() < 0 || index.column() >= columnCount())
        return {};
    const auto& r = acquisition_->references[static_cast<std::size_t>(index.row())];
    if (role == Qt::CheckStateRole && index.column() == 0 && r.check)
        return selected_.contains(r.id) ? Qt::Checked : Qt::Unchecked;
    if (role == Qt::ToolTipRole)
        return "Select only held-out references whose stored world coordinates were independently surveyed. "
               "Shared board spacing, anchors or orientation alone are correlated evidence. Selection does "
               "not alter calibration or certify independence.";
    if (role != Qt::DisplayRole)
        return {};
    switch (index.column()) {
    case 0:
        return r.check ? "eligible" : "fit — ineligible";
    case 1:
        return QStringLiteral("%1 / %2").arg(r.id).arg(r.check ? "Check" : "Fit");
    case 2:
        return r.point.world.x.value;
    case 3:
        return r.point.world.y.value;
    case 4: {
        if (!acquisition_->profile.homography)
            return {};
        auto p = projectWorkspacePoint(*acquisition_->profile.homography, r.point.image);
        return p ? QVariant{1000 * std::hypot(p->x.value - r.point.world.x.value,
                                              p->y.value - r.point.world.y.value)}
                 : QVariant{};
    }
    case 5: {
        if (!acquisition_->profile.homography)
            return {};
        const auto p = projectWorkspacePoint(*acquisition_->profile.homography, r.point.image);
        if (!p)
            return {};
        const auto relative = workspaceRelativeErrorPercent(
            std::hypot(p->x.value - r.point.world.x.value, p->y.value - r.point.world.y.value),
            acquisition_->profile.bounds);
        return relative ? QVariant{*relative} : QVariant{QStringLiteral("unavailable")};
    }
    default:
        return r.source ? QStringLiteral("Board %1 / corner %2")
                              .arg(r.source->placementId)
                              .arg(r.source->cornerIndex)
                        : QStringLiteral("Manual / boundary");
    }
}
Qt::ItemFlags WorkspaceSurveyModel::flags(const QModelIndex& i) const
{
    auto f = QAbstractTableModel::flags(i);
    if (acquisition_ && i.isValid() && i.row() >= 0 && i.row() < rowCount() && i.column() == 0 &&
        acquisition_->references[static_cast<std::size_t>(i.row())].check)
        f |= Qt::ItemIsUserCheckable;
    return f;
}
bool WorkspaceSurveyModel::setData(const QModelIndex& i, const QVariant& value, int role)
{
    if (role != Qt::CheckStateRole || !(flags(i) & Qt::ItemIsUserCheckable))
        return false;
    const auto id = acquisition_->references[static_cast<std::size_t>(i.row())].id;
    if (value.toInt() == Qt::Checked)
        selected_.insert(id);
    else
        selected_.erase(id);
    emit dataChanged(i, i, {Qt::CheckStateRole});
    return true;
}
WorkspaceDiagnosticsDialog::WorkspaceDiagnosticsDialog(WorkspaceDiagnosticsController& controller,
                                                       QWidget* parent)
    : QDialog(parent), controller_(controller)
{
    setWindowTitle("Workspace scale, observed error and conditional uncertainty");
    resize(1000, 850);
    auto* outer = new QVBoxLayout(this);
    auto* scroll = new QScrollArea(this);
    scroll->setWidgetResizable(true);
    auto* body = new QWidget(scroll);
    auto* layout = new QVBoxLayout(body);
    scroll->setWidget(body);
    outer->addWidget(scroll, 1);
    identity_ = new QLabel(body);
    identity_->setObjectName("mapIdentity");
    identity_->setWordWrap(true);
    identity_->setTextFormat(Qt::PlainText);
    layout->addWidget(identity_);
    mode_ = new QComboBox(body);
    mode_->setObjectName("mapMode");
    mode_->addItems({"Local scale along world X (px/cm)", "Local scale along world Y (px/cm)",
                     "Interpolated surveyed absolute error (mm)",
                     "Interpolated surveyed error (% of workspace diagonal)",
                     "Conditional pointwise 95% prediction radius (mm)"});
    layout->addWidget(mode_);
    map_ = new WorkspaceMapView(body);
    map_->setObjectName("workspaceMap");
    layout->addWidget(map_, 1);
    auto* explanation = new QLabel(
        "Green squares: fitted residuals. Yellow circles: held-out Checks (independence unconfirmed). White "
        "crosses: explicitly surveyed Checks. All reference positions retained; up to 32 IDs labelled. Grey "
        "cells: unsupported / extrapolated / missing evidence. Error fields interpolate observed error; they "
        "are not uncertainty bounds. Scale is geometric, not a statement of accuracy.",
        body);
    explanation->setWordWrap(true);
    layout->addWidget(explanation);
    auto* table = new QTableView(body);
    table->setObjectName("surveyReferences");
    survey_ = new WorkspaceSurveyModel(table);
    table->setModel(survey_);
    table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    table->setMinimumHeight(110);
    table->setMaximumHeight(170);
    layout->addWidget(table);
    independent_ = new QCheckBox("Confirm independent survey of selected stored Check coordinates", body);
    independent_->setObjectName("surveyIndependent");
    layout->addWidget(independent_);
    assumptions_ = new QCheckBox("Confirm conditional Gaussian affine residual-model assumptions", body);
    assumptions_->setObjectName("surveyAssumptions");
    layout->addWidget(assumptions_);
    auto* evidenceRow = new QHBoxLayout;
    floor_ = new QDoubleSpinBox(body);
    floor_->setObjectName("surveyFloor");
    floor_->setDecimals(6);
    floor_->setRange(0, MaximumWorkspaceSurveyStandardUncertaintyMeters * 1000);
    floor_->setSuffix(" mm additional per-axis standard uncertainty");
    evidenceRow->addWidget(floor_);
    method_ = new QLineEdit(body);
    method_->setObjectName("surveyMethod");
    method_->setMaxLength(static_cast<int>(MaximumWorkspaceSurveyMethodBytes));
    method_->setPlaceholderText("Survey method, uncertainty budget and model justification (required)");
    evidenceRow->addWidget(method_, 1);
    layout->addLayout(evidenceRow);
    auto* caveat = new QLabel(
        "Displayed maps use the last applied evidence. Stored Check coordinates must be independently "
        "surveyed; board spacing/anchors alone do not qualify. The model assumes independent Gaussian, "
        "constant-variance residual observations conditional on fixed H-mapped pixel predictors. Prediction "
        "needs ≥6 surveyed Checks spanning a well-constrained region and a positive uncertainty floor. "
        "Coverage is conditional and pointwise; the map is not a simultaneous 95% guarantee. Unmodelled "
        "camera/target systematics or model failure invalidate it. Coordinates remain those of H. No "
        "independent evidence means uncertainty unavailable.",
        body);
    caveat->setWordWrap(true);
    layout->addWidget(caveat);
    status_ = new QLabel(this);
    status_->setObjectName("mapStatus");
    status_->setTextFormat(Qt::PlainText);
    status_->setWordWrap(true);
    outer->addWidget(status_);
    auto* actions = new QHBoxLayout;
    compute_ = new QPushButton("Apply survey evidence / recompute", this);
    compute_->setObjectName("mapCompute");
    export_ = new QPushButton("Export samples and evidence…", this);
    export_->setObjectName("mapExport");
    auto* close = new QPushButton("Close", this);
    actions->addWidget(compute_);
    actions->addWidget(export_);
    actions->addWidget(close);
    outer->addLayout(actions);
    connect(mode_, &QComboBox::currentIndexChanged, this, [this](int i) {
        map_->mode = i;
        map_->update();
    });
    connect(compute_, &QPushButton::clicked, this, &WorkspaceDiagnosticsDialog::recompute);
    connect(export_, &QPushButton::clicked, this, [this] {
        const auto path = QFileDialog::getSaveFileName(this, "Export current diagnostic snapshot", {},
                                                       "Workspace diagnostics (*.json)");
        if (!path.isEmpty())
            controller_.exportSnapshot(path);
    });
    connect(close, &QPushButton::clicked, this, &QDialog::close);
    connect(&controller_, &WorkspaceDiagnosticsController::changed, this,
            &WorkspaceDiagnosticsDialog::refresh);
    connect(&controller_, &WorkspaceDiagnosticsController::calibrationChanged, this,
            &WorkspaceDiagnosticsDialog::resetCalibration);
    resetCalibration();
    refresh();
}
void WorkspaceDiagnosticsDialog::resetCalibration()
{
    const auto& e = controller_.evidence();
    survey_->setAcquisition(controller_.acquisition(), e.referenceIds);
    independent_->setChecked(e.independentSurveyConfirmed);
    assumptions_->setChecked(e.gaussianAffineModelConfirmed);
    floor_->setValue(e.additionalStandardUncertaintyMeters * 1000);
    method_->setText(QString::fromStdString(e.method));
}
void WorkspaceDiagnosticsDialog::recompute()
{
    controller_.request({survey_->selectedIds(), independent_->isChecked(), assumptions_->isChecked(),
                         floor_->value() / 1000, method_->text().toStdString()});
}
void WorkspaceDiagnosticsDialog::refresh()
{
    map_->snapshot = controller_.snapshot();
    map_->update();
    status_->setText(controller_.message());
    compute_->setEnabled(bool(controller_.profile()));
    export_->setEnabled(bool(controller_.snapshot()) && !controller_.busy());
    if (const auto& p = controller_.profile())
        identity_->setText(
            QStringLiteral("Active workspace %1 · frame %2 · acquisition %3\nCamera %4 · %5×%6 native-K "
                           "undistorted pixels · grid %7×%8 · known fit coverage: %9")
                .arg(QString::fromStdString(p->workspaceId))
                .arg(QString::fromStdString(p->frameId))
                .arg(p->acquisitionId.empty() ? "unavailable" : QString::fromStdString(p->acquisitionId))
                .arg(QString::fromStdString(p->intrinsics.sourceId))
                .arg(p->intrinsics.width)
                .arg(p->intrinsics.height)
                .arg(controller_.snapshot() ? controller_.snapshot()->columns : DefaultWorkspaceMapColumns)
                .arg(controller_.snapshot() ? controller_.snapshot()->rows : DefaultWorkspaceMapRows)
                .arg(controller_.acquisition() ? "yes" : "unavailable (geometric scale only)"));
    else
        identity_->setText("No active compatible workspace calibration. Maps and evidence cleared.");
}
} // namespace micro_robot
