#include "ui/WorkspaceDistanceDialog.hpp"
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QImage>
#include <QLabel>
#include <QLineEdit>
#include <QMouseEvent>
#include <QPainter>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

namespace micro_robot
{
class WorkspaceDistanceView final : public QWidget
{
public:
    explicit WorkspaceDistanceView(QWidget* parent) : QWidget(parent) { setMinimumSize(360, 260); }
    FramePtr frame;
    std::shared_ptr<const WorkspaceDiagnostics> geometry;
    std::array<std::optional<WorkspaceDistanceEndpoint>, WorkspaceDistanceEndpointCount> endpoints;
    bool worldView{};
    std::function<void(ImageCorner)> imagePicked;
    std::function<void(WorldPoint2D)> worldPicked;
    QRectF worldRect() const
    {
        if (!geometry)
            return {};
        const auto& b = geometry->profile.bounds;
        const QSizeF fitted = QSizeF(b.width().value, b.height().value)
                                  .scaled(QSizeF(width() - 70, height() - 55), Qt::KeepAspectRatio);
        return {{(width() - fitted.width()) / 2, (height() - fitted.height()) / 2}, fitted};
    }
    std::optional<QPointF> display(WorldPoint2D q) const
    {
        if (!geometry)
            return {};
        if (worldView) {
            const auto p = workspaceToView(geometry->profile.bounds, q);
            if (!p)
                return {};
            const auto r = worldRect();
            return QPointF{r.left() + p->u * r.width(), r.top() + p->v * r.height()};
        }
        if (!frame)
            return {};
        const auto pixel = unprojectWorkspacePoint(*geometry->profile.homography, q);
        const auto viewport = workspaceCameraViewport(frame->width, frame->height, width(), height());
        if (!pixel || !viewport)
            return {};
        const auto p = workspaceCameraDisplay(*viewport, *pixel);
        if (!p)
            return {};
        return QPointF{p->u, p->v};
    }

protected:
    void mousePressEvent(QMouseEvent* event) override
    {
        if (event->button() != Qt::LeftButton || !geometry)
            return;
        if (worldView) {
            const auto r = worldRect();
            if (!r.contains(event->position()))
                return;
            auto p =
                viewToWorkspace(geometry->profile.bounds, {(event->position().x() - r.left()) / r.width(),
                                                           (event->position().y() - r.top()) / r.height()});
            if (p && worldPicked)
                worldPicked(*p);
        } else if (frame) {
            const auto viewport = workspaceCameraViewport(frame->width, frame->height, width(), height());
            if (!viewport)
                return;
            const auto pixel = workspaceCameraPick(*viewport, event->position().x(), event->position().y());
            if (pixel && imagePicked)
                imagePicked(*pixel);
        }
    }
    void paintEvent(QPaintEvent*) override
    {
        QPainter painter(this);
        painter.fillRect(rect(), QColor{25, 28, 33});
        painter.setPen(Qt::white);
        if (!geometry) {
            painter.drawText(rect(), Qt::AlignCenter,
                             "Activate calibration and wait for supported geometry.");
            return;
        }
        if (worldView) {
            const auto r = worldRect();
            painter.fillRect(r, QColor{45, 50, 55});
            painter.drawRect(r);
            const auto& b = geometry->profile.bounds;
            painter.drawText(QPointF{r.left(), r.bottom() + 18},
                             QStringLiteral("%1 mm").arg(b.minimum().x.value * 1000));
            painter.drawText(QRectF{r.left(), r.bottom() + 3, r.width(), 22}, Qt::AlignRight,
                             QStringLiteral("%1 mm +X").arg(b.maximum().x.value * 1000));
            painter.drawText(QPointF{r.left() - 34, r.top() - 5},
                             QStringLiteral("+Y %1 mm").arg(b.maximum().y.value * 1000));
            painter.drawText(QPointF{r.left() - 34, r.bottom()},
                             QStringLiteral("%1 mm").arg(b.minimum().y.value * 1000));
            if (auto origin = display({})) {
                painter.setPen(Qt::gray);
                painter.drawLine(*origin + QPointF{-5, 0}, *origin + QPointF{5, 0});
                painter.drawLine(*origin + QPointF{0, -5}, *origin + QPointF{0, 5});
            }
        } else {
            if (!frame) {
                painter.drawText(rect(), Qt::AlignCenter,
                                 "Freeze a measurement frame to pick native-K undistorted pixels.");
                return;
            }
            const auto viewport = workspaceCameraViewport(frame->width, frame->height, width(), height());
            if (!viewport)
                return;
            const auto format = frame->pixelFormat == PixelFormat::Rgb8    ? QImage::Format_RGB888
                                : frame->pixelFormat == PixelFormat::Rgba8 ? QImage::Format_RGBA8888
                                                                           : QImage::Format_Grayscale8;
            QImage image(reinterpret_cast<const uchar*>(frame->pixels->data()),
                         static_cast<int>(frame->width), static_cast<int>(frame->height),
                         static_cast<qsizetype>(frame->strideBytes), format);
            painter.drawImage(QRectF{viewport->left, viewport->top, viewport->width, viewport->height},
                              image);
        }
        const auto a = endpoints[0] ? display(endpoints[0]->world) : std::nullopt,
                   b = endpoints[1] ? display(endpoints[1]->world) : std::nullopt;
        painter.setPen(QPen(Qt::white, 2));
        if (a && b)
            painter.drawLine(*a, *b);
        for (std::size_t i = 0; i < endpoints.size(); ++i)
            if (endpoints[i])
                if (auto at = display(endpoints[i]->world)) {
                    painter.setPen(QPen(i == 0 ? Qt::cyan : Qt::magenta, 2));
                    painter.drawEllipse(*at, 6, 6);
                    painter.drawText(*at + QPointF{8, -8}, i == 0 ? "A" : "B");
                }
    }
};
WorkspaceDistanceDialog::WorkspaceDistanceDialog(WorkspaceCalibrationController& source, QWidget* parent)
    : QDialog(parent), controller_(source)
{
    setWindowTitle("Measure tabletop distance");
    resize(1000, 800);
    auto* outer = new QVBoxLayout(this);
    auto* scroll = new QScrollArea(this);
    scroll->setWidgetResizable(true);
    auto* body = new QWidget(scroll);
    auto* layout = new QVBoxLayout(body);
    scroll->setWidget(body);
    outer->addWidget(scroll, 1);
    identity_ = new QLabel(body);
    identity_->setObjectName("distanceIdentity");
    identity_->setWordWrap(true);
    identity_->setTextFormat(Qt::PlainText);
    layout->addWidget(identity_);
    auto* viewRow = new QHBoxLayout;
    viewMode_ = new QComboBox(body);
    viewMode_->setObjectName("distanceViewMode");
    viewMode_->addItems({"Frozen undistorted camera", "World diagram (+Y up)"});
    target_ = new QComboBox(body);
    target_->setObjectName("distanceTarget");
    target_->addItems({"Pick / replace A", "Pick / replace B"});
    viewRow->addWidget(viewMode_);
    viewRow->addWidget(target_);
    layout->addLayout(viewRow);
    view_ = new WorkspaceDistanceView(body);
    view_->setObjectName("distanceView");
    layout->addWidget(view_, 1);
    auto* instructions =
        new QLabel("Click to set the selected endpoint. World coordinates persist when switching views. "
                   "Re-freezing or changing active calibration clears picks. Edit mapped-world coordinates "
                   "below; edits reset the selection bound. Keep camera/table fixed.",
                   body);
    instructions->setWordWrap(true);
    layout->addWidget(instructions);
    for (std::size_t i = 0; i < controls_.size(); ++i) {
        auto& c = controls_[i];
        const auto name = QString::number(i);
        auto* row = new QGridLayout;
        row->addWidget(new QLabel(i == 0 ? "A" : "B", body), 0, 0);
        auto coordinate = [&](const char* axis) {
            auto* spin = new QDoubleSpinBox(body);
            spin->setObjectName("distance" + name + axis);
            spin->setDecimals(WorkspaceDistanceCoordinateDecimals);
            spin->setRange(-MinimumWorkspaceDistanceEditorMagnitudeMeters * 1000,
                           MinimumWorkspaceDistanceEditorMagnitudeMeters * 1000);
            spin->setSuffix(QStringLiteral(" mm %1").arg(axis));
            return spin;
        };
        c.x = coordinate("X");
        c.y = coordinate("Y");
        row->addWidget(c.x, 0, 1);
        row->addWidget(c.y, 0, 2);
        c.apply = new QPushButton("Apply coordinates", body);
        c.apply->setObjectName("distance" + name + "Apply");
        row->addWidget(c.apply, 0, 3);
        auto* clear = new QPushButton("Clear", body);
        clear->setObjectName("distance" + name + "Clear");
        row->addWidget(clear, 0, 4);
        c.known = new QCheckBox("Selection disk bound known", body);
        c.known->setObjectName("distance" + name + "Known");
        row->addWidget(c.known, 1, 0, 1, 2);
        c.space = new QComboBox(body);
        c.space->setObjectName("distance" + name + "Space");
        c.space->addItems({"Native undistorted pixels", "World millimetres"});
        row->addWidget(c.space, 1, 2);
        c.radius = new QDoubleSpinBox(body);
        c.radius->setObjectName("distance" + name + "Radius");
        c.radius->setDecimals(WorkspaceDistanceCoordinateDecimals);
        c.radius->setRange(0, MaximumWorkspaceSelectionRadiusPixels);
        row->addWidget(c.radius, 1, 3);
        c.native = new QLabel(body);
        c.native->setObjectName("distance" + name + "Native");
        c.native->setTextFormat(Qt::PlainText);
        row->addWidget(c.native, 3, 1, 1, 4);
        c.justification = new QLineEdit(body);
        c.justification->setObjectName("distance" + name + "Method");
        c.justification->setMaxLength(static_cast<int>(MaximumWorkspaceSelectionMethodBytes));
        c.justification->setPlaceholderText(
            "Maximum selection-disk justification (required for ±; zero needs exact-endpoint evidence)");
        row->addWidget(c.justification, 2, 1, 1, 4);
        layout->addLayout(row);
        connect(c.justification, &QLineEdit::textEdited, this, [this, i] { selectionEdited(i); });
        connect(c.apply, &QPushButton::clicked, this, [this, i] {
            const auto& c = controls_[i];
            controller_.setWorld(i, {Meters{c.x->value() / 1000}, Meters{c.y->value() / 1000}});
        });
        connect(clear, &QPushButton::clicked, this, [this, i] { controller_.clear(i); });
        connect(c.known, &QCheckBox::toggled, this, [this, i] { selectionEdited(i); });
        connect(c.radius, &QDoubleSpinBox::valueChanged, this, [this, i] { selectionEdited(i); });
        connect(c.space, &QComboBox::currentIndexChanged, this, [this, i] { selectionEdited(i, true); });
    }
    auto* caveat = new QLabel(
        "Selection bounds are deterministic maximum disks, not default click precision or standard "
        "deviations. Zero requires an exact-endpoint justification. Calibration ± uses independently "
        "surveyed conditional prediction model and bounds the whole selection footprint. Coverage is "
        "conditional ≥95% for a fixed endpoint pair; shared calibration errors are allowed. Unsupported "
        "footprints or missing evidence show uncertainty unavailable.",
        body);
    caveat->setWordWrap(true);
    layout->addWidget(caveat);
    distance_ = new QLabel(this);
    distance_->setObjectName("distanceValue");
    distance_->setTextFormat(Qt::PlainText);
    distance_->setWordWrap(true);
    outer->addWidget(distance_);
    uncertainty_ = new QLabel(this);
    uncertainty_->setObjectName("distanceUncertainty");
    uncertainty_->setTextFormat(Qt::PlainText);
    uncertainty_->setWordWrap(true);
    outer->addWidget(uncertainty_);
    status_ = new QLabel(this);
    status_->setObjectName("distanceStatus");
    status_->setTextFormat(Qt::PlainText);
    status_->setWordWrap(true);
    outer->addWidget(status_);
    auto* actions = new QHBoxLayout;
    freeze_ = new QPushButton("Freeze measurement frame", this);
    freeze_->setObjectName("distanceFreeze");
    auto* clearAll = new QPushButton("Clear both", this);
    clearAll->setObjectName("distanceClearBoth");
    auto* close = new QPushButton("Close", this);
    actions->addWidget(freeze_);
    actions->addWidget(clearAll);
    actions->addWidget(close);
    outer->addLayout(actions);
    connect(freeze_, &QPushButton::clicked, &controller_, &WorkspaceDistanceController::freeze);
    connect(clearAll, &QPushButton::clicked, this, [this] { controller_.clear(); });
    connect(close, &QPushButton::clicked, this, &QDialog::close);
    connect(viewMode_, &QComboBox::currentIndexChanged, this, [this](int mode) {
        view_->worldView = mode == 1;
        view_->update();
    });
    view_->imagePicked = [this](ImageCorner p) {
        if (controller_.pickImage(static_cast<std::size_t>(target_->currentIndex()), p) &&
            target_->currentIndex() == 0)
            target_->setCurrentIndex(1);
    };
    view_->worldPicked = [this](WorldPoint2D p) {
        if (controller_.setWorld(static_cast<std::size_t>(target_->currentIndex()), p) &&
            target_->currentIndex() == 0)
            target_->setCurrentIndex(1);
    };
    connect(&controller_, &WorkspaceDistanceController::changed, this, &WorkspaceDistanceDialog::refresh);
    refresh();
}
void WorkspaceDistanceDialog::selectionEdited(std::size_t i, bool spaceChanged)
{
    auto& c = controls_[i];
    const bool world = c.space->currentIndex() == 1;
    controller_.setSelection(
        i, {world ? WorkspaceSelectionSpace::WorldMeters : WorkspaceSelectionSpace::NativeUndistortedPixels,
            spaceChanged || !c.known->isChecked()
                ? std::optional<double>{}
                : std::optional<double>{c.radius->value() / (world ? 1000 : 1)},
            spaceChanged ? std::string{} : c.justification->text().toStdString()});
}
void WorkspaceDistanceDialog::refresh()
{
    view_->frame = controller_.frame();
    view_->geometry = controller_.geometry();
    view_->endpoints = controller_.endpoints();
    view_->update();
    freeze_->setEnabled(controller_.canFreeze());
    const auto& geometry = controller_.geometry();
    if (geometry) {
        const auto& p = geometry->profile;
        identity_->setText(QStringLiteral("Workspace %1 · frame %2 · camera %3 · %4×%5 native-K undistorted "
                                          "pixels\n%6 · fit coverage %7")
                               .arg(QString::fromStdString(p.workspaceId))
                               .arg(QString::fromStdString(p.frameId))
                               .arg(QString::fromStdString(p.intrinsics.sourceId))
                               .arg(p.intrinsics.width)
                               .arg(p.intrinsics.height)
                               .arg(view_->frame
                                        ? QStringLiteral("Frozen frame %1").arg(view_->frame->sequence)
                                        : QStringLiteral("No camera frame frozen"))
                               .arg(geometry->acquisition ? "known" : "unavailable; declared geometry only"));
    } else
        identity_->setText("Active compatible calibration diagnostics unavailable.");
    for (std::size_t i = 0; i < controls_.size(); ++i) {
        auto& c = controls_[i];
        const auto& endpoint = controller_.endpoints()[i];
        double editorMagnitude = MinimumWorkspaceDistanceEditorMagnitudeMeters;
        if (geometry) {
            const auto& bounds = geometry->profile.bounds;
            editorMagnitude = std::max(
                {editorMagnitude, std::abs(bounds.minimum().x.value), std::abs(bounds.maximum().x.value),
                 std::abs(bounds.minimum().y.value), std::abs(bounds.maximum().y.value)});
        }
        const bool editable = editorMagnitude <= std::numeric_limits<double>::max() / 1000;
        if (editable) {
            c.x->setRange(-editorMagnitude * 1000, editorMagnitude * 1000);
            c.y->setRange(-editorMagnitude * 1000, editorMagnitude * 1000);
        }
        c.x->setEnabled(editable);
        c.y->setEnabled(editable);
        const bool same = initialized_ && bool(shownWorld_[i]) == bool(endpoint) &&
                          (!endpoint || (shownWorld_[i]->x.value == endpoint->world.x.value &&
                                         shownWorld_[i]->y.value == endpoint->world.y.value));
        if (!same) {
            shownWorld_[i] = endpoint ? std::optional<WorldPoint2D>{endpoint->world} : std::nullopt;
            c.x->setValue(endpoint ? endpoint->world.x.value * 1000 : 0);
            c.y->setValue(endpoint ? endpoint->world.y.value * 1000 : 0);
        }
        c.apply->setEnabled(bool(geometry) && editable);
        const QSignalBlocker a(c.known), b(c.space), r(c.radius), j(c.justification);
        const auto selection = endpoint ? endpoint->selection : WorkspaceEndpointSelection{};
        const bool world = selection.space == WorkspaceSelectionSpace::WorldMeters;
        c.justification->setText(QString::fromStdString(selection.justification));
        c.justification->setEnabled(bool(endpoint));
        c.space->setCurrentIndex(world ? 1 : 0);
        c.known->setChecked(bool(selection.maximumRadius));
        c.radius->setRange(0, world ? MaximumWorkspaceSelectionRadiusMeters * 1000
                                    : MaximumWorkspaceSelectionRadiusPixels);
        c.radius->setSuffix(world ? " mm maximum" : " px maximum");
        c.radius->setValue(selection.maximumRadius ? *selection.maximumRadius * (world ? 1000 : 1) : 0);
        c.known->setEnabled(bool(endpoint));
        c.space->setEnabled(bool(endpoint));
        c.radius->setEnabled(endpoint && bool(selection.maximumRadius));
        if (endpoint && geometry) {
            auto p = unprojectWorkspacePoint(*geometry->profile.homography, endpoint->world);
            c.native->setText(p ? QStringLiteral("Mapped world (%1, %2) mm · native undistorted (%3, %4) px")
                                      .arg(endpoint->world.x.value * 1000, 0, 'g', 9)
                                      .arg(endpoint->world.y.value * 1000, 0, 'g', 9)
                                      .arg(p->u, 0, 'g', 9)
                                      .arg(p->v, 0, 'g', 9)
                                : "Native projection unavailable");
        } else
            c.native->setText("Endpoint not selected");
    }
    initialized_ = true;
    status_->setText(controller_.message());
    if (const auto& m = controller_.measurement(); m.result) {
        const auto& result = *m.result;
        distance_->setText(QStringLiteral("Planar distance: %1 cm (%2 mm)")
                               .arg(result.distanceMeters * 100, 0, 'g', 9)
                               .arg(result.distanceMeters * 1000, 0, 'g', 9));
        uncertainty_->setText(
            result.plusMinusMeters
                ? QStringLiteral("± %1 mm · conditional ≥95% for a fixed endpoint pair · interval [%2, %3] "
                                 "mm\nIncludes declared selection bounds and calibration prediction; "
                                 "endpoint independence is not assumed.")
                      .arg(*result.plusMinusMeters * 1000, 0, 'g', 6)
                      .arg(*result.lowerMeters * 1000, 0, 'g', 8)
                      .arg(*result.upperMeters * 1000, 0, 'g', 8)
                : QStringLiteral("Uncertainty unavailable: %1")
                      .arg(QString::fromStdString(result.uncertaintyUnavailable)));
    } else {
        distance_->setText("Select two supported endpoints to measure planar distance.");
        uncertainty_->setText(
            controller_.measurement().error.empty()
                ? QStringLiteral(
                      "Uncertainty unavailable until both endpoints and sufficient evidence are present.")
                : QString::fromStdString(controller_.measurement().error));
    }
}
} // namespace micro_robot
