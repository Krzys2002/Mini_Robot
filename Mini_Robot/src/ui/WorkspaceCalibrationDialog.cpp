#include "ui/WorkspaceCalibrationDialog.hpp"
#include <QDoubleSpinBox>
#include <QHeaderView>
#include <QImage>
#include <QLabel>
#include <QLineEdit>
#include <QLocale>
#include <QMouseEvent>
#include <QPainter>
#include <QPushButton>
#include <QSignalBlocker>
#include <QTableWidget>
#include <QVBoxLayout>
#include <cmath>
#include <functional>
#include <limits>

namespace micro_robot {
class WorkspaceImageView final : public QWidget {
public:
    using QWidget::QWidget;
    FramePtr frame;
    std::vector<std::pair<ImageCorner, bool>> markers;
    std::function<void(ImageCorner)> clicked;

    QRectF imageBounds() const
    {
        if (!frame)
            return {};
        const QSizeF fitted = QSizeF(frame->width, frame->height).scaled(size(), Qt::KeepAspectRatio);
        return {{(width() - fitted.width()) / 2, (height() - fitted.height()) / 2}, fitted};
    }

protected:
    void mousePressEvent(QMouseEvent* event) override
    {
        const auto bounds = imageBounds();
        if (event->button() != Qt::LeftButton || !frame || !bounds.contains(event->position()))
            return;
        const double u = (event->position().x() - bounds.x()) * frame->width / bounds.width() - 0.5;
        const double v = (event->position().y() - bounds.y()) * frame->height / bounds.height() - 0.5;
        if (u >= 0 && v >= 0 && u <= frame->width - 1 && v <= frame->height - 1 && clicked)
            clicked({u, v});
    }

    void paintEvent(QPaintEvent*) override
    {
        QPainter painter(this);
        painter.fillRect(rect(), Qt::black);
        if (!frame) {
            painter.setPen(Qt::white);
            painter.drawText(rect(), Qt::AlignCenter,
                             QStringLiteral("Freeze a compatible undistorted frame to select points"));
            return;
        }
        auto format = frame->pixelFormat == PixelFormat::Rgb8    ? QImage::Format_RGB888
                      : frame->pixelFormat == PixelFormat::Rgba8 ? QImage::Format_RGBA8888
                                                                 : QImage::Format_Grayscale8;
        const QImage image(reinterpret_cast<const uchar*>(frame->pixels->data()),
                           static_cast<int>(frame->width), static_cast<int>(frame->height),
                           static_cast<qsizetype>(frame->strideBytes), format);
        const auto bounds = imageBounds();
        painter.drawImage(bounds, image);
        int index = 0;
        for (const auto& [p, check] : markers) {
            ++index;
            if (!std::isfinite(p.u) || !std::isfinite(p.v))
                continue;
            painter.setPen(QPen(check ? Qt::yellow : Qt::green, 2));
            const QPointF at{bounds.x() + (p.u + 0.5) * bounds.width() / frame->width,
                             bounds.y() + (p.v + 0.5) * bounds.height() / frame->height};
            painter.drawEllipse(at, 4, 4);
            painter.drawText(at + QPointF(6, -6), QString::number(index));
        }
    }
};

WorkspaceCalibrationDialog::WorkspaceCalibrationDialog(WorkspaceCalibrationController& controller,
                                                       QWidget* parent)
    : QDialog(parent), controller_(controller)
{
    setWindowTitle(QStringLiteral("Tabletop homography calibration"));
    resize(900, 780);
    auto* layout = new QVBoxLayout(this);
    auto* instructions = new QLabel(
        QStringLiteral(
            "Freeze an undistorted image. Click or enter native (u,v) pixels and measured world (x,y) "
            "metres.\n"
            "Fit 4–64 paired points in any order. Mark independent references as Check.\n"
            "World origin: lower-left; +X right, +Y up. Keep camera/table fixed. "
            "Side views are planar only: use visible references at near/far edges, never guessed occluded points. "
            "Raise an almost edge-on camera; objects above the table have parallax error."),
        this);
    instructions->setWordWrap(true);
    layout->addWidget(instructions);
    auto* settings = new QHBoxLayout;
    identity_ = new QLineEdit(this);
    identity_->setObjectName("workspaceIdentity");
    identity_->setPlaceholderText("Workspace identity (required)");
    settings->addWidget(identity_, 2);
    auto dimension = [this](const char* name, const QString& suffix) {
        auto* value = new QDoubleSpinBox(this);
        value->setObjectName(name);
        value->setDecimals(6);
        value->setRange(0, 1000);
        value->setSuffix(suffix);
        value->setSpecialValueText("Not measured");
        return value;
    };
    width_ = dimension("workspaceWidth", " m wide");
    height_ = dimension("workspaceHeight", " m high");
    settings->addWidget(width_);
    settings->addWidget(height_);
    layout->addLayout(settings);
    image_ = new WorkspaceImageView(this);
    image_->setObjectName("workspaceImage");
    image_->setMinimumSize(320, 180);
    image_->clicked = [this](ImageCorner point) { addPoint(point); };
    layout->addWidget(image_, 1);
    points_ = new QTableWidget(0, 5, this);
    points_->setObjectName("workspacePoints");
    points_->setHorizontalHeaderLabels({"u (px)", "v (px)", "x (m)", "y (m)", "Check only"});
    points_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    points_->setMinimumHeight(130);
    layout->addWidget(points_);
    auto* rows = new QHBoxLayout;
    auto* add = new QPushButton("Add point", this);
    add->setObjectName("workspaceAddPoint");
    rows->addWidget(add);
    auto* remove = new QPushButton("Remove selected row", this);
    remove->setObjectName("workspaceRemovePoint");
    rows->addWidget(remove);
    layout->addLayout(rows);
    message_ = new QLabel(this);
    message_->setObjectName("workspaceMessage");
    message_->setTextFormat(Qt::PlainText);
    message_->setWordWrap(true);
    message_->setMinimumHeight(message_->fontMetrics().lineSpacing() * 4);
    layout->addWidget(message_);
    auto* actions = new QHBoxLayout;
    auto action = [this, actions](const char* name, const QString& text) {
        auto* b = new QPushButton(text, this);
        b->setObjectName(name);
        actions->addWidget(b);
        return b;
    };
    freeze_ = action("workspaceFreeze", "Freeze undistorted frame");
    calculate_ = action("workspaceCalculate", "Calculate");
    save_ = action("workspaceSave", "Save transform");
    load_ = action("workspaceLoad", "Load transform");
    verify_ = action("workspaceVerify", "Verify checks");
    activate_ = action("workspaceActivate", "Activate transform");
    auto* close = action("workspaceClose", "Close");
    layout->addLayout(actions);
    auto configure = [this] { controller_.configure(identity_->text(), width_->value(), height_->value()); };
    connect(identity_, &QLineEdit::textChanged, this, configure);
    connect(width_, &QDoubleSpinBox::valueChanged, this, configure);
    connect(height_, &QDoubleSpinBox::valueChanged, this, configure);
    connect(points_, &QTableWidget::itemChanged, this, [this] { pointsChanged(); });
    connect(add, &QPushButton::clicked, this, [this] { addPoint({}); });
    connect(remove, &QPushButton::clicked, this, [this] {
        const int row = points_->currentRow();
        if (row >= 0) {
            points_->removeRow(row);
            pointsChanged();
        }
    });
    connect(freeze_, &QPushButton::clicked, &controller_, &WorkspaceCalibrationController::freeze);
    connect(calculate_, &QPushButton::clicked, &controller_, &WorkspaceCalibrationController::calculate);
    connect(save_, &QPushButton::clicked, &controller_, &WorkspaceCalibrationController::save);
    connect(load_, &QPushButton::clicked, &controller_, &WorkspaceCalibrationController::load);
    connect(verify_, &QPushButton::clicked, &controller_, &WorkspaceCalibrationController::verify);
    connect(activate_, &QPushButton::clicked, &controller_, &WorkspaceCalibrationController::activate);
    connect(close, &QPushButton::clicked, this, &QDialog::close);
    connect(&controller_, &WorkspaceCalibrationController::changed, this,
            &WorkspaceCalibrationDialog::refresh);
    const QSignalBlocker identityBlocker(identity_), widthBlocker(width_), heightBlocker(height_);
    identity_->setText(controller_.workspaceId());
    if (const auto bounds = controller_.draftBounds()) {
        width_->setValue(bounds->width().value);
        height_->setValue(bounds->height().value);
    }
    refresh();
}

void WorkspaceCalibrationDialog::addPoint(std::optional<ImageCorner> point)
{
    if (points_->rowCount() >= 128 || controller_.busy())
        return;
    const QSignalBlocker blocker(points_);
    const int row = points_->rowCount();
    points_->insertRow(row);
    for (int col = 0; col < 4; ++col)
        points_->setItem(row, col, new QTableWidgetItem);
    if (point) {
        points_->item(row, 0)->setText(QString::number(point->u, 'g', 15));
        points_->item(row, 1)->setText(QString::number(point->v, 'g', 15));
    }
    auto* check = new QTableWidgetItem;
    check->setFlags(Qt::ItemIsEnabled | Qt::ItemIsUserCheckable | Qt::ItemIsSelectable);
    check->setCheckState(controller_.draftProfile() ? Qt::Checked : Qt::Unchecked);
    points_->setItem(row, 4, check);
    points_->setCurrentCell(row, 2);
    pointsChanged();
}

void WorkspaceCalibrationDialog::pointsChanged()
{
    std::vector<WorkspaceCorrespondence> fit, check;
    image_->markers.clear();
    for (int row = 0; row < points_->rowCount(); ++row) {
        double values[4];
        for (int col = 0; col < 4; ++col) {
            bool valid = false;
            const auto* item = points_->item(row, col);
            values[col] = item ? QLocale::c().toDouble(item->text(), &valid) : 0;
            if (!valid)
                values[col] = std::numeric_limits<double>::quiet_NaN();
        }
        const bool isCheck = points_->item(row, 4) && points_->item(row, 4)->checkState() == Qt::Checked;
        image_->markers.emplace_back(ImageCorner{values[0], values[1]}, isCheck);
        (isCheck ? check : fit).push_back({{values[0], values[1]}, {Meters{values[2]}, Meters{values[3]}}});
    }
    image_->update();
    controller_.setPoints(std::move(fit), std::move(check));
}

void WorkspaceCalibrationDialog::refresh()
{
    const auto frame = controller_.frozenFrame();
    if (frame != shownFrame_) {
        shownFrame_ = frame;
        image_->frame = frame;
        image_->markers.clear();
        image_->update();
        const QSignalBlocker blocker(points_);
        points_->setRowCount(0);
        auto restore = [this](const auto& references, bool isCheck) {
            for (const auto& p : references) {
                const int row = points_->rowCount();
                points_->insertRow(row);
                const double values[]{p.image.u, p.image.v, p.world.x.value, p.world.y.value};
                for (int col = 0; col < 4; ++col)
                    points_->setItem(row, col, new QTableWidgetItem(QString::number(values[col], 'g', 15)));
                auto* check = new QTableWidgetItem;
                check->setFlags(Qt::ItemIsEnabled | Qt::ItemIsUserCheckable | Qt::ItemIsSelectable);
                check->setCheckState(isCheck ? Qt::Checked : Qt::Unchecked);
                points_->setItem(row, 4, check);
                image_->markers.emplace_back(p.image, isCheck);
            }
        };
        restore(controller_.fitPoints(), false);
        restore(controller_.checkPoints(), true);
    }
    if (const auto& p = controller_.draftProfile()) {
        const QSignalBlocker a(width_), b(height_);
        width_->setValue(p->bounds.width().value);
        height_->setValue(p->bounds.height().value);
    }
    message_->setText(controller_.message());
    freeze_->setEnabled(controller_.canFreeze());
    calculate_->setEnabled(controller_.canCalculate());
    save_->setEnabled(controller_.canSave());
    load_->setEnabled(controller_.canLoad());
    verify_->setEnabled(controller_.canVerify());
    activate_->setEnabled(controller_.canActivate());
    points_->setEnabled(!controller_.busy());
}
} // namespace micro_robot
