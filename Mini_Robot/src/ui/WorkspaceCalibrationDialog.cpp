#include "ui/WorkspaceCalibrationDialog.hpp"
#include "ui/WorkspaceDiagnosticsDialog.hpp"
#include "ui/WorkspaceDistanceDialog.hpp"
#include "ui/WorkspaceReferenceModel.hpp"
#include "ui/WorkspaceBoardPanel.hpp"
#include <QScrollArea>
#include <QTableView>
#include <QComboBox>
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

namespace micro_robot {
class WorkspaceImageView final : public QWidget {
public:
    using QWidget::QWidget;
    FramePtr frame;
    std::vector<std::pair<ImageCorner, bool>> markers;
    std::vector<ImageCorner> corners;
    std::optional<ImageCorner> origin;
    std::vector<ImageCorner> board;
    std::size_t boardAnchor{};
    int boardColumns{WorkspaceBoardTarget.columns};
    QString dimensions;
    std::function<void(ImageCorner)> clicked;
    QRectF imageBounds() const
    {
        if (!frame) return {};
        const QSizeF fitted = QSizeF(frame->width, frame->height).scaled(size(), Qt::KeepAspectRatio);
        return {{(width() - fitted.width()) / 2, (height() - fitted.height()) / 2}, fitted};
    }
protected:
    void mousePressEvent(QMouseEvent* event) override
    {
        const auto bounds = imageBounds();
        if (event->button() != Qt::LeftButton || !frame || !bounds.contains(event->position())) return;
        const double u = (event->position().x() - bounds.x()) * frame->width / bounds.width() - 0.5;
        const double v = (event->position().y() - bounds.y()) * frame->height / bounds.height() - 0.5;
        if (u >= 0 && v >= 0 && u <= frame->width - 1 && v <= frame->height - 1 && clicked) clicked({u,v});
    }
    void paintEvent(QPaintEvent*) override
    {
        QPainter painter(this);
        painter.fillRect(rect(), Qt::black);
        if (!frame) {
            painter.setPen(Qt::white);
            painter.drawText(rect(), Qt::AlignCenter, "Freeze a compatible undistorted frame to select corners");
            return;
        }
        const auto format = frame->pixelFormat == PixelFormat::Rgb8 ? QImage::Format_RGB888
            : frame->pixelFormat == PixelFormat::Rgba8 ? QImage::Format_RGBA8888 : QImage::Format_Grayscale8;
        const QImage image(reinterpret_cast<const uchar*>(frame->pixels->data()),
            static_cast<int>(frame->width), static_cast<int>(frame->height),
            static_cast<qsizetype>(frame->strideBytes), format);
        const auto bounds = imageBounds();
        painter.drawImage(bounds, image);
        auto at = [&](ImageCorner p) {
            return QPointF{bounds.x() + (p.u + 0.5) * bounds.width() / frame->width,
                           bounds.y() + (p.v + 0.5) * bounds.height() / frame->height};
        };
        painter.setPen(QPen(Qt::cyan, 2));
        for (std::size_t i = 0; i < corners.size(); ++i) {
            painter.drawEllipse(at(corners[i]), 4, 4);
            painter.drawText(at(corners[i]) + QPointF(6,-6), QStringLiteral("C%1").arg(i+1));
            if (i) painter.drawLine(at(corners[i-1]), at(corners[i]));
        }
        if (corners.size() == WorkspaceCornerCount) {
            painter.drawLine(at(corners.back()), at(corners.front()));
            painter.drawText((at(corners[0])+at(corners[1]))/2, "+X");
            painter.drawText((at(corners[0])+at(corners[3]))/2, "+Y");
            // Arrows point along the selected physical edges, independent of screen orientation.
            for (std::size_t i : {std::size_t{1}, std::size_t{3}}) {
                const auto start = at(corners[0]), end = at(corners[i]);
                const auto delta = end-start;
                const auto tip = start + delta*0.4;
                const double length = std::hypot(delta.x(), delta.y());
                if (length > 0) {
                    const QPointF direction = delta / length, normal{-direction.y(), direction.x()};
                    painter.drawLine(start, tip);
                    painter.drawLine(tip, tip-direction*10+normal*5);
                    painter.drawLine(tip, tip-direction*10-normal*5);
                }
            }
        }
        int index = 0;
        for (const auto& [p, check] : markers) {
            ++index;
            if (!std::isfinite(p.u) || !std::isfinite(p.v)) continue;
            painter.setPen(QPen(check ? Qt::yellow : Qt::green, 2));
            painter.drawEllipse(at(p), 4, 4);
            if (index<=MaximumWorkspaceOverlayReferenceLabels) painter.drawText(at(p)+QPointF(6,12), QString::number(index));
        }
        if (!board.empty()) {
            painter.setPen(QPen(Qt::cyan,2));
            for (std::size_t i=0;i<board.size();++i) {
                painter.drawEllipse(at(board[i]),3,3);
                if (i==0 || i==boardAnchor || i+1==board.size() || i+1==static_cast<std::size_t>(boardColumns)
                    || i==board.size()-static_cast<std::size_t>(boardColumns))
                    painter.drawText(at(board[i])+QPointF(4,-4),QStringLiteral("B%1").arg(i));
            }
            if (boardAnchor<board.size()) {
                const auto anchor=at(board[boardAnchor]);
                painter.setPen(QPen(Qt::magenta,2)); painter.drawEllipse(anchor,7,7);
                painter.drawText(anchor+QPointF(8,8),"Anchor");
                const auto column=boardAnchor%static_cast<std::size_t>(boardColumns);
                const auto row=boardAnchor/static_cast<std::size_t>(boardColumns);
                auto axis=[&](std::size_t other,bool reverse,const char* label) {
                    const auto delta=reverse?anchor-at(board[other]):at(board[other])-anchor;
                    painter.drawLine(anchor,anchor+delta*2);
                    painter.drawText(anchor+delta*2,label);
                };
                axis(column+1<static_cast<std::size_t>(boardColumns)?boardAnchor+1:boardAnchor-1,column+1==static_cast<std::size_t>(boardColumns),"+cols");
                axis((row+1)*static_cast<std::size_t>(boardColumns)<board.size()?boardAnchor+static_cast<std::size_t>(boardColumns):boardAnchor-static_cast<std::size_t>(boardColumns),
                     (row+1)*static_cast<std::size_t>(boardColumns)>=board.size(),"+rows");
            }
        }
        if (origin) {
            painter.setPen(QPen(Qt::magenta, 2));
            painter.drawEllipse(at(*origin), 6, 6);
            painter.drawText(at(*origin)+QPointF(8,0), "(0,0)");
        }
        painter.fillRect(QRectF{0,0,static_cast<double>(width()),25}, QColor(0,0,0,190));
        painter.setPen(Qt::white);
        painter.drawText(QRectF{5,0,static_cast<double>(width()-10),25}, Qt::AlignVCenter, dimensions);
    }
};

WorkspaceCalibrationDialog::WorkspaceCalibrationDialog(WorkspaceCalibrationController& controller, QWidget* parent)
    : QDialog(parent), controller_(controller)
{
    setWindowTitle("Tabletop homography calibration");
    resize(1000,900);
    auto* outer = new QVBoxLayout(this);
    auto* scroll = new QScrollArea(this); scroll->setWidgetResizable(true);
    auto* body = new QWidget(scroll); auto* layout = new QVBoxLayout(body);
    scroll->setObjectName("workspaceScroll"); scroll->setWidget(body); outer->addWidget(scroll,1);
    auto* instructions = new QLabel(
        "1. Freeze compatible undistorted pixels. Select physical C1 (base), C2 (+X neighbour), C3 (opposite), C4 (+Y neighbour).\n"
        "2. Enter measured edge lengths and origin offsets from C1 along +X/+Y. Review the labelled perimeter and Confirm boundary.\n"
        "3. Add extra paired references / independent Checks, Calculate, review errors, then explicitly Activate. Keep camera/table fixed.", this);
    instructions->setWordWrap(true);
    layout->addWidget(instructions);
    auto* settings = new QHBoxLayout;
    identity_ = new QLineEdit(this);
    identity_->setObjectName("workspaceIdentity");
    identity_->setPlaceholderText("Workspace identity (required)");
    settings->addWidget(identity_,2);
    auto dimension = [this](const char* name, const char* suffix) {
        auto* value = new QDoubleSpinBox(this);
        value->setObjectName(name);
        value->setDecimals(CoordinateDisplayDecimals);
        value->setRange(0, MaximumWorkspaceDimensionMeters);
        value->setSuffix(suffix);
        return value;
    };
    width_ = dimension("workspaceWidth", " m along +X");
    height_ = dimension("workspaceHeight", " m along +Y");
    settings->addWidget(width_); settings->addWidget(height_);
    layout->addLayout(settings);
    auto* originRow = new QHBoxLayout;
    originCorner_ = new QComboBox(this);
    originCorner_->setObjectName("workspaceOriginCorner");
    originCorner_->addItems({"Origin: measured offsets", "Origin at C1", "Origin at C2", "Origin at C3", "Origin at C4"});
    originX_ = dimension("workspaceOriginX", " m from C1 along +X");
    originY_ = dimension("workspaceOriginY", " m from C1 along +Y");
    originRow->addWidget(originCorner_); originRow->addWidget(originX_); originRow->addWidget(originY_);
    originPick_ = new QPushButton("Pick origin after Calculate", this);
    originPick_->setObjectName("workspacePickOrigin"); originPick_->setCheckable(true);
    originRow->addWidget(originPick_); layout->addLayout(originRow);
    image_ = new WorkspaceImageView(this); image_->setObjectName("workspaceImage");
    image_->setMinimumSize(320,180); layout->addWidget(image_,1);
    corners_ = new QTableWidget(0,2,this); corners_->setObjectName("workspaceCorners");
    corners_->setHorizontalHeaderLabels({"Corner u (px)", "Corner v (px)"});
    corners_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    corners_->setMaximumHeight(150); layout->addWidget(corners_);
    auto* cornerActions = new QHBoxLayout;
    auto button = [this](QHBoxLayout* row, const char* name, const char* text) {
        auto* b = new QPushButton(text,this); b->setObjectName(name); row->addWidget(b); return b;
    };
    undo_ = button(cornerActions,"workspaceUndoCorner","Undo last corner");
    reset_ = button(cornerActions,"workspaceResetCorners","Reselect all corners");
    reselect_ = button(cornerActions,"workspaceReselectCorner","Reselect selected corner");
    confirm_ = button(cornerActions,"workspaceConfirmBoundary","Confirm boundary / axes");
    layout->addLayout(cornerActions);
    points_ = new QTableView(this); points_->setObjectName("workspacePoints");
    referenceModel_ = new WorkspaceReferenceModel(controller_,points_); points_->setModel(referenceModel_);
    points_->setSelectionBehavior(QAbstractItemView::SelectRows);
    board_ = new WorkspaceBoardPanel(controller_,[this]() -> std::optional<std::uint64_t> {
        const int row=points_->currentIndex().row();
        if (row<0 || static_cast<std::size_t>(row)>=controller_.references().size()) return {};
        return controller_.references()[static_cast<std::size_t>(row)].id;
    },this);
    layout->addWidget(board_); connect(board_,&QGroupBox::toggled,this,[this]{refresh();});
    points_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    points_->setMaximumHeight(160); layout->addWidget(points_);
    auto* rows = new QHBoxLayout;
    add_ = button(rows,"workspaceAddPoint","Add reference");
    remove_ = button(rows,"workspaceRemovePoint","Remove selected reference");
    layout->addLayout(rows);
    message_ = new QLabel(this); message_->setObjectName("workspaceMessage");
    message_->setTextFormat(Qt::PlainText); message_->setWordWrap(true);
    message_->setMinimumHeight(message_->fontMetrics().lineSpacing()*3); layout->addWidget(message_);
    auto* actions = new QHBoxLayout;
    freeze_ = button(actions,"workspaceFreeze","Freeze / reset references");
    calculate_ = button(actions,"workspaceCalculate","Calculate");
    verify_ = button(actions,"workspaceVerify","Verify checks");
    activate_ = button(actions,"workspaceActivate","Activate transform");
    outer->addLayout(actions);
    auto* storage = new QHBoxLayout;
    save_ = button(storage,"workspaceSave","Save transform");
    load_ = button(storage,"workspaceLoad","Load transform");
    distance_ = button(storage,"workspaceDistance","Measure distance…");
    connect(distance_, &QPushButton::clicked, this, [this] {
        if (!distanceDialog_) { distanceDialog_ = new WorkspaceDistanceDialog(controller_, this); distanceDialog_->setAttribute(Qt::WA_DeleteOnClose); }
        distanceDialog_->show(); distanceDialog_->raise(); distanceDialog_->activateWindow();
    });
    diagnostics_ = button(storage,"workspaceDiagnostics","Scale / error maps…");
    connect(diagnostics_, &QPushButton::clicked, this, [this] {
        if (!diagnosticsDialog_) {
            diagnosticsDialog_ = new WorkspaceDiagnosticsDialog(controller_.diagnostics(), this);
            diagnosticsDialog_->setAttribute(Qt::WA_DeleteOnClose);
        }
        diagnosticsDialog_->show(); diagnosticsDialog_->raise(); diagnosticsDialog_->activateWindow();
    });
    auto* close = button(storage,"workspaceClose","Close"); outer->addLayout(storage);
    image_->clicked = [this](ImageCorner p) {
        if (controller_.busy()) return;
        if (originPick_->isChecked()) {
            originPick_->setChecked(false); controller_.setOriginFromImage(p);
        } else if (replacingCorner_) {
            const auto index = replacingCorner_; replacingCorner_.reset(); controller_.selectCorner(p,index);
        } else if (!controller_.canAddReferences()) controller_.selectCorner(p);
        else if (!board_->isChecked()) addPoint(p);
    };
    for (auto* value : {width_,height_,originX_,originY_})
        connect(value,&QDoubleSpinBox::valueChanged,this,[this]{ configure(); });
    connect(identity_,&QLineEdit::textChanged,this,[this]{ configure(); });
    connect(originCorner_,&QComboBox::activated,this,[this](int index) {
        if (index == 0) return;
        { const QSignalBlocker x(originX_), y(originY_);
          originX_->setValue(index == 2 || index == 3 ? width_->value() : 0);
          originY_->setValue(index == 3 || index == 4 ? height_->value() : 0); }
        configure();
    });
    connect(corners_,&QTableWidget::itemChanged,this,[this](QTableWidgetItem* item) {
        const int row = item->row(); bool a{}, b{};
        const double u = QLocale::c().toDouble(corners_->item(row,0)->text(),&a);
        const double v = QLocale::c().toDouble(corners_->item(row,1)->text(),&b);
        controller_.selectCorner({a ? u : std::numeric_limits<double>::quiet_NaN(),
                                 b ? v : std::numeric_limits<double>::quiet_NaN()}, static_cast<std::size_t>(row));
    });
    connect(undo_,&QPushButton::clicked,&controller_,&WorkspaceCalibrationController::undoCorner);
    connect(reset_,&QPushButton::clicked,this,[this]{ replacingCorner_.reset(); controller_.clearCorners(); });
    connect(reselect_,&QPushButton::clicked,this,[this]{
        if (corners_->currentRow() >= 0) {
            replacingCorner_ = static_cast<std::size_t>(corners_->currentRow());
            message_->setText("Click the replacement for the selected labelled corner.");
        }
    });
    connect(confirm_,&QPushButton::clicked,&controller_,&WorkspaceCalibrationController::confirmBoundary);
    connect(add_,&QPushButton::clicked,this,[this]{ addPoint({}); });
    connect(remove_,&QPushButton::clicked,this,[this]{
        const int row = points_->currentIndex().row();
        if (row>=0) controller_.removeReference(static_cast<std::size_t>(row));
    });
    connect(freeze_,&QPushButton::clicked,&controller_,&WorkspaceCalibrationController::freeze);
    connect(calculate_,&QPushButton::clicked,&controller_,&WorkspaceCalibrationController::calculate);
    connect(verify_,&QPushButton::clicked,&controller_,&WorkspaceCalibrationController::verify);
    connect(activate_,&QPushButton::clicked,&controller_,&WorkspaceCalibrationController::activate);
    connect(save_,&QPushButton::clicked,&controller_,&WorkspaceCalibrationController::save);
    connect(load_,&QPushButton::clicked,&controller_,&WorkspaceCalibrationController::load);
    connect(close,&QPushButton::clicked,this,&QDialog::close);
    connect(&controller_,&WorkspaceCalibrationController::changed,this,&WorkspaceCalibrationDialog::refresh);
    { const QSignalBlocker block(identity_); identity_->setText(controller_.workspaceId()); }
    refresh();
}
void WorkspaceCalibrationDialog::configure()
{
    { const QSignalBlocker block(originCorner_); originCorner_->setCurrentIndex(0); }
    controller_.configure(identity_->text(),width_->value(),height_->value(),originX_->value(),originY_->value());
}
void WorkspaceCalibrationDialog::addPoint(std::optional<ImageCorner> point)
{
    const auto before=controller_.references().size(); controller_.appendReference(point);
    if (controller_.references().size()>before)
        points_->setCurrentIndex(referenceModel_->index(static_cast<int>(before),2));
}
void WorkspaceCalibrationDialog::restoreReferences()
{
    referenceModel_->refresh(); image_->markers.clear();
    for (const auto& r:controller_.references()) {
        if (r.boundary) continue;
        if (image_->markers.size()==MaximumWorkspaceOverlayReferences) break;
        image_->markers.emplace_back(r.point.image,r.check);
    }
}
void WorkspaceCalibrationDialog::refresh()
{
    diagnostics_->setEnabled(bool(controller_.profile()));
    distance_->setEnabled(bool(controller_.profile()));
    const auto frame = controller_.frozenFrame();
    if (frame != shownFrame_ || shownRevision_ != controller_.referencesRevision()) {
        originPick_->setChecked(false); replacingCorner_.reset();
        shownFrame_ = frame; shownRevision_ = controller_.referencesRevision();
        image_->frame = frame; restoreReferences();
        const QSignalBlocker block(corners_); corners_->setRowCount(0);
        const QStringList labels{"C1 base", "C2 +X", "C3 opposite", "C4 +Y"};
        for (const auto& p : controller_.corners()) {
            const int row = corners_->rowCount(); corners_->insertRow(row);
            corners_->setVerticalHeaderItem(row,new QTableWidgetItem(labels[row]));
            corners_->setItem(row,0,new QTableWidgetItem(QString::number(p.u,'g',17)));
            corners_->setItem(row,1,new QTableWidgetItem(QString::number(p.v,'g',17)));
        }
    }
    restoreReferences();
    image_->dimensions.clear();
    image_->corners = controller_.corners(); image_->origin.reset(); image_->board.clear();
    if (board_->isChecked() && controller_.boardReview()) {
        for (const auto& p:controller_.boardPoints()) image_->board.push_back(p.image);
        image_->boardAnchor=controller_.boardReview()->anchorIndex;
    } else if (board_->isChecked() && controller_.detectedBoard()) {
        image_->board=controller_.detectedBoard()->detected; image_->boardAnchor=0;
    }
    if (const auto b = controller_.draftBounds()) {
        const QSignalBlocker w(width_), h(height_), x(originX_), y(originY_);
        width_->setValue(b->width().value); height_->setValue(b->height().value);
        originX_->setValue(-b->minimum().x.value); originY_->setValue(-b->minimum().y.value);
        image_->dimensions = QStringLiteral("+X edge %1 m · +Y edge %2 m · origin from C1 (%3, %4) m")
            .arg(b->width().value).arg(b->height().value).arg(-b->minimum().x.value).arg(-b->minimum().y.value);
    }
    if (const auto& p = controller_.draftProfile(); p && p->homography) {
        image_->origin = unprojectWorkspacePoint(*p->homography,{});
        if (image_->corners.empty() && p->boundary && frame)
            image_->corners.assign(p->boundary->begin(),p->boundary->end());
    }
    image_->dimensions += QStringLiteral(" · %1 extra reference markers shown; %2 dataset rows (labels ≤%3; all rows retained)")
        .arg(image_->markers.size()).arg(controller_.references().size()).arg(MaximumWorkspaceOverlayReferenceLabels);
    image_->update(); message_->setText(controller_.message());
    const bool idle = !controller_.busy();
    freeze_->setEnabled(controller_.canFreeze()); calculate_->setEnabled(controller_.canCalculate());
    save_->setEnabled(controller_.canSave()); load_->setEnabled(controller_.canLoad());
    verify_->setEnabled(controller_.canVerify()); activate_->setEnabled(controller_.canActivate());
    corners_->setEnabled(idle); points_->setEnabled(idle);
    confirm_->setEnabled(idle && frame && controller_.corners().size() == WorkspaceCornerCount);
    undo_->setEnabled(idle && !controller_.corners().empty()); reset_->setEnabled(idle && frame);
    reselect_->setEnabled(idle && !controller_.corners().empty());
    originPick_->setEnabled(idle && frame && controller_.draftProfile().has_value());
    add_->setEnabled(idle && frame && controller_.canAddReferences()); remove_->setEnabled(idle);
}
} // namespace micro_robot
