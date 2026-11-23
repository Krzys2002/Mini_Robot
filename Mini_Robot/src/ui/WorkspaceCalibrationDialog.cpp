#include "ui/WorkspaceCalibrationDialog.hpp"
#include "ui/CalibrationForm.hpp"
#include "ui/CalibrationImageView.hpp"
#include "ui/WorkspaceDiagnosticsDialog.hpp"
#include "ui/WorkspaceDistanceDialog.hpp"
#include "ui/WorkspaceReferenceModel.hpp"
#include "ui/WorkspaceBoardPanel.hpp"
#include "ui/ElevatedLocalizationDialog.hpp"
#include "ui/ReferenceMarkersDialog.hpp"
#include <QScrollArea>
#include <QScrollBar>
#include <QMessageBox>
#include <QFormLayout>
#include <QKeyEvent>
#include <QTableView>
#include <QComboBox>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QLocale>
#include <QPushButton>
#include <QSignalBlocker>
#include <QScopedValueRollback>
#include <QTableWidget>
#include <QVBoxLayout>
#include <cmath>

namespace micro_robot {
namespace {
class WorkspaceField final : public QLineEdit {
public:
    using QLineEdit::QLineEdit;
protected:
    void keyPressEvent(QKeyEvent* event) override
    {
        if (event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter) {
            event->accept();
            return;
        }
        QLineEdit::keyPressEvent(event);
    }
};

// Keep an unchanged rendered length bound to its original metre value. A unit
// presentation round trip must not allocate a new calibration frame through rounding.
inline void setWorkspaceLengthText(QLineEdit* field, double meters, WorkspaceLengthUnit unit)
{
    field->setText(workspaceLengthText(meters*workspaceUnitsPerMeter(unit)));
    field->setProperty("workspaceRenderedText",field->text());
    field->setProperty("workspaceMeters",meters);
}
}

WorkspaceCalibrationDialog::WorkspaceCalibrationDialog(WorkspaceCalibrationController& controller, QWidget* parent)
    : QDialog(parent), controller_(controller)
{
    setWindowTitle("Tabletop homography calibration");
    resize(1000,900);
    auto* outer = new QVBoxLayout(this);
    auto* scroll = new QScrollArea(this); scroll->setWidgetResizable(true);
    auto* body = new QWidget(scroll); auto* layout = new QVBoxLayout(body);
    scroll->setObjectName("workspaceScroll"); scroll->setWidget(body); outer->addWidget(scroll,1);
    instructions_ = new QLabel(
        "1. Freeze compatible undistorted pixels. Select physical C1 (base), C2 (+X neighbour), C3 (opposite), C4 (+Y neighbour).\n"
        "2. Enter measured edge lengths and origin offsets from C1 along +X/+Y. Review the labelled perimeter and Confirm boundary.\n"
        "3. Add extra paired references / independent Checks, Calculate, review errors, then explicitly Activate. Keep camera/table fixed.\n"
        "Side views are planar only: use visible near/far references, never guessed occluded points. Raise an almost edge-on camera; elevated objects have parallax error.", this);
    instructions_->setWordWrap(true);
    layout->addWidget(instructions_);
    geometryFields_ = new QWidget(this);
    auto* geometryLayout = new QVBoxLayout(geometryFields_);
    geometryLayout->setContentsMargins(0,0,0,0);
    auto* settings = new QFormLayout;
    settings->setRowWrapPolicy(QFormLayout::WrapLongRows);
    identity_ = new WorkspaceField(this);
    identity_->setObjectName("workspaceIdentity");
    settings->addRow("Workspace identity",identity_);
    units_=new QComboBox(this); units_->setObjectName("workspaceLengthUnit");
    units_->addItems({"m", "cm"}); settings->addRow("Length unit (whole window)",units_);
    auto dimension = [this,settings](const char* name,const char* text) {
        auto* value = new WorkspaceField(this); value->setObjectName(name);
        auto* label=new QLabel(this); label->setBuddy(value);
        label->setProperty("workspaceLengthLabel",QString::fromUtf8(text));
        settings->addRow(label,value);
        return value;
    };
    width_=dimension("workspaceWidth","Edge length along +X");
    height_=dimension("workspaceHeight","Edge length along +Y");
    originX_=dimension("workspaceOriginX","Origin offset from C1 along +X");
    originY_=dimension("workspaceOriginY","Origin offset from C1 along +Y");
    geometryLayout->addLayout(settings);
    auto* originRow = new QHBoxLayout;
    originCorner_ = new QComboBox(this);
    originCorner_->setObjectName("workspaceOriginCorner");
    originCorner_->addItems({"Origin: measured offsets", "Origin at C1", "Origin at C2", "Origin at C3", "Origin at C4"});
    originRow->addWidget(originCorner_);
    originPick_ = new QPushButton("Pick origin after Calculate", this);
    originPick_->setObjectName("workspacePickOrigin"); originPick_->setCheckable(true);
    originRow->addWidget(originPick_); geometryLayout->addLayout(originRow);
    layout->addWidget(geometryFields_);
    image_ = new WorkspaceImageView(this);
    layout->addWidget(image_->createControls(this));
    layout->addWidget(image_,1);
    imageDetails_ = new QLabel(this);
    imageDetails_->setWordWrap(true);
    layout->addWidget(imageDetails_);
    corners_ = new QTableWidget(0,2,this); corners_->setObjectName("workspaceCorners");
    corners_->setHorizontalHeaderLabels({"Corner u (px)", "Corner v (px)"});
    corners_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    corners_->setMaximumHeight(150); layout->addWidget(corners_);
    cornerTools_ = new QWidget(this);
    auto* cornerActions = new QHBoxLayout(cornerTools_);
    cornerActions->setContentsMargins(0,0,0,0);
    auto button = [this](QHBoxLayout* row, const char* name, const char* text) {
        auto* b = new QPushButton(text,this); b->setObjectName(name); row->addWidget(b); return b;
    };
    undo_ = button(cornerActions,"workspaceUndoCorner","Undo last corner");
    reset_ = button(cornerActions,"workspaceResetCorners","Reselect all corners");
    reselect_ = button(cornerActions,"workspaceReselectCorner","Reselect selected corner");
    confirm_ = button(cornerActions,"workspaceConfirmBoundary","Confirm boundary / axes");
    layout->addWidget(cornerTools_);
    points_ = new QTableView(this); points_->setObjectName("workspacePoints");
    referenceModel_ = new WorkspaceReferenceModel(controller_,points_); points_->setModel(referenceModel_);
    points_->setSelectionBehavior(QAbstractItemView::SelectRows);
    board_ = new WorkspaceBoardPanel(controller_,[this]() -> std::optional<std::uint64_t> {
        const int row=points_->currentIndex().row();
        if (row<0 || static_cast<std::size_t>(row)>=controller_.references().size()) return {};
        return controller_.references()[static_cast<std::size_t>(row)].id;
    },this);
    localization_ = new QPushButton("Elevated tag / axle-body geometry…", this);
    localization_->setObjectName("elevatedLocalization");
    layout->addWidget(localization_);
    surveyedReferences_ = new QPushButton("Surveyed references / drift monitoring…", this);
    surveyedReferences_->setObjectName("surveyedReferences"); layout->addWidget(surveyedReferences_);
    connect(surveyedReferences_, &QPushButton::clicked, this, [this] { ReferenceMarkersDialog dialog{controller_, this}; dialog.exec(); });
    connect(localization_, &QPushButton::clicked, this, [this] {
        ElevatedLocalizationDialog dialog{controller_, this}; dialog.exec();
    });
    layout->addWidget(board_); connect(board_,&QGroupBox::toggled,this,[this]{refresh();});
    points_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    points_->setMaximumHeight(160); layout->addWidget(points_);
    referenceTools_ = new QWidget(this);
    auto* rows = new QHBoxLayout(referenceTools_);
    rows->setContentsMargins(0,0,0,0);
    add_ = button(rows,"workspaceAddPoint","Add reference");
    remove_ = button(rows,"workspaceRemovePoint","Remove selected reference");
    layout->addWidget(referenceTools_);
    message_ = new QLabel(this); message_->setObjectName("workspaceMessage");
    message_->setTextFormat(Qt::PlainText); message_->setWordWrap(true);
    connect(referenceModel_, &WorkspaceReferenceModel::editRejected, message_, &QLabel::setText);
    message_->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    auto messagePolicy = message_->sizePolicy();
    messagePolicy.setVerticalPolicy(QSizePolicy::Minimum);
    message_->setSizePolicy(messagePolicy);
    outer->addWidget(message_);
    auto* actions = new QHBoxLayout;
    freeze_ = button(actions,"workspaceFreeze","Freeze / reset references");
    calculate_ = button(actions,"workspaceCalculate","Calculate");
    verify_ = button(actions,"workspaceVerify","Verify checks");
    activate_ = button(actions,"workspaceActivate","Activate transform");
    next_ = button(actions,"workspaceCalibrationNext","Accept and next");
    next_->hide();
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
        const auto frame = controller_.frozenFrame();
        const auto revision = controller_.referencesRevision();
        if (originPick_->isChecked()) {
            originPick_->setChecked(false);
            if (confirmCalibrationLoss(this, "Move workspace origin", "Changing the origin clears the draft transform and references.")
                && frame == controller_.frozenFrame() && revision == controller_.referencesRevision())
                controller_.setOriginFromImage(p);
        } else if (replacingCorner_) {
            const auto index = replacingCorner_; replacingCorner_.reset();
            if ((!hasGeometryDraft() || confirmCalibrationLoss(this, "Replace workspace corner", "Changing a corner clears the draft transform and dependent references."))
                && frame == controller_.frozenFrame() && revision == controller_.referencesRevision())
                controller_.selectCorner(p,index);
        } else if (stage_ != Stage::Tabletop && !controller_.canAddReferences()) controller_.selectCorner(p);
        else if (stage_ != Stage::Workspace && !board_->isChecked()) addPoint(p);
    };
    for (auto* value : {width_,height_,originX_,originY_})
        connect(value,&QLineEdit::editingFinished,this,[this]{ configure(); });
    connect(identity_,&QLineEdit::editingFinished,this,[this]{ configure(); });
    connect(units_,&QComboBox::currentIndexChanged,this,&WorkspaceCalibrationDialog::changeUnit);
    connect(originCorner_,&QComboBox::activated,this,[this](int index) {
        if (index == 0) return;
        { const QSignalBlocker x(originX_), y(originY_);
          originX_->setText(index == 2 || index == 3 ? width_->text() : "0");
          originY_->setText(index == 3 || index == 4 ? height_->text() : "0"); }
        configure();
    });
    connect(corners_,&QTableWidget::itemChanged,this,[this](QTableWidgetItem* item) {
        const int row = item->row();
        if (static_cast<std::size_t>(row) >= controller_.corners().size()) return;
        const auto original = controller_.corners()[static_cast<std::size_t>(row)];
        const auto revision = controller_.referencesRevision();
        bool uValid{}, vValid{};
        const double u = QLocale::c().toDouble(corners_->item(row,0)->text(),&uValid);
        const double v = QLocale::c().toDouble(corners_->item(row,1)->text(),&vValid);
        const auto frame = controller_.frozenFrame();
        if (frame && uValid && vValid && u >= 0 && v >= 0 && u <= frame->width - 1 && v <= frame->height - 1 &&
            (!hasGeometryDraft() || confirmCalibrationLoss(this, "Replace workspace corner", "Changing a corner clears the draft transform and dependent references."))) {
            if (revision == controller_.referencesRevision())
                controller_.selectCorner({u,v}, static_cast<std::size_t>(row));
            return;
        }
        if (revision != controller_.referencesRevision()) return;
        const QSignalBlocker block(corners_);
        corners_->item(row,0)->setText(QString::number(original.u,'g',17));
        corners_->item(row,1)->setText(QString::number(original.v,'g',17));
        message_->setText(QStringLiteral("Corner unchanged. Enter a pixel position inside the image."));
    });
    connect(undo_,&QPushButton::clicked,this,[this] {
        if (confirmCalibrationLoss(this, "Undo corner", "This removes the last corner and clears any dependent draft references."))
            controller_.undoCorner();
    });
    connect(reset_,&QPushButton::clicked,this,[this] {
        if (!controller_.corners().empty() &&
            !confirmCalibrationLoss(this, "Reselect workspace corners", "This clears the selected corners and dependent draft references.")) return;
        replacingCorner_.reset(); controller_.clearCorners();
    });
    connect(reselect_,&QPushButton::clicked,this,[this]{
        if (corners_->currentRow() >= 0) {
            replacingCorner_ = static_cast<std::size_t>(corners_->currentRow());
            message_->setText("Click the replacement for the selected labelled corner.");
        }
    });
    const auto confirmBoundary = [this] {
        if (inputError_.isEmpty() && !controller_.boundaryConfirmed()) controller_.confirmBoundary();
    };
    connect(confirm_,&QPushButton::clicked,this,confirmBoundary);
    connect(next_,&QPushButton::clicked,this,[this, confirmBoundary] {
        if (stage_ == Stage::Workspace) {
            if (controller_.frozenFrame()) confirmBoundary();
            if (inputError_.isEmpty() &&
                (controller_.boundaryConfirmed() || (!controller_.frozenFrame() && controller_.draftProfile()))) accept();
        } else if (stage_ == Stage::Tabletop && controller_.canActivate()) {
            if (controller_.profile() &&
                !confirmCalibrationLoss(this, "Replace tabletop calibration", "This replaces the active tabletop transform and its attached calibration.")) return;
            if (controller_.canActivate()) {
                controller_.activate();
                accept();
            }
        }
    });
    connect(add_,&QPushButton::clicked,this,[this]{ addPoint({}); });
    connect(remove_,&QPushButton::clicked,this,[this]{
        const int row = points_->currentIndex().row();
        const auto revision = controller_.referencesRevision();
        if (row >= 0 && static_cast<std::size_t>(row) < controller_.references().size() &&
            !controller_.references()[static_cast<std::size_t>(row)].boundary &&
            confirmCalibrationLoss(this, "Remove reference", "This removes the selected calibration reference.") &&
            revision == controller_.referencesRevision())
            controller_.removeReference(static_cast<std::size_t>(row));
    });
    connect(freeze_,&QPushButton::clicked,this,[this] {
        if (stage_ == Stage::Tabletop && controller_.boundaryConfirmed()) {
            if ((controller_.boardReview() || controller_.detectedBoard()) &&
                !confirmCalibrationLoss(this, "Replace frozen frame", "This clears the current chessboard review.")) return;
            controller_.freezeBoard();
        } else {
            if ((controller_.frozenFrame() || !controller_.corners().empty() || hasGeometryDraft()) &&
                !confirmCalibrationLoss(this, "Replace frozen frame", "This replaces the frozen frame and clears the selected corners and references.")) return;
            if (stage_ == Stage::Workspace) controller_.freezeWorkspace();
            else controller_.freeze();
        }
    });
    connect(calculate_,&QPushButton::clicked,this,[this] {
        if (controller_.draftProfile() &&
            !confirmCalibrationLoss(this, "Recalculate transform", "This replaces the current draft transform and any attached robot or reference calibration.")) return;
        controller_.calculate();
    });
    connect(verify_,&QPushButton::clicked,&controller_,&WorkspaceCalibrationController::verify);
    connect(activate_,&QPushButton::clicked,this,[this] {
        if (!controller_.profile() || confirmCalibrationLoss(this, "Replace active transform", "This replaces the currently active calibration."))
            controller_.activate();
    });
    connect(save_,&QPushButton::clicked,this,[this] {
        if (confirmCalibrationLoss(this, "Save transform", "Saving may replace the transform stored for this workspace."))
            controller_.save();
    });
    connect(load_,&QPushButton::clicked,this,[this] {
        if ((hasGeometryDraft() || controller_.frozenFrame() || !controller_.corners().empty()) &&
            !confirmCalibrationLoss(this, "Load transform", "Loading may replace the draft transform and its points.")) return;
        controller_.load();
    });
    connect(close,&QPushButton::clicked,this,&QDialog::close);
    connect(&controller_,&WorkspaceCalibrationController::changed,this,&WorkspaceCalibrationDialog::refresh);
    { const QSignalBlocker block(identity_); identity_->setText(controller_.workspaceId()); }
    updateUnitLabels();
    for (auto* action : findChildren<QPushButton*>()) action->setAutoDefault(false);
    refresh();
}
void WorkspaceCalibrationDialog::setStage(Stage stage)
{
    stage_ = stage;
    const bool tabletop = stage == Stage::Tabletop;
    const bool workspace = stage == Stage::Workspace;
    setWindowTitle(workspace ? "Workspace calibration" : "Tabletop calibration");
    instructions_->setText(workspace
        ? "Enter measured edge lengths and freeze an image. Select C1 (base), C2 (+X), C3 (opposite) and C4 (+Y). Leave origin offsets blank to place the origin at C1."
        : "Inspect the accepted workspace. Add measured references or independent Check points as needed, calculate the transform and review the errors before accepting.");
    geometryFields_->setVisible(!tabletop);
    corners_->setVisible(!tabletop);
    cornerTools_->setVisible(!tabletop);
    confirm_->setVisible(!workspace);
    originPick_->setVisible(!tabletop);
    points_->setVisible(!workspace);
    board_->setVisible(!workspace);
    referenceTools_->setVisible(!workspace);
    localization_->hide();
    surveyedReferences_->hide();
    calculate_->setVisible(!workspace);
    verify_->setVisible(!workspace);
    save_->setVisible(!workspace);
    activate_->hide();
    load_->setVisible(workspace);
    next_->setVisible(true);
    freeze_->setText(tabletop ? "Freeze reference image" : "Freeze workspace image");
    if (workspace) {
        findChild<QScrollArea*>("workspaceScroll")->verticalScrollBar()->setValue(0);
        originPick_->setChecked(false);
        replacingCorner_.reset();
    }
    refresh();
}
bool WorkspaceCalibrationDialog::hasGeometryDraft() const
{
    return controller_.boundaryConfirmed() || controller_.draftProfile() || !controller_.references().empty();
}
void WorkspaceCalibrationDialog::syncFields()
{
    identity_->setText(controller_.workspaceId());
    if (const auto bounds = controller_.draftBounds()) {
        setWorkspaceLengthText(width_, bounds->width().value, unit_);
        setWorkspaceLengthText(height_, bounds->height().value, unit_);
        setWorkspaceLengthText(originX_, -bounds->minimum().x.value, unit_);
        setWorkspaceLengthText(originY_, -bounds->minimum().y.value, unit_);
    }
    for (auto* field : {identity_, width_, height_, originX_, originY_}) {
        field->setStyleSheet({});
        field->setToolTip({});
    }
}
void WorkspaceCalibrationDialog::configure(bool confirmLoss)
{
    if (configuring_) return;
    const QScopedValueRollback configuring(configuring_, true);
    { const QSignalBlocker block(originCorner_); originCorner_->setCurrentIndex(0); }
    const double scale=workspaceUnitsPerMeter(unit_);
    bool valid=true;
    auto number=[&](QLineEdit* field,bool positive) {
        bool ok=false; double value=parseWorkspaceLength(field->text(),&ok)/scale;
        if (!positive && field->text().trimmed().isEmpty()) { value = 0; ok = true; }
        if (ok && field->property("workspaceMeters").isValid()
            && field->property("workspaceRenderedText").toString()==field->text())
            value=field->property("workspaceMeters").toDouble();
        ok=ok && std::isfinite(value) && value>=0 && value<=MaximumWorkspaceDimensionMeters && (!positive || value>0);
        field->setStyleSheet(ok ? "" : "QLineEdit { border: 1px solid #b42318; }");
        field->setToolTip(ok ? "" : QStringLiteral("Enter a finite %1 in %2 (maximum %3).")
            .arg(positive ? "positive length" : "non-negative offset",workspaceUnitLabel(unit_))
            .arg(MaximumWorkspaceDimensionMeters*scale));
        if (ok) {
            field->setProperty("workspaceRenderedText",field->text());
            field->setProperty("workspaceMeters",value);
        }
        valid=valid && ok;
        return ok ? value : std::numeric_limits<double>::quiet_NaN();
    };
    const double width=number(width_,true),height=number(height_,true),x=number(originX_,false),y=number(originY_,false);
    inputError_.clear();
    if (!valid || x>width || y>height) {
        inputError_="Enter positive edge lengths and origin offsets inside the rectangle in the selected unit.";
        for (auto pair:{std::pair{originX_,x>width},std::pair{originY_,y>height}})
            if (pair.second) {
                pair.first->setStyleSheet("QLineEdit { border: 1px solid #b42318; }");
                pair.first->setToolTip("Origin offset must be inside the measured edge length.");
            }
        if (!controller_.draftBounds() && !hasGeometryDraft())
            controller_.configure(identity_->text(),width,height,x,y);
        refresh();
        return;
    }
    const auto old = controller_.draftBounds();
    const bool changed = identity_->text() != controller_.workspaceId() || !old
        || old->width().value != width || old->height().value != height
        || old->minimum().x.value != -x || old->minimum().y.value != -y;
    if (changed && confirmLoss && hasGeometryDraft() &&
        !confirmCalibrationLoss(this, "Change workspace geometry", "Changing the workspace dimensions, identity or origin clears the draft transform and its references.")) {
        syncFields();
    } else if (changed) {
        controller_.configure(identity_->text(),width,height,x,y);
    }
    refresh();
}
void WorkspaceCalibrationDialog::updateUnitLabels()
{
    for (auto* label:findChildren<QLabel*>()) {
        const auto text=label->property("workspaceLengthLabel");
        if (text.isValid()) label->setText(text.toString()+" ("+workspaceUnitLabel(unit_)+")");
    }
}
void WorkspaceCalibrationDialog::changeUnit(int index)
{
    const auto next=index==1 ? WorkspaceLengthUnit::Centimeters : WorkspaceLengthUnit::Meters;
    if (next==unit_) return;
    const auto answer=QMessageBox::warning(this,"Change length unit",
        "Recalculate the typed values to preserve their physical lengths?\n"
        "Yes converts the numbers. No keeps the numbers and interprets them in the new unit, which clears dependent draft references when geometry changes.\n"
        "Cancel keeps the current unit.",
        QMessageBox::Yes|QMessageBox::No|QMessageBox::Cancel,QMessageBox::Yes);
    if (answer == QMessageBox::Cancel) {
        const QSignalBlocker block(units_);
        units_->setCurrentIndex(unit_ == WorkspaceLengthUnit::Centimeters ? 1 : 0);
        return;
    }
    const bool recalculate = answer == QMessageBox::Yes;
    if (recalculate) for (auto* field:{width_,height_,originX_,originY_}) {
        bool ok=false; const double value=parseWorkspaceLength(field->text(),&ok);
        if (ok && std::isfinite(value)) {
            const bool unchanged=field->property("workspaceMeters").isValid()
                && field->property("workspaceRenderedText").toString()==field->text();
            const double meters=unchanged ? field->property("workspaceMeters").toDouble() : value/workspaceUnitsPerMeter(unit_);
            setWorkspaceLengthText(field,meters,next);
        }
    }
    unit_=next;
    board_->setLengthUnit(next,recalculate);
    referenceModel_->setLengthUnit(next);
    updateUnitLabels();
    if (!recalculate) {
        for (auto* field:{width_,height_,originX_,originY_}) field->setProperty("workspaceMeters",QVariant{});
        configure(false);
    }
    board_->finishUnitChange();
    refresh();
}
void WorkspaceCalibrationDialog::addPoint(std::optional<ImageCorner> point)
{
    if (controller_.boardReview() &&
        !confirmCalibrationLoss(this, "Add reference", "Adding a reference clears the pending chessboard review.")) return;
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
    const bool loaded = !frame && controller_.draftProfile() && shownRevision_ != controller_.referencesRevision();
    if (frame != shownFrame_ || shownRevision_ != controller_.referencesRevision()) {
        originPick_->setChecked(false); replacingCorner_.reset();
        shownFrame_ = frame; shownRevision_ = controller_.referencesRevision();
        image_->setFrame(frame); restoreReferences();
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
    QString dimensions;
    image_->corners = controller_.corners(); image_->origin.reset(); image_->board.clear();
    if (board_->isChecked() && controller_.boardReview()) {
        for (const auto& p:controller_.boardPoints()) image_->board.push_back(p.image);
        image_->boardAnchor=controller_.boardReview()->anchorIndex;
    } else if (board_->isChecked() && controller_.detectedBoard()) {
        image_->board=controller_.detectedBoard()->detected; image_->boardAnchor=0;
    }
    const auto bounds=controller_.draftBounds();
    if (bounds) {
        const auto& b=*bounds; const double scale=workspaceUnitsPerMeter(unit_);
        if (!configuring_ && (!shownBounds_ || *shownBounds_!=b || loaded)) {
            inputError_.clear();
            syncFields();
        }
        dimensions = QStringLiteral("+X edge %1 %5 · +Y edge %2 %5 · origin from C1 (%3, %4) %5")
            .arg(b.width().value*scale).arg(b.height().value*scale)
            .arg(-b.minimum().x.value*scale).arg(-b.minimum().y.value*scale).arg(workspaceUnitLabel(unit_));
    }
    shownBounds_=bounds;
    if (const auto& p = controller_.draftProfile(); p && p->homography) {
        image_->origin = unprojectWorkspacePoint(*p->homography,{});
        if (image_->corners.empty() && p->boundary && frame)
            image_->corners.assign(p->boundary->begin(),p->boundary->end());
    }
    if (image_->markers.size() > MaximumWorkspaceOverlayReferenceLabels)
        dimensions += QStringLiteral(" · Showing %1 reference markers with the first %2 labelled.")
            .arg(image_->markers.size()).arg(MaximumWorkspaceOverlayReferenceLabels);
    if (imageDetails_->text() != dimensions) imageDetails_->setText(dimensions);
    imageDetails_->setVisible(!dimensions.isEmpty());
    image_->update(); message_->setText(inputError_.isEmpty() ? controller_.message() : inputError_);
    const bool idle = !controller_.busy();
    freeze_->setEnabled(controller_.canFreeze()); calculate_->setEnabled(controller_.canCalculate() && inputError_.isEmpty());
    const bool readyBoundary = idle && frame && controller_.draftBounds() && !controller_.workspaceId().isEmpty()
        && controller_.corners().size() == WorkspaceCornerCount;
    next_->setEnabled(stage_ == Stage::Workspace
        ? idle && inputError_.isEmpty() && (readyBoundary || (!frame && controller_.draftProfile()))
        : controller_.canActivate());
    save_->setEnabled(controller_.canSave()); load_->setEnabled(controller_.canLoad());
    verify_->setEnabled(controller_.canVerify()); activate_->setEnabled(controller_.canActivate());
    corners_->setEnabled(idle); points_->setEnabled(idle);
    units_->setEnabled(idle);
    for (auto* field : {identity_,width_,height_,originX_,originY_}) field->setEnabled(idle);
    confirm_->setEnabled(readyBoundary && inputError_.isEmpty());
    undo_->setEnabled(idle && !controller_.corners().empty()); reset_->setEnabled(idle && frame && !controller_.corners().empty());
    reselect_->setEnabled(idle && !controller_.corners().empty());
    originPick_->setEnabled(idle && frame && controller_.draftProfile().has_value());
    add_->setEnabled(idle && frame && controller_.canAddReferences()); remove_->setEnabled(idle);
}
} // namespace micro_robot
