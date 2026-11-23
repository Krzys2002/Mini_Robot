#include "ui/WorkspaceBoardPanel.hpp"
#include "ui/CalibrationForm.hpp"
#include <QCheckBox>
#include <QComboBox>
#include <QGridLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSignalBlocker>
#include <QTableWidget>
#include <QVBoxLayout>
#include <numbers>
#include <QPainter>
#include <QImage>
#include <QMouseEvent>
#include <cmath>
#include <algorithm>
namespace micro_robot {
namespace {
// Presentation-only crop/zoom. Native image/world measurements are never changed.
class WorkspaceBoardZoom final : public QWidget {
public:
    WorkspaceBoardZoom(WorkspaceCalibrationController& controller,std::function<std::size_t()> anchor,
                       std::function<void(std::size_t)> picked,QWidget* parent)
        : QWidget(parent),controller_(controller),anchor_(std::move(anchor)),picked_(std::move(picked))
    { setObjectName("workspaceBoardZoom"); setMinimumHeight(240); setMaximumHeight(320); }
protected:
    struct Geometry { FramePtr frame; std::vector<ImageCorner> points; QRectF source,destination; };
    Geometry geometry() const
    {
        Geometry g; g.frame=controller_.frozenFrame();
        if (!g.frame) return g;
        if (controller_.boardReview()) for (const auto& p:controller_.boardPoints()) g.points.push_back(p.image);
        else if (controller_.detectedBoard()) g.points=controller_.detectedBoard()->detected;
        if (g.points.empty()) return g;
        double loU=g.frame->width,loV=g.frame->height,hiU=0,hiV=0;
        for (const auto& p:g.points) { loU=std::min(loU,p.u); loV=std::min(loV,p.v); hiU=std::max(hiU,p.u); hiV=std::max(hiV,p.v); }
        const double left=std::max(0.0,loU+0.5-WorkspaceBoardZoomMarginPixels), top=std::max(0.0,loV+0.5-WorkspaceBoardZoomMarginPixels);
        g.source={left,top,std::min(static_cast<double>(g.frame->width),hiU+0.5+WorkspaceBoardZoomMarginPixels)-left,
            std::min(static_cast<double>(g.frame->height),hiV+0.5+WorkspaceBoardZoomMarginPixels)-top};
        const QSizeF area{static_cast<double>(width()),static_cast<double>(height()-25)};
        const QSizeF fitted=g.source.size().scaled(area,Qt::KeepAspectRatio);
        g.destination={{(width()-fitted.width())/2,25+(height()-25-fitted.height())/2},fitted}; return g;
    }
    static QPointF at(const Geometry& g,ImageCorner p)
    { return {g.destination.x()+(p.u+0.5-g.source.x())*g.destination.width()/g.source.width(),
              g.destination.y()+(p.v+0.5-g.source.y())*g.destination.height()/g.source.height()}; }
    void mousePressEvent(QMouseEvent* event) override
    {
        if (event->button()!=Qt::LeftButton || controller_.busy()) return;
        const auto g=geometry(); if (g.points.empty()) return;
        double distance=WorkspaceBoardAnchorPickRadiusPixels; std::optional<std::size_t> selected;
        for (std::size_t i=0;i<g.points.size();++i) {
            const auto delta=at(g,g.points[i])-event->position(); const double d=std::hypot(delta.x(),delta.y());
            if (d<distance) {distance=d; selected=i;}
        }
        if (selected) picked_(*selected);
    }
    void paintEvent(QPaintEvent*) override
    {
        QPainter painter(this); painter.fillRect(rect(),Qt::black); painter.setPen(Qt::white);
        const auto g=geometry();
        if (!g.frame || g.points.empty()) {painter.drawText(rect(),Qt::AlignCenter,"Detect the exact frozen frame to review a labelled board zoom"); return;}
        const auto format=g.frame->pixelFormat==PixelFormat::Rgb8?QImage::Format_RGB888:g.frame->pixelFormat==PixelFormat::Rgba8?QImage::Format_RGBA8888:QImage::Format_Grayscale8;
        QImage image(reinterpret_cast<const uchar*>(g.frame->pixels->data()),static_cast<int>(g.frame->width),static_cast<int>(g.frame->height),
                     static_cast<qsizetype>(g.frame->strideBytes),format);
        painter.drawImage(g.destination,image,g.source);
        for (std::size_t i=0;i<g.points.size();++i) {
            painter.setPen(QPen(controller_.boardReview() && !controller_.boardReview()->included[i]?Qt::gray:Qt::cyan,2));
            painter.drawEllipse(at(g,g.points[i]),4,4); painter.drawText(at(g,g.points[i])+QPointF(5,-5),QString::number(i));
        }
        const auto index=anchor_();
        if (index<g.points.size()) {
            const auto anchor=at(g,g.points[index]); painter.setPen(QPen(Qt::magenta,2)); painter.drawEllipse(anchor,8,8);
            painter.drawText(anchor+QPointF(8,12),"Anchor");
            if (controller_.boardReview()) {
                const auto columns=static_cast<std::size_t>(WorkspaceBoardTarget.columns);
                const bool lastColumn=index%columns+1==columns, lastRow=index+columns>=g.points.size();
                const auto column=lastColumn?anchor-at(g,g.points[index-1]):at(g,g.points[index+1])-anchor;
                const auto row=lastRow?anchor-at(g,g.points[index-columns]):at(g,g.points[index+columns])-anchor;
                auto arrow=[&](QPointF direction,bool last,QColor color) {
                    const auto start=last?anchor-direction*0.8:anchor, tip=last?anchor:anchor+direction*0.8;
                    const double length=std::hypot(direction.x(),direction.y()); if (length==0) return;
                    const auto unit=direction/length; const QPointF normal{-unit.y(),unit.x()};
                    painter.setPen(QPen(color,2)); painter.drawLine(start,tip);
                    painter.drawLine(tip,tip-unit*8+normal*4); painter.drawLine(tip,tip-unit*8-normal*4);
                };
                arrow(column,lastColumn,Qt::red); arrow(row,lastRow,Qt::yellow);
            }
        }
        painter.setPen(Qt::white);
        painter.drawText(QRect{5,0,width()-10,25},Qt::AlignVCenter,controller_.boardReview()
            ?"Logical board zoom · red arrow +cols, yellow arrow +rows · click anchor · display crop only; native coordinates retained"
            :"Detector-order zoom · Review measured geometry for logical ordering · no physical corner identity inferred");
    }
private:
    WorkspaceCalibrationController& controller_;
    std::function<std::size_t()> anchor_;
    std::function<void(std::size_t)> picked_;
};
} // namespace
WorkspaceBoardPanel::WorkspaceBoardPanel(WorkspaceCalibrationController& controller,
    std::function<std::optional<std::uint64_t>()> selectedReference,QWidget* parent)
    : QGroupBox("Tabletop chessboard acquisition",parent),controller_(controller),selectedReference_(std::move(selectedReference))
{
    setObjectName("workspaceBoardPanel"); setCheckable(true); setChecked(false);
    auto* layout=new QVBoxLayout(this); auto* body=new QWidget(this); auto* content=new QVBoxLayout(body); layout->addWidget(body);
    body->setVisible(false); connect(this,&QGroupBox::toggled,body,&QWidget::setVisible);
    auto* help=new QLabel("Keep the 9×6 inner-corner / nominal 25 mm target FLAT on the table. Keep camera/table fixed between placements. Record corner-surface elevation and board/backing thickness below, or leave explicitly unknown; these metadata do not compensate H. "
        "Enter the measured spacing. Every placement needs a measured anchor and explicit +columns/+rows orientation. Detector ordering does not identify a physical corner. "
        "Review index labels, reversals and exclusions; manual corrections remain possible after import.",body);
    help->setWordWrap(true); content->addWidget(help);
    auto* actions=new QHBoxLayout;
    auto button=[&](const char* name,const char* text) { auto* b=new QPushButton(text,body); b->setObjectName(name); actions->addWidget(b); return b; };
    freeze_=button("workspaceBoardFreeze","Freeze next board frame");
    detect_=button("workspaceBoardDetect","Detect exact frozen frame");
    review_=button("workspaceBoardReview","Review measured geometry"); content->addLayout(actions);
    settings_=new QWidget(body); auto* grid=new QGridLayout(settings_);
    fields_=new CalibrationForm(this);
    auto number=[&](const char* name,double lo,double hi,bool length,std::optional<double> fallback={}) {
        auto* field=new QLineEdit(settings_);
        field->setObjectName(name);
        fields_->watch(field,[field]{return field->text();},[=,this] {
            const double scale=length ? workspaceUnitsPerMeter(unit_) : 1.0;
            const double value=CalibrationForm::value(field,fallback.value_or(std::numeric_limits<double>::quiet_NaN()));
            return std::isfinite(value) && value>=lo*scale && value<=hi*scale ? QString{}
                : QStringLiteral("Enter a number between %1 and %2.").arg(lo*scale).arg(hi*scale);
        });
        return field;
    };
    anchor_=new QLineEdit(settings_);
    anchor_->setObjectName("workspaceBoardAnchorIndex");
    fields_->watch(anchor_,[this]{return anchor_->text();},[this] {
        if (anchor_->text().trimmed().isEmpty()) return QString{};
        bool ok=false;
        const int index=anchor_->text().toInt(&ok);
        const int limit=WorkspaceBoardTarget.columns*WorkspaceBoardTarget.rows;
        return ok && index>=0 && index<limit ? QString{}
            : QStringLiteral("Enter a corner index between 0 and %1.").arg(limit-1);
    });
    x_=number("workspaceBoardAnchorX",-MaximumAnchorMeters,MaximumAnchorMeters,true);
    y_=number("workspaceBoardAnchorY",-MaximumAnchorMeters,MaximumAnchorMeters,true);
    angle_=number("workspaceBoardColumnAngle",-MaximumAngleDegrees,MaximumAngleDegrees,false,0);
    spacing_=number("workspaceBoardSpacing",MinimumSpacingMeters,MaximumSpacingMeters,true);
    rowDirection_=new QComboBox(settings_); rowDirection_->setObjectName("workspaceBoardRowDirection");
    rowDirection_->addItems({"Choose +rows direction","+rows = +90° from +columns","+rows = -90° from +columns"});
    auto labelled=[&](QWidget* input,const QString& name,int row,int column,bool length=false) {
        auto* group=new QWidget(settings_); auto* box=new QVBoxLayout(group); box->setContentsMargins(0,0,0,0);
        auto* label=new QLabel(name,group); label->setBuddy(input);
        label->setWordWrap(true);
        if (length) label->setProperty("workspaceLengthLabel",name);
        box->addWidget(label); box->addWidget(input); grid->addWidget(group,row,column);
    };
    labelled(anchor_,"Anchor corner index · blank uses 0",0,0);
    labelled(x_,"Anchor X / measured offset",0,1,true);
    labelled(y_,"Anchor Y / measured offset",0,2,true);
    labelled(angle_,"+columns angle from world +X (degrees) · blank uses 0",1,0);
    labelled(rowDirection_,"Board row direction",1,1);
    labelled(spacing_,"Measured corner spacing",1,2,true);
    reverseColumns_=new QCheckBox("Reverse detector columns",settings_); reverseColumns_->setObjectName("workspaceBoardReverseColumns");
    reverseRows_=new QCheckBox("Reverse detector rows",settings_); reverseRows_->setObjectName("workspaceBoardReverseRows");
    relative_=new QCheckBox("Measured offset from reference",settings_); relative_->setObjectName("workspaceBoardRelative");
    reference_=new QPushButton("Use selected reference",settings_); reference_->setObjectName("workspaceBoardUseReference");
    grid->addWidget(reverseColumns_,2,0); grid->addWidget(reverseRows_,2,1); grid->addWidget(relative_,2,2); grid->addWidget(reference_,3,2);
    note_=new QLineEdit(settings_);
    note_->setObjectName("workspaceBoardNote");
    labelled(note_,"Measurement / setup note",3,0);
    content->addWidget(settings_);
    details_=new QLabel(body); details_->setObjectName("workspaceBoardDetails"); details_->setWordWrap(true); content->addWidget(details_);
    auto* zoom=new WorkspaceBoardZoom(controller_,[this]{return static_cast<std::size_t>(anchor_->text().toInt());},
        [this](std::size_t index){anchor_->setText(QString::number(index));},body);
    content->addWidget(zoom);
    connect(&controller_,&WorkspaceCalibrationController::changed,zoom,[zoom]{zoom->update();});
    connect(anchor_,&QLineEdit::editingFinished,zoom,[zoom]{zoom->update();});
    connect(anchor_,&QLineEdit::textChanged,zoom,[anchor=anchor_,zoom]{if (!anchor->hasFocus()) zoom->update();});
    points_=new QTableWidget(0,7,body); points_->setObjectName("workspaceBoardPoints");
    points_->setHorizontalHeaderLabels({"Index","u px","v px","x m","y m","Include","Review status"});
    points_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch); points_->setMaximumHeight(200); content->addWidget(points_);
    identity_=new QCheckBox("I confirm physical anchor identity, +columns/+rows arrows, ordering and inclusion mask",body);
    identity_->setObjectName("workspaceBoardIdentityConfirmed"); content->addWidget(identity_);
    auto* footer=new QHBoxLayout;
    check_=new QCheckBox("Check-only placement (measure independently; never fitted)",body); check_->setObjectName("workspaceBoardCheck"); footer->addWidget(check_);
    import_=new QPushButton("Import reviewed placement",body); import_->setObjectName("workspaceBoardImport"); footer->addWidget(import_); content->addLayout(footer);
    connect(freeze_,&QPushButton::clicked,this,[this] {
        if (confirmReplacingReview("Freeze next board frame?")) controller_.freezeBoard();
    });
    connect(detect_,&QPushButton::clicked,this,[this] {
        if (confirmReplacingReview("Detect board again?")) controller_.detectBoard();
    });
    connect(review_,&QPushButton::clicked,this,[this]{review();});
    connect(import_,&QPushButton::clicked,this,[this] {
        if (reviewDirty_) return;
        if (controller_.draftProfile() && !confirmCalibrationLoss(this, "Change tabletop references?",
                "Importing this placement invalidates the corresponding solved transform or verification evidence.")) return;
        controller_.importBoardPlacement(check_->isChecked(),identity_->isChecked());
    });
    for (auto* field:{x_,y_,angle_,spacing_,anchor_,note_})
        connect(field,&QLineEdit::textChanged,this,[this]{edited();});
    connect(rowDirection_,&QComboBox::currentIndexChanged,this,[this]{edited();});
    for (auto* box:{reverseColumns_,reverseRows_,relative_}) connect(box,&QCheckBox::toggled,this,[this]{edited();});
    int metadataRow=4;
    for (auto entry : {std::pair{&surface_, "Corner surface elevation above tabletop; blank = unknown"},
                       std::pair{&boardThickness_, "Board thickness; blank = unknown"},
                       std::pair{&backingThickness_, "Backing thickness; blank = unknown"},
                       std::pair{&elevationMethod_, "Elevation measurement method/provenance"}}) {
        *entry.first=new QLineEdit(settings_);
        (*entry.first)->setObjectName(entry.first==&surface_ ? "workspaceBoardSurfaceElevation" :
            entry.first==&boardThickness_ ? "workspaceBoardThickness" :
            entry.first==&backingThickness_ ? "workspaceBoardBackingThickness" : "workspaceBoardElevationMethod");
        auto* label=new QLabel(entry.second,settings_); label->setBuddy(*entry.first);
        if (entry.first!=&elevationMethod_) label->setProperty("workspaceLengthLabel",QString::fromUtf8(entry.second));
        grid->addWidget(label,metadataRow,0);
        grid->addWidget(*entry.first,metadataRow++,1);
        connect(*entry.first,&QLineEdit::textChanged,this,[this]{edited();});
    }
    for (auto* field:{note_, elevationMethod_}) fields_->watch(field,[field]{return field->text();},[field] {
        return field->text().toUtf8().size() <= static_cast<qsizetype>(MaximumWorkspacePlacementNoteBytes) ? QString{}
            : QStringLiteral("Measurement note exceeds the supported length.");
    });
    for (auto* field:{surface_,boardThickness_,backingThickness_}) fields_->watch(field,[field]{return field->text();},[=,this] {
        if (field->text().trimmed().isEmpty()) return QString{};
        const double value=CalibrationForm::value(field);
        if (!std::isfinite(value) || (field!=surface_ && value<0))
            return QStringLiteral("Enter a valid measured length or leave blank.");
        if (field==surface_ && elevationMethod_->text().trimmed().isEmpty())
            return QStringLiteral("Add the measurement method for a known surface elevation.");
        return QString{};
    });
    connect(reference_,&QPushButton::clicked,this,[this]{relativeId_=selectedReference_(); edited();});
    connect(check_,&QCheckBox::toggled,this,[this]{identity_->setChecked(false);});
    connect(identity_,&QCheckBox::toggled,this,[this]{refresh();});
    connect(points_,&QTableWidget::itemChanged,this,[this](QTableWidgetItem* item) {
        if (item->column()!=5) return;
        identity_->setChecked(false);
        controller_.setBoardPointIncluded(static_cast<std::size_t>(item->row()),item->checkState()==Qt::Checked);
    });
    connect(&controller_,&WorkspaceCalibrationController::changed,this,[this]{refresh();}); refresh();
}
void WorkspaceBoardPanel::setLengthUnit(WorkspaceLengthUnit unit,bool recalculate)
{
    const double scale=workspaceUnitsPerMeter(unit),ratio=scale/workspaceUnitsPerMeter(unit_);
    preserveInputs_=true;
    if (recalculate) for (auto* field:{x_,y_,spacing_,surface_,boardThickness_,backingThickness_}) {
        const QSignalBlocker block(field); bool ok=false;
        const double value=parseWorkspaceLength(field->text(),&ok);
        if (ok && std::isfinite(value)) field->setText(workspaceLengthText(value*ratio));
    }
    unit_=unit;
    points_->setHorizontalHeaderLabels({"Index","u px","v px","x ("+workspaceUnitLabel(unit)+")",
        "y ("+workspaceUnitLabel(unit)+")","Include","Review status"});
    if (!recalculate) edited();
    shownRevision_=std::numeric_limits<std::uint64_t>::max(); refresh();
}
void WorkspaceBoardPanel::edited()
{
    reviewDirty_=true;
    identity_->setChecked(false);
    refresh();
    if (controller_.boardReview()) details_->setText("Review measured geometry again before importing the changed placement.");
}
bool WorkspaceBoardPanel::confirmReplacingReview(const QString& title)
{
    return !controller_.boardReview() || confirmCalibrationLoss(this,title,
        "This action discards the pending board review and its inclusion choices.");
}
void WorkspaceBoardPanel::review()
{
    if (relative_->isChecked() && !relativeId_) { details_->setText("Select a reference row and press Use selected reference first."); return; }
    const auto error=fields_->validate();
    if (!error.isEmpty()) { details_->setText(error); return; }
    if (rowDirection_->currentIndex()==0) { details_->setText("Choose the measured board row direction."); return; }
    if (!confirmReplacingReview("Replace board review?")) return;
    const double scale=workspaceUnitsPerMeter(unit_);
    WorkspaceBoardPlacement p;
    p.anchorIndex=static_cast<std::size_t>(anchor_->text().toInt());
    p.anchor={Meters{CalibrationForm::value(x_)/scale},Meters{CalibrationForm::value(y_)/scale}};
    p.columnAngle=Radians{CalibrationForm::value(angle_,0)*std::numbers::pi/180};
    p.rowDirection=rowDirection_->currentIndex()==1?1:rowDirection_->currentIndex()==2?-1:0;
    p.reverseColumns=reverseColumns_->isChecked(); p.reverseRows=reverseRows_->isChecked();
    p.target.squareSizeMeters=CalibrationForm::value(spacing_)/scale;
    p.note=note_->text().toStdString();
    auto measurement=[scale](QLineEdit* field)->std::optional<double> {
        if (field->text().trimmed().isEmpty()) return {};
        return CalibrationForm::value(field)/scale;
    };
    p.surfaceElevationMeters=measurement(surface_);
    p.boardThicknessMeters=measurement(boardThickness_);
    p.backingThicknessMeters=measurement(backingThickness_);
    p.elevationMethod=elevationMethod_->text().toStdString();
    if (relative_->isChecked()) { p.relativeReferenceId=relativeId_; p.measuredOffset=p.anchor; }
    reviewDirty_=false;
    controller_.reviewBoardPlacement(std::move(p));
}
void WorkspaceBoardPanel::refresh()
{
    const bool idle=!controller_.busy(); settings_->setEnabled(idle);
    freeze_->setEnabled(idle && controller_.boundaryConfirmed() && controller_.canFreeze());
    detect_->setEnabled(controller_.canDetectBoard()); review_->setEnabled(controller_.canReviewBoard());
    identity_->setEnabled(!reviewDirty_ && controller_.canImportBoard()); points_->setEnabled(idle && !reviewDirty_);
    import_->setEnabled(!reviewDirty_ && controller_.canImportBoard() && identity_->isChecked());
    if (shownRevision_==controller_.boardRevision()) return;
    shownRevision_=controller_.boardRevision();
    if (frameId_!=controller_.frameId()) {
        frameId_=controller_.frameId(); relativeId_.reset();
        const QSignalBlocker a(x_),b(y_),c(angle_),d(anchor_),e(rowDirection_),f(reverseColumns_),g(reverseRows_),h(relative_),
            j(note_),k(spacing_),l(surface_),m(boardThickness_),n(backingThickness_),o(elevationMethod_);
        if (!preserveInputs_) {
            for (auto* field:{x_,y_,angle_,anchor_,spacing_,note_,surface_,boardThickness_,backingThickness_,elevationMethod_}) {
                field->clear();
                field->setStyleSheet({});
                field->setToolTip({});
            }
            rowDirection_->setCurrentIndex(0);
            reverseColumns_->setChecked(false); reverseRows_->setChecked(false); relative_->setChecked(false);
        } else { relative_->setChecked(false); }
    }
    if (!preserveInputs_) { const QSignalBlocker b(identity_); identity_->setChecked(false); }
    import_->setEnabled(!reviewDirty_ && controller_.canImportBoard() && identity_->isChecked());
    const QSignalBlocker block(points_); points_->setRowCount(0);
    if (const auto& p=controller_.boardReview()) {
        details_->setText(QStringLiteral("Placement %1 · frame %2 · resolved anchor (%3,%4) %8 · +columns %5 rad · +rows sign %6 · %7 stored placements. "
                                        "Original detection and the explicit exclusion mask are retained.")
            .arg(p->id).arg(p->frameSequence).arg(p->anchor.x.value*workspaceUnitsPerMeter(unit_)).arg(p->anchor.y.value*workspaceUnitsPerMeter(unit_)).arg(p->columnAngle.value).arg(p->rowDirection).arg(controller_.placements().size()).arg(workspaceUnitLabel(unit_)));
        for (std::size_t i=0;i<controller_.boardPoints().size();++i) {
            const auto& pair=controller_.boardPoints()[i]; const int row=points_->rowCount(); points_->insertRow(row);
            const double values[]{static_cast<double>(i),pair.image.u,pair.image.v,pair.world.x.value*workspaceUnitsPerMeter(unit_),pair.world.y.value*workspaceUnitsPerMeter(unit_)};
            for (int c=0;c<5;++c) { auto* item=new QTableWidgetItem(QString::number(values[c],'g',12)); item->setFlags(Qt::ItemIsEnabled|Qt::ItemIsSelectable); points_->setItem(row,c,item); }
            const auto rejection=controller_.boardAssessment()[i];
            auto* include=new QTableWidgetItem; include->setFlags(rejection==WorkspaceReferenceRejection::None?Qt::ItemIsEnabled|Qt::ItemIsUserCheckable:Qt::ItemIsEnabled);
            include->setCheckState(p->included[i]?Qt::Checked:Qt::Unchecked); points_->setItem(row,5,include);
            auto* status=new QTableWidgetItem(rejection==WorkspaceReferenceRejection::None?"Reviewed candidate":rejection==WorkspaceReferenceRejection::Duplicate?"Duplicate — excluded":"Outside — excluded");
            status->setFlags(Qt::ItemIsEnabled|Qt::ItemIsSelectable); points_->setItem(row,6,status);
        }
    } else details_->setText(QStringLiteral("%1 placements retained. %2 · Use explicit absolute coordinates or a measured world-axis offset. Nothing is inferred from previous board motion.")
        .arg(controller_.placements().size()).arg(relativeId_?QStringLiteral("Offset reference #%1").arg(*relativeId_):QStringLiteral("No offset reference selected")));
}
} // namespace micro_robot
