#include "ui/WorkspaceReferenceModel.hpp"
#include <limits>
#include <algorithm>
namespace micro_robot {
WorkspaceReferenceModel::WorkspaceReferenceModel(WorkspaceCalibrationController& controller,QObject* parent)
    : QAbstractTableModel(parent),controller_(controller)
{
    connect(&controller_,&WorkspaceCalibrationController::changed,this,[this]{refresh();}); refresh();
}
int WorkspaceReferenceModel::rowCount(const QModelIndex& parent) const { return parent.isValid()?0:static_cast<int>(controller_.references().size()); }
int WorkspaceReferenceModel::columnCount(const QModelIndex& parent) const { return parent.isValid()?0:ReferenceColumnCount; }
QVariant WorkspaceReferenceModel::headerData(int section,Qt::Orientation orientation,int role) const
{
    if (role!=Qt::DisplayRole) return {};
    if (orientation==Qt::Vertical) return section+1;
    const QStringList labels{"u (px)","v (px)","x ("+workspaceUnitLabel(unit_)+")","y ("+workspaceUnitLabel(unit_)+")","Check only","Provenance"};
    return section>=0 && section<labels.size()?QVariant{labels[section]}:QVariant{};
}
QVariant WorkspaceReferenceModel::data(const QModelIndex& index,int role) const
{
    if (!index.isValid() || index.row()<0 || index.row()>=rowCount()) return {};
    const auto& r=controller_.references()[static_cast<std::size_t>(index.row())];
    if (role==Qt::ToolTipRole && r.source) {
        const auto placement=std::find_if(controller_.placements().begin(),controller_.placements().end(),[&](const auto& p){return p.id==r.source->placementId;});
        if (placement!=controller_.placements().end()) {
            const auto original=generateWorkspaceBoard(*placement,controller_.draftBounds());
            if (r.source->cornerIndex<original.points.size()) {
                const auto& p=original.points[r.source->cornerIndex];
                return QStringLiteral("Placement %1 · source frame %2 · anchor index %3 at (%4,%5) %12 · +columns %6 rad · +rows sign %7.\n"
                                      "Original native pixel (%8,%9); original generated world (%10,%11) %12. Current editable values are shown in the row.\n"
                                      "Original detection, measured anchor/offset and review mask remain in the acquisition record. Shared placement measurements can have correlated errors.")
                    .arg(placement->id).arg(placement->frameSequence).arg(placement->anchorIndex)
                    .arg(placement->anchor.x.value*workspaceUnitsPerMeter(unit_),0,'g',12).arg(placement->anchor.y.value*workspaceUnitsPerMeter(unit_),0,'g',12)
                    .arg(placement->columnAngle.value,0,'g',12).arg(placement->rowDirection)
                    .arg(p.image.u,0,'g',12).arg(p.image.v,0,'g',12).arg(p.world.x.value*workspaceUnitsPerMeter(unit_),0,'g',12).arg(p.world.y.value*workspaceUnitsPerMeter(unit_),0,'g',12).arg(workspaceUnitLabel(unit_));
            }
        }
    }
    if (index.column()==4) return role==Qt::CheckStateRole?QVariant{r.check?Qt::Checked:Qt::Unchecked}:QVariant{};
    if (role!=Qt::DisplayRole && role!=Qt::EditRole && role!=Qt::ToolTipRole) return {};
    if (index.column()==5) {
        if (r.boundary) return QStringLiteral("Boundary #%1").arg(r.id);
        if (r.source) return QStringLiteral("Placement %1 · corner %2 (editable)").arg(r.source->placementId).arg(r.source->cornerIndex);
        return QStringLiteral("Manual #%1").arg(r.id);
    }
    const double values[]{r.point.image.u,r.point.image.v,r.point.world.x.value*workspaceUnitsPerMeter(unit_),r.point.world.y.value*workspaceUnitsPerMeter(unit_)};
    if (index.column()<0 || index.column()>3) return {};
    return QString::number(values[index.column()],'g',role==Qt::DisplayRole?12:std::numeric_limits<double>::max_digits10);
}
Qt::ItemFlags WorkspaceReferenceModel::flags(const QModelIndex& index) const
{
    if (!index.isValid() || index.row()>=rowCount()) return Qt::NoItemFlags;
    auto result=Qt::ItemIsEnabled|Qt::ItemIsSelectable;
    if (!controller_.references()[static_cast<std::size_t>(index.row())].boundary) {
        if (index.column()<4) result|=Qt::ItemIsEditable;
        if (index.column()==4) result|=Qt::ItemIsUserCheckable;
    }
    return result;
}
bool WorkspaceReferenceModel::setData(const QModelIndex& index,const QVariant& value,int role)
{
    if (!index.isValid() || index.row()<0 || index.row()>=rowCount() || controller_.busy()) return false;
    const auto& r=controller_.references()[static_cast<std::size_t>(index.row())];
    if (r.boundary) return false;
    auto point=r.point; bool check=r.check;
    if (index.column()==4 && role==Qt::CheckStateRole) check=value.toInt()==Qt::Checked;
    else if (index.column()>=0 && index.column()<4 && role==Qt::EditRole) {
        bool valid{}; double number=index.column()>=2 ? parseWorkspaceLength(value.toString(),&valid) : value.toDouble(&valid);
        if (!valid) number=std::numeric_limits<double>::quiet_NaN();
        if (index.column()==0) point.image.u=number;
        if (index.column()==1) point.image.v=number;
        if (index.column()==2) point.world.x=Meters{number/workspaceUnitsPerMeter(unit_)};
        if (index.column()==3) point.world.y=Meters{number/workspaceUnitsPerMeter(unit_)};
    } else return false;
    controller_.editReference(static_cast<std::size_t>(index.row()),point,check); return true;
}
void WorkspaceReferenceModel::setLengthUnit(WorkspaceLengthUnit unit)
{
    if (unit==unit_) return;
    unit_=unit;
    emit headerDataChanged(Qt::Horizontal,2,3);
    if (rowCount()) emit dataChanged(index(0,2),index(rowCount()-1,3));
}
void WorkspaceReferenceModel::refresh()
{
    if (shownRevision_==controller_.referencesRevision()) return;
    shownRevision_=controller_.referencesRevision();
    if (shownRows_!=rowCount()) { beginResetModel(); shownRows_=rowCount(); endResetModel(); }
    else if (shownRows_) emit dataChanged(index(0,0),index(shownRows_-1,ReferenceColumnCount-1));
}
} // namespace micro_robot
