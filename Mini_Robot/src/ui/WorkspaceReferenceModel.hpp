#pragma once
#include <limits>
#include "app/WorkspaceCalibrationController.hpp"
#include <QAbstractTableModel>
#include "ui/WorkspaceLengthUnit.hpp"
namespace micro_robot {
// GUI-thread projection of typed application references. No per-cell widgets or
// repeated full-dataset geometry validation; edits go through the service.
class WorkspaceReferenceModel final : public QAbstractTableModel {
public:
    explicit WorkspaceReferenceModel(WorkspaceCalibrationController& controller, QObject* parent=nullptr);
    int rowCount(const QModelIndex& parent={}) const override;
    int columnCount(const QModelIndex& parent={}) const override;
    QVariant data(const QModelIndex& index,int role=Qt::DisplayRole) const override;
    QVariant headerData(int section,Qt::Orientation orientation,int role=Qt::DisplayRole) const override;
    Qt::ItemFlags flags(const QModelIndex& index) const override;
    bool setData(const QModelIndex& index,const QVariant& value,int role=Qt::EditRole) override;
    void refresh();
    void setLengthUnit(WorkspaceLengthUnit unit);
private:
    WorkspaceCalibrationController& controller_;
    WorkspaceLengthUnit unit_{WorkspaceLengthUnit::Meters};
    std::uint64_t shownRevision_{std::numeric_limits<std::uint64_t>::max()};
    int shownRows_{};
    static constexpr int ReferenceColumnCount=6;
};
} // namespace micro_robot
