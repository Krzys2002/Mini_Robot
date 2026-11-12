#pragma once
#include "app/WorkspaceDiagnosticsController.hpp"
#include <QAbstractTableModel>
#include <QDialog>
#include <set>
class QCheckBox;
class QDoubleSpinBox;
class QLineEdit;
class QLabel;
class QPushButton;
class QComboBox;

namespace micro_robot
{
inline constexpr int WorkspaceMapLegendStops = 32;
inline constexpr std::size_t MaximumWorkspaceMapReferenceLabels = 32;
class WorkspaceMapView;
class WorkspaceSurveyModel final : public QAbstractTableModel
{
public:
    explicit WorkspaceSurveyModel(QObject* parent = nullptr) : QAbstractTableModel(parent) {}
    void setAcquisition(std::shared_ptr<const WorkspaceAcquisitionRecord>,
                        const std::vector<std::uint64_t>& selected = {});
    [[nodiscard]] std::vector<std::uint64_t> selectedIds() const;
    int rowCount(const QModelIndex& parent = {}) const override;
    int columnCount(const QModelIndex& parent = {}) const override;
    QVariant data(const QModelIndex&, int role = Qt::DisplayRole) const override;
    QVariant headerData(int, Qt::Orientation, int role = Qt::DisplayRole) const override;
    Qt::ItemFlags flags(const QModelIndex&) const override;
    bool setData(const QModelIndex&, const QVariant&, int role = Qt::EditRole) override;

private:
    std::shared_ptr<const WorkspaceAcquisitionRecord> acquisition_;
    std::set<std::uint64_t> selected_;
};
class WorkspaceDiagnosticsDialog final : public QDialog
{
    Q_OBJECT
public:
    explicit WorkspaceDiagnosticsDialog(WorkspaceDiagnosticsController&, QWidget* parent = nullptr);

private:
    void resetCalibration();
    void refresh();
    void recompute();
    WorkspaceDiagnosticsController& controller_;
    WorkspaceSurveyModel* survey_{};
    WorkspaceMapView* map_{};
    QComboBox* mode_{};
    QCheckBox *independent_{}, *assumptions_{};
    QDoubleSpinBox* floor_{};
    QLineEdit* method_{};
    QLabel *identity_{}, *status_{};
    QPushButton *compute_{}, *export_{};
};
} // namespace micro_robot
