#pragma once
#include "app/WorkspaceCalibrationController.hpp"
#include <QDialog>
namespace micro_robot {
// SI survey import/review; computation and validation belong to application/domain services.
class ElevatedLocalizationDialog final : public QDialog {
public:
    explicit ElevatedLocalizationDialog(WorkspaceCalibrationController&, QWidget* parent=nullptr);
};
}
