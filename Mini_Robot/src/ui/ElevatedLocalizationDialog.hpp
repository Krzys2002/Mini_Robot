#pragma once
#include "app/WorkspaceCalibrationController.hpp"
#include <QDialog>
#include <functional>
namespace micro_robot {
// SI survey import/review; computation and validation belong to application/domain services.
class ElevatedLocalizationDialog final : public QDialog {
public:
    explicit ElevatedLocalizationDialog(WorkspaceCalibrationController&, QWidget* parent=nullptr, bool guided=false);
    void reject() override;
private:
    std::function<bool()> hasPendingInput_;
};
}
