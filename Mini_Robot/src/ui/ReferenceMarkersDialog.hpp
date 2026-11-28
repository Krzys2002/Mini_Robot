#pragma once
#include "app/WorkspaceCalibrationController.hpp"
#include <QDialog>
#include <functional>
namespace micro_robot {
class ReferenceMarkersDialog final:public QDialog {
public:
    explicit ReferenceMarkersDialog(WorkspaceCalibrationController&, QWidget* parent=nullptr, bool guided=false);
    void reject() override;
private:
    std::function<bool()> hasPendingInput_;
};
}
