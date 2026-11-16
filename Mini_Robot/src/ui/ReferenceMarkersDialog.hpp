#pragma once
#include "app/WorkspaceCalibrationController.hpp"
#include <QDialog>
namespace micro_robot {
class ReferenceMarkersDialog final:public QDialog {
public:
    explicit ReferenceMarkersDialog(WorkspaceCalibrationController&,QWidget* parent=nullptr);
};
}
