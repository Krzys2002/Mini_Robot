#pragma once

#include <QMainWindow>

namespace micro_robot {

class MainWindow final : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
};

} // namespace micro_robot
