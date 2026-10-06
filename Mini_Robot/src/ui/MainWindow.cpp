#include "ui/MainWindow.hpp"

#include <QLabel>

namespace micro_robot {

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    setWindowTitle(QStringLiteral("Micro Robot"));
    resize(960, 640);

    auto* welcome = new QLabel(QStringLiteral("Micro Robot"), this);
    welcome->setAlignment(Qt::AlignCenter);
    setCentralWidget(welcome);
}

} // namespace micro_robot
