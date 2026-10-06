#include "ui/MainWindow.hpp"

#include <QApplication>

int main(int argc, char* argv[])
{
    QApplication application(argc, argv);
    QApplication::setApplicationName(QStringLiteral("Micro Robot"));

    micro_robot::MainWindow window;
    window.show();

    return application.exec();
}
