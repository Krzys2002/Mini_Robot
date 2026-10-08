#include <QApplication>
#include <QByteArray>
#include <gtest/gtest.h>

int main(int argc, char* argv[])
{
    // Keep widget smoke tests independent of a desktop session.
    qputenv("QT_QPA_PLATFORM", QByteArrayLiteral("offscreen"));
    ::testing::InitGoogleTest(&argc, argv);
    QApplication application(argc, argv);
    return RUN_ALL_TESTS();
}
