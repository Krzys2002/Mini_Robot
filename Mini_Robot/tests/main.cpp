#include <QApplication>
#include <QByteArray>
#include <gtest/gtest.h>

int main(int argc, char* argv[])
{
    // Keep widget tests independent of a desktop session. Setting
    // MICRO_ROBOT_TEST_QPA_PLATFORM (for example wayland or xcb) repeats them on that
    // platform plugin inside a running desktop session.
    const auto platform = qgetenv("MICRO_ROBOT_TEST_QPA_PLATFORM");
    qputenv("QT_QPA_PLATFORM", platform.isEmpty() ? QByteArrayLiteral("offscreen") : platform);
    ::testing::InitGoogleTest(&argc, argv);
    QApplication application(argc, argv);
    return RUN_ALL_TESTS();
}
