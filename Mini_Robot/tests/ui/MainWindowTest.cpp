#include "ui/MainWindow.hpp"

#include <QApplication>
#include <gtest/gtest.h>

TEST(MainWindowSmokeTest, OpensAndCloses)
{
    micro_robot::MainWindow window;
    window.show();
    QApplication::processEvents();

    ASSERT_TRUE(window.isVisible());
    ASSERT_NE(window.centralWidget(), nullptr);

    EXPECT_TRUE(window.close());
    QApplication::processEvents();
    EXPECT_FALSE(window.isVisible());
}
