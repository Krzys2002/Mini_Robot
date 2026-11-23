#include "ui/CalibrationImageView.hpp"

#include <QApplication>
#include <QImage>
#include <QMouseEvent>
#include <QPushButton>
#include <QWheelEvent>
#include <gtest/gtest.h>

namespace {
using namespace micro_robot;

constexpr unsigned ImageWidth = 320;
constexpr unsigned ImageHeight = 180;

FramePtr imageFrame()
{
    auto pixels = std::make_shared<std::vector<std::byte>>(ImageWidth * ImageHeight);
    for (unsigned y = 0; y < ImageHeight; ++y) {
        for (unsigned x = 0; x < ImageWidth; ++x) {
            const auto level = 40 + (x >= ImageWidth / 2 ? 50 : 0) + (y >= ImageHeight / 2 ? 100 : 0);
            (*pixels)[y * ImageWidth + x] = static_cast<std::byte>(level);
        }
    }
    return std::make_shared<Frame>(Frame{0, {}, ImageWidth, ImageHeight, ImageWidth, PixelFormat::Gray8, pixels});
}

class CalibrationImageViewTest : public testing::Test {
protected:
    CalibrationImageViewTest() : view_(&owner_), controls_(view_.createControls(&owner_))
    {
        view_.resize(ImageWidth, ImageHeight);
        view_.clicked = [this](ImageCorner point) { clicks_.push_back(point); };
    }

    void pressButton(const char* name)
    {
        auto* button = controls_->findChild<QPushButton*>(name);
        ASSERT_NE(button, nullptr);
        button->click();
    }

    void pointer(QEvent::Type type, QPointF location, Qt::MouseButton button, Qt::MouseButtons held)
    {
        QMouseEvent event(type, location, location, button, held, Qt::NoModifier);
        QApplication::sendEvent(&view_, &event);
    }

    void click(QPointF location)
    {
        pointer(QEvent::MouseButtonPress, location, Qt::LeftButton, Qt::LeftButton);
        pointer(QEvent::MouseButtonRelease, location, Qt::LeftButton, Qt::NoButton);
    }

    void expectPixel(double u, double v)
    {
        ASSERT_FALSE(clicks_.empty());
        EXPECT_NEAR(clicks_.back().u, u, 1e-9);
        EXPECT_NEAR(clicks_.back().v, v, 1e-9);
    }

    int grayAt(QPointF location)
    {
        const QImage image = view_.grab().toImage();
        return image.pixelColor((location * image.devicePixelRatio()).toPoint()).red();
    }

    QWidget owner_;
    WorkspaceImageView view_;
    QWidget* controls_;
    std::vector<ImageCorner> clicks_;
};

TEST_F(CalibrationImageViewTest, FlippedRasterAndPointSelectionUseTheSameNativePixels)
{
    EXPECT_FALSE(controls_->isEnabled());
    view_.setFrame(imageFrame());
    ASSERT_TRUE(controls_->isEnabled());
    const QPointF pick{60.5, 55.5};
    click(pick);
    expectPixel(60, 55);
    EXPECT_EQ(grayAt(pick), 40);

    pressButton("workspaceImageFlipHorizontal");
    click(pick);
    expectPixel(259, 55);
    EXPECT_EQ(grayAt(pick), 90);

    pressButton("workspaceImageFlipVertical");
    click(pick);
    expectPixel(259, 124);
    EXPECT_EQ(grayAt(pick), 190);

    pressButton("workspaceImageFlipHorizontal");
    click(pick);
    expectPixel(60, 124);
    EXPECT_EQ(grayAt(pick), 140);
    EXPECT_EQ(clicks_.size(), 4U);
}

TEST_F(CalibrationImageViewTest, ZoomAndPanPreservePointIdentityAndFitRestoresViewport)
{
    view_.setFrame(imageFrame());
    const QPointF anchor{124.5, 72.5};
    QWheelEvent wheel(anchor, anchor, {}, {0, 120}, Qt::NoButton, Qt::NoModifier, Qt::NoScrollPhase, false);
    QApplication::sendEvent(&view_, &wheel);
    click(anchor);
    expectPixel(124, 72);
    click(anchor + QPointF{25, 0});
    expectPixel(144, 72);

    pressButton("workspaceImagePan");
    const QPointF delta{20, 10};
    pointer(QEvent::MouseButtonPress, anchor, Qt::LeftButton, Qt::LeftButton);
    pointer(QEvent::MouseMove, anchor + delta, Qt::NoButton, Qt::LeftButton);
    pointer(QEvent::MouseButtonRelease, anchor + delta, Qt::LeftButton, Qt::NoButton);
    ASSERT_EQ(clicks_.size(), 2U);
    pressButton("workspaceImagePan");
    click(anchor + delta);
    expectPixel(124, 72);

    pressButton("workspaceImageFit");
    click(anchor);
    expectPixel(124, 72);
    click(anchor + QPointF{25, 0});
    expectPixel(149, 72);
}

TEST_F(CalibrationImageViewTest, LetterboxAndMissingFrameCannotAddPoints)
{
    click({60.5, 55.5});
    view_.resize(400, 200);
    view_.setFrame(imageFrame());
    click({10, 100});
    EXPECT_TRUE(clicks_.empty());
    click({200, 100});
    expectPixel(159.5, 89.5);
    view_.setFrame({});
    click({200, 100});
    EXPECT_EQ(clicks_.size(), 1U);
    EXPECT_FALSE(controls_->isEnabled());
}

} // namespace
