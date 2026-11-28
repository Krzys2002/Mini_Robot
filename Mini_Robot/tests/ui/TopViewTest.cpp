#include "ui/TopViewDialog.hpp"
#include "../calibration/IntrinsicFixtures.hpp"

#include <QLabel>
#include <QTest>
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <semaphore>

namespace {
using namespace micro_robot;
using namespace std::chrono_literals;

constexpr std::uint32_t CameraWidth = 160;
constexpr std::uint32_t CameraHeight = 120;
const WorkspaceBounds Bounds{Meters{0.75}, Meters{0.6}, {Meters{-0.25}, Meters{-0.2}}};
constexpr WorkspaceHomography Oblique{{0.01, 0, -0.5, 0, -0.01, 0.7, 0, 0.005, 1}};

FramePtr gradientFrame(std::uint64_t sequence = 1, Timestamp at = MonotonicClock::now())
{
    constexpr std::size_t stride = CameraWidth * 3 + 4;
    auto bytes = std::make_shared<std::vector<std::byte>>(CameraHeight * stride);
    for (std::uint32_t v = 0; v < CameraHeight; ++v) {
        for (std::uint32_t u = 0; u < CameraWidth; ++u) {
            const auto index = v * stride + u * 3;
            (*bytes)[index] = static_cast<std::byte>(u);
            (*bytes)[index + 1] = static_cast<std::byte>(v);
            (*bytes)[index + 2] = std::byte{100};
        }
    }
    return std::make_shared<const Frame>(Frame{sequence, at, CameraWidth, CameraHeight, stride,
                                              PixelFormat::Rgb8, std::move(bytes)});
}

WorkspaceCalibrationProfile calibration()
{
    auto intrinsics = test::validIntrinsicCalibration();
    intrinsics.width = CameraWidth;
    intrinsics.height = CameraHeight;
    intrinsics.cameraMatrix = {120, 0, 80, 0, 125, 60, 0, 0, 1};
    return {WorkspaceCalibrationProfileVersion, "table-a", Bounds, std::move(intrinsics),
            WorkspaceImageCoordinates::UndistortedNativeCameraMatrix, Oblique};
}

TEST(TopViewTest, RectifiesAnObliqueImageInSignedMetricAxes)
{
    const auto frame = gradientFrame();
    const auto image = renderTopView(*frame, Bounds, Oblique);
    ASSERT_FALSE(image.isNull());
    EXPECT_NEAR(double(image.width()) / image.height(), 0.75 / 0.6, 0.002);
    for (const double screenX : {0.1, 0.5, 0.9}) {
        for (const double screenY : {0.1, 0.5, 0.9}) {
            const int sx = static_cast<int>(screenX * image.width()), sy = static_cast<int>(screenY * image.height());
            const double worldX = -0.25 + 0.75 * screenX, worldY = 0.4 - 0.6 * screenY;
            // Independent inverse of the explicit synthetic camera model.
            const double v = (0.7 - worldY) / (0.01 + 0.005 * worldY);
            const double u = 50 + 100 * worldX * (1 + 0.005 * v);
            const auto color = image.pixelColor(sx, sy);
            EXPECT_NEAR(color.red(), u, 2) << screenX << ", " << screenY;
            EXPECT_NEAR(color.green(), v, 2) << screenX << ", " << screenY;
            EXPECT_EQ(color.blue(), 100);
        }
    }
    auto negative = Oblique;
    for (auto& entry : negative.imageToWorld) entry *= -1e180;
    EXPECT_EQ(renderTopView(*frame, Bounds, negative), image);
}

TEST(TopViewTest, RejectsAHorizonCrossingTheWorkspace)
{
    // In the inverse, u = 1/(0.2 - x). Bounds straddle the horizon x = 0.2.
    const WorkspaceHomography crossing{{0.2, 0, -1, 0, 0.01, 0, 1, 0, 0}};
    EXPECT_TRUE(renderTopView(*gradientFrame(), Bounds, crossing).isNull());
}

TEST(TopViewTest, RendersReversedWindingWithAHorizonOutsideTheTable)
{
    const WorkspaceHomography reversed{{-0.01, 0, 0.5, 0, -0.016, 1.5, 0, 0.02, -1}};
    const auto image = renderTopView(*gradientFrame(), Bounds, reversed);
    ASSERT_FALSE(image.isNull());
    const double worldY = 0.1;
    const double v = (1.5 + worldY) / (0.016 + 0.02 * worldY);
    const auto left = image.pixelColor(image.width() / 4, image.height() / 2);
    const auto right = image.pixelColor(3 * image.width() / 4, image.height() / 2);
    EXPECT_NEAR(left.red(), 50 + 6.25 * (0.02 * v - 1), 2);
    EXPECT_NEAR(right.red(), 50 - 31.25 * (0.02 * v - 1), 2);
    EXPECT_NEAR(left.green(), v, 2);
    EXPECT_NEAR(right.green(), v, 2);
}

struct UndistorterState {
    std::atomic<int> calls{};
    std::atomic<std::uint64_t> latestSequence{};
    std::binary_semaphore releaseFirst{0};
};

class BlockingUndistorter final : public ICalibrationUndistorter {
public:
    explicit BlockingUndistorter(UndistorterState& state) : state_(state) {}
    UndistortedPreviewOutcome undistort(const Frame& frame, const IntrinsicCalibration&) override
    {
        state_.latestSequence = frame.sequence;
        if (++state_.calls == 1 && !state_.releaseFirst.try_acquire_for(2s))
            return {{}, "Test did not release first frame."};
        return {std::make_shared<const Frame>(frame), {}};
    }
private:
    UndistorterState& state_;
};

TEST(TopViewTest, OnlyProcessesCompatibleVisibleFramesAndFencesOldWork)
{
    UndistorterState state;
    TopViewDialog dialog{std::make_unique<BlockingUndistorter>(state)};
    auto* status = dialog.findChild<QLabel*>(QStringLiteral("topViewStatus"));
    ASSERT_NE(status, nullptr);
    dialog.setCalibration(calibration());
    dialog.setInput(QStringLiteral("camera-a"), true);
    dialog.receiveFrame(gradientFrame(1)); // Hidden.
    dialog.show();
    QTest::qWait(200);
    EXPECT_EQ(state.calls.load(), 0);

    dialog.receiveFrame(gradientFrame(2));
    ASSERT_TRUE(QTest::qWaitFor([&] { return state.calls.load() == 1; }));
    dialog.receiveFrame(gradientFrame(3)); // Superseded by the newer frame before dispatch.
    dialog.receiveFrame(gradientFrame(4));
    dialog.setInput(QStringLiteral("other-camera"), true);
    state.releaseFirst.release();
    QTest::qWait(200);
    EXPECT_TRUE(status->text().contains(QStringLiteral("Accept tabletop calibration")));
    EXPECT_EQ(state.calls.load(), 1);
    dialog.receiveFrame(gradientFrame(5));
    dialog.setInput(QStringLiteral("camera-a"), true);
    dialog.receiveFrame(gradientFrame(6, MonotonicClock::now() - CameraFrameTimeout));
    dialog.receiveFrame(gradientFrame(7));
    dialog.receiveFrame(gradientFrame(8));
    ASSERT_TRUE(QTest::qWaitFor([&] { return status->text() == QStringLiteral("Live rectified camera"); }));
    EXPECT_EQ(state.calls.load(), 2);
    EXPECT_EQ(state.latestSequence.load(), 8U);
    dialog.setCalibration({});
    EXPECT_TRUE(status->text().contains(QStringLiteral("Accept tabletop calibration")));
}
} // namespace
