#include "ui/VisionDebugView.hpp"
#include "ui/MainWindow.hpp"
#include "app/VisionController.hpp"
#include "app/DesktopController.hpp"
#include "ManualBackends.hpp"

#include <QApplication>
#include <QElapsedTimer>
#include <QImage>
#include <QPlainTextEdit>
#include <QPushButton>
#include <gtest/gtest.h>
#include <functional>
#include <algorithm>

namespace {
using namespace micro_robot;
using namespace std::chrono_literals;
std::shared_ptr<VisionDebugFrame> debugPreview(PixelFormat format = PixelFormat::Gray8)
{
    auto frame = std::make_shared<Frame>();
    frame->sequence = 42; frame->capturedAt = MonotonicClock::now();
    frame->width = 320; frame->height = 200; frame->pixelFormat = format;
    const std::size_t channels = format == PixelFormat::Gray8 ? 1 : format == PixelFormat::Rgb8 ? 3 : 4;
    frame->strideBytes = 320 * channels + 8; // Exercise padded rows, including RGBA alpha.
    auto pixels = std::make_shared<std::vector<std::byte>>(frame->strideBytes * frame->height, std::byte{32});
    if (channels == 4) for (std::size_t y = 0; y < frame->height; ++y)
        for (std::size_t x = 0; x < frame->width; ++x) (*pixels)[y * frame->strideBytes + 4*x + 3] = std::byte{255};
    // A visible raw-image patch independently checks the image transform, not
    // just the overlay transform over an otherwise uniform background.
    for (std::size_t y = 40; y < 60; ++y) for (std::size_t x = 130; x < 150; ++x)
        for (std::size_t c = 0; c < std::min<std::size_t>(channels, 3); ++c)
            (*pixels)[y * frame->strideBytes + channels*x + c] = std::byte{240};
    frame->pixels = pixels;
    auto detections = std::make_shared<MarkerDetections>();
    detections->frameSequence = 42; detections->capturedAt = frame->capturedAt;
    detections->width = 320; detections->height = 200; detections->processingTime = 2ms;
    detections->markers = {
        {MarkerFamily::AprilTag36h11, 0, {{{40,40},{100,40},{100,100},{40,100}}}, std::nullopt},
        {MarkerFamily::AprilTag36h11, 2, {{{250,90},{250,150},{190,150},{190,90}}}, 37.5}};
    return makeVisionDebugFrame(frame, detections, RobotMarkerMap{});
}
bool nearColor(const QImage& image, QPoint point, const std::function<bool(QColor)>& matches)
{
    const double ratio = image.devicePixelRatio();
    const int x = static_cast<int>(point.x() * ratio), y = static_cast<int>(point.y() * ratio);
    const int radius = static_cast<int>(6 * ratio);
    for (int dy = -radius; dy <= radius; ++dy) for (int dx = -radius; dx <= radius; ++dx)
        if (image.rect().contains(x+dx,y+dy) && matches(image.pixelColor(x+dx,y+dy))) return true;
    return false;
}
const auto cyan = [](QColor c) { return c.red() < 80 && c.green() > 170 && c.blue() > 170; };
const auto green = [](QColor c) { return c.red() < 80 && c.green() > 170 && c.blue() < 160; };
const auto yellow = [](QColor c) { return c.red() > 170 && c.green() > 170 && c.blue() < 80; };
TEST(VisionDebugViewTest, ImageAndMultipleOverlaysAlignAfterWideAndTallResizingForAllFormats)
{
    for (auto format : {PixelFormat::Gray8, PixelFormat::Rgb8, PixelFormat::Rgba8}) {
        VisionOverlay view;
        view.present(debugPreview(format));
        view.resize(800,600); view.show(); QApplication::processEvents();
        EXPECT_EQ(view.imageRect(), QRectF(0,50,800,500));
        auto image = view.grab().toImage();
        EXPECT_TRUE(nearColor(image, {350,176}, [](QColor c) { return c.red()>220 && c.green()>220 && c.blue()>220; }));
        EXPECT_TRUE(nearColor(image, {176,226}, cyan)); // Pixel center (70,70), scale 2.5, +50 letterbox.
        EXPECT_TRUE(nearColor(image, {101,226}, green));
        EXPECT_TRUE(nearColor(image, {551,351}, cyan)); // Independent second robot center.
        EXPECT_TRUE(nearColor(image, {630,351}, yellow)); // Second front points right.
        EXPECT_TRUE(nearColor(image, {176,140}, yellow)); // First front points up.
        view.resize(600,800); QApplication::processEvents();
        EXPECT_EQ(view.imageRect(), QRectF(0,212.5,600,375));
        image = view.grab().toImage();
        EXPECT_TRUE(nearColor(image, {263,307}, [](QColor c) { return c.red()>220 && c.green()>220 && c.blue()>220; }));
        EXPECT_TRUE(nearColor(image, {132,345}, cyan));
        EXPECT_TRUE(nearColor(image, {76,345}, green));
        EXPECT_TRUE(nearColor(image, {413,438}, cyan));
        EXPECT_TRUE(nearColor(image, {480,438}, yellow));
    }
}
TEST(VisionDebugViewTest, OrientationArrowRotatesThroughEveryQuarterTurn)
{
    auto preview = debugPreview();
    auto detections = std::make_shared<MarkerDetections>(*preview->detections);
    detections->markers.resize(1);
    const QPoint expected[]{QPoint{141,80}, QPoint{202,141}, QPoint{141,202}, QPoint{80,141}};
    VisionOverlay view; view.resize(640,400); view.show();
    for (const QPoint point : expected) {
        view.present(makeVisionDebugFrame(preview->frame, detections, RobotMarkerMap{}));
        QApplication::processEvents();
        EXPECT_TRUE(nearColor(view.grab().toImage(), point, yellow));
        for (auto& corner : detections->markers[0].corners)
            corner = {140 - corner.v, corner.u}; // Clockwise around (70,70).
    }
}
TEST(VisionDebugViewTest, ClearsInvalidPairsAndReleasesOldFrameStorage)
{
    VisionOverlay view;
    auto preview = debugPreview();
    const std::weak_ptr<const Frame> frame = preview->frame;
    view.present(preview); preview.reset();
    EXPECT_FALSE(frame.expired());
    view.present({});
    EXPECT_TRUE(frame.expired());
    auto invalid = debugPreview();
    auto changed = std::make_shared<MarkerDetections>(*invalid->detections);
    ++changed->frameSequence; invalid->detections = changed;
    view.present(invalid);
    EXPECT_FALSE(view.preview());
}
class NoMarkers final : public IMarkerDetector {
public:
    MarkerDetections detect(const Frame&) override { return {}; }
};
TEST(VisionDebugViewTest, DialogShowsIndependentIdsQualityAgeTimingAndOutputFps)
{
    Logger logger;
    VisionController vision{std::make_unique<NoMarkers>(), logger.channel("vision")};
    VisionDebugDialog dialog{vision};
    dialog.show(); QApplication::processEvents();
    auto preview = debugPreview(); preview->outputFramesPerSecond = 25.0;
    emit vision.previewAvailable("camera-a", preview);
    const auto text = dialog.findChild<QPlainTextEdit*>("visionDiagnostics")->toPlainText();
    EXPECT_TRUE(text.contains("Source camera-a"));
    EXPECT_TRUE(text.contains("frame 42"));
    EXPECT_TRUE(text.contains("Age "));
    EXPECT_TRUE(text.contains("detector 2.00 ms"));
    EXPECT_TRUE(text.contains("output FPS 25.0"));
    EXPECT_TRUE(text.contains("Tag 0 · Robot 1 · min edge 60.0 px · margin unavailable"));
    EXPECT_TRUE(text.contains("Tag 2 · Robot 3 · min edge 60.0 px · margin 37.5"));
    dialog.show(); QApplication::processEvents();
    const auto screenshot = qEnvironmentVariable("MICRO_ROBOT_VISION_SCREENSHOT");
    if (!screenshot.isEmpty()) EXPECT_TRUE(dialog.grab().save(screenshot));
    emit vision.previewAvailable("camera-a", {});
    EXPECT_FALSE(dialog.findChild<VisionOverlay*>("visionOverlay")->preview());
    vision.shutdown();
}
TEST(VisionDebugViewTest, HiddenDialogDefersDiagnosticsAndRefreshesOnShow)
{
    Logger logger;
    VisionController vision{std::make_unique<NoMarkers>(), logger.channel("vision")};
    VisionDebugDialog dialog{vision};
    auto* diagnostics = dialog.findChild<QPlainTextEdit*>("visionDiagnostics");
    emit vision.previewAvailable("camera-a", debugPreview());
    EXPECT_TRUE(diagnostics->toPlainText().isEmpty());
    dialog.show(); QApplication::processEvents();
    EXPECT_TRUE(diagnostics->toPlainText().contains("Source camera-a"));
    dialog.hide();
    const auto previous = diagnostics->toPlainText();
    auto stale = debugPreview();
    auto frame = std::make_shared<Frame>(*stale->frame);
    frame->capturedAt = MonotonicClock::now() - 4s;
    auto detections = std::make_shared<MarkerDetections>(*stale->detections);
    detections->capturedAt = frame->capturedAt;
    stale->frame = frame; stale->detections = detections;
    emit vision.previewAvailable("camera-b", stale);
    EXPECT_EQ(diagnostics->toPlainText(), previous);
    dialog.show(); QApplication::processEvents();
    EXPECT_TRUE(diagnostics->toPlainText().contains("expired"));
    EXPECT_FALSE(dialog.findChild<VisionOverlay*>("visionOverlay")->preview());
    vision.shutdown();
}
TEST(VisionDebugViewTest, WorkerUsesInjectedRobotMarkerMap)
{
    class OneMarker final : public IMarkerDetector {
    public:
        MarkerDetections detect(const Frame&) override { return *debugPreview()->detections; }
    };
    Logger logger;
    VisionController vision{std::make_unique<OneMarker>(), logger.channel("vision"),
        RobotMarkerMap{RobotMarkerProfile{MarkerFamily::AprilTag36h11, 0, 16, 101}}};
    VisionDebugFramePtr received;
    QObject::connect(&vision, &VisionController::previewAvailable, &vision,
        [&](const QString&, VisionDebugFramePtr preview) { received = std::move(preview); });
    vision.setInput("camera-a", true);
    vision.receiveFrame(debugPreview()->frame);
    QElapsedTimer timeout; timeout.start();
    while (vision.busy() && timeout.elapsed() < 5000) {
        QApplication::processEvents(); QThread::yieldCurrentThread();
    }
    ASSERT_FALSE(vision.busy());
    ASSERT_TRUE(received);
    ASSERT_EQ(received->markers.size(), 2U);
    EXPECT_EQ(received->markers[0].robotId, RobotId{101});
    EXPECT_EQ(received->markers[1].robotId, RobotId{103});
    vision.shutdown();
}
TEST(VisionDebugViewTest, OldCapturedFramesExpireImmediatelyInsteadOfShowingLiveOverlay)
{
    Logger logger;
    VisionController vision{std::make_unique<NoMarkers>(), logger.channel("vision")};
    VisionDebugDialog dialog{vision};
    dialog.show(); QApplication::processEvents();
    auto preview = debugPreview();
    auto frame = std::make_shared<Frame>(*preview->frame);
    frame->capturedAt = MonotonicClock::now() - 4s;
    auto detections = std::make_shared<MarkerDetections>(*preview->detections);
    detections->capturedAt = frame->capturedAt;
    preview->frame = frame; preview->detections = detections;
    emit vision.previewAvailable("camera-a", preview);
    EXPECT_FALSE(dialog.findChild<VisionOverlay*>("visionOverlay")->preview());
    EXPECT_TRUE(dialog.findChild<QPlainTextEdit*>("visionDiagnostics")->toPlainText().contains("expired"));
    vision.shutdown();
}
TEST(VisionDebugViewTest, ReentrantInputInvalidationCannotRestoreAnOlderPreview)
{
    Logger logger;
    VisionController vision{std::make_unique<NoMarkers>(), logger.channel("vision")};
    QObject::connect(&vision, &VisionController::previewAvailable, &vision,
        [&](const QString&, VisionDebugFramePtr preview) { if (preview) vision.setInput("camera-b", true); });
    VisionDebugDialog dialog{vision};
    dialog.show(); QApplication::processEvents(); // Receives the outer signal after invalidation.
    vision.setInput("camera-a", true);
    vision.receiveFrame(debugPreview()->frame);
    QElapsedTimer timeout; timeout.start();
    while (vision.busy() && timeout.elapsed() < 5000) {
        QApplication::processEvents(); QThread::yieldCurrentThread();
    }
    ASSERT_FALSE(vision.busy());
    EXPECT_FALSE(dialog.findChild<VisionOverlay*>("visionOverlay")->preview());
    vision.shutdown();
}

} // namespace
