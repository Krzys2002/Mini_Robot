#include "vision/VisionDebugFrame.hpp"
#include <gtest/gtest.h>

namespace {
using namespace micro_robot;
using namespace std::chrono_literals;
struct Input {
    std::shared_ptr<Frame> frame = std::make_shared<Frame>();
    std::shared_ptr<MarkerDetections> detections = std::make_shared<MarkerDetections>();
    Input()
    {
        frame->sequence = detections->frameSequence = 42;
        frame->capturedAt = detections->capturedAt = Timestamp{1s};
        frame->width = detections->width = 100;
        frame->height = detections->height = 80;
        frame->pixelFormat = PixelFormat::Gray8; frame->strideBytes = 100;
        frame->pixels = std::make_shared<const std::vector<std::byte>>(8000);
    }
};
MarkerObservation marker(int id)
{ return {MarkerFamily::AprilTag36h11, id, {{{10,10},{50,10},{50,50},{10,50}}}, 37.0}; }
TEST(VisionDebugFrameTest, KeepsExactImmutableFrameAndIndependentIdentities)
{
    Input input;
    input.detections->markers = {marker(2), marker(0)};
    const auto preview = makeVisionDebugFrame(input.frame, input.detections, RobotMarkerMap{});
    ASSERT_TRUE(preview);
    EXPECT_EQ(preview->frame.get(), input.frame.get());
    EXPECT_EQ(preview->detections.get(), input.detections.get());
    ASSERT_EQ(preview->markers.size(), 2U);
    EXPECT_EQ(preview->markers[0].marker.id, 0);
    EXPECT_EQ(preview->markers[0].robotId, RobotId{1});
    EXPECT_EQ(preview->markers[1].marker.id, 2);
    EXPECT_EQ(preview->markers[1].robotId, RobotId{3});
    EXPECT_DOUBLE_EQ(preview->markers[1].geometry->center.u, 30);
    EXPECT_DOUBLE_EQ(preview->markers[1].geometry->minimumEdgePixels, 40);
    EXPECT_EQ(preview->markers[1].marker.decisionMargin, 37.0);
}
TEST(VisionDebugFrameTest, RejectsMismatchedFrameSequenceTimestampAndDimensions)
{
    Input input;
    EXPECT_FALSE(makeVisionDebugFrame({}, input.detections, RobotMarkerMap{}));
    EXPECT_FALSE(makeVisionDebugFrame(input.frame, {}, RobotMarkerMap{}));
    ++input.detections->frameSequence;
    EXPECT_FALSE(makeVisionDebugFrame(input.frame, input.detections, RobotMarkerMap{}));
    --input.detections->frameSequence; input.detections->capturedAt += 1ms;
    EXPECT_FALSE(makeVisionDebugFrame(input.frame, input.detections, RobotMarkerMap{}));
    input.detections->capturedAt = input.frame->capturedAt; ++input.detections->width;
    EXPECT_FALSE(makeVisionDebugFrame(input.frame, input.detections, RobotMarkerMap{}));
    --input.detections->width; ++input.detections->height;
    EXPECT_FALSE(makeVisionDebugFrame(input.frame, input.detections, RobotMarkerMap{}));
}
TEST(VisionDebugFrameTest, RejectedTagsStayVisibleButNeverAcquireARobotId)
{
    Input input;
    auto invalid = marker(3); invalid.corners = {};
    input.detections->markers = {marker(1), marker(1), marker(16), invalid};
    const auto preview = makeVisionDebugFrame(input.frame, input.detections, RobotMarkerMap{});
    ASSERT_EQ(preview->markers.size(), 4U);
    for (const auto& overlay : preview->markers) {
        EXPECT_FALSE(overlay.robotId);
        EXPECT_TRUE(overlay.rejection);
    }
    EXPECT_EQ(preview->markers[0].rejection, MarkerRejection::DuplicateRobot);
    EXPECT_EQ(preview->markers[2].rejection, MarkerRejection::UnassignedId);
    EXPECT_FALSE(preview->markers[3].geometry);
}
TEST(VisionDebugFrameTest, DetectorErrorRetainsMatchingImageWithoutPartialOverlays)
{
    Input input;
    input.detections->markers = {marker(0)};
    input.detections->error = "failed";
    const auto preview = makeVisionDebugFrame(input.frame, input.detections, RobotMarkerMap{});
    ASSERT_TRUE(preview);
    EXPECT_TRUE(preview->markers.empty());
    EXPECT_EQ(preview->detections->error, "failed");
}
TEST(VisionOutputRateTest, MeasuresCompletionCadenceAndResetsWithoutCameraFpsClaims)
{
    VisionOutputRate rate;
    EXPECT_FALSE(rate.record(Timestamp{1s}));
    ASSERT_TRUE(rate.record(Timestamp{1100ms}));
    EXPECT_NEAR(*rate.record(Timestamp{1200ms}), 10, 1e-10);
    rate.reset();
    EXPECT_FALSE(rate.record(Timestamp{5s}));
    EXPECT_FALSE(rate.record(Timestamp{5s}));
    EXPECT_FALSE(rate.record(Timestamp{4s}));
    EXPECT_NEAR(*rate.record(Timestamp{4500ms}), 2, 1e-10);
}
TEST(VisionOutputRateTest, RollingWindowEventuallyDropsAnOldSlowInterval)
{
    VisionOutputRate rate;
    (void)rate.record(Timestamp{});
    (void)rate.record(Timestamp{100s});
    std::optional<double> fps;
    for (int i = 1; i <= 20; ++i) fps = rate.record(Timestamp{100s + i * 10ms});
    ASSERT_TRUE(fps);
    EXPECT_NEAR(*fps, 100, 1e-8);
}
} // namespace
