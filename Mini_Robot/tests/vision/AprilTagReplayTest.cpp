#include "vision/OpenCvAprilTagDetector.hpp"
#include "camera/RecordedVideoSource.hpp"
#include "MarkerFixtures.hpp"
#include <opencv2/videoio.hpp>
#include <gtest/gtest.h>
#include <filesystem>

namespace {
using namespace micro_robot;
using namespace micro_robot::test;
struct RecordedTags {
    std::filesystem::path path{std::filesystem::temp_directory_path()
        / ("mini-robot-tags-" + std::to_string(MonotonicClock::now().time_since_epoch().count()) + ".avi")};
    ~RecordedTags() { std::error_code ignored; std::filesystem::remove(path, ignored); }
};
class DetectingSink final : public ICameraEvents {
public:
    void onFrame(FramePtr frame) override { results.push_back(detector.detect(*frame)); }
    void onCameraStateChanged(CameraState, Timestamp) override {}
    void onCameraError(const CameraError& error) override { errors.push_back(error); }
    OpenCvAprilTagDetector detector;
    std::vector<MarkerDetections> results;
    std::vector<CameraError> errors;
};
TEST(AprilTagReplayTest, DecodesRecordedFramesThroughTheSameDetectorContract)
{
    RecordedTags file;
    cv::VideoWriter writer{file.path.string(), cv::VideoWriter::fourcc('M','J','P','G'), 25, {400,400}};
    ASSERT_TRUE(writer.isOpened()) << "MJPEG encoder required for deterministic replay verification";
    for (const int id : {0, 15}) {
        cv::Mat bgr;
        cv::cvtColor(tagScene(id), bgr, cv::COLOR_GRAY2BGR);
        writer.write(bgr);
    }
    writer.release();
    auto sink = std::make_shared<DetectingSink>();
    RecordedVideoSource source{{file.path, std::chrono::milliseconds{40}, false}};
    source.setEventSink(sink);
    const auto start = Timestamp{} + std::chrono::seconds{1};
    source.start();
    source.advanceTo(start);
    source.advanceTo(start + std::chrono::milliseconds{40});
    ASSERT_TRUE(sink->errors.empty());
    ASSERT_EQ(sink->results.size(), 2U);
    for (std::size_t i = 0; i < 2; ++i) {
        const auto& result = sink->results[i];
        ASSERT_TRUE(result.error.empty()) << result.error;
        ASSERT_EQ(result.markers.size(), 1U);
        EXPECT_EQ(result.markers[0].id, i == 0 ? 0 : 15);
        EXPECT_EQ(result.capturedAt, start + std::chrono::milliseconds{40 * i});
    }
    EXPECT_GT(sink->results[1].frameSequence, sink->results[0].frameSequence);
    source.stop();
    source.advanceTo(start + std::chrono::milliseconds{80});
}
} // namespace
