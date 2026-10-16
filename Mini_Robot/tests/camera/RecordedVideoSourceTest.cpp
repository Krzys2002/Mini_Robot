#include "camera/RecordedVideoSource.hpp"

#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include <filesystem>
#include <fstream>
#include <functional>
#include <stdexcept>
#include <vector>

namespace micro_robot {
namespace {
using namespace std::chrono_literals;

struct Events final : ICameraEvents {
    std::vector<FramePtr> frames;
    std::vector<CameraState> states;
    std::vector<CameraError> errors;
    std::function<void()> onStreaming;
    std::function<void()> afterFrame;
    void onFrame(FramePtr frame) override
    {
        frames.push_back(std::move(frame));
        if (afterFrame) { afterFrame(); }
    }
    void onCameraStateChanged(CameraState state, Timestamp) override
    {
        states.push_back(state);
        if (state == CameraState::Streaming && onStreaming) { onStreaming(); }
    }
    void onCameraError(const CameraError& error) override { errors.push_back(error); }
};

class RecordedVideoTest : public testing::Test {
protected:
    void SetUp() override
    {
        // Exclusive directory creation prevents collisions between CTest processes.
        const auto seed = MonotonicClock::now().time_since_epoch().count();
        for (int attempt = 0; attempt < 100; ++attempt) {
            directory = std::filesystem::temp_directory_path()
                / ("micro-robot-replay-" + std::to_string(seed) + "-" + std::to_string(attempt));
            if (std::filesystem::create_directory(directory)) { break; }
            directory.clear();
        }
        ASSERT_FALSE(directory.empty());
        file = directory / "fixture.avi";
        writeFixture();
    }
    void TearDown() override
    {
        std::error_code error;
        if (!directory.empty()) { std::filesystem::remove_all(directory, error); }
    }
    void writeFixture()
    {
        cv::VideoWriter writer(file.string(), cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                               25.0, cv::Size{32, 24});
        ASSERT_TRUE(writer.isOpened()) << "MJPEG writer required for replay integration tests";
        for (int index = 0; index < 3; ++index) {
            writer.write(cv::Mat(24, 32, CV_8UC3, cv::Scalar(20 + index * 10, 70, 180)));
        }
    }
    std::filesystem::path directory;
    std::filesystem::path file;
};

TEST_F(RecordedVideoTest, DeliversOwnedRgbFramesThroughCameraInterfaceThenStopsAtEof)
{
    auto events = std::make_shared<Events>();
    {
        RecordedVideoSource source({file});
        ICameraSource& camera = source;
        camera.setEventSink(events);
        EXPECT_EQ(camera.state(), CameraState::Stopped);
        EXPECT_FALSE(camera.info().sourceId.empty());
        EXPECT_EQ(camera.info().displayName, "fixture.avi");
        camera.start();
        camera.start();
        EXPECT_TRUE(events->states.empty());
        for (int tick = 0; tick < 4; ++tick) {
            source.advanceTo(Timestamp{} + tick * 40ms);
        }
        ASSERT_EQ(events->frames.size(), 3u);
        EXPECT_EQ(events->states, (std::vector{CameraState::Streaming, CameraState::Stopped}));
        EXPECT_EQ(camera.state(), CameraState::Stopped);
        EXPECT_TRUE(events->errors.empty());
        source.advanceTo(Timestamp{} + 1s);
        EXPECT_EQ(events->frames.size(), 3u);
    }
    for (std::size_t index = 0; index < events->frames.size(); ++index) {
        const auto& frame = *events->frames[index];
        EXPECT_TRUE(isValidFrame(frame));
        EXPECT_EQ(frame.sequence, index);
        EXPECT_EQ(frame.capturedAt, Timestamp{} + index * 40ms);
        EXPECT_EQ(frame.width, 32u);
        EXPECT_EQ(frame.height, 24u);
        EXPECT_EQ(frame.strideBytes, 96u);
        EXPECT_EQ(frame.pixelFormat, PixelFormat::Rgb8);
        // Codec tolerance; explicitly verify BGR -> RGB channel order.
        EXPECT_NEAR(std::to_integer<int>((*frame.pixels)[0]), 180, 5);
        EXPECT_NEAR(std::to_integer<int>((*frame.pixels)[1]), 70, 5);
        EXPECT_NEAR(std::to_integer<int>((*frame.pixels)[2]), 20 + index * 10, 5);
    }
}

TEST_F(RecordedVideoTest, IndependentRunsHaveIdenticalPixelsSequencesAndVirtualTimes)
{
    auto replay = [this] {
        auto events = std::make_shared<Events>();
        RecordedVideoSource source({file, 10ms});
        source.setEventSink(events);
        source.start();
        // Irregular driver timing must not change scheduled capture timestamps.
        source.advanceTo(Timestamp{});
        source.advanceTo(Timestamp{} + 5ms);
        source.advanceTo(Timestamp{} + 50ms);
        source.advanceTo(Timestamp{} + 50ms);
        source.advanceTo(Timestamp{} + 50ms);
        return events;
    };
    const auto first = replay();
    const auto second = replay();
    ASSERT_EQ(first->frames.size(), 3u);
    ASSERT_EQ(second->frames.size(), 3u);
    for (std::size_t index = 0; index < 3; ++index) {
        EXPECT_EQ(*first->frames[index]->pixels, *second->frames[index]->pixels);
        EXPECT_EQ(first->frames[index]->sequence, second->frames[index]->sequence);
        EXPECT_EQ(first->frames[index]->capturedAt, Timestamp{} + index * 10ms);
        EXPECT_EQ(first->frames[index]->capturedAt, second->frames[index]->capturedAt);
    }
}

TEST_F(RecordedVideoTest, StopCancelsPendingStartAndRestartRewindsWithIncreasingIdentity)
{
    RecordedVideoSource source({file});
    auto events = std::make_shared<Events>();
    source.setEventSink(events);
    source.start();
    source.stop();
    EXPECT_TRUE(events->states.empty()); // No inline cancellation callback.
    source.advanceTo(Timestamp{});
    EXPECT_EQ(events->states, std::vector{CameraState::Stopped});
    EXPECT_TRUE(events->frames.empty());
    source.stop();
    source.advanceTo(Timestamp{});
    EXPECT_EQ(events->states.size(), 1U); // Repeated stop adds no event.
    source.start();
    source.advanceTo(Timestamp{});
    ASSERT_EQ(events->frames.size(), 1u);
    source.stop();
    EXPECT_EQ(source.state(), CameraState::Streaming);
    source.advanceTo(Timestamp{});
    EXPECT_EQ(source.state(), CameraState::Stopped);
    source.stop();
    source.advanceTo(Timestamp{});
    source.start();
    source.advanceTo(Timestamp{}); // restart cadence cannot go backward
    EXPECT_EQ(events->frames.size(), 1u);
    source.advanceTo(Timestamp{} + 40ms);
    ASSERT_EQ(events->frames.size(), 2u);
    EXPECT_EQ(*events->frames[0]->pixels, *events->frames[1]->pixels);
    EXPECT_EQ(events->frames[1]->sequence, 1u);
    EXPECT_EQ(events->frames[1]->capturedAt, Timestamp{} + 40ms);
}

TEST_F(RecordedVideoTest, EofRestartAndLoopRewindWithoutResettingTimeline)
{
    RecordedVideoSource looping({file, 40ms, true});
    auto events = std::make_shared<Events>();
    looping.setEventSink(events);
    looping.start();
    for (int tick = 0; tick < 7; ++tick) { looping.advanceTo(Timestamp{} + tick * 40ms); }
    ASSERT_EQ(events->frames.size(), 7u);
    EXPECT_EQ(events->states, (std::vector{CameraState::Streaming}));
    for (std::size_t index = 0; index < 7; ++index) {
        EXPECT_EQ(*events->frames[index]->pixels, *events->frames[index % 3]->pixels);
        EXPECT_EQ(events->frames[index]->sequence, index);
        EXPECT_EQ(events->frames[index]->capturedAt, Timestamp{} + index * 40ms);
    }
    looping.stop();
    looping.advanceTo(Timestamp{} + 300ms);
    EXPECT_EQ(looping.state(), CameraState::Stopped);

    RecordedVideoSource single({file});
    events = std::make_shared<Events>();
    single.setEventSink(events);
    single.start();
    for (int tick = 0; tick < 4; ++tick) { single.advanceTo(Timestamp{} + tick * 40ms); }
    single.start();
    single.advanceTo(Timestamp{} + 160ms);
    ASSERT_EQ(events->frames.size(), 4u);
    EXPECT_EQ(*events->frames[0]->pixels, *events->frames[3]->pixels);
    EXPECT_EQ(events->frames[3]->sequence, 3u);
    EXPECT_EQ(events->frames[3]->capturedAt, Timestamp{} + 160ms);
}

TEST_F(RecordedVideoTest, CallbackStopPreventsFirstFrameAndDoesNotNestEvents)
{
    RecordedVideoSource source({file});
    auto events = std::make_shared<Events>();
    events->onStreaming = [&] {
        EXPECT_EQ(source.state(), CameraState::Streaming);
        source.stop();
        EXPECT_EQ(events->states.size(), 1u);
    };
    source.setEventSink(events);
    source.start();
    source.advanceTo(Timestamp{});
    EXPECT_TRUE(events->frames.empty());
    EXPECT_EQ(source.state(), CameraState::Stopped);
    EXPECT_EQ(events->states, (std::vector{CameraState::Streaming, CameraState::Stopped}));
}

TEST_F(RecordedVideoTest, FrameCallbackStopAndSinkReplacementTakeEffectNextDispatch)
{
    RecordedVideoSource source({file});
    auto first = std::make_shared<Events>();
    auto second = std::make_shared<Events>();
    first->afterFrame = [&] { source.setEventSink(second); source.stop(); };
    source.setEventSink(first);
    source.start();
    source.advanceTo(Timestamp{});
    source.advanceTo(Timestamp{} + 40ms);
    EXPECT_EQ(first->frames.size(), 1u);
    EXPECT_TRUE(second->frames.empty());
    EXPECT_EQ(second->states, (std::vector{CameraState::Stopped}));
    source.start();
    source.advanceTo(Timestamp{} + 80ms);
    EXPECT_EQ(second->frames.size(), 1u);
}

TEST_F(RecordedVideoTest, WeakSinkExpiresAndDetachedSourceStillReachesEof)
{
    RecordedVideoSource source({file});
    auto events = std::make_shared<Events>();
    std::weak_ptr<Events> weak = events;
    source.setEventSink(events);
    events.reset();
    EXPECT_TRUE(weak.expired());
    source.start();
    for (int tick = 0; tick < 4; ++tick) { source.advanceTo(Timestamp{} + tick * 40ms); }
    EXPECT_EQ(source.state(), CameraState::Stopped);
}

TEST_F(RecordedVideoTest, MissingFileFailsAsynchronouslyAndCanRecover)
{
    const auto selected = directory / "selected.avi";
    RecordedVideoSource source({selected});
    const auto identity = source.info().sourceId;
    auto events = std::make_shared<Events>();
    source.setEventSink(events);
    source.start();
    EXPECT_TRUE(events->errors.empty());
    source.advanceTo(Timestamp{});
    EXPECT_EQ(source.state(), CameraState::Fault);
    ASSERT_EQ(events->errors.size(), 1u);
    EXPECT_EQ(events->errors[0].code, CameraErrorCode::Unavailable);
    source.advanceTo(Timestamp{} + 40ms);
    EXPECT_EQ(events->errors.size(), 1u); // no implicit retry
    std::filesystem::copy_file(file, selected);
    source.start();
    source.advanceTo(Timestamp{} + 80ms);
    EXPECT_EQ(source.info().sourceId, identity);
    EXPECT_EQ(source.state(), CameraState::Streaming);
    ASSERT_EQ(events->frames.size(), 1u);
}

TEST_F(RecordedVideoTest, InvalidLocalVideoFailsWithoutPublishingFrames)
{
    const auto invalid = directory / "invalid.avi";
    { std::ofstream output(invalid); output << "not a video"; }
    RecordedVideoSource source({invalid, 40ms, true});
    auto events = std::make_shared<Events>();
    source.setEventSink(events);
    source.start();
    source.advanceTo(Timestamp{});
    EXPECT_EQ(source.state(), CameraState::Fault);
    EXPECT_TRUE(events->frames.empty());
    ASSERT_EQ(events->errors.size(), 1u);
    EXPECT_EQ(events->errors[0].code, CameraErrorCode::BackendFailure);
}

TEST_F(RecordedVideoTest, RejectsInvalidClockAndRecursiveAdvancement)
{
    EXPECT_THROW(RecordedVideoSource(RecordedVideoConfig{}), std::invalid_argument);
    EXPECT_THROW(RecordedVideoSource((RecordedVideoConfig{file, 0ms})), std::invalid_argument);
    RecordedVideoSource source({file});
    auto events = std::make_shared<Events>();
    events->afterFrame = [&] { EXPECT_THROW(source.advanceTo(Timestamp{}), std::logic_error); };
    source.setEventSink(events);
    source.start();
    source.advanceTo(Timestamp{} + 1s);
    EXPECT_THROW(source.advanceTo(Timestamp{}), std::invalid_argument);
    source.stop();
    source.advanceTo(Timestamp{} + 1s);
    EXPECT_EQ(source.state(), CameraState::Stopped);
}

TEST_F(RecordedVideoTest, TimestampOverflowFailsWithoutPublishing)
{
    RecordedVideoSource source({file});
    auto events = std::make_shared<Events>();
    source.setEventSink(events);
    source.start();
    source.advanceTo(Timestamp::max());
    EXPECT_EQ(source.state(), CameraState::Fault);
    EXPECT_TRUE(events->frames.empty());
    ASSERT_EQ(events->errors.size(), 1u);
}

} // namespace
} // namespace micro_robot
