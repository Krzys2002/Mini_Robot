#include "calibration/CalibrationCapture.hpp"
#include "calibration/IntrinsicCalibration.hpp"
#include "CaptureFixtures.hpp"
#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace {
using namespace micro_robot;
using namespace micro_robot::test;

TEST(CalibrationCaptureTest, TargetUsesInnerCornersAndMetricObjectPoints)
{
    const auto points = calibrationObjectPoints({});
    ASSERT_EQ(points.size(), 54U);
    EXPECT_DOUBLE_EQ(points.front().x, 0);
    EXPECT_DOUBLE_EQ(points.back().x, 0.2);
    EXPECT_DOUBLE_EQ(points.back().y, 0.125);
    EXPECT_DOUBLE_EQ(points.back().z, 0);
    EXPECT_THROW((void)calibrationObjectPoints({6, 6, 0.025}), std::invalid_argument);
    EXPECT_THROW((void)calibrationObjectPoints({9, 6, -1}), std::invalid_argument);
    EXPECT_THROW((void)calibrationObjectPoints({1000000, 6, 1}), std::invalid_argument);
}
TEST(CalibrationCaptureTest, ConfigurationRejectsInvalidOrUnboundedThresholds)
{
    CalibrationCaptureConfig config;
    config.maximumViews = 0;
    EXPECT_THROW(CalibrationCapture{config}, std::invalid_argument);
    config = {};
    config.minimumSharpness = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(CalibrationCapture{config}, std::invalid_argument);
    config = {};
    config.minimumLocationCells = 10;
    EXPECT_THROW(CalibrationCapture{config}, std::invalid_argument);
}
TEST(CalibrationCaptureTest, RejectsBadObservationsWithSpecificFeedback)
{
    CalibrationCapture session;
    EXPECT_FALSE(session.add(captureFrame(), captureDetection()).accepted);
    session.restart("camera-a");
    EXPECT_FALSE(session.add({}, {}).accepted);
    auto noBoard = session.add(captureFrame(), {});
    EXPECT_NE(noBoard.message.find("not found"), std::string::npos);
    auto blurry = captureDetection();
    blurry.sharpness = 0;
    EXPECT_NE(session.add(captureFrame(), blurry).message.find("blurry"), std::string::npos);
    EXPECT_NE(session.add(captureFrame(), captureDetection(30, 30)).message.find("clipped"), std::string::npos);
    EXPECT_NE(session.add(captureFrame(), captureDetection(500, 400, 5)).message.find("small"), std::string::npos);
    auto invalid = captureDetection();
    invalid.corners[0].u = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(session.add(captureFrame(), invalid).accepted);
    invalid = captureDetection();
    invalid.error = "Detector failed";
    EXPECT_EQ(session.add(captureFrame(), invalid).message, "Detector failed");
    EXPECT_TRUE(session.views().empty());
}
TEST(CalibrationCaptureTest, RejectsRepeatedReversedAndNearlyIdenticalViews)
{
    CalibrationCapture session;
    session.restart("camera-a");
    ASSERT_TRUE(session.add(captureFrame(), captureDetection()).accepted);
    EXPECT_NE(session.add(captureFrame(), captureDetection(600)).message.find("repeated"), std::string::npos);
    EXPECT_NE(session.add(captureFrame(2), captureDetection(503)).message.find("Redundant"), std::string::npos);
    auto reversed = captureDetection();
    std::reverse(reversed.corners.begin(), reversed.corners.end());
    EXPECT_FALSE(session.add(captureFrame(2), reversed).accepted);
    EXPECT_TRUE(session.add(captureFrame(2), captureDetection(700)).accepted);
    EXPECT_FALSE(session.add(captureFrame(3, 900), captureDetection()).accepted);
}
TEST(CalibrationCaptureTest, CompletionRequiresCountAndPositionScaleRotationDiversity)
{
    CalibrationCaptureConfig config;
    config.minimumViews = 3;
    CalibrationCapture session{config};
    session.restart("camera-a");
    ASSERT_TRUE(session.add(captureFrame(1), captureDetection(250)).accepted);
    ASSERT_TRUE(session.add(captureFrame(2), captureDetection(500)).accepted);
    ASSERT_TRUE(session.add(captureFrame(3), captureDetection(750)).accepted);
    auto progress = session.progress();
    EXPECT_EQ(progress.locationCells, 3U);
    EXPECT_DOUBLE_EQ(progress.scaleSpread, 0);
    EXPECT_FALSE(progress.ready);
    EXPECT_FALSE(session.finish());
    ASSERT_TRUE(session.add(captureFrame(4), captureDetection(500, 400, 55, 0.3)).accepted);
    progress = session.progress();
    EXPECT_GE(progress.scaleSpread, config.minimumScaleSpread);
    EXPECT_GE(progress.angleSpreadRadians, config.minimumAngleSpreadRadians);
    EXPECT_TRUE(progress.ready);
    EXPECT_TRUE(session.finish());
    EXPECT_EQ(session.state(), CalibrationCaptureState::Complete);
    EXPECT_FALSE(session.add(captureFrame(5), captureDetection()).accepted);
    EXPECT_FALSE(session.remove(99));
    EXPECT_EQ(session.state(), CalibrationCaptureState::Complete);
    EXPECT_TRUE(session.remove(0));
    EXPECT_EQ(session.state(), CalibrationCaptureState::Collecting);
    EXPECT_EQ(session.views().size(), 3U);
    EXPECT_EQ(session.views().front().frame->sequence, 2U);
    EXPECT_EQ(session.sourceId(), "camera-a");
}
TEST(CalibrationCaptureTest, RemoveRecaptureRestartAndCancelAreExplicitTransitions)
{
    CalibrationCapture session;
    session.restart("camera-a");
    ASSERT_TRUE(session.add(captureFrame(), captureDetection()).accepted);
    EXPECT_FALSE(session.remove(2));
    EXPECT_TRUE(session.remove(0));
    EXPECT_EQ(session.progress().count, 0U);
    EXPECT_TRUE(session.add(captureFrame(), captureDetection()).accepted);
    session.restart("camera-b");
    EXPECT_TRUE(session.views().empty());
    EXPECT_EQ(session.sourceId(), "camera-b");
    session.cancel();
    EXPECT_EQ(session.state(), CalibrationCaptureState::Cancelled);
    EXPECT_TRUE(session.sourceId().empty());
    session.restart("");
    EXPECT_EQ(session.state(), CalibrationCaptureState::Idle);
}
TEST(CalibrationCaptureTest, StorageBudgetAndViewLimitAreBoundedAndReleasedOnRemoval)
{
    CalibrationCaptureConfig config;
    config.maximumStoredBytes = 800000;
    CalibrationCapture session{config};
    session.restart("camera");
    EXPECT_TRUE(session.add(captureFrame(1), captureDetection()).accepted);
    EXPECT_NE(session.add(captureFrame(2), captureDetection(700)).message.find("memory"), std::string::npos);
    EXPECT_TRUE(session.remove(0));
    EXPECT_TRUE(session.add(captureFrame(2), captureDetection(700)).accepted);
    config.maximumStoredBytes = 10000000;
    config.minimumViews = config.maximumViews = 3;
    CalibrationCapture limited{config};
    limited.restart("camera");
    ASSERT_TRUE(limited.add(captureFrame(1), captureDetection(250)).accepted);
    ASSERT_TRUE(limited.add(captureFrame(2), captureDetection(500)).accepted);
    ASSERT_TRUE(limited.add(captureFrame(3), captureDetection(750)).accepted);
    EXPECT_NE(limited.add(captureFrame(4), captureDetection(500, 400, 50)).message.find("limit"), std::string::npos);
}
} // namespace

namespace {
TEST(CalibrationGuidanceTest, StartAndFirstViewTrackSessionLifecycle)
{
    CalibrationCapture session;
    EXPECT_EQ(session.guidance().suggestion, CalibrationSuggestion::Start);
    session.restart("camera");
    EXPECT_EQ(session.guidance().suggestion, CalibrationSuggestion::FirstView);
    EXPECT_FALSE(session.add(captureFrame(), {}).accepted);
    EXPECT_EQ(session.guidance().suggestion, CalibrationSuggestion::FirstView);
    ASSERT_TRUE(session.add(captureFrame(), captureDetection()).accepted);
    session.cancel();
    EXPECT_EQ(session.guidance().suggestion, CalibrationSuggestion::Start);
    EXPECT_EQ(session.progress().locationCells, 0u);
    session.restart("other");
    EXPECT_EQ(session.guidance().suggestion, CalibrationSuggestion::FirstView);
}
TEST(CalibrationGuidanceTest, SuggestsAnUnoccupiedBoardCentreCellFromAcceptedViewsOnly)
{
    CalibrationCapture session;
    session.restart("camera");
    ASSERT_TRUE(session.add(captureFrame(1), captureDetection()).accepted);
    auto progress = session.progress();
    EXPECT_TRUE(progress.occupiedLocationCells[4]);
    EXPECT_EQ(session.guidance().suggestion, CalibrationSuggestion::ChangeLocation);
    ASSERT_TRUE(session.guidance().nextLocationCell);
    EXPECT_FALSE(progress.occupiedLocationCells[*session.guidance().nextLocationCell]);
    auto blurry = captureDetection(200, 150, 20);
    blurry.sharpness = 0;
    EXPECT_FALSE(session.add(captureFrame(2), blurry).accepted);
    EXPECT_FALSE(session.progress().occupiedLocationCells[0]);
    ASSERT_TRUE(session.add(captureFrame(3), captureDetection(200, 150, 20)).accepted);
    EXPECT_TRUE(session.progress().occupiedLocationCells[0]);
    EXPECT_NE(session.guidance().nextLocationCell, 0u);
    ASSERT_TRUE(session.remove(1));
    EXPECT_EQ(session.guidance().nextLocationCell, 0u);
    EXPECT_FALSE(session.progress().occupiedLocationCells[0]);
}
TEST(CalibrationGuidanceTest, ScaleThenInPlaneAngleThenReadyReflectMeasuredGaps)
{
    CalibrationCaptureConfig config;
    config.minimumViews = 3;
    CalibrationCapture session{config};
    session.restart("camera");
    ASSERT_TRUE(session.add(captureFrame(1), captureDetection(250)).accepted);
    ASSERT_TRUE(session.add(captureFrame(2), captureDetection(500)).accepted);
    ASSERT_TRUE(session.add(captureFrame(3), captureDetection(750)).accepted);
    EXPECT_EQ(session.guidance().suggestion, CalibrationSuggestion::ChangeScale);
    EXPECT_FALSE(session.guidance().nextLocationCell);
    ASSERT_TRUE(session.add(captureFrame(4), captureDetection(500, 400, 55)).accepted);
    EXPECT_EQ(session.guidance().suggestion, CalibrationSuggestion::RotateInPlane);
    ASSERT_TRUE(session.add(captureFrame(5), captureDetection(500, 400, 55, 0.4)).accepted);
    EXPECT_EQ(session.guidance().suggestion, CalibrationSuggestion::Ready);
    ASSERT_TRUE(session.finish());
    EXPECT_EQ(session.guidance().suggestion, CalibrationSuggestion::ReviewResult);
    ASSERT_TRUE(session.remove(4));
    EXPECT_EQ(session.guidance().suggestion, CalibrationSuggestion::RotateInPlane);
}
TEST(CalibrationGuidanceTest, DisabledDiversityGatesStillRequireCount)
{
    CalibrationCaptureConfig config;
    config.minimumLocationCells = 1;
    config.minimumScaleSpread = config.minimumAngleSpreadRadians = 0;
    CalibrationCapture session{config};
    session.restart("camera");
    ASSERT_TRUE(session.add(captureFrame(), captureDetection()).accepted);
    EXPECT_EQ(session.guidance().suggestion, CalibrationSuggestion::MoreViews);
    EXPECT_FALSE(session.guidance().nextLocationCell);
}
TEST(CalibrationGuidanceTest, FullDatasetWithMissingDiversitySuggestsReplacement)
{
    CalibrationCaptureConfig config;
    config.minimumViews = config.maximumViews = 3;
    config.minimumLocationCells = 1;
    CalibrationCapture session{config};
    session.restart("camera");
    ASSERT_TRUE(session.add(captureFrame(1), captureDetection(250)).accepted);
    ASSERT_TRUE(session.add(captureFrame(2), captureDetection(500)).accepted);
    ASSERT_TRUE(session.add(captureFrame(3), captureDetection(750)).accepted);
    EXPECT_EQ(session.guidance().suggestion, CalibrationSuggestion::ReplaceView);
    EXPECT_FALSE(session.progress().ready);
    ASSERT_TRUE(session.remove(0));
    EXPECT_EQ(session.guidance().suggestion, CalibrationSuggestion::ChangeScale);
}
} // namespace

namespace {
FramePtr storageFrame(std::uint64_t sequence, std::uint32_t width, std::uint32_t height,
                      PixelFormat format, std::size_t padding = 0)
{
    const auto channels = format == PixelFormat::Gray8 ? 1u : format == PixelFormat::Rgb8 ? 3u : 4u;
    auto f = std::make_shared<Frame>(*captureFrame(sequence, width, height));
    f->pixelFormat = format;
    f->strideBytes = static_cast<std::size_t>(width) * channels + padding;
    f->pixels = std::make_shared<const std::vector<std::byte>>(f->strideBytes * height, std::byte{127});
    return f;
}
TEST(CalibrationStorageTest, CompactsPaddedColourAndUnusedCapacityWithoutChangingGeometry)
{
    for (auto format : {PixelFormat::Gray8, PixelFormat::Rgb8, PixelFormat::Rgba8}) {
        auto input = std::make_shared<Frame>(*storageFrame(1, 6, 4, format, 32));
        auto pixels = std::make_shared<std::vector<std::byte>>(*input->pixels);
        pixels->reserve(pixels->size() + 8192);
        input->pixels = pixels;
        std::weak_ptr<const std::vector<std::byte>> original = pixels;
        const auto output = prepareCalibrationReviewFrame(input);
        EXPECT_EQ(output->width, input->width);
        EXPECT_EQ(output->height, input->height);
        EXPECT_EQ(output->sequence, input->sequence);
        EXPECT_EQ(output->capturedAt, input->capturedAt);
        EXPECT_EQ(output->pixelFormat, PixelFormat::Gray8);
        EXPECT_EQ(output->strideBytes, 6u);
        EXPECT_EQ(output->pixels->size(), 24u);
        EXPECT_EQ(output->pixels->capacity(), 24u);
        EXPECT_TRUE(std::all_of(output->pixels->begin(), output->pixels->end(),
            [](auto p) { return p == std::byte{127}; }));
        pixels.reset(); input.reset();
        EXPECT_TRUE(original.expired());
    }
    const auto gray = storageFrame(1, 6, 4, PixelFormat::Gray8);
    EXPECT_EQ(prepareCalibrationReviewFrame(gray), gray); // Already compact: no copy.
}
TEST(CalibrationStorageTest, BudgetBoundaryUsesPackedBytesAndPreflightRetainsNothing)
{
    CalibrationCaptureConfig config;
    config.maximumStoredBytes = 800000;
    CalibrationCapture exact(config);
    exact.restart("camera");
    auto padded = storageFrame(1, 1000, 800, PixelFormat::Rgba8, 4096);
    std::weak_ptr<const Frame> raw = padded;
    EXPECT_TRUE(exact.assess(padded, captureDetection()).accepted);
    EXPECT_TRUE(exact.views().empty());
    EXPECT_EQ(exact.storedPixelBytes(), 0u);
    EXPECT_TRUE(exact.add(padded, captureDetection()).accepted);
    padded.reset();
    EXPECT_TRUE(raw.expired());
    EXPECT_EQ(exact.storedPixelBytes(), 800000u);
    EXPECT_EQ(exact.remainingPixelBytes(), 0u);
    EXPECT_FALSE(exact.add(captureFrame(2), captureDetection(700)).accepted);
    ASSERT_TRUE(exact.remove(0));
    EXPECT_EQ(exact.storedPixelBytes(), 0u);
    config.maximumStoredBytes = 799999;
    CalibrationCapture tooSmall(config); tooSmall.restart("camera");
    bool prepared = false;
    auto rejected = tooSmall.add(captureFrame(), captureDetection(), [&](FramePtr frame) {
        prepared = true; return frame;
    });
    EXPECT_FALSE(prepared);
    EXPECT_FALSE(rejected.accepted);
    EXPECT_NE(rejected.message.find("799999"), std::string::npos);
    EXPECT_EQ(tooSmall.storedPixelBytes(), 0u);
}
TEST(CalibrationStorageTest, AllTwentyFourFourMegapixelViewsFitTheUnchangedDefaultBudget)
{
    CalibrationCaptureConfig config; config.duplicateDistance = 0.00001;
    CalibrationCapture session(config); session.restart("camera");
    const auto source = storageFrame(1, 2000, 2000, PixelFormat::Rgba8);
    for (std::uint64_t n = 1; n <= CalibrationMaximumViews; ++n) {
        auto frame = std::make_shared<Frame>(*source);
        frame->sequence = n; frame->capturedAt = Timestamp{std::chrono::seconds{n}};
        ASSERT_TRUE(session.add(frame, captureDetection(1000 + n * 2, 1000, 80)).accepted) << n;
    }
    EXPECT_EQ(session.views().size(), CalibrationMaximumViews);
    EXPECT_EQ(session.storedPixelBytes(), CalibrationMaximumViews * CalibrationMaximumImagePixels);
    EXPECT_LE(session.storedPixelBytes(), config.maximumStoredBytes);
    std::weak_ptr<const Frame> review = session.views().front().frame;
    session.cancel();
    EXPECT_EQ(session.storedPixelBytes(), 0u);
    EXPECT_TRUE(review.expired());
}
TEST(CalibrationStorageTest, AllocationAndLengthFailuresPreserveExistingViewsAndAccounting)
{
    CalibrationCapture session; session.restart("camera");
    ASSERT_TRUE(session.add(captureFrame(), captureDetection()).accepted);
    const auto kept = session.views().front().frame;
    const auto bytes = session.storedPixelBytes();
    const auto fail = session.add(captureFrame(2), captureDetection(700), [](FramePtr) -> FramePtr {
        throw std::bad_alloc{};
    });
    EXPECT_FALSE(fail.accepted);
    EXPECT_NE(fail.message.find("allocation failed"), std::string::npos);
    EXPECT_FALSE(session.add(captureFrame(2), captureDetection(700), [](FramePtr) -> FramePtr {
        throw std::length_error("Synthetic allocation size failure");
    }).accepted);
    EXPECT_EQ(session.views().size(), 1u);
    EXPECT_EQ(session.views().front().frame, kept);
    EXPECT_EQ(session.storedPixelBytes(), bytes);
    EXPECT_TRUE(session.add(captureFrame(2), captureDetection(700)).accepted);
    EXPECT_EQ(session.storedPixelBytes(), bytes * 2);
}
TEST(CalibrationStorageTest, MalformedAndOverflowingMetadataNeverAllocatesOrCorruptsState)
{
    EXPECT_TRUE(supportedCalibrationResolution(2000, 2000));
    EXPECT_FALSE(supportedCalibrationResolution(2000, 2001));
    EXPECT_FALSE(supportedCalibrationResolution(4097, 1));
    auto malformed = std::make_shared<Frame>(*captureFrame());
    malformed->width = malformed->height = std::numeric_limits<std::uint32_t>::max();
    malformed->strideBytes = std::numeric_limits<std::size_t>::max();
    EXPECT_FALSE(isValidFrame(*malformed));
    EXPECT_THROW((void)prepareCalibrationReviewFrame(malformed), std::invalid_argument);
    CalibrationCapture session; session.restart("camera");
    EXPECT_FALSE(session.add(malformed, captureDetection()).accepted);
    EXPECT_EQ(session.storedPixelBytes(), 0u);
    CalibrationCaptureConfig unsupported;
    unsupported.maximumViews = CalibrationMaximumViews + 1;
    EXPECT_THROW(CalibrationCapture{unsupported}, std::invalid_argument);
}
TEST(CalibrationStorageTest, RepeatedRemoveRestartAndCancelReleasePixelsWithoutBudgetDrift)
{
    CalibrationCapture session; session.restart("camera");
    for (int i = 0; i < 16; ++i) {
        ASSERT_TRUE(session.add(captureFrame(), captureDetection()).accepted);
        std::weak_ptr<const Frame> frame = session.views().front().frame;
        ASSERT_TRUE(session.remove(0));
        EXPECT_TRUE(frame.expired());
        EXPECT_EQ(session.storedPixelBytes(), 0u);
    }
    ASSERT_TRUE(session.add(captureFrame(), captureDetection()).accepted);
    std::weak_ptr<const Frame> first = session.views().front().frame;
    session.restart("different");
    EXPECT_TRUE(first.expired());
    ASSERT_TRUE(session.add(captureFrame(), captureDetection()).accepted);
    std::weak_ptr<const Frame> second = session.views().front().frame;
    session.cancel();
    EXPECT_TRUE(second.expired());
    EXPECT_EQ(session.remainingPixelBytes(), session.config().maximumStoredBytes);
}
} // namespace

namespace {
TEST(CalibrationStorageTest, SolverObservationsKeepOriginalCoordinatesResolutionAndCameraBinding)
{
    CalibrationCaptureConfig config;
    config.minimumViews = 3; config.minimumLocationCells = 1;
    config.minimumScaleSpread = config.minimumAngleSpreadRadians = 0;
    CalibrationCapture session(config); session.restart("original-camera");
    for (std::uint64_t n = 1; n <= 3; ++n)
        ASSERT_TRUE(session.add(storageFrame(n, 1000, 800, PixelFormat::Rgba8, 2048),
            captureDetection(n * 250)).accepted);
    ASSERT_TRUE(session.finish());
    const auto input = intrinsicCalibrationInput(session);
    EXPECT_EQ(input.sourceId, "original-camera");
    EXPECT_EQ(input.width, 1000u); EXPECT_EQ(input.height, 800u);
    ASSERT_EQ(input.observations.size(), 3u);
    const auto expected = captureDetection(250);
    for (std::size_t i = 0; i < expected.corners.size(); ++i) {
        EXPECT_DOUBLE_EQ(input.observations[0].corners[i].u, expected.corners[i].u);
        EXPECT_DOUBLE_EQ(input.observations[0].corners[i].v, expected.corners[i].v);
    }
    session.cancel();
    EXPECT_EQ(input.observations[0].corners.size(), expected.corners.size());
}
} // namespace
