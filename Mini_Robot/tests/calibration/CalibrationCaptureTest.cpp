#include "calibration/CalibrationCapture.hpp"
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
