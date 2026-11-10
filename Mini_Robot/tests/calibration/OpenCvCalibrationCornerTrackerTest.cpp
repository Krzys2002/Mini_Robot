#include "calibration/OpenCvCalibrationCornerTracker.hpp"
#include "calibration/OpenCvCalibrationDetector.hpp"
#include <opencv2/imgproc.hpp>
#include <gtest/gtest.h>
#include <cstring>
#include <limits>

namespace {
using namespace micro_robot;
using namespace std::chrono_literals;
cv::Mat board()
{
    cv::Mat image(480, 640, CV_8UC1, cv::Scalar{255});
    for (int y=0; y<7; ++y) for (int x=0; x<10; ++x)
        if ((x+y)%2 == 0) cv::rectangle(image, {100+x*40,100+y*40,40,40}, cv::Scalar{0}, cv::FILLED);
    return image;
}
Frame frame(const cv::Mat& gray, std::uint64_t sequence, PixelFormat format = PixelFormat::Gray8)
{
    cv::Mat image;
    if (format == PixelFormat::Rgb8) cv::cvtColor(gray, image, cv::COLOR_GRAY2RGB);
    else if (format == PixelFormat::Rgba8) cv::cvtColor(gray, image, cv::COLOR_GRAY2RGBA);
    else image = gray;
    Frame f{sequence, Timestamp{} + std::chrono::milliseconds{static_cast<int>(sequence)*33},
        640, 480, image.cols*image.elemSize()+8, format, {}};
    auto pixels=std::make_shared<std::vector<std::byte>>(f.strideBytes*f.height);
    for (int y=0; y<image.rows; ++y) std::memcpy(pixels->data()+y*f.strideBytes, image.ptr(y), image.cols*image.elemSize());
    f.pixels=pixels;
    return f;
}
TEST(CalibrationCornerTrackerTest, TracksTranslationInAllFormatsWithPaddedImmutableFrames)
{
    OpenCvCalibrationDetector detector;
    OpenCvCalibrationCornerTracker tracker;
    cv::Mat moved;
    cv::warpAffine(board(), moved, cv::Matx23d{1,0,5,0,1,3}, {640,480}, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar{255});
    for (auto format : {PixelFormat::Gray8, PixelFormat::Rgb8, PixelFormat::Rgba8}) {
        const auto from=frame(board(),1,format), to=frame(moved,2,format);
        const auto original=*to.pixels;
        const auto seed=detector.detect(from,{});
        ASSERT_EQ(seed.corners.size(),54U);
        const auto result=tracker.track(from,to,seed.corners);
        ASSERT_EQ(result.size(),54U);
        for (std::size_t i=0; i<result.size(); ++i) {
            EXPECT_NEAR(result[i].u, seed.corners[i].u+5, .1);
            EXPECT_NEAR(result[i].v, seed.corners[i].v+3, .1);
        }
        EXPECT_EQ(*to.pixels, original);
    }
}
TEST(CalibrationCornerTrackerTest, OcclusionAndAmbiguousLargeMovementHideTheWholeGrid)
{
    OpenCvCalibrationDetector detector;
    OpenCvCalibrationCornerTracker tracker;
    const auto from=frame(board(),1);
    const auto corners=detector.detect(from,{}).corners;
    cv::Mat moved;
    cv::warpAffine(board(), moved, cv::Matx23d{1,0,20,0,1,0}, {640,480}, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar{255});
    EXPECT_TRUE(tracker.track(from,frame(moved,2),corners).empty());
    auto occluded=board();
    cv::rectangle(occluded,{200,150,100,100},cv::Scalar{127},cv::FILLED);
    EXPECT_TRUE(tracker.track(from,frame(occluded,2),corners).empty());
    EXPECT_TRUE(tracker.track(from,frame(cv::Mat(480,640,CV_8UC1,cv::Scalar{127}),2),corners).empty());
}
TEST(CalibrationCornerTrackerTest, RejectsBadOrderingExpiredStepsInvalidPointsAndInput)
{
    OpenCvCalibrationDetector detector;
    OpenCvCalibrationCornerTracker tracker;
    const auto from=frame(board(),1);
    auto to=frame(board(),2);
    auto points=detector.detect(from,{}).corners;
    EXPECT_TRUE(tracker.track(from,from,points).empty());
    to.capturedAt=from.capturedAt+151ms;
    EXPECT_TRUE(tracker.track(from,to,points).empty());
    to=frame(board(),2); to.width=1;
    EXPECT_TRUE(tracker.track(from,to,points).empty());
    to=frame(board(),2); points[0].u=std::numeric_limits<double>::quiet_NaN();
    EXPECT_TRUE(tracker.track(from,to,points).empty());
    EXPECT_TRUE(tracker.track(from,to,{}).empty());
    points=detector.detect(from,{}).corners; points[1]=points[0];
    EXPECT_TRUE(tracker.track(from,to,points).empty());
}
TEST(CalibrationCornerTrackerTest, RejectsInvalidPolicy)
{
    CalibrationCornerTrackingConfig config;
    config.maximumSpacingFraction=.5;
    EXPECT_THROW(OpenCvCalibrationCornerTracker{config}, std::invalid_argument);
    config={}; config.maximumRoundTripPixels=std::numeric_limits<double>::infinity();
    EXPECT_THROW(OpenCvCalibrationCornerTracker{config}, std::invalid_argument);
}
} // namespace
