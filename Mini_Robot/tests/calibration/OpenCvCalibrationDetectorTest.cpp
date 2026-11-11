#include "calibration/OpenCvCalibrationDetector.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR >= 5
#include <opencv2/geometry.hpp>
#endif
#include <gtest/gtest.h>

#include <algorithm>
#include <cstring>
#include <array>
#include <chrono>
#include <cmath>

namespace {
using namespace micro_robot;

cv::Mat chessboard()
{
    cv::Mat image(800, 1000, CV_8UC1, cv::Scalar{255});
    for (int row = 0; row < 7; ++row) {
        for (int col = 0; col < 10; ++col) {
            if ((row + col) % 2 == 0) cv::rectangle(image, {200 + col * 50, 225 + row * 50, 50, 50},
                                                 cv::Scalar{0}, cv::FILLED);
        }
    }
    return image;
}
Frame frameFrom(const cv::Mat& input, PixelFormat format = PixelFormat::Gray8)
{
    cv::Mat image;
    if (format == PixelFormat::Rgb8) cv::cvtColor(input, image, cv::COLOR_GRAY2RGB);
    else if (format == PixelFormat::Rgba8) cv::cvtColor(input, image, cv::COLOR_GRAY2RGBA);
    else image = input;
    Frame frame;
    frame.width = static_cast<std::uint32_t>(image.cols);
    frame.height = static_cast<std::uint32_t>(image.rows);
    frame.strideBytes = image.cols * image.elemSize() + 8; // Exercise padded rows.
    frame.pixelFormat = format;
    auto storage = std::make_shared<std::vector<std::byte>>(frame.strideBytes * frame.height);
    for (int row = 0; row < image.rows; ++row) {
        std::memcpy(storage->data() + row * frame.strideBytes, image.ptr(row), image.cols * image.elemSize());
    }
    frame.pixels = storage;
    return frame;
}
TEST(CalibrationDetectorTest, DetectsAllFormatsWithPaddingAndSubpixelCorners)
{
    OpenCvCalibrationDetector detector;
    for (const auto format : {PixelFormat::Gray8, PixelFormat::Rgb8, PixelFormat::Rgba8}) {
        const auto frame = frameFrom(chessboard(), format);
        const auto before = *frame.pixels;
        const auto result = detector.detect(frame, {});
        ASSERT_TRUE(result.error.empty()) << result.error;
        ASSERT_EQ(result.corners.size(), 54U);
        EXPECT_GT(result.sharpness, 40);
        EXPECT_EQ(*frame.pixels, before);
        const auto [min, max] = std::minmax_element(result.corners.begin(), result.corners.end(),
            [](const ImageCorner& a, const ImageCorner& b) { return a.u < b.u; });
        EXPECT_NEAR(min->u, 249.5, 1);
        EXPECT_NEAR(max->u, 649.5, 1);
    }
}
TEST(CalibrationDetectorTest, DetectsVariedPositionsRotationAndPerspective)
{
    OpenCvCalibrationDetector detector;
    const auto image = chessboard();
    std::vector<cv::Mat> transforms;
    transforms.push_back(cv::getRotationMatrix2D({500, 400}, 22, 0.8));
    transforms.push_back(cv::getRotationMatrix2D({500, 400}, -18, 1.1));
    for (const auto& transform : transforms) {
        cv::Mat moved;
        cv::warpAffine(image, moved, transform, image.size(), cv::INTER_LINEAR,
                       cv::BORDER_CONSTANT, cv::Scalar{255});
        EXPECT_EQ(detector.detect(frameFrom(moved), {}).corners.size(), 54U);
    }
    const std::vector<cv::Point2f> from{{200,225}, {700,225}, {700,575}, {200,575}};
    const std::vector<cv::Point2f> to{{140,130}, {680,220}, {720,590}, {210,620}};
    cv::Mat perspective;
    cv::warpPerspective(image, perspective, cv::getPerspectiveTransform(from, to), image.size(),
                        cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar{255});
    EXPECT_EQ(detector.detect(frameFrom(perspective), {}).corners.size(), 54U);
}
TEST(CalibrationDetectorTest, DiverseDetectedBoardsCompleteTheDefaultCapturePolicy)
{
    OpenCvCalibrationDetector detector;
    CalibrationCapture session;
    session.restart("synthetic-camera");
    const auto image = chessboard();
    const std::array<std::array<double, 4>, 10> poses{{
        {280, 250, 0, .7}, {500, 250, 8, .8}, {720, 250, -8, .7},
        {280, 400, 15, .7}, {500, 400, -15, 1.1}, {720, 400, 15, .7},
        {280, 550, -15, .7}, {500, 550, 0, .8}, {720, 550, -8, .7},
        {500, 400, 30, .9}
    }};
    std::uint64_t sequence = 0;
    for (const auto& pose : poses) {
        auto transform = cv::getRotationMatrix2D({450, 400}, pose[2], pose[3]);
        transform.at<double>(0, 2) += pose[0] - 450;
        transform.at<double>(1, 2) += pose[1] - 400;
        cv::Mat moved;
        cv::warpAffine(image, moved, transform, image.size(), cv::INTER_LINEAR,
                       cv::BORDER_CONSTANT, cv::Scalar{255});
        auto frame = std::make_shared<Frame>(frameFrom(moved));
        frame->sequence = ++sequence;
        frame->capturedAt = Timestamp{std::chrono::seconds{static_cast<int>(sequence)}};
        const auto decision = session.add(frame, detector.detect(*frame, {}));
        ASSERT_TRUE(decision.accepted) << "View " << sequence << ": " << decision.message;
    }
    EXPECT_EQ(session.views().size(), 10U);
    EXPECT_TRUE(session.progress().ready);
    EXPECT_TRUE(session.finish());
}
TEST(CalibrationDetectorTest, RejectsBlankPartialInvalidAndOversizedFrames)
{
    OpenCvCalibrationDetector detector;
    EXPECT_TRUE(detector.detect(frameFrom(cv::Mat(800, 1000, CV_8UC1, cv::Scalar{127})), {}).corners.empty());
    auto partial = chessboard();
    cv::rectangle(partial, {0,0,400,800}, cv::Scalar{255}, cv::FILLED);
    EXPECT_TRUE(detector.detect(frameFrom(partial), {}).corners.empty());
    EXPECT_FALSE(detector.detect({}, {}).error.empty());
    auto frame = frameFrom(chessboard());
    frame.width = 100000;
    EXPECT_FALSE(detector.detect(frame, {}).error.empty());
    EXPECT_FALSE(detector.detect(frameFrom(chessboard()), {1, 1, 0}).error.empty());
}
TEST(CalibrationDetectorTest, BlurReducesTargetSharpness)
{
    OpenCvCalibrationDetector detector;
    auto sharp = chessboard();
    cv::Mat blurred;
    cv::GaussianBlur(sharp, blurred, {31,31}, 5);
    const auto good = detector.detect(frameFrom(sharp), {});
    const auto bad = detector.detect(frameFrom(blurred), {});
    ASSERT_EQ(good.corners.size(), 54U);
    if (!bad.corners.empty()) EXPECT_LT(bad.sharpness, 40);
}
} // namespace

#include "calibration/WorkspaceAcquisition.hpp"
#include "calibration/OpenCvWorkspaceHomographySolver.hpp"
#include "HomographyFixtures.hpp"

TEST(CalibrationDetectorWorkspaceTest, SharedRasterTargetGeneratesMeasuredNativeWorkspacePairs)
{
    using namespace micro_robot::test;
    auto frame=frameFrom(chessboard()); frame.sequence=42;
    OpenCvCalibrationDetector detector;
    const auto detected=detector.detect(frame,WorkspaceBoardTarget);
    ASSERT_EQ(detected.corners.size(),54);
    // Independent synthetic plane map: 50 native pixels correspond to a measured
    // 25 mm square. Physical axes are declared here, not inferred by production UI.
    auto world=[](ImageCorner p) {return WorldPoint2D{Meters{(p.u-49.5)/2000},Meters{(724.5-p.v)/2000}};};
    WorkspaceBoardPlacement placement; placement.id=1; placement.detected=detected.corners;
    placement.included.assign(54,true); placement.frameSequence=frame.sequence;
    placement.anchor=world(detected.corners[0]);
    const auto columnA=world(detected.corners[0]),columnB=world(detected.corners[1]),rowB=world(detected.corners[9]);
    const double cx=columnB.x.value-columnA.x.value,cy=columnB.y.value-columnA.y.value;
    placement.columnAngle=Radians{std::atan2(cy,cx)};
    placement.rowDirection=cx*(rowB.y.value-columnA.y.value)-cy*(rowB.x.value-columnA.x.value)>0?1:-1;
    auto input=homographyInput(); input.profile.bounds=WorkspaceBounds{Meters{0.4},Meters{0.3}};
    input.profile.intrinsics.distortion={0,0,0,0,0}; input.fitPoints.clear(); input.checkPoints.clear();
    const auto generated=generateWorkspaceBoard(placement,input.profile.bounds);
    ASSERT_TRUE(generated.error.empty())<<generated.error;
    input.fitPoints=generated.points;
    input.checkPoints={{{403.5,438.5},{Meters{0.177},Meters{0.143}}}};
    const auto result=OpenCvWorkspaceHomographySolver{}.solve(input);
    ASSERT_TRUE(result.result)<<result.error;
    EXPECT_LT(result.result->fit.maximumMeters,0.0001);
    EXPECT_LT(result.result->check->maximumMeters,0.0001);
    EXPECT_EQ(placement.frameSequence,42);
}
