#include "MarkerPrint.hpp"

#include <gtest/gtest.h>
#include <opencv2/imgproc.hpp>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>

using namespace micro_robot::marker_print;

TEST(MarkerPrintTest, RejectsUnallocatedIdsAndInvalidSizes)
{
    EXPECT_THROW((void)markerCells(-1), std::invalid_argument);
    EXPECT_THROW((void)markerCells(16), std::invalid_argument);
    EXPECT_THROW((void)markerImage(0, 0), std::invalid_argument);
    EXPECT_THROW((void)markerImage(0, 101), std::invalid_argument);
    EXPECT_THROW((void)sheetSvg(30), std::invalid_argument);
}

TEST(MarkerPrintTest, EveryRobotIdDecodesAtAllQuarterTurns)
{
    for (int id = 0; id < robotCount; ++id) {
        auto image = markerImage(id, 10);
        for (int rotation = 0; rotation < 4; ++rotation) {
            SCOPED_TRACE(::testing::Message() << "tag=" << id << " quarter turns=" << rotation);
            EXPECT_EQ(detectIds(image), std::vector<int>{id});
            cv::rotate(image, image, cv::ROTATE_90_CLOCKWISE);
        }
    }
}

TEST(MarkerPrintTest, ImagePreservesOneCellQuietZoneAndBlackBorder)
{
    const auto image = markerImage(0, 1);
    ASSERT_EQ(image.rows, 10);
    ASSERT_EQ(image.cols, 10);
    for (int i = 0; i < 10; ++i) {
        EXPECT_EQ(image.at<unsigned char>(0, i), 255);
        EXPECT_EQ(image.at<unsigned char>(9, i), 255);
        EXPECT_EQ(image.at<unsigned char>(i, 0), 255);
        EXPECT_EQ(image.at<unsigned char>(i, 9), 255);
    }
    for (int i = 1; i < 9; ++i) {
        EXPECT_EQ(image.at<unsigned char>(1, i), 0);
        EXPECT_EQ(image.at<unsigned char>(8, i), 0);
        EXPECT_EQ(image.at<unsigned char>(i, 1), 0);
        EXPECT_EQ(image.at<unsigned char>(i, 8), 0);
    }
}

TEST(MarkerPrintTest, CommittedSheetsMatchDictionaryAndLayout)
{
    for (const int size : {20, 40}) {
        const auto path = std::filesystem::path{MICRO_ROBOT_MARKER_ASSETS}
            / ("tag36h11-" + std::to_string(size) + "mm-a4.svg");
        std::ifstream input{path};
        ASSERT_TRUE(input.is_open()) << path;
        const std::string actual{std::istreambuf_iterator<char>{input}, {}};
        EXPECT_EQ(actual, sheetSvg(size)) << path;
    }
}
