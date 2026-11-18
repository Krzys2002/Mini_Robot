#include "planning/MetricPath.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace {
using namespace micro_robot;

TEST(MetricPathTest, StoresWorldWaypointsInMetresWithoutReinterpretation)
{
    const WorkspaceBounds bounds{Meters{1.2}, Meters{0.8},
                                 {Meters{-0.4}, Meters{0.15}}};
    MetricPath path;
    const WorldPoint2D first{Meters{-0.123456789}, Meters{0.234567891}};
    const WorldPoint2D second{Meters{0.651234567}, Meters{0.765432109}};

    ASSERT_TRUE(path.append(first, bounds));
    ASSERT_TRUE(path.append(second, bounds));

    ASSERT_EQ(path.size(), 2U);
    ASSERT_EQ(path.points().size(), 2U);
    EXPECT_DOUBLE_EQ(path.points()[0].x.value, first.x.value);
    EXPECT_DOUBLE_EQ(path.points()[0].y.value, first.y.value);
    EXPECT_DOUBLE_EQ(path.points()[1].x.value, second.x.value);
    EXPECT_DOUBLE_EQ(path.points()[1].y.value, second.y.value);
}

TEST(MetricPathTest, AcceptsClosedWorkspaceBoundaryAndRejectsInvalidPointsAtomically)
{
    const WorkspaceBounds bounds{Meters{1.0}, Meters{0.6}};
    MetricPath path;
    ASSERT_TRUE(path.append({Meters{0.0}, Meters{0.0}}, bounds));
    ASSERT_TRUE(path.append({Meters{1.0}, Meters{0.6}}, bounds));

    const auto original = path.points();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double infinity = std::numeric_limits<double>::infinity();
    for (const WorldPoint2D point : {
             WorldPoint2D{Meters{-0.001}, Meters{0.3}},
             WorldPoint2D{Meters{1.001}, Meters{0.3}},
             WorldPoint2D{Meters{0.5}, Meters{-0.001}},
             WorldPoint2D{Meters{0.5}, Meters{0.601}},
             WorldPoint2D{Meters{nan}, Meters{0.3}},
             WorldPoint2D{Meters{0.5}, Meters{infinity}}}) {
        EXPECT_FALSE(path.append(point, bounds));
        ASSERT_EQ(path.points().size(), original.size());
        for (std::size_t index = 0; index < original.size(); ++index) {
            EXPECT_DOUBLE_EQ(path.points()[index].x.value, original[index].x.value);
            EXPECT_DOUBLE_EQ(path.points()[index].y.value, original[index].y.value);
        }
    }
}

TEST(MetricPathTest, UndoAndClearRespectEmptyAndPreserveRemainingWaypoints)
{
    const WorkspaceBounds bounds{Meters{1.0}, Meters{0.6}};
    MetricPath path;
    EXPECT_TRUE(path.empty());
    EXPECT_FALSE(path.undo());

    const WorldPoint2D first{Meters{0.2}, Meters{0.3}};
    const WorldPoint2D second{Meters{0.7}, Meters{0.4}};
    ASSERT_TRUE(path.append(first, bounds));
    ASSERT_TRUE(path.append(second, bounds));
    ASSERT_TRUE(path.undo());
    ASSERT_EQ(path.size(), 1U);
    EXPECT_DOUBLE_EQ(path.points().front().x.value, first.x.value);
    EXPECT_DOUBLE_EQ(path.points().front().y.value, first.y.value);
    ASSERT_TRUE(path.undo());
    EXPECT_TRUE(path.empty());
    EXPECT_FALSE(path.undo());

    ASSERT_TRUE(path.append(second, bounds));
    path.clear();
    EXPECT_TRUE(path.empty());
    EXPECT_EQ(path.size(), 0U);
    EXPECT_TRUE(path.points().empty());
}

} // namespace
