#include <array>
#include "world/Workspace.hpp"
#include "world/WorldModel.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>

namespace {
using namespace micro_robot;
TEST(WorkspaceTest, RequiresFinitePositiveMetricDimensions)
{
    for (double invalid : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                           -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
        EXPECT_THROW((WorkspaceBounds{Meters{invalid}, Meters{0.8}}), std::invalid_argument);
        EXPECT_THROW((WorkspaceBounds{Meters{1.2}, Meters{invalid}}), std::invalid_argument);
    }
    const WorkspaceBounds bounds{Meters{1.2}, Meters{0.8}};
    EXPECT_DOUBLE_EQ(bounds.width().value, 1.2);
    EXPECT_DOUBLE_EQ(bounds.height().value, 0.8);
}
TEST(WorkspaceTest, ClosedBoundsIncludeCornersAndRejectOutsideAndNonFinitePoints)
{
    const WorkspaceBounds bounds{Meters{1.2}, Meters{0.8}};
    for (double x : {0.0, 0.6, 1.2}) for (double y : {0.0, 0.4, 0.8})
        EXPECT_TRUE(bounds.contains({Meters{x}, Meters{y}}));
    for (double x : {-0.001, 1.201, std::numeric_limits<double>::infinity(),
                      std::numeric_limits<double>::quiet_NaN()})
        EXPECT_FALSE(bounds.contains({Meters{x}, Meters{0.4}}));
    for (double y : {-0.001, 0.801, std::numeric_limits<double>::infinity(),
                      std::numeric_limits<double>::quiet_NaN()})
        EXPECT_FALSE(bounds.contains({Meters{0.6}, Meters{y}}));
}
TEST(WorkspaceTest, LowerLeftOriginAndTopLeftViewHaveExplicitCornerMapping)
{
    const WorkspaceBounds bounds{Meters{1.2}, Meters{0.8}};
    for (double x : {0.0, 1.2}) for (double y : {0.0, 0.8}) {
        const auto view = workspaceToView(bounds, {Meters{x}, Meters{y}});
        ASSERT_TRUE(view);
        EXPECT_DOUBLE_EQ(view->u, x == 0 ? 0 : 1);
        EXPECT_DOUBLE_EQ(view->v, y == 0 ? 1 : 0);
    }
}
TEST(WorkspaceTest, LayoutConversionRoundTripsAcrossDifferentAspectRatios)
{
    for (double width : {0.001, 1.2, 100.0}) for (double height : {0.002, 0.8, 30.0}) {
        const WorkspaceBounds bounds{Meters{width}, Meters{height}};
        for (int u = 0; u <= 10; ++u) for (int v = 0; v <= 10; ++v) {
            const WorkspaceViewPoint view{u / 10.0, v / 10.0};
            const auto world = viewToWorkspace(bounds, view);
            ASSERT_TRUE(world);
            EXPECT_TRUE(bounds.contains(*world));
            const auto restored = workspaceToView(bounds, *world);
            ASSERT_TRUE(restored);
            EXPECT_NEAR(restored->u, view.u, 1e-14);
            EXPECT_NEAR(restored->v, view.v, 1e-14);
        }
    }
}
TEST(WorkspaceTest, LayoutConversionRejectsInvalidInputWithoutClamping)
{
    const WorkspaceBounds bounds{Meters{1}, Meters{1}};
    for (double invalid : {-0.1, 1.1, std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()}) {
        EXPECT_FALSE(viewToWorkspace(bounds, {invalid, 0.5}));
        EXPECT_FALSE(viewToWorkspace(bounds, {0.5, invalid}));
        EXPECT_FALSE(workspaceToView(bounds, {Meters{invalid}, Meters{0.5}}));
        EXPECT_FALSE(workspaceToView(bounds, {Meters{0.5}, Meters{invalid}}));
    }
}
TEST(WorkspaceTest, PositiveWorldHeadingRotatesFromViewRightTowardViewUp)
{
    const WorkspaceBounds bounds{Meters{2}, Meters{2}};
    const double pi = std::numbers::pi;
    const double expectedU[]{1, 0.5, 0, 0.5};
    const double expectedV[]{0.5, 0, 0.5, 1};
    for (int quarter = 0; quarter < 4; ++quarter) {
        const Pose2D pose{Meters{1}, Meters{1}, Radians{quarter * pi / 2}};
        const auto front = workspaceToView(bounds,
            {Meters{pose.x.value + std::cos(pose.heading.value)},
             Meters{pose.y.value + std::sin(pose.heading.value)}});
        ASSERT_TRUE(front);
        EXPECT_NEAR(front->u, expectedU[quarter], 1e-14);
        EXPECT_NEAR(front->v, expectedV[quarter], 1e-14);
    }
}
TEST(WorkspaceTest, WorldOwnsOptionalBoundsAndSnapshotsRemainDetached)
{
    WorldModel world;
    EXPECT_FALSE(world.snapshot().workspace);
    EXPECT_FALSE(world.workspaceBounds());
    world.setWorkspaceBounds(WorkspaceBounds{Meters{1.2}, Meters{0.8}});
    auto snapshot = world.snapshot();
    ASSERT_TRUE(snapshot.workspace);
    world.setWorkspaceBounds(WorkspaceBounds{Meters{2}, Meters{3}});
    EXPECT_DOUBLE_EQ(snapshot.workspace->width().value, 1.2);
    snapshot.workspace.reset();
    EXPECT_DOUBLE_EQ(world.snapshot().workspace->height().value, 3);
    EXPECT_DOUBLE_EQ(world.workspaceBounds()->height().value, 3);
    world.setWorkspaceBounds(std::nullopt);
    EXPECT_FALSE(world.snapshot().workspace);
    EXPECT_FALSE(world.workspaceBounds());
}
TEST(WorkspaceTest, BoundsChangesDoNotClampOrReinterpretRobotPoses)
{
    WorldModel world;
    ASSERT_TRUE(world.registerRobot(RobotId{1}, Timestamp{}));
    ASSERT_EQ(world.updatePose(RobotId{1}, {Meters{-1}, Meters{3}, Radians{8}}, Timestamp{}),
              PoseUpdateResult::Applied);
    world.setWorkspaceBounds(WorkspaceBounds{Meters{1}, Meters{1}});
    EXPECT_DOUBLE_EQ(world.robot(RobotId{1})->pose->x.value, -1);
    EXPECT_DOUBLE_EQ(world.robot(RobotId{1})->pose->heading.value, 8);
}
} // namespace

TEST(WorkspaceFrameTest, InteriorOriginRoundTripsSignedCoordinatesAndRejectsInvalidExtents)
{
    const WorkspaceBounds b{Meters{1.5}, Meters{0.6}, {Meters{-0.75}, Meters{-0.2}}};
    for (const auto p : std::array<WorldPoint2D, 3>{b.minimum(), WorldPoint2D{}, b.maximum()}) {
        ASSERT_TRUE(b.contains(p));
        auto v = workspaceToView(b, p);
        ASSERT_TRUE(v);
        auto restored = viewToWorkspace(b, *v);
        ASSERT_TRUE(restored);
        EXPECT_NEAR(restored->x.value, p.x.value, 1e-15);
        EXPECT_NEAR(restored->y.value, p.y.value, 1e-15);
    }
    const auto origin = workspaceToView(b, {});
    ASSERT_TRUE(origin);
    EXPECT_DOUBLE_EQ(origin->u, 0.5);
    EXPECT_NEAR(origin->v, 2.0 / 3, 1e-15);
    EXPECT_FALSE(b.contains({Meters{-0.751}, Meters{0}}));
    EXPECT_THROW((WorkspaceBounds{Meters{1}, Meters{1}, {Meters{1e300}, Meters{0}}}), std::invalid_argument);
}

TEST(WorkspaceFrameTest, ExplicitFrameReplacementClearsMeasurementsWithoutRemovingRobots)
{
    WorldModel world;
    const RobotId id{1};
    ASSERT_TRUE(world.registerRobot(id, Timestamp{}));
    ASSERT_EQ(world.updateTrackedPose(id, {Meters{-0.2}, Meters{0.1}, Radians{0.7}},
        Timestamp{} + std::chrono::seconds{1}, {std::chrono::seconds{1}, std::chrono::seconds{2}}), PoseUpdateResult::Applied);
    world.clearMeasurements();
    ASSERT_TRUE(world.robot(id));
    EXPECT_FALSE(world.robot(id)->pose);
    EXPECT_FALSE(world.robot(id)->poseUpdatedAt);
    EXPECT_FALSE(world.robot(id)->tracking);
}

TEST(WorkspaceFrameTest, AuthoritativeFrameIdentityAndSignedBoundsSurviveSnapshotCopy)
{
    WorldModel world;
    world.setWorkspaceFrame(WorkspaceBounds{Meters{1},Meters{0.6},{Meters{-0.5},Meters{-0.3}}},"frame-A");
    auto first = world.snapshot();
    ASSERT_TRUE(first.workspaceFrameId);
    EXPECT_EQ(*first.workspaceFrameId,"frame-A");
    world.setWorkspaceFrame(WorkspaceBounds{Meters{1},Meters{0.6}},"frame-B");
    EXPECT_EQ(*first.workspaceFrameId,"frame-A");
    EXPECT_DOUBLE_EQ(first.workspace->minimum().x.value,-0.5);
    world.setWorkspaceBounds({});
    EXPECT_FALSE(world.snapshot().workspaceFrameId);
}
