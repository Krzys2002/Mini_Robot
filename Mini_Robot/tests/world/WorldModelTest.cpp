#include "world/WorldModel.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <numbers>

namespace micro_robot {
namespace {

using namespace std::chrono_literals;

TEST(WorldModelTest, EmptyRegistryAndUnknownRobotAreExplicit)
{
    const WorldModel world;
    EXPECT_EQ(world.robotCount(), 0U);
    EXPECT_FALSE(world.robot(RobotId{7}));
    EXPECT_TRUE(world.snapshot().robots.empty());
}

TEST(WorldModelTest, RegisteredRobotHasUnknownPoseAndRegistrationTimestamp)
{
    WorldModel world;
    EXPECT_TRUE(world.registerRobot(RobotId{0}, Timestamp{}));
    const auto state = world.robot(RobotId{0});
    ASSERT_TRUE(state);
    EXPECT_EQ(state->id, RobotId{0});
    EXPECT_EQ(state->registeredAt, Timestamp{});
    EXPECT_FALSE(state->pose);
    EXPECT_FALSE(state->poseUpdatedAt);
    EXPECT_EQ(world.robotCount(), 1U);
    // Epoch is a valid timestamp, not a missing-value sentinel.
    EXPECT_EQ(world.updatePose(RobotId{0}, Pose2D{}, Timestamp{}), PoseUpdateResult::Applied);
    ASSERT_TRUE(world.robot(RobotId{0})->poseUpdatedAt);
    EXPECT_EQ(*world.robot(RobotId{0})->poseUpdatedAt, Timestamp{});
}

TEST(WorldModelTest, StoresCopiedPoseAndTimestampWithoutFilteringOrNormalization)
{
    WorldModel world;
    ASSERT_TRUE(world.registerRobot(RobotId{7}, Timestamp{1s}));
    Pose2D pose{Meters{1.25}, Meters{-0.5}, Radians{3.0 * std::numbers::pi}};
    EXPECT_EQ(world.updatePose(RobotId{7}, pose, Timestamp{2s}), PoseUpdateResult::Applied);
    pose.x = Meters{100.0};
    auto state = world.robot(RobotId{7});
    ASSERT_TRUE(state);
    ASSERT_TRUE(state->pose);
    EXPECT_DOUBLE_EQ(state->pose->x.value, 1.25);
    EXPECT_DOUBLE_EQ(state->pose->y.value, -0.5);
    EXPECT_DOUBLE_EQ(state->pose->heading.value, 3.0 * std::numbers::pi);
    EXPECT_EQ(state->poseUpdatedAt, Timestamp{2s});
    EXPECT_EQ(state->registeredAt, Timestamp{1s});
    state->pose->x = Meters{200.0};
    EXPECT_DOUBLE_EQ(world.robot(RobotId{7})->pose->x.value, 1.25);
}

TEST(WorldModelTest, DuplicateRegistrationDoesNotResetCurrentState)
{
    WorldModel world;
    ASSERT_TRUE(world.registerRobot(RobotId{7}, Timestamp{1s}));
    ASSERT_EQ(world.updatePose(RobotId{7}, {Meters{2.0}, Meters{3.0}, Radians{0.5}},
                              Timestamp{2s}), PoseUpdateResult::Applied);
    EXPECT_FALSE(world.registerRobot(RobotId{7}, Timestamp{3s}));
    EXPECT_EQ(world.robotCount(), 1U);
    const auto state = world.robot(RobotId{7});
    ASSERT_TRUE(state);
    EXPECT_EQ(state->registeredAt, Timestamp{1s});
    EXPECT_EQ(state->poseUpdatedAt, Timestamp{2s});
    ASSERT_TRUE(state->pose);
    EXPECT_DOUBLE_EQ(state->pose->x.value, 2.0);
}

TEST(WorldModelTest, RejectsUnknownRobotsWithoutRegisteringThem)
{
    WorldModel world;
    EXPECT_EQ(world.updatePose(RobotId{7}, Pose2D{}, Timestamp{1s}), PoseUpdateResult::UnknownRobot);
    EXPECT_EQ(world.robotCount(), 0U);
    EXPECT_FALSE(world.robot(RobotId{7}));
}

TEST(WorldModelTest, RejectsOlderAndEqualTimestampsWithoutOverwritingPose)
{
    WorldModel world;
    ASSERT_TRUE(world.registerRobot(RobotId{7}, Timestamp{2s}));
    EXPECT_EQ(world.updatePose(RobotId{7}, Pose2D{}, Timestamp{1s}), PoseUpdateResult::StaleTimestamp);
    EXPECT_FALSE(world.robot(RobotId{7})->pose);
    EXPECT_FALSE(world.robot(RobotId{7})->poseUpdatedAt);
    ASSERT_EQ(world.updatePose(RobotId{7}, {Meters{1.0}, Meters{2.0}, Radians{0.5}},
                              Timestamp{3s}), PoseUpdateResult::Applied);
    for (const Timestamp at : {Timestamp{2s}, Timestamp{3s}}) {
        EXPECT_EQ(world.updatePose(RobotId{7}, Pose2D{}, at), PoseUpdateResult::StaleTimestamp);
        EXPECT_DOUBLE_EQ(world.robot(RobotId{7})->pose->x.value, 1.0);
        EXPECT_EQ(world.robot(RobotId{7})->poseUpdatedAt, Timestamp{3s});
    }
    EXPECT_EQ(world.updatePose(RobotId{7}, Pose2D{}, Timestamp{4s}), PoseUpdateResult::Applied);
    EXPECT_DOUBLE_EQ(world.robot(RobotId{7})->pose->x.value, 0.0);
    EXPECT_EQ(world.robot(RobotId{7})->poseUpdatedAt, Timestamp{4s});
}

TEST(WorldModelTest, RejectsNonfinitePoseComponentsWithoutMutatingState)
{
    WorldModel world;
    ASSERT_TRUE(world.registerRobot(RobotId{7}, Timestamp{1s}));
    ASSERT_EQ(world.updatePose(RobotId{7}, {Meters{1.0}, Meters{2.0}, Radians{0.5}},
                              Timestamp{2s}), PoseUpdateResult::Applied);
    for (const double invalid : {std::numeric_limits<double>::quiet_NaN(),
                                std::numeric_limits<double>::infinity(),
                                -std::numeric_limits<double>::infinity()}) {
        const Pose2D poses[]{
            {Meters{invalid}, Meters{0.0}, Radians{0.0}},
            {Meters{0.0}, Meters{invalid}, Radians{0.0}},
            {Meters{0.0}, Meters{0.0}, Radians{invalid}}
        };
        for (const auto& pose : poses) {
            EXPECT_EQ(world.updatePose(RobotId{7}, pose, Timestamp{3s}), PoseUpdateResult::InvalidPose);
            EXPECT_DOUBLE_EQ(world.robot(RobotId{7})->pose->x.value, 1.0);
            EXPECT_EQ(world.robot(RobotId{7})->poseUpdatedAt, Timestamp{2s});
        }
    }
}

TEST(WorldModelTest, RobotUpdatesHaveIndependentTimestampOrdering)
{
    WorldModel world;
    ASSERT_TRUE(world.registerRobot(RobotId{9}, Timestamp{1s}));
    ASSERT_TRUE(world.registerRobot(RobotId{2}, Timestamp{1s}));
    EXPECT_EQ(world.updatePose(RobotId{9}, {Meters{9.0}, Meters{0.0}, Radians{0.0}},
                              Timestamp{10s}), PoseUpdateResult::Applied);
    EXPECT_EQ(world.updatePose(RobotId{2}, {Meters{2.0}, Meters{0.0}, Radians{0.0}},
                              Timestamp{2s}), PoseUpdateResult::Applied);
    EXPECT_DOUBLE_EQ(world.robot(RobotId{9})->pose->x.value, 9.0);
    EXPECT_EQ(world.robot(RobotId{9})->poseUpdatedAt, Timestamp{10s});
    EXPECT_DOUBLE_EQ(world.robot(RobotId{2})->pose->x.value, 2.0);
    EXPECT_EQ(world.robot(RobotId{2})->poseUpdatedAt, Timestamp{2s});
}

TEST(WorldModelTest, SnapshotsAreIndependentAndOrderedByRobotId)
{
    WorldModel world;
    ASSERT_TRUE(world.registerRobot(RobotId{9}, Timestamp{1s}));
    ASSERT_TRUE(world.registerRobot(RobotId{2}, Timestamp{1s}));
    ASSERT_EQ(world.updatePose(RobotId{9}, Pose2D{}, Timestamp{2s}), PoseUpdateResult::Applied);
    auto snapshot = world.snapshot();
    ASSERT_EQ(snapshot.robots.size(), 2U);
    EXPECT_EQ(snapshot.robots.begin()->first, RobotId{2});
    ASSERT_EQ(world.updatePose(RobotId{9}, {Meters{5.0}, Meters{0.0}, Radians{0.0}},
                              Timestamp{3s}), PoseUpdateResult::Applied);
    EXPECT_DOUBLE_EQ(snapshot.robots.at(RobotId{9}).pose->x.value, 0.0);
    EXPECT_EQ(snapshot.robots.at(RobotId{9}).poseUpdatedAt, Timestamp{2s});
    snapshot.robots.at(RobotId{9}).pose->x = Meters{100.0};
    snapshot.robots.erase(RobotId{2});
    EXPECT_DOUBLE_EQ(world.robot(RobotId{9})->pose->x.value, 5.0);
    EXPECT_EQ(world.robotCount(), 2U);
    ASSERT_TRUE(world.removeRobot(RobotId{9}));
    EXPECT_TRUE(snapshot.robots.contains(RobotId{9}));
}

TEST(WorldModelTest, RemovalAndReregistrationDoNotRestoreOldPose)
{
    WorldModel world;
    ASSERT_TRUE(world.registerRobot(RobotId{7}, Timestamp{1s}));
    ASSERT_TRUE(world.registerRobot(RobotId{8}, Timestamp{1s}));
    ASSERT_EQ(world.updatePose(RobotId{7}, Pose2D{}, Timestamp{2s}), PoseUpdateResult::Applied);
    EXPECT_FALSE(world.removeRobot(RobotId{9}));
    EXPECT_TRUE(world.removeRobot(RobotId{7}));
    EXPECT_FALSE(world.removeRobot(RobotId{7}));
    EXPECT_FALSE(world.robot(RobotId{7}));
    EXPECT_TRUE(world.robot(RobotId{8}));
    EXPECT_EQ(world.robotCount(), 1U);
    ASSERT_TRUE(world.registerRobot(RobotId{7}, Timestamp{3s}));
    const auto state = world.robot(RobotId{7});
    ASSERT_TRUE(state);
    EXPECT_EQ(state->registeredAt, Timestamp{3s});
    EXPECT_FALSE(state->pose);
    EXPECT_FALSE(state->poseUpdatedAt);
    EXPECT_EQ(world.updatePose(RobotId{7}, Pose2D{}, Timestamp{2s}), PoseUpdateResult::StaleTimestamp);
}

} // namespace
} // namespace micro_robot
