#include "tracking/RobotTracker.hpp"
#include "tracking/MetricTracking.hpp"
#include <gtest/gtest.h>
#include <limits>
#include <stdexcept>
using namespace micro_robot;
using namespace std::chrono_literals;
namespace {
const Pose2D pose{Meters{.2}, Meters{.3}, Radians{1}};
const TrackingThresholds thresholds{200ms, 1s};
Timestamp at(Duration t) { return Timestamp{} + t; }
TEST(RobotTrackerTest, IndependentTracksAgeAtExplicitBoundaries)
{
    WorldModel world;
    RobotTracker tracker(world, thresholds);
    EXPECT_EQ(world.tracking(RobotId{1}, at(2s)).status, TrackingStatus::Unknown);
    ASSERT_EQ(tracker.observe(RobotId{1}, pose, at(1s), at(1s)), TrackerUpdate::Applied);
    ASSERT_EQ(tracker.observe(RobotId{2}, pose, at(1500ms), at(1500ms)), TrackerUpdate::Applied);
    EXPECT_EQ(world.tracking(RobotId{1}, at(1199ms)).status, TrackingStatus::Fresh);
    const auto stale = world.tracking(RobotId{1}, at(1200ms));
    EXPECT_EQ(stale.status, TrackingStatus::Stale);
    ASSERT_TRUE(stale.age);
    EXPECT_EQ(*stale.age, 200ms);
    EXPECT_EQ(world.tracking(RobotId{1}, at(2s)).status, TrackingStatus::Lost);
    EXPECT_EQ(world.tracking(RobotId{2}, at(2s)).status, TrackingStatus::Stale);
    EXPECT_EQ(world.tracking(RobotId{2}, at(1s)).status, TrackingStatus::Lost);
}
TEST(RobotTrackerTest, RejectsOldFutureInvalidAndExpiredWithoutCorruptingWorld)
{
    WorldModel world;
    RobotTracker tracker(world, thresholds);
    ASSERT_EQ(tracker.observe(RobotId{1}, pose, at(1s), at(1s)), TrackerUpdate::Applied);
    EXPECT_EQ(tracker.observe(RobotId{1}, pose, at(1s), at(1100ms)), TrackerUpdate::RejectedByWorld);
    EXPECT_EQ(tracker.observe(RobotId{1}, pose, at(900ms), at(1100ms)), TrackerUpdate::RejectedByWorld);
    EXPECT_EQ(tracker.observe(RobotId{2}, pose, at(3s), at(2s)), TrackerUpdate::FutureTimestamp);
    EXPECT_EQ(tracker.observe(RobotId{2}, pose, at(1s), at(2s)), TrackerUpdate::ExpiredMeasurement);
    auto invalid = pose; invalid.x.value = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(tracker.observe(RobotId{2}, invalid, at(2s), at(2s)), TrackerUpdate::InvalidMeasurement);
    EXPECT_FALSE(world.robot(RobotId{2}));
    EXPECT_EQ(world.robot(RobotId{1})->poseUpdatedAt, at(1s));
}
TEST(RobotTrackerTest, InvalidationAndReappearanceRequireNewerMeasurement)
{
    WorldModel world;
    RobotTracker tracker(world, thresholds);
    ASSERT_EQ(tracker.observe(RobotId{1}, pose, at(1s), at(1s)), TrackerUpdate::Applied);
    tracker.invalidate(at(1050ms));
    EXPECT_EQ(world.tracking(RobotId{1}, at(1050ms)).status, TrackingStatus::Lost);
    EXPECT_EQ(tracker.observe(RobotId{1}, pose, at(1040ms), at(1060ms)), TrackerUpdate::RejectedByWorld);
    ASSERT_EQ(tracker.observe(RobotId{1}, pose, at(1100ms), at(1100ms)), TrackerUpdate::Applied);
    EXPECT_EQ(world.tracking(RobotId{1}, at(1100ms)).status, TrackingStatus::Fresh);
    ASSERT_EQ(world.updatePose(RobotId{1}, pose, at(1200ms)), PoseUpdateResult::Applied);
    EXPECT_EQ(world.tracking(RobotId{1}, at(1200ms)).status, TrackingStatus::Unknown);
}
TEST(RobotTrackerTest, RejectsInvalidThresholds)
{
    WorldModel world;
    EXPECT_THROW((RobotTracker{world, {0ms, 1s}}), std::invalid_argument);
    EXPECT_THROW((RobotTracker{world, {1s, 1s}}), std::invalid_argument);
}
}

TEST(MetricTrackingTest, ValidMeasurementsAgeAndInputInvalidationStopsTracking)
{
    WorldModel world;
    RobotTracker tracker(world, thresholds);
    auto batch = std::make_shared<MetricObservationBatch>();
    batch->status = MetricObservationStatus::Valid;
    batch->measurements.push_back({RobotId{1}, pose, at(1s), 1, 10, {}});
    ingestMetricBatch(tracker, batch, at(1s));
    EXPECT_EQ(world.tracking(RobotId{1}, at(1s)).status, TrackingStatus::Fresh);
    batch->measurements.clear();
    ingestMetricBatch(tracker, batch, at(1200ms));
    EXPECT_EQ(world.tracking(RobotId{1}, at(1200ms)).status, TrackingStatus::Stale);
    ingestMetricBatch(tracker, {}, at(1300ms));
    EXPECT_FALSE(world.robot(RobotId{1})->pose);
    EXPECT_FALSE(world.robot(RobotId{1})->tracking);
    EXPECT_EQ(world.tracking(RobotId{1}, at(1300ms)).status, TrackingStatus::Unknown);
    batch->measurements.push_back({RobotId{1}, pose, at(1250ms), 2, 10, {}});
    ingestMetricBatch(tracker, batch, at(1350ms));
    EXPECT_EQ(world.tracking(RobotId{1}, at(1350ms)).status, TrackingStatus::Unknown);
    batch->measurements[0].capturedAt = at(1400ms);
    ingestMetricBatch(tracker, batch, at(1400ms));
    EXPECT_EQ(world.tracking(RobotId{1}, at(1400ms)).status, TrackingStatus::Fresh);
}
