#include "tracking/RobotTracker.hpp"
#include "tracking/MetricTracking.hpp"
#include <gtest/gtest.h>
#include <limits>
#include <numbers>
#include <cmath>
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
    EXPECT_EQ(world.tracking(RobotId{1}, at(1300ms)).status, TrackingStatus::Lost);
    batch->measurements.push_back({RobotId{1}, pose, at(1250ms), 2, 10, {}});
    ingestMetricBatch(tracker, batch, at(1350ms));
    EXPECT_EQ(world.tracking(RobotId{1}, at(1350ms)).status, TrackingStatus::Lost);
    batch->measurements[0].capturedAt = at(1400ms);
    ingestMetricBatch(tracker, batch, at(1400ms));
    EXPECT_EQ(world.tracking(RobotId{1}, at(1400ms)).status, TrackingStatus::Fresh);
}

TEST(PoseSmoothingTest, WrapSafeAndRawDiagnostics)
{
    WorldModel world;
    RobotTracker tracker(world, thresholds, {.25, .5});
    auto sample = pose; sample.heading = Radians{179 * std::numbers::pi / 180};
    ASSERT_EQ(tracker.observe(RobotId{1}, sample, at(1s), at(1s)), TrackerUpdate::Applied);
    sample.x = Meters{1}; sample.heading = Radians{-179 * std::numbers::pi / 180};
    ASSERT_EQ(tracker.observe(RobotId{1}, sample, at(1100ms), at(1100ms)), TrackerUpdate::Applied);
    const auto state = world.robot(RobotId{1});
    EXPECT_NEAR(state->pose->x.value, .4, 1e-12);
    EXPECT_NEAR(state->pose->heading.value, std::numbers::pi, 1e-12);
    EXPECT_DOUBLE_EQ(state->tracking->rawPose->x.value, 1);
    EXPECT_NEAR(state->tracking->continuousHeading, 181 * std::numbers::pi / 180, 1e-12);
}
TEST(PoseSmoothingTest, LostAndInvalidatedTracksInitializeWithoutTransient)
{
    WorldModel world;
    RobotTracker tracker(world, thresholds, {.1, .1});
    ASSERT_EQ(tracker.observe(RobotId{1}, pose, at(1s), at(1s)), TrackerUpdate::Applied);
    auto sample = pose; sample.x = Meters{2};
    ASSERT_EQ(tracker.observe(RobotId{1}, sample, at(2s), at(2s)), TrackerUpdate::Applied);
    EXPECT_DOUBLE_EQ(world.robot(RobotId{1})->pose->x.value, 2);
    tracker.invalidate(at(2100ms)); sample.x = Meters{3};
    ASSERT_EQ(tracker.observe(RobotId{1}, sample, at(2200ms), at(2200ms)), TrackerUpdate::Applied);
    EXPECT_DOUBLE_EQ(world.robot(RobotId{1})->pose->x.value, 3);
    EXPECT_THROW((RobotTracker{world, thresholds, {0, 1}}), std::invalid_argument);
}
TEST(PoseSmoothingTest, StationaryNoiseIsReducedAndRepeatedSamplesStayFixed)
{
    WorldModel world;
    RobotTracker tracker(world, thresholds, {.2, .2});
    double rawEnergy = 0, filteredEnergy = 0;
    for (int i = 0; i < 200; ++i) {
        Pose2D sample{Meters{i == 0 ? 0.0 : (i % 2 ? .01 : -.01)}, Meters{}, Radians{}};
        const auto t = at(1s + i * 10ms);
        ASSERT_EQ(tracker.observe(RobotId{1}, sample, t, t), TrackerUpdate::Applied);
        rawEnergy += sample.x.value * sample.x.value;
        filteredEnergy += std::pow(world.robot(RobotId{1})->pose->x.value, 2);
    }
    EXPECT_LT(filteredEnergy, rawEnergy / 4);
}

TEST(MotionEstimateTest, KnownMotionAcrossWrapAndAgeConfidence)
{
    WorldModel world;
    RobotTracker tracker(world, thresholds);
    auto sample = pose; sample.heading = Radians{3.13};
    ASSERT_EQ(tracker.observe(RobotId{1}, sample, at(1s), at(1s), .8), TrackerUpdate::Applied);
    EXPECT_FALSE(world.robot(RobotId{1})->tracking->velocity);
    sample.x.value += .1; sample.y.value -= .2; sample.heading = Radians{-3.13};
    ASSERT_EQ(tracker.observe(RobotId{1}, sample, at(1100ms), at(1100ms), .8), TrackerUpdate::Applied);
    const auto v = world.robot(RobotId{1})->tracking->velocity;
    ASSERT_TRUE(v);
    EXPECT_NEAR(v->xMetersPerSecond, 1, 1e-12);
    EXPECT_NEAR(v->yMetersPerSecond, -2, 1e-12);
    EXPECT_NEAR(v->radiansPerSecond, (2 * std::numbers::pi - 6.26) / .1, 1e-12);
    EXPECT_NEAR(*world.tracking(RobotId{1}, at(1600ms)).confidence, .4, 1e-12);
    EXPECT_EQ(world.tracking(RobotId{1}, at(2100ms)).confidence, 0);
}
TEST(MotionEstimateTest, StationaryTinyDtGapAndInvalidQuality)
{
    WorldModel world;
    RobotTracker tracker(world, thresholds);
    ASSERT_EQ(tracker.observe(RobotId{1}, pose, at(1s), at(1s)), TrackerUpdate::Applied);
    ASSERT_EQ(tracker.observe(RobotId{1}, pose, at(1100ms), at(1100ms)), TrackerUpdate::Applied);
    ASSERT_TRUE(world.robot(RobotId{1})->tracking->velocity);
    EXPECT_DOUBLE_EQ(world.robot(RobotId{1})->tracking->velocity->xMetersPerSecond, 0);
    EXPECT_DOUBLE_EQ(world.robot(RobotId{1})->tracking->velocity->radiansPerSecond, 0);
    ASSERT_EQ(tracker.observe(RobotId{1}, pose, at(1100ms + 1ns), at(1100ms + 1ns)), TrackerUpdate::Applied);
    EXPECT_FALSE(world.robot(RobotId{1})->tracking->velocity);
    ASSERT_EQ(tracker.observe(RobotId{1}, pose, at(3s), at(3s)), TrackerUpdate::Applied);
    EXPECT_FALSE(world.robot(RobotId{1})->tracking->velocity);
    EXPECT_EQ(tracker.observe(RobotId{1}, pose, at(3100ms), at(3100ms), 2), TrackerUpdate::InvalidMeasurement);
    EXPECT_EQ(world.robot(RobotId{1})->poseUpdatedAt, at(3s));
    tracker.invalidate(at(3200ms));
    EXPECT_EQ(world.tracking(RobotId{1}, at(3200ms)).confidence, 0);
}
