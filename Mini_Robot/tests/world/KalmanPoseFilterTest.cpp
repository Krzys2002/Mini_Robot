#include "tracking/KalmanPoseFilter.hpp"
#include "tracking/RobotTracker.hpp"
#include "tracking/MetricTracking.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <stdexcept>
using namespace micro_robot;
using namespace std::chrono_literals;

TEST(KalmanPoseFilterTest, ConstantVelocityWithIrregularTimestamps)
{
    KalmanPoseFilter filter;
    double t = 0;
    for (int i = 0; i < 200; ++i) {
        const double dt = i % 2 ? .02 : .04;
        t += dt;
        ASSERT_TRUE(filter.update({Meters{t}, Meters{-2*t}, Radians{.2*t}}, dt));
    }
    EXPECT_NEAR(filter.pose()->x.value, t, .001);
    EXPECT_NEAR(filter.pose()->y.value, -2*t, .001);
    EXPECT_NEAR(filter.pose()->heading.value, .2*t, .001);
}
TEST(KalmanPoseFilterTest, WrapStationaryAndInvalidUpdates)
{
    KalmanPoseFilter filter;
    ASSERT_TRUE(filter.update({Meters{}, Meters{}, Radians{3.13}}, 0));
    ASSERT_TRUE(filter.update({Meters{}, Meters{}, Radians{-3.13}}, .1));
    EXPECT_GT(std::abs(filter.pose()->heading.value), 3);
    const auto before = *filter.pose();
    for (double dt : {0.0, -1.0, 11.0, std::numeric_limits<double>::infinity()})
        EXPECT_FALSE(filter.update({Meters{1}, Meters{}, Radians{}}, dt));
    EXPECT_DOUBLE_EQ(filter.pose()->x.value, before.x.value);
    EXPECT_THROW((KalmanPoseFilter{KalmanNoise{.1, .1, 0, .1}}), std::invalid_argument);
}
TEST(KalmanPoseFilterTest, TrackerSelectionRejectionAndReset)
{
    WorldModel world;
    RobotTracker tracker(world, {200ms, 1s}, {1, 1, PoseFilterKind::Kalman});
    const RobotId id{1};
    const Timestamp start{};
    ASSERT_EQ(tracker.observe(id, {Meters{}, Meters{}, Radians{}}, start, start), TrackerUpdate::Applied);
    ASSERT_EQ(tracker.observe(id, {Meters{.1}, Meters{}, Radians{}}, start+100ms, start+100ms), TrackerUpdate::Applied);
    const auto before = world.robot(id)->pose->x.value;
    EXPECT_EQ(tracker.observe(id, {Meters{100}, Meters{}, Radians{}}, start, start+100ms), TrackerUpdate::RejectedByWorld);
    EXPECT_DOUBLE_EQ(world.robot(id)->pose->x.value, before);
    tracker.invalidate(start+200ms);
    ASSERT_EQ(tracker.observe(id, {Meters{2}, Meters{}, Radians{}}, start+300ms, start+300ms), TrackerUpdate::Applied);
    EXPECT_DOUBLE_EQ(world.robot(id)->pose->x.value, 2);
    ASSERT_TRUE(world.removeRobot(id));
    ASSERT_EQ(tracker.observe(id, {Meters{3}, Meters{}, Radians{}}, start+400ms, start+400ms), TrackerUpdate::Applied);
    EXPECT_DOUBLE_EQ(world.robot(id)->pose->x.value, 3);
}

TEST(KalmanPoseFilterTest, StableStationaryNoiseAndMetricBatchSelection)
{
    double rawEnergy=0, filteredEnergy=0;
    KalmanPoseFilter filter;
    for (int i=0; i<1000; ++i) {
        const double x=.01*std::sin(17*i);
        ASSERT_TRUE(filter.update({Meters{x}, Meters{}, Radians{}}, .02));
        if (i>25) {rawEnergy+=x*x;filteredEnergy+=std::pow(filter.pose()->x.value,2);}
    }
    EXPECT_LT(filteredEnergy, rawEnergy/2);
    WorldModel world;
    RobotTracker tracker(world, {250ms,1s}, {.2,.2,PoseFilterKind::Kalman});
    auto batch=std::make_shared<MetricObservationBatch>();
    batch->status=MetricObservationStatus::Valid;
    batch->measurements.push_back({RobotId{1}, Pose2D{}, Timestamp{}+1s, 1, 10, {}});
    ingestMetricBatch(tracker,batch,Timestamp{}+1s);
    ASSERT_TRUE(world.robot(RobotId{1}));
    EXPECT_EQ(world.tracking(RobotId{1},Timestamp{}+1s).status,TrackingStatus::Fresh);
    batch->measurements[0].pose.x=Meters{.1};
    batch->measurements[0].capturedAt+=20ms;
    ingestMetricBatch(tracker,batch,Timestamp{}+1020ms);
    EXPECT_GT(world.robot(RobotId{1})->pose->x.value,.02);
    EXPECT_LT(world.robot(RobotId{1})->pose->x.value,.1);
}
