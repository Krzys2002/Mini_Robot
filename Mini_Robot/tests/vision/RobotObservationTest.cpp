#include "vision/RobotObservation.hpp"

#include "common/Angle.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>

namespace {
using namespace micro_robot;
constexpr double pi = std::numbers::pi;
const auto otherFamily = static_cast<MarkerFamily>(99);

// Canonical TL, TR, BR, BL of a square of half-size h centred at (u, v) whose
// front edge has the given heading (counterclockwise as displayed, +v down).
std::array<MarkerCorner, 4> square(double u, double v, double heading, double h = 20)
{
    const double fu = std::cos(heading) * h, fv = -std::sin(heading) * h;
    const double ru = -fv, rv = fu; // Marker right side, as seen from above.
    return {{{u + fu - ru, v + fv - rv}, {u + fu + ru, v + fv + rv},
             {u - fu + ru, v - fv + rv}, {u - fu - ru, v - fv - rv}}};
}
MarkerObservation marker(int id, std::array<MarkerCorner, 4> corners = square(100, 100, pi / 2),
                         MarkerFamily family = MarkerFamily::AprilTag36h11)
{
    return {family, id, corners, std::nullopt};
}
MarkerDetections frame(std::vector<MarkerObservation> markers)
{
    MarkerDetections detections;
    detections.frameSequence = 42;
    detections.capturedAt = Timestamp{} + std::chrono::milliseconds{1234};
    detections.width = 1920;
    detections.height = 1080;
    detections.markers = std::move(markers);
    return detections;
}

TEST(RobotMarkerMapTest, DefaultProfileMapsTags0To15ToRobots1To16)
{
    const RobotMarkerMap map;
    for (int tag = 0; tag < 16; ++tag) {
        const auto id = map.robotFor(MarkerFamily::AprilTag36h11, tag);
        ASSERT_TRUE(id) << tag;
        EXPECT_EQ(*id, RobotId{static_cast<std::uint32_t>(tag + 1)});
    }
    for (const int tag : {-1, 16, 17, 586, 587, std::numeric_limits<int>::min(),
                          std::numeric_limits<int>::max()}) {
        EXPECT_FALSE(map.robotFor(MarkerFamily::AprilTag36h11, tag)) << tag;
    }
    EXPECT_FALSE(map.robotFor(otherFamily, 0));
}

TEST(RobotMarkerMapTest, MapsCustomRangesAndRejectsInvalidProfiles)
{
    const RobotMarkerMap map{{MarkerFamily::AprilTag36h11, 100, 3, 7}};
    EXPECT_FALSE(map.robotFor(MarkerFamily::AprilTag36h11, 99));
    EXPECT_EQ(map.robotFor(MarkerFamily::AprilTag36h11, 100), RobotId{7});
    EXPECT_EQ(map.robotFor(MarkerFamily::AprilTag36h11, 102), RobotId{9});
    EXPECT_FALSE(map.robotFor(MarkerFamily::AprilTag36h11, 103));
    EXPECT_FALSE(map.robotFor(MarkerFamily::AprilTag36h11, 0));

    const auto maximum = std::numeric_limits<std::uint32_t>::max();
    EXPECT_NO_THROW((RobotMarkerMap{{MarkerFamily::AprilTag36h11, 586, 1, maximum}}));
    EXPECT_NO_THROW((RobotMarkerMap{{MarkerFamily::AprilTag36h11, 0, 587, 0}}));
    EXPECT_THROW((RobotMarkerMap{{MarkerFamily::AprilTag36h11, 0, 0, 1}}), std::invalid_argument);
    EXPECT_THROW((RobotMarkerMap{{MarkerFamily::AprilTag36h11, -1, 16, 1}}), std::invalid_argument);
    EXPECT_THROW((RobotMarkerMap{{MarkerFamily::AprilTag36h11, 587, 1, 1}}), std::invalid_argument);
    EXPECT_THROW((RobotMarkerMap{{MarkerFamily::AprilTag36h11, 580, 8, 1}}), std::invalid_argument);
    EXPECT_THROW((RobotMarkerMap{{MarkerFamily::AprilTag36h11, 0, 2, maximum}}), std::invalid_argument);
}

TEST(RobotMarkerMapTest, RejectsUnsupportedProfileFamilies)
{
    for (const int family : {-1, 1, 99, std::numeric_limits<int>::max()}) {
        EXPECT_THROW((RobotMarkerMap{{static_cast<MarkerFamily>(family), 0, 16, 1}}),
                     std::invalid_argument) << family;
    }
}

TEST(RobotObservationTest, KnownMarkersBecomeOrderedTimestampedObservations)
{
    auto detections = frame({marker(3, square(300, 200, 0)), marker(0, square(50, 60, pi / 2))});
    detections.markers[0].decisionMargin = 37.5;
    const auto result = observeRobots(detections, RobotMarkerMap{});

    EXPECT_TRUE(result.error.empty());
    EXPECT_TRUE(result.rejected.empty());
    EXPECT_EQ(result.frameSequence, 42U);
    EXPECT_EQ(result.capturedAt, detections.capturedAt);
    EXPECT_EQ(result.width, 1920U);
    EXPECT_EQ(result.height, 1080U);
    ASSERT_EQ(result.robots.size(), 2U);
    EXPECT_EQ(result.robots[0].robotId, RobotId{1});
    EXPECT_EQ(result.robots[1].robotId, RobotId{4});

    const auto& robot = result.robots[1];
    EXPECT_EQ(robot.family, MarkerFamily::AprilTag36h11);
    EXPECT_EQ(robot.markerId, 3);
    EXPECT_EQ(robot.frameSequence, 42U);
    EXPECT_EQ(robot.capturedAt, detections.capturedAt);
    EXPECT_EQ(robot.decisionMargin, 37.5);
    EXPECT_FALSE(result.robots[0].decisionMargin);
    for (std::size_t i = 0; i < 4; ++i) {
        EXPECT_EQ(robot.corners[i].u, detections.markers[0].corners[i].u);
        EXPECT_EQ(robot.corners[i].v, detections.markers[0].corners[i].v);
    }
    EXPECT_NEAR(robot.center.u, 300, 1e-9);
    EXPECT_NEAR(robot.center.v, 200, 1e-9);
    EXPECT_NEAR(robot.imageHeading.value, 0, 1e-12);
    EXPECT_NEAR(robot.minimumEdgePixels, 40, 1e-9);
}

TEST(RobotObservationTest, UnknownFamiliesAndIdsNeverBecomeRobotMeasurements)
{
    const auto result = observeRobots(frame({marker(16), marker(-1), marker(586), marker(2),
        marker(5, square(100, 100, pi / 2), otherFamily)}), RobotMarkerMap{});
    ASSERT_EQ(result.robots.size(), 1U);
    EXPECT_EQ(result.robots[0].robotId, RobotId{3});
    ASSERT_EQ(result.rejected.size(), 4U);
    const int ids[]{16, -1, 586, 5};
    for (std::size_t i = 0; i < 4; ++i) {
        EXPECT_EQ(result.rejected[i].marker.id, ids[i]);
        EXPECT_EQ(result.rejected[i].reason,
                  i == 3 ? MarkerRejection::UnsupportedFamily : MarkerRejection::UnassignedId);
    }

    // Exhaustively: only the allocated range ever yields a robot identity.
    std::vector<MarkerObservation> all;
    for (int id = -5; id < aprilTag36h11MarkerCount + 5; ++id) all.push_back(marker(id));
    const auto exhaustive = observeRobots(frame(all), RobotMarkerMap{});
    ASSERT_EQ(exhaustive.robots.size(), 16U);
    for (std::size_t i = 0; i < 16; ++i) {
        EXPECT_EQ(exhaustive.robots[i].markerId, static_cast<int>(i));
        EXPECT_EQ(exhaustive.robots[i].robotId, RobotId{static_cast<std::uint32_t>(i + 1)});
    }
    EXPECT_EQ(exhaustive.rejected.size(), all.size() - 16);
}

TEST(RobotObservationTest, DuplicateRobotMarkersAreRejectedWithoutChoosingOne)
{
    const auto result = observeRobots(frame({marker(2, square(100, 100, 0)), marker(4),
        marker(2, square(500, 300, pi))}), RobotMarkerMap{});
    ASSERT_EQ(result.robots.size(), 1U);
    EXPECT_EQ(result.robots[0].robotId, RobotId{5});
    ASSERT_EQ(result.rejected.size(), 2U);
    for (const auto& rejected : result.rejected) {
        EXPECT_EQ(rejected.marker.id, 2);
        EXPECT_EQ(rejected.reason, MarkerRejection::DuplicateRobot);
    }
}

TEST(RobotObservationTest, DetectionErrorsPropagateWithoutPartialObservations)
{
    auto detections = frame({marker(0)});
    detections.error = "decode failed";
    const auto result = observeRobots(detections, RobotMarkerMap{});
    EXPECT_EQ(result.error, "decode failed");
    EXPECT_EQ(result.frameSequence, 42U);
    EXPECT_EQ(result.capturedAt, detections.capturedAt);
    EXPECT_TRUE(result.robots.empty());
    EXPECT_TRUE(result.rejected.empty());

    const auto empty = observeRobots(frame({}), RobotMarkerMap{});
    EXPECT_TRUE(empty.error.empty());
    EXPECT_TRUE(empty.robots.empty());
    EXPECT_TRUE(empty.rejected.empty());
}

TEST(MarkerGeometryTest, CenterFollowsTranslationAtEveryHeading)
{
    for (const double heading : {0.0, 0.3, pi / 2, 2.0, pi, -2.5, -pi / 2}) {
        const auto origin = markerGeometry(square(0, 0, heading));
        ASSERT_TRUE(origin);
        for (const auto [du, dv] : {std::pair{0.0, 0.0}, {12.25, -3.5}, {1919.5, 1079.5}, {-40.0, 700.0}}) {
            const auto moved = markerGeometry(square(du, dv, heading));
            ASSERT_TRUE(moved);
            EXPECT_NEAR(moved->center.u, origin->center.u + du, 1e-9);
            EXPECT_NEAR(moved->center.v, origin->center.v + dv, 1e-9);
            EXPECT_NEAR(moved->imageHeading.value, origin->imageHeading.value, 1e-12);
            EXPECT_NEAR(moved->minimumEdgePixels, origin->minimumEdgePixels, 1e-9);
        }
    }
}

TEST(MarkerGeometryTest, CenterIsTheProjectiveDiagonalIntersection)
{
    // A tilted square imaged as a trapezoid: its vertex mean would be (5, 5).
    const auto geometry = markerGeometry({{{0, 0}, {10, 0}, {15, 10}, {-5, 10}}});
    ASSERT_TRUE(geometry);
    EXPECT_NEAR(geometry->center.u, 5, 1e-12);
    EXPECT_NEAR(geometry->center.v, 10.0 / 3.0, 1e-12);
    EXPECT_NEAR(geometry->imageHeading.value, pi / 2, 1e-12);
    EXPECT_NEAR(geometry->minimumEdgePixels, 10, 1e-12);
}

TEST(MarkerGeometryTest, CardinalHeadingsFollowTheDisplayedCounterclockwiseConvention)
{
    // Front edge toward image right, up, left and down. Exact corners are used
    // so that the left-facing case lands on the -pi/+pi boundary.
    const std::array<std::pair<std::array<MarkerCorner, 4>, double>, 4> cases{{
        {{{{1, -1}, {1, 1}, {-1, 1}, {-1, -1}}}, 0.0},
        {{{{-1, -1}, {1, -1}, {1, 1}, {-1, 1}}}, pi / 2},
        {{{{-1, 1}, {-1, -1}, {1, -1}, {1, 1}}}, pi},
        {{{{1, 1}, {-1, 1}, {-1, -1}, {1, -1}}}, -pi / 2},
    }};
    for (const auto& [corners, expected] : cases) {
        const auto geometry = markerGeometry(corners);
        ASSERT_TRUE(geometry);
        EXPECT_EQ(geometry->imageHeading.value, expected);
        EXPECT_EQ(geometry->center.u, 0);
        EXPECT_EQ(geometry->center.v, 0);
    }
}

TEST(MarkerGeometryTest, HeadingFollowsTheFullRotationRangeConsistently)
{
    for (int degrees = -720; degrees <= 720; ++degrees) {
        const double heading = degrees * pi / 180;
        const auto geometry = markerGeometry(square(640, 360, heading));
        ASSERT_TRUE(geometry) << degrees;
        const double value = geometry->imageHeading.value;
        EXPECT_GT(value, -pi) << degrees;
        EXPECT_LE(value, pi) << degrees;
        EXPECT_NEAR(normalizeAngle(Radians{value - heading}).value, 0, 1e-9) << degrees;
        EXPECT_NEAR(geometry->center.u, 640, 1e-9) << degrees;
        EXPECT_NEAR(geometry->center.v, 360, 1e-9) << degrees;
        EXPECT_NEAR(geometry->minimumEdgePixels, 40, 1e-9) << degrees;
    }
}

TEST(MarkerGeometryTest, RejectsNonFiniteDegenerateNonConvexAndMirroredCorners)
{
    const auto nan = std::numeric_limits<double>::quiet_NaN();
    const auto infinity = std::numeric_limits<double>::infinity();
    const std::array<std::array<MarkerCorner, 4>, 7> invalid{{
        {{{nan, 0}, {1, 0}, {1, 1}, {0, 1}}},
        {{{0, 0}, {1, infinity}, {1, 1}, {0, 1}}},
        {{{5, 5}, {5, 5}, {5, 5}, {5, 5}}},              // Collapsed.
        {{{0, 0}, {1, 0}, {2, 0}, {3, 0}}},              // Collinear.
        {{{0, 0}, {1, 1}, {1, 0}, {0, 1}}},              // Self-intersecting.
        {{{0, 0}, {0, 1}, {1, 1}, {1, 0}}},              // Mirrored winding.
        {{{0, 0}, {4, 0}, {1, 1}, {0, 4}}},              // Concave at BR.
    }};
    for (std::size_t i = 0; i < invalid.size(); ++i) {
        EXPECT_FALSE(markerGeometry(invalid[i])) << i;
    }
    const auto result = observeRobots(frame({marker(1, invalid[5]), marker(0)}), RobotMarkerMap{});
    ASSERT_EQ(result.robots.size(), 1U);
    EXPECT_EQ(result.robots[0].robotId, RobotId{1});
    ASSERT_EQ(result.rejected.size(), 1U);
    EXPECT_EQ(result.rejected[0].marker.id, 1);
    EXPECT_EQ(result.rejected[0].reason, MarkerRejection::InvalidGeometry);
}
} // namespace
