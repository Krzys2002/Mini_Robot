#include "calibration/WorkspaceCalibration.hpp"
#include "IntrinsicFixtures.hpp"

#include <gtest/gtest.h>
#include <limits>
#include <type_traits>

namespace {
using namespace micro_robot;
WorkspaceCalibrationProfile profile()
{
    return {WorkspaceCalibrationProfileVersion, "bench-A", WorkspaceBounds{Meters{1.2}, Meters{0.8}},
            test::validIntrinsicCalibration()};
}
TEST(WorkspaceCalibrationTest, SeparateVersionedProfileStartsWithoutATransform)
{
    static_assert(!std::is_same_v<WorkspaceCalibrationProfile, IntrinsicCalibration>);
    auto p = profile();
    EXPECT_EQ(p.version, WorkspaceCalibrationProfileVersion);
    EXPECT_FALSE(p.homography);
    EXPECT_TRUE(validateWorkspaceCalibrationProfile(p).empty());
    EXPECT_TRUE(workspaceCalibrationCompatibilityError(p, p.intrinsics).empty());
}
TEST(WorkspaceCalibrationTest, RejectsUnknownVersionMissingIdentityAndUnsupportedPixelDomain)
{
    auto p = profile();
    p.version = WorkspaceCalibrationProfileVersion + 1;
    EXPECT_FALSE(validateWorkspaceCalibrationProfile(p).empty());
    p = profile(); p.workspaceId.clear();
    EXPECT_FALSE(validateWorkspaceCalibrationProfile(p).empty());
    p = profile(); p.imageCoordinates = static_cast<WorkspaceImageCoordinates>(99);
    EXPECT_FALSE(validateWorkspaceCalibrationProfile(p).empty());
    p = profile(); p.intrinsics.width = 0;
    EXPECT_FALSE(validateWorkspaceCalibrationProfile(p).empty());
}
TEST(WorkspaceCalibrationTest, BindsExactCameraResolutionAndIntrinsicOpticalModel)
{
    const auto p = profile();
    auto active = p.intrinsics;
    active.sourceId += "-other";
    EXPECT_FALSE(workspaceCalibrationCompatibilityError(p, active).empty());
    active = p.intrinsics; ++active.width;
    EXPECT_FALSE(workspaceCalibrationCompatibilityError(p, active).empty());
    active = p.intrinsics; ++active.height;
    EXPECT_FALSE(workspaceCalibrationCompatibilityError(p, active).empty());
    active = p.intrinsics; active.cameraMatrix[0] += 1;
    EXPECT_FALSE(workspaceCalibrationCompatibilityError(p, active).empty());
    active = p.intrinsics; active.distortion[0] += 0.001;
    EXPECT_FALSE(workspaceCalibrationCompatibilityError(p, active).empty());
    // The profile owns a snapshot, not a reference to mutable active intrinsics.
    EXPECT_NE(active.distortion, p.intrinsics.distortion);
}
TEST(WorkspaceCalibrationTest, OptionalHomographyMustBeFiniteAndNonsingularAtAnyHomogeneousScale)
{
    auto p = profile();
    // Illustrative pixel-to-metre storage only; no solver/application is tested here.
    for (double scale : {1.0, -1.0, 1e-100, 1e100}) {
        p.homography = WorkspaceHomography{{0.001 * scale, 0, 0, 0, -0.001 * scale,
                                           0.8 * scale, 0, 0, scale}};
        EXPECT_TRUE(validateWorkspaceCalibrationProfile(p).empty());
    }
    for (double invalid : {std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity()}) {
        p.homography->imageToWorld[0] = invalid;
        EXPECT_FALSE(validateWorkspaceCalibrationProfile(p).empty());
    }
    p.homography = WorkspaceHomography{};
    EXPECT_FALSE(validateWorkspaceCalibrationProfile(p).empty());
    p.homography = WorkspaceHomography{{1, 2, 3, 2, 4, 6, 0, 0, 1}};
    EXPECT_FALSE(validateWorkspaceCalibrationProfile(p).empty());
}
} // namespace
