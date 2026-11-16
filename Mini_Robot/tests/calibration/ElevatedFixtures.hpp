#pragma once
#include "calibration/WorkspaceProfile.hpp"
#include "vision/MetricObservation.hpp"
#include "IntrinsicFixtures.hpp"
#include <cmath>
namespace micro_robot::test {
inline WorkspaceCalibrationProfile elevatedProfile(double surfaceHeight=0.008) {
    auto k=validIntrinsicCalibration(); k.width=1920; k.height=1080;
    k.cameraMatrix={1100,0,960,0,1100,540,0,0,1}; k.distortion={0,0,0,0,0};
    WorkspaceCalibrationProfile p{WorkspaceCalibrationProfileVersion,"elevated-table",
        WorkspaceBounds{Meters{1},Meters{.5},{Meters{-.5},Meters{-.25}}},k};
    p.frameId="measured-table-frame";
    const double a=.45/std::hypot(.45,.75),b=.75/std::hypot(.45,.75),depth=std::hypot(.45,.75);
    // Inverse of analytic z=0 projection: v-540=-1100*b*y/(a*y+depth), u-960=1100*x/(a*y+depth).
    p.homography=WorkspaceHomography{{depth*b,0,-960*depth*b,0,-depth,540*depth,
        0,a,1100*b-540*a}};
    ElevatedLocalization e;
    e.revision="camera-pose-r1"; e.tabletopBinding=tabletopProfileBinding(p);
    e.worldToCamera={1,0,0,0,-b,-a,0,a,-b}; e.translationMeters={0,0,depth};
    e.survey.surfaceHeightMeters=surfaceHeight; e.survey.boardThicknessMeters=surfaceHeight; e.survey.backingThicknessMeters=0;
    e.survey.measurementMethod="independent surveyed reference fixture";
    e.mounts={{RobotId{0},.05,.015,-.008,.2,"mount-r1","caliper and angle fixture"}};
    // Near/far/edge fits with distinct held-out interior checks.
    for (bool check:{false,true}) for (double x:(check?std::array{-.3,.3}:std::array{-.45,.45}))
        for(double y:(check?std::array{-.15,.15}:std::array{-.22,.22})) {
            WorldPoint2D world{Meters{x},Meters{y}};
            // Independent analytic pinhole formula, including measured surface height.
            const double z=a*y-b*surfaceHeight+depth;
            e.survey.references.push_back({{960+1100*x/z,540+1100*(-b*y-a*surfaceHeight)/z},world,check});
        }
    p.localization=e; return p;
}
inline std::array<MarkerCorner,4> elevatedCorners(double x,double y,double height,double heading) {
    const double a=.45/std::hypot(.45,.75),b=.75/std::hypot(.45,.75),depth=std::hypot(.45,.75);
    std::array<MarkerCorner,4> pixels;
    const double front[]{1,1,-1,-1},right[]{-1,1,1,-1};
    for(std::size_t i=0;i<4;++i) {
        const double wx=x+.016*(front[i]*std::cos(heading)+right[i]*std::sin(heading));
        const double wy=y+.016*(front[i]*std::sin(heading)-right[i]*std::cos(heading));
        const double z=a*wy-b*height+depth;
        pixels[i]={960+1100*wx/z,540+1100*(-b*wy-a*height)/z};
    }
    return pixels;
}
} // namespace micro_robot::test
