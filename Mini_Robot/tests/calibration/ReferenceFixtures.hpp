#pragma once
#include "ElevatedFixtures.hpp"
namespace micro_robot::test {
inline WorkspaceCalibrationProfile referenceProfile() {
    auto p=elevatedProfile();
    const double a=.45/std::hypot(.45,.75),b=.75/std::hypot(.45,.75),depth=std::hypot(.45,.75);
    p.intrinsics.cameraMatrix={700,0,960,0,700,540,0,0,1};
    p.homography=WorkspaceHomography{{depth*b,0,-960*depth*b,0,-depth,540*depth,0,a,700*b-540*a}};
    p.localization->tabletopBinding=tabletopProfileBinding(p);
    for(auto& ref:p.localization->survey.references) ref.image=*projectReference(*p.localization,p.intrinsics,ref.world,p.localization->survey.surfaceHeightMeters);
    ReferenceConfiguration c; c.revision="registry-r1"; c.tabletopBinding=tabletopProfileBinding(p); c.sharedErrorNotes="Independent test surveys; shared instrument bias not eliminated.";
    for(int role=0;role<3;++role) for(double x:{-1.,1.}) for(double y:{-1.,1.}) {
        SurveyedReferenceMarker m; m.id=100+static_cast<int>(c.markers.size()); m.role=static_cast<ReferenceRole>(role);
        m.surfaceHeightMeters=.004+.01*(m.id%3); m.blackEdgeMeters=.032; m.standardUncertaintyMeters=.0005;
        m.installationRevision="install-1"; m.surveyRevision="survey-1"; m.method="Analytic fixture; independent measured world corners"; m.independentlySurveyed=role==1;
        const double cx=x*(.58+.07*role),cy=y*(.32+.05*role),h=m.blackEdgeMeters/2;
        m.corners={WorldPoint2D{Meters{cx+h},Meters{cy+h}},WorldPoint2D{Meters{cx+h},Meters{cy-h}},
            WorldPoint2D{Meters{cx-h},Meters{cy-h}},WorldPoint2D{Meters{cx-h},Meters{cy+h}}};
        c.markers.push_back(m);
    }
    p.references=c; return p;
}
inline ReferenceFrame referenceFrame(const WorkspaceCalibrationProfile& p,Timestamp when=Timestamp{}+std::chrono::seconds{10},
    const ElevatedLocalization* actual=nullptr) {
    ReferenceFrame frame; frame.cameraBinding=referenceCameraBinding(p.intrinsics); frame.worldFrameId=p.frameId;
    frame.registryRevision=p.references->revision; frame.inputCalibrationRevision=p.localization->revision;
    frame.frameSequence=42; frame.inputGeneration=7; frame.capturedAt=when;
    for(const auto& m:p.references->markers) {
        ReferenceObservation row; row.id=m.id; row.rejection=ReferenceRejection::Included;
        for(std::size_t j=0;j<4;++j) {
            auto pixel=projectReference(actual?*actual:*p.localization,p.intrinsics,m.corners[j],m.surfaceHeightMeters);
            if(pixel) row.raw[j]=row.undistorted[j]={pixel->u,pixel->v};
        }
        frame.observations.push_back(row);
    }
    return frame;
}
inline WorkspaceCalibrationProfile commissionedReferenceProfile() {
    auto p=referenceProfile(); const auto frame=referenceFrame(p);
    std::string error; auto input=referenceExtrinsicInput(frame,*p.references,p,"commissioned-r2",error);
    if(!input) throw std::runtime_error(error);
    p.localization->revision=input->revision; p.localization->survey=input->survey;
    p.references->acquisition=frame; p.references->validatedLocalizationRevision=p.localization->revision;
    return p;
}
} // namespace micro_robot::test
