#include "calibration/ElevatedLocalization.hpp"
#include "calibration/WorkspaceProfile.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <set>
#include <tuple>
#include <sstream>

namespace micro_robot {
namespace {
bool note(const std::string& s) { return !s.empty() && s.size() <= MaximumGeometryNoteBytes; }
bool finite(double v) { return std::isfinite(v); }
bool rotation(const std::array<double,9>& r) {
    for (double v:r) if (!finite(v)) return false;
    for (std::size_t i=0;i<3;++i) for (std::size_t j=0;j<3;++j) {
        double dot=0;
        for (std::size_t k=0;k<3;++k) dot+=r[i*3+k]*r[j*3+k];
        if (std::abs(dot-(i==j?1.0:0.0))>RotationTolerance) return false;
    }
    const double det=r[0]*(r[4]*r[8]-r[5]*r[7])-r[1]*(r[3]*r[8]-r[5]*r[6])+r[2]*(r[3]*r[7]-r[4]*r[6]);
    return std::abs(det-1)<RotationTolerance;
}
}
std::string tabletopProfileBinding(const WorkspaceCalibrationProfile& p) {
    auto tabletop=p; tabletop.localization.reset(); tabletop.references.reset();
    return serializeWorkspaceProfile(tabletop);
}
std::string validateRobotMounts(const std::vector<RobotMount>& mounts) {
    if (mounts.empty() || mounts.size()>MaximumRobotMounts) return "Configure 1–64 measured robot mounts.";
    std::set<RobotId> ids;
    for (const auto& m:mounts)
        if (!ids.insert(m.robotId).second || !finite(m.tagHeightMeters) || m.tagHeightMeters<0
            || !finite(m.bodyForwardMeters) || !finite(m.bodyLeftMeters) || !finite(m.bodyHeadingOffsetRadians)
            || !note(m.revision) || !note(m.measurementMethod))
            return "Mounts require unique RobotIds, measured nonnegative height, finite SI offsets, revision and measurement method.";
    return {};
}
std::string validateExtrinsicSurvey(const ExtrinsicSurvey& s, const IntrinsicCalibration& k) {
    if (!validateIntrinsicCalibration(k).empty()) return "Invalid calibrated intrinsics.";
    if (!finite(s.surfaceHeightMeters) || s.surfaceHeightMeters<0 || !finite(s.boardThicknessMeters)
        || !finite(s.backingThicknessMeters) || s.boardThicknessMeters<0 || s.backingThicknessMeters<0
        || s.surfaceHeightMeters+RayTolerance<s.boardThicknessMeters+s.backingThicknessMeters
        || !note(s.measurementMethod) || !finite(s.maximumReprojectionPixels) || s.maximumReprojectionPixels<=0
        || !finite(s.maximumCheckAxisMeters) || s.maximumCheckAxisMeters<=0)
        return "Measure surface z from tabletop, board/backing thickness and survey method; use positive error limits.";
    if (s.references.size()>MaximumExtrinsicReferences) return "Extrinsic survey exceeds 256 references.";
    std::set<std::pair<double,double>> pixels;
    std::set<std::tuple<double,double,double>> world;
    std::vector<WorldPoint2D> fit,check;
    for (const auto& p:s.references) {
        if (!finite(p.image.u) || !finite(p.image.v) || p.image.u<0 || p.image.v<0 || p.image.u>k.width-1
            || p.image.v>k.height-1 || !finite(p.world.x.value) || !finite(p.world.y.value)
            || (p.heightMeters && (!finite(*p.heightMeters) || *p.heightMeters<0))
            || !pixels.insert({p.image.u,p.image.v}).second || !world.insert({p.world.x.value,p.world.y.value,p.heightMeters.value_or(s.surfaceHeightMeters)}).second)
            return "Survey references must be distinct finite world points and undistorted native pixels; Checks must be held out.";
        (p.check?check:fit).push_back(p.world);
    }
    for (const auto* group:{&fit,&check}) {
        if (group->size()<4) return "Use at least four Fit and four independently surveyed Check references.";
        double span=0, area=0;
        const auto a=group->front();
        for (const auto b:*group) for (const auto c:*group) {
            const double bx=b.x.value-a.x.value, by=b.y.value-a.y.value;
            const double cx=c.x.value-a.x.value, cy=c.y.value-a.y.value;
            span=std::max({span,std::abs(bx),std::abs(by),std::abs(cx),std::abs(cy)});
            area=std::max(area,std::abs(bx*cy-by*cx));
        }
        if (!finite(area) || span==0 || area<=RayTolerance*span*span) return "Spread Fit and Check references across both world axes.";
    }
    return {};
}
std::optional<std::array<double,3>> cameraWorldPosition(const ElevatedLocalization& e) {
    if (!rotation(e.worldToCamera)) return {};
    for (double v:e.translationMeters) if (!finite(v)) return {};
    std::array<double,3> c{};
    for (std::size_t i=0;i<3;++i) for (std::size_t j=0;j<3;++j) c[i]-=e.worldToCamera[j*3+i]*e.translationMeters[j];
    for (double v:c) if (!finite(v)) return {};
    return c;
}
std::optional<WorldPoint2D> intersectTagPlane(const ElevatedLocalization& e, const IntrinsicCalibration& k, ImageCorner p, double height) {
    const auto c=cameraWorldPosition(e);
    if (!c || !finite(height) || height<0 || (*c)[2]<=height+RayTolerance || !finite(p.u) || !finite(p.v)) return {};
    const std::array<double,3> cameraRay{(p.u-k.cameraMatrix[2])/k.cameraMatrix[0],(p.v-k.cameraMatrix[5])/k.cameraMatrix[4],1};
    std::array<double,3> ray{};
    for (std::size_t i=0;i<3;++i) for (std::size_t j=0;j<3;++j) ray[i]+=e.worldToCamera[j*3+i]*cameraRay[j];
    const double norm=std::hypot(ray[0],ray[1],ray[2]);
    if (!finite(norm) || std::abs(ray[2])<=RayTolerance*norm) return {};
    const double distance=(height-(*c)[2])/ray[2];
    if (!finite(distance) || distance<=RayTolerance) return {};
    const double x=(*c)[0]+distance*ray[0], y=(*c)[1]+distance*ray[1];
    if (!finite(x) || !finite(y)) return {};
    return WorldPoint2D{Meters{x},Meters{y}};
}
std::optional<ImageCorner> projectReference(const ElevatedLocalization& e,const IntrinsicCalibration& k,WorldPoint2D p,double height) {
    const std::array<double,3> w{p.x.value,p.y.value,height};
    auto camera=e.translationMeters;
    for (std::size_t i=0;i<3;++i) for (std::size_t j=0;j<3;++j) camera[i]+=e.worldToCamera[i*3+j]*w[j];
    if (!finite(camera[2]) || camera[2]<=RayTolerance) return {};
    const double u=k.cameraMatrix[0]*camera[0]/camera[2]+k.cameraMatrix[2];
    const double v=k.cameraMatrix[4]*camera[1]/camera[2]+k.cameraMatrix[5];
    if (!finite(u) || !finite(v)) return {};
    return ImageCorner{u,v};
}
std::string validateElevatedLocalization(const ElevatedLocalization& e, const WorkspaceCalibrationProfile& p) {
    if (!note(e.revision)) return "Extrinsics require a bounded calibration revision.";
    try {
        if (e.tabletopBinding!=tabletopProfileBinding(p))
            return "Extrinsics belong to another camera/K/resolution, frame or calibration revision.";
    } catch (const std::exception& error) {
        return std::string{"Invalid tabletop binding: "}+error.what();
    }
    if (auto error=validateExtrinsicSurvey(e.survey,p.intrinsics); !error.empty()) return error;
    if (auto error=validateRobotMounts(e.mounts); !error.empty()) return error;
    const auto camera=cameraWorldPosition(e);
    if (!camera || (*camera)[2]<=e.survey.surfaceHeightMeters+RayTolerance) return "Invalid rotation/extrinsics or camera below the measured surface.";
    for (const auto& ref:e.survey.references) if ((*camera)[2]<=ref.heightMeters.value_or(e.survey.surfaceHeightMeters)+RayTolerance) return "Reference plane must lie below camera.";
    for (const auto& m:e.mounts) if ((*camera)[2]<=m.tagHeightMeters+RayTolerance) return "Tag plane must lie below the camera.";
    if (e.alternativeGapPixels && (!finite(*e.alternativeGapPixels) || *e.alternativeGapPixels<MinimumPoseSeparationPixels)) return "Planar camera pose is ambiguous; improve reference geometry/view and survey.";
    for (const auto& ref:e.survey.references) {
        const auto pixel=projectReference(e,p.intrinsics,ref.world,ref.heightMeters.value_or(e.survey.surfaceHeightMeters));
        const auto world=intersectTagPlane(e,p.intrinsics,ref.image,ref.heightMeters.value_or(e.survey.surfaceHeightMeters));
        if (!pixel || !world) return "Reference behind camera or unsupported/horizon ray.";
        if (std::hypot(pixel->u-ref.image.u,pixel->v-ref.image.v)>e.survey.maximumReprojectionPixels) return "Reference exceeds configured reprojection limit.";
        if (ref.check && (std::abs(world->x.value-ref.world.x.value)>e.survey.maximumCheckAxisMeters
            || std::abs(world->y.value-ref.world.y.value)>e.survey.maximumCheckAxisMeters)) return "Held-out survey check exceeds per-axis metric limit.";
    }
    return {};
}
std::string serializeElevatedLocalization(const ElevatedLocalization& e) {
    std::ostringstream o; o.imbue(std::locale::classic()); o<<std::setprecision(std::numeric_limits<double>::max_digits10);
    const auto& s=e.survey;
    const bool heights=std::any_of(s.references.begin(),s.references.end(),[](const auto& p){return p.heightMeters.has_value();});
    o<<"micro_robot_elevated "<<(heights?2:1)<<"\n"<<std::quoted(e.revision)<<' '<<std::quoted(e.tabletopBinding)<<' '
        <<s.surfaceHeightMeters<<' '<<s.boardThicknessMeters<<' '<<s.backingThicknessMeters<<' '<<std::quoted(s.measurementMethod)<<' '
        <<s.maximumReprojectionPixels<<' '<<s.maximumCheckAxisMeters<<'\n';
    for(double v:e.worldToCamera) o<<v<<' ';
    for(double v:e.translationMeters) o<<v<<' ';
    o<<e.alternativeGapPixels.has_value()<<' '<<e.alternativeGapPixels.value_or(0)<<'\n'<<s.references.size()<<'\n';
    for(const auto& p:s.references) {
        o<<p.image.u<<' '<<p.image.v<<' '<<p.world.x.value<<' '<<p.world.y.value<<' '<<p.check;
        if(heights) o<<' '<<p.heightMeters.has_value()<<' '<<p.heightMeters.value_or(0);
        o<<'\n';
    }
    o<<e.mounts.size()<<'\n';
    for(const auto& m:e.mounts) o<<m.robotId.value()<<' '<<m.tagHeightMeters<<' '<<m.bodyForwardMeters<<' '<<m.bodyLeftMeters<<' '
        <<m.bodyHeadingOffsetRadians<<' '<<std::quoted(m.revision)<<' '<<std::quoted(m.measurementMethod)<<'\n';
    return o.str();
}
std::optional<ElevatedLocalization> parseElevatedLocalization(std::string_view text) {
    if (text.size()>MaximumWorkspaceProfileBytes) return {};
    std::istringstream in{std::string{text}}; in.imbue(std::locale::classic());
    auto quoted=[&](std::string& s) { in>>std::ws; return in.peek()=='"' && bool(in>>std::quoted(s)); };
    ElevatedLocalization e; auto& s=e.survey; std::string token; int version,flag; double gap; std::size_t count;
    if (!(in>>token>>version) || token!="micro_robot_elevated" || (version!=1 && version!=2) || !quoted(e.revision) || !quoted(e.tabletopBinding)
        || !(in>>s.surfaceHeightMeters>>s.boardThicknessMeters>>s.backingThicknessMeters) || !quoted(s.measurementMethod)
        || !(in>>s.maximumReprojectionPixels>>s.maximumCheckAxisMeters)) return {};
    for(double& v:e.worldToCamera) if(!(in>>v)) return {};
    for(double& v:e.translationMeters) if(!(in>>v)) return {};
    if (!(in>>flag>>gap) || (flag!=0 && flag!=1) || (!flag && gap!=0)) return {};
    if(flag) e.alternativeGapPixels=gap;
    if(!(in>>count) || count>MaximumExtrinsicReferences) return {};
    for(std::size_t i=0;i<count;++i) {
        ExtrinsicReference p;
        if(!(in>>p.image.u>>p.image.v>>p.world.x.value>>p.world.y.value>>flag) || (flag!=0 && flag!=1)) return {};
        p.check=flag;
        if(version==2) { double height; if(!(in>>flag>>height) || (flag!=0 && flag!=1) || (!flag && height!=0)) return {}; if(flag) p.heightMeters=height; }
        s.references.push_back(p);
    }
    if(!(in>>count) || count>MaximumRobotMounts) return {};
    for(std::size_t i=0;i<count;++i) {
        RobotMount m; std::uint64_t id;
        if(!(in>>id>>m.tagHeightMeters>>m.bodyForwardMeters>>m.bodyLeftMeters>>m.bodyHeadingOffsetRadians)
            || id>std::numeric_limits<std::uint32_t>::max() || !quoted(m.revision) || !quoted(m.measurementMethod)) return {};
        m.robotId=RobotId{static_cast<std::uint32_t>(id)}; e.mounts.push_back(m);
    }
    in>>std::ws; if(!in.eof()) return {};
    return e;
}
} // namespace micro_robot
