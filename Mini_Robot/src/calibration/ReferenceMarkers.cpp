#include "calibration/ReferenceMarkers.hpp"
#include "calibration/WorkspaceProfile.hpp"
#include <algorithm>
#include <cmath>
#include <set>

namespace micro_robot {
namespace {
bool note(const std::string& s) { return !s.empty() && s.size()<=MaximumGeometryNoteBytes; }
bool validImageQuad(const std::array<MarkerCorner,4>& q) {
    for(std::size_t i=0;i<4;++i) {
        const auto a=q[i],b=q[(i+1)%4],c=q[(i+2)%4];
        if(!std::isfinite(a.u) || !std::isfinite(a.v) || std::hypot(b.u-a.u,b.v-a.v)<MinimumReferenceEdgePixels || (b.u-a.u)*(c.v-b.v)-(b.v-a.v)*(c.u-b.u)<=0) return false;
    }
    return true;
}
bool imageSpread(const std::vector<std::array<double,2>>& points) {
    if(points.size()<MinimumReferenceSupportMarkers) return false;
    double minU=points.front()[0],maxU=minU,minV=points.front()[1],maxV=minV,area=0;
    for(const auto& a:points) {
        minU=std::min(minU,a[0]); maxU=std::max(maxU,a[0]); minV=std::min(minV,a[1]); maxV=std::max(maxV,a[1]);
        for(const auto& b:points) for(const auto& c:points)
            area=std::max(area,std::abs((b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]))/2);
    }
    return maxU-minU>=MinimumReferenceImageSpan && maxV-minV>=MinimumReferenceImageSpan && area>=MinimumReferenceImageTriangleArea;
}
WorldPoint2D centre(const SurveyedReferenceMarker& m) {
    WorldPoint2D result;
    for(const auto& p:m.corners) { result.x.value+=p.x.value/4; result.y.value+=p.y.value/4; }
    return result;
}
double cross(WorldPoint2D a,WorldPoint2D b,WorldPoint2D c) {
    return (b.x.value-a.x.value)*(c.y.value-a.y.value)-(b.y.value-a.y.value)*(c.x.value-a.x.value);
}
}
std::string referenceCameraBinding(const IntrinsicCalibration& k) { return serializeCalibrationProfile(k); }
bool referenceSupportCoversWorkspace(const std::vector<WorldPoint2D>& points,const WorkspaceBounds& bounds) {
    if(points.size()<MinimumReferenceSupportMarkers) return false;
    auto sorted=points;
    std::sort(sorted.begin(),sorted.end(),[](auto a,auto b){return a.x.value<b.x.value || (a.x.value==b.x.value && a.y.value<b.y.value);});
    std::vector<WorldPoint2D> hull;
    for(auto p:sorted) {
        while(hull.size()>1 && cross(hull[hull.size()-2],hull.back(),p)<=0) hull.pop_back();
        hull.push_back(p);
    }
    const auto lower=hull.size();
    for(auto i=sorted.rbegin()+1;i!=sorted.rend();++i) {
        while(hull.size()>lower && cross(hull[hull.size()-2],hull.back(),*i)<=0) hull.pop_back();
        hull.push_back(*i);
    }
    if(hull.size()<4) return false;
    for(const auto corner:workspaceCorners(bounds)) for(std::size_t i=1;i<hull.size();++i)
        if(cross(hull[i-1],hull[i],corner)<-ReferenceSquareToleranceMeters*ReferenceSquareToleranceMeters) return false;
    return true;
}
std::string validateReferenceFrame(const ReferenceFrame& f,const ReferenceConfiguration& config,const WorkspaceCalibrationProfile& p) {
    if(f.cameraBinding!=referenceCameraBinding(p.intrinsics) || f.worldFrameId!=p.frameId || f.registryRevision!=config.revision)
        return "Reference observations belong to another camera/K/resolution, world frame or registry revision.";
    if(!f.error.empty()) return f.error;
    if(f.observations.size()!=config.markers.size()) return "Reference observation snapshot is incomplete or exceeds registry capacity.";
    for(std::size_t i=0;i<f.observations.size();++i) {
        const auto& row=f.observations[i];
        if(row.id!=config.markers[i].id || row.rejection<ReferenceRejection::Included || row.rejection>ReferenceRejection::UnsupportedRay)
            return "Reference snapshot identity/rejection mismatch.";
        for(const auto* corners:{&row.raw,&row.undistorted}) for(const auto pixel:*corners)
            if(!std::isfinite(pixel.u) || !std::isfinite(pixel.v)) return "Non-finite reference pixels.";
        if(row.rejection==ReferenceRejection::Included) {
            if(!validImageQuad(row.raw) || !validImageQuad(row.undistorted)) return "Invalid reference corner order/geometry.";
            for(const auto* corners:{&row.raw,&row.undistorted}) for(const auto pixel:*corners)
                if(pixel.u<0 || pixel.v<0 || pixel.u>p.intrinsics.width-1 || pixel.v>p.intrinsics.height-1) return "Reference pixels outside native image support.";
        }
    }
    return {};
}
std::string validateReferenceConfiguration(const ReferenceConfiguration& config,const WorkspaceCalibrationProfile& p,const RobotMarkerProfile& allocation) {
    if(!note(config.revision) || !note(config.sharedErrorNotes) || config.tabletopBinding!=tabletopProfileBinding(p)) return "Reference registry revision, shared-error notes or exact tabletop binding is invalid.";
    if(config.markers.empty() || config.markers.size()>MaximumReferenceMarkers) return "Use 1–32 surveyed reference markers.";
    const auto& policy=config.policy;
    if(policy.cadence<MinimumReferenceCadence || policy.cadence>MaximumReferenceCadence || policy.maximumEvidenceAge<policy.cadence
        || policy.maximumEvidenceAge>MaximumReferenceCadence || policy.minimumDriftDuration<policy.cadence
        || policy.minimumDriftDuration>MaximumReferenceCadence || policy.consecutiveDriftSamples<MinimumReferenceDriftSamples || policy.consecutiveDriftSamples>MaximumReferenceMarkers
        || !std::isfinite(policy.maximumResidualPixels) || policy.maximumResidualPixels<=0
        || !std::isfinite(policy.maximumResidualAxisMeters) || policy.maximumResidualAxisMeters<=0)
        return "Use bounded positive monitoring limits, at least two consecutive drift samples and a persistence duration of at least one cadence.";
    std::set<int> identities;
    for(const auto& m:config.markers) {
        if(m.family!=MarkerFamily::AprilTag36h11 || m.id<0 || m.id>=aprilTag36h11MarkerCount || !identities.insert(m.id).second
            || (m.family==allocation.family && m.id>=allocation.firstMarkerId && static_cast<std::uint32_t>(m.id-allocation.firstMarkerId)<allocation.markerCount))
            return "Reference family/ID is duplicated, unsupported or conflicts with the robot marker allocation.";
        if(m.role<ReferenceRole::Fit || m.role>ReferenceRole::DriftMonitor || !note(m.installationRevision) || !note(m.surveyRevision) || !note(m.method)
            || (m.role==ReferenceRole::IndependentCheck && !m.independentlySurveyed)
            || !std::isfinite(m.surfaceHeightMeters) || m.surfaceHeightMeters<0 || !std::isfinite(m.blackEdgeMeters) || m.blackEdgeMeters<=0
            || !std::isfinite(m.standardUncertaintyMeters) || m.standardUncertaintyMeters<=0) return "References require measured height/black edge/uncertainty, role, survey/mount revisions and method; Checks require independent survey declaration.";
        for(std::size_t i=0;i<4;++i) {
            const auto a=m.corners[i],b=m.corners[(i+1)%4],c=m.corners[(i+2)%4];
            const double edge=std::hypot(b.x.value-a.x.value,b.y.value-a.y.value);
            const double dot=(b.x.value-a.x.value)*(c.x.value-b.x.value)+(b.y.value-a.y.value)*(c.y.value-b.y.value);
            if(!std::isfinite(a.x.value) || !std::isfinite(a.y.value) || !std::isfinite(edge) || !std::isfinite(dot)
                || std::abs(edge-m.blackEdgeMeters)>ReferenceSquareToleranceMeters+ReferenceSizeRelativeTolerance*m.blackEdgeMeters
                || cross(a,b,c)>=0 || std::abs(dot)>ReferenceSizeRelativeTolerance*m.blackEdgeMeters*m.blackEdgeMeters)
                return "Reference corners must describe the measured square in decoded TL/TR/BR/BL order, clockwise in world X/Y.";
        }
    }
    if(config.acquisition) {
        if(auto error=validateReferenceFrame(*config.acquisition,config,p); !error.empty()) return error;
    }
    if(!config.validatedLocalizationRevision.empty()) {
        if(!config.acquisition || !p.localization || p.localization->revision!=config.validatedLocalizationRevision)
            return "Reference activation revision lacks its exact validated geometry/acquisition.";
        const auto& survey=p.localization->survey;
        if(survey.measurementMethod!="Surveyed reference registry "+config.revision
            || survey.maximumReprojectionPixels!=config.policy.maximumResidualPixels
            || survey.maximumCheckAxisMeters!=config.policy.maximumResidualAxisMeters)
            return "Validated reference survey/policy no longer matches the calibrated geometry.";
        std::size_t index=0; std::vector<WorldPoint2D> fit,checks;
        std::vector<std::array<double,2>> fitImage,checkImage;
        for(std::size_t i=0;i<config.markers.size();++i) {
            const auto& marker=config.markers[i]; const auto& row=config.acquisition->observations[i];
            if(marker.role==ReferenceRole::DriftMonitor || row.rejection!=ReferenceRejection::Included) continue;
            (marker.role==ReferenceRole::Fit?fit:checks).push_back(centre(marker));
            std::array<double,2> image{};
            for(auto pixel:row.undistorted) { image[0]+=pixel.u/(4*p.intrinsics.width); image[1]+=pixel.v/(4*p.intrinsics.height); }
            (marker.role==ReferenceRole::Fit?fitImage:checkImage).push_back(image);
            for(std::size_t j=0;j<4;++j) {
                if(index>=survey.references.size()) return "Validated survey reference count mismatch.";
                const auto& ref=survey.references[index++];
                if(ref.world.x.value!=marker.corners[j].x.value || ref.world.y.value!=marker.corners[j].y.value
                    || ref.image.u!=row.undistorted[j].u || ref.image.v!=row.undistorted[j].v
                    || ref.heightMeters!=std::optional<double>{marker.surfaceHeightMeters}
                    || ref.check!=(marker.role==ReferenceRole::IndependentCheck)) return "Validated reference geometry was changed or relabelled.";
            }
        }
        if(index!=survey.references.size() || !referenceSupportCoversWorkspace(fit,p.bounds) || !referenceSupportCoversWorkspace(checks,p.bounds)
            || !imageSpread(fitImage) || !imageSpread(checkImage))
            return "Validated Fit/Check support is incomplete.";
    }
    return {};
}
std::optional<ExtrinsicSolveInput> referenceExtrinsicInput(const ReferenceFrame& frame,const ReferenceConfiguration& config,
    const WorkspaceCalibrationProfile& p,std::string revision,std::string& error) {
    error=validateReferenceConfiguration(config,p); if(!error.empty()) return {};
    error=validateReferenceFrame(frame,config,p); if(!error.empty()) return {};
    if(!p.localization) { error="Configure measured robot mounts and a height-aware draft first."; return {}; }
    ExtrinsicSolveInput input; input.revision=std::move(revision); input.mounts=p.localization->mounts;
    input.survey.measurementMethod="Surveyed reference registry "+config.revision;
    input.survey.maximumReprojectionPixels=config.policy.maximumResidualPixels;
    input.survey.maximumCheckAxisMeters=config.policy.maximumResidualAxisMeters;
    std::vector<WorldPoint2D> fit,checks;
    std::vector<std::array<double,2>> fitImage,checkImage;
    for(std::size_t i=0;i<config.markers.size();++i) {
        const auto& m=config.markers[i]; const auto& row=frame.observations[i];
        if(row.rejection!=ReferenceRejection::Included || m.role==ReferenceRole::DriftMonitor) continue;
        (m.role==ReferenceRole::Fit?fit:checks).push_back(centre(m));
        std::array<double,2> image{};
        for(auto pixel:row.undistorted) { image[0]+=pixel.u/(4*p.intrinsics.width); image[1]+=pixel.v/(4*p.intrinsics.height); }
        (m.role==ReferenceRole::Fit?fitImage:checkImage).push_back(image);
        for(std::size_t j=0;j<4;++j) input.survey.references.push_back({{row.undistorted[j].u,row.undistorted[j].v},m.corners[j],m.role==ReferenceRole::IndependentCheck,m.surfaceHeightMeters});
    }
    if(!referenceSupportCoversWorkspace(fit,p.bounds) || !referenceSupportCoversWorkspace(checks,p.bounds)) {
        error="Visible Fit and independent Check marker centres must each surround the travel rectangle; a cluster cannot establish workspace support."; return {};
    }
    if(!imageSpread(fitImage) || !imageSpread(checkImage)) {
        error="Reference image support is too clustered/edge-on. Spread both Fit and Check markers across the native image; preserve usable corner sampling."; return {};
    }
    return input;
}
void ReferenceMonitor::configure(const ReferenceConfiguration& config,std::string revision,std::uint64_t generation) {
    // Configuration serialization binds registry, policy, immutable evidence and all revisions.
    cadence_=config.policy.cadence;
    const auto key=serializeReferenceConfiguration(config);
    if(key!=configurationKey_ || revision!=calibrationRevision_) {
        configurationKey_=key; calibrationRevision_=std::move(revision); driftLatched_=false;
        lastAssessment_.reset(); driftStarted_.reset(); driftSamples_=0; report_={};
    }
    if(generation!=generation_) {
        generation_=generation; lastAssessment_.reset(); driftStarted_.reset(); driftSamples_=0;
        report_.permitsMeasurements=false; report_.state=driftLatched_?PlacementConsistency::ConfirmedDrift:PlacementConsistency::Unvalidated;
    }
}
bool ReferenceMonitor::due(Timestamp now) const { return !lastAssessment_ || now-*lastAssessment_>=cadence_; }
PlacementReport ReferenceMonitor::status(Timestamp now,const ReferencePolicy& policy) const {
    auto result=report_;
    if(driftLatched_) { result.state=PlacementConsistency::ConfirmedDrift; result.permitsMeasurements=false; result.message="Confirmed global placement inconsistency; explicit fresh revalidation/activation required."; }
    else if(!lastAssessment_ || now<*lastAssessment_ || now-*lastAssessment_>policy.maximumEvidenceAge) {
        result.state=PlacementConsistency::InsufficientEvidence; result.permitsMeasurements=false; result.message="Reference monitoring expired/unavailable; camera movement or calibration health is unproven.";
    }
    return result;
}
PlacementReport ReferenceMonitor::observe(const ReferenceFrame& f,const ReferenceConfiguration& config,const WorkspaceCalibrationProfile& p) {
    if(lastAssessment_ && f.capturedAt-*lastAssessment_<config.policy.cadence) return status(f.capturedAt,config.policy);
    if(lastAssessment_ && f.capturedAt-*lastAssessment_>config.policy.maximumEvidenceAge) { driftSamples_=0; driftStarted_.reset(); }
    PlacementReport report; report.assessedAt=f.capturedAt; report.frameSequence=f.frameSequence; report.registryRevision=config.revision;
    report.calibrationRevision=p.localization?p.localization->revision:std::string{};
    const auto error=validateReferenceFrame(f,config,p);
    std::vector<WorldPoint2D> visible,good,bad;
    if(error.empty() && p.localization && config.validatedLocalizationRevision==p.localization->revision && !config.validatedLocalizationRevision.empty()) {
        for(std::size_t i=0;i<config.markers.size();++i) {
            const auto& m=config.markers[i]; const auto& row=f.observations[i];
            if(m.role!=ReferenceRole::DriftMonitor || row.rejection!=ReferenceRejection::Included) continue;
            bool supported=true,outlier=false;
            for(std::size_t j=0;j<4;++j) {
                const auto expected=projectReference(*p.localization,p.intrinsics,m.corners[j],m.surfaceHeightMeters);
                const auto mapped=intersectTagPlane(*p.localization,p.intrinsics,{row.undistorted[j].u,row.undistorted[j].v},m.surfaceHeightMeters);
                if(!expected || !mapped) { supported=false; break; }
                const double ex=row.undistorted[j].u-expected->u,ey=row.undistorted[j].v-expected->v;
                outlier=outlier || std::hypot(ex,ey)>config.policy.maximumResidualPixels
                    || std::abs(mapped->x.value-m.corners[j].x.value)>config.policy.maximumResidualAxisMeters
                    || std::abs(mapped->y.value-m.corners[j].y.value)>config.policy.maximumResidualAxisMeters;
            }
            if(!supported) continue;
            const auto c=centre(m); visible.push_back(c); (outlier?bad:good).push_back(c);
        }
    }
    report.visible=visible.size(); report.outliers=bad.size();
    report.state=PlacementConsistency::InsufficientEvidence;
    report.message=error.empty()?"Insufficient visible spatial support; missing/occluded references do not prove movement or health.":error;
    bool global=false;
    if(referenceSupportCoversWorkspace(visible,p.bounds)) {
        if(bad.empty()) { report.state=PlacementConsistency::Consistent; report.permitsMeasurements=true; report.message="Reference placement consistent within configured limits; physical accuracy remains unverified."; }
        else {
            // Spatially widespread failures include rotation/scale drift; a common
            // image-translation direction is not required. This confirms inconsistency,
            // not whether camera, table or a shared reference support moved.
            global=bad.size()>=MinimumReferenceSupportMarkers && static_cast<double>(bad.size())/visible.size()>=ReferenceGlobalFraction
                && referenceSupportCoversWorkspace(bad,p.bounds);
            if(bad.size()==1) {
                report.state=PlacementConsistency::IndividualOutlier;
                report.permitsMeasurements=referenceSupportCoversWorkspace(good,p.bounds);
                report.message=report.permitsMeasurements?"Isolated reference outlier; remaining support surrounds workspace.":"Isolated reference outlier; remaining spatial support insufficient, measurements withheld.";
            } else if(global || !referenceSupportCoversWorkspace(good,p.bounds)) {
                report.state=PlacementConsistency::SuspectedDrift;
                report.message=global?"Widespread spatial residuals; confirming persistent global placement inconsistency.":"Multiple/outlying references or inadequate remaining support; cause unresolved, revalidation required.";
            } else { report.state=PlacementConsistency::IndividualOutlier; report.permitsMeasurements=true; report.message="Isolated reference outlier(s); remaining monitor support surrounds workspace. Inspect moved marker/survey before reuse."; }
        }
    }
    if(global) {
        if(!driftStarted_) driftStarted_=f.capturedAt;
        ++driftSamples_;
        if(driftSamples_>=config.policy.consecutiveDriftSamples && f.capturedAt-*driftStarted_>=config.policy.minimumDriftDuration) driftLatched_=true;
    } else { driftSamples_=0; driftStarted_.reset(); }
    lastAssessment_=f.capturedAt; report_=std::move(report);
    return status(f.capturedAt,config.policy);
}
} // namespace micro_robot
