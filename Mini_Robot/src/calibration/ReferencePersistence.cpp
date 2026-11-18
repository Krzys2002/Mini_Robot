#include "calibration/ReferenceMarkers.hpp"
#include "calibration/WorkspaceProfile.hpp"
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
namespace micro_robot {
std::string serializeReferenceConfiguration(const ReferenceConfiguration& c) {
    std::ostringstream out; out.imbue(std::locale::classic()); out<<std::setprecision(std::numeric_limits<double>::max_digits10);
    out<<"micro_robot_references 1\n"<<std::quoted(c.revision)<<' '<<std::quoted(c.tabletopBinding)<<' '<<std::quoted(c.sharedErrorNotes)<<' '
        <<std::quoted(c.validatedLocalizationRevision)<<'\n';
    const auto& p=c.policy;
    out<<p.cadence.count()<<' '<<p.maximumEvidenceAge.count()<<' '<<p.minimumDriftDuration.count()<<' '<<p.consecutiveDriftSamples<<' '
        <<p.maximumResidualPixels<<' '<<p.maximumResidualAxisMeters<<'\n'<<c.markers.size()<<'\n';
    for(const auto& m:c.markers) {
        out<<static_cast<int>(m.family)<<' '<<m.id<<' '<<static_cast<int>(m.role)<<' '<<m.surfaceHeightMeters<<' '
            <<m.blackEdgeMeters<<' '<<m.standardUncertaintyMeters<<' '<<m.independentlySurveyed<<' '
            <<std::quoted(m.installationRevision)<<' '<<std::quoted(m.surveyRevision)<<' '<<std::quoted(m.method);
        for(const auto& corner:m.corners) out<<' '<<corner.x.value<<' '<<corner.y.value;
        out<<'\n';
    }
    out<<c.acquisition.has_value()<<'\n';
    if(c.acquisition) {
        const auto& f=*c.acquisition;
        out<<std::quoted(f.cameraBinding)<<' '<<std::quoted(f.worldFrameId)<<' '<<std::quoted(f.registryRevision)<<' '
            <<std::quoted(f.inputCalibrationRevision)<<' '<<f.frameSequence<<' '<<f.inputGeneration<<' '
            <<std::chrono::duration_cast<std::chrono::nanoseconds>(f.capturedAt.time_since_epoch()).count()<<' '
            <<std::quoted(f.error)<<' '<<f.observations.size()<<'\n';
        for(const auto& row:f.observations) {
            out<<row.id<<' '<<static_cast<int>(row.rejection);
            for(const auto* corners:{&row.raw,&row.undistorted}) for(auto pixel:*corners) out<<' '<<pixel.u<<' '<<pixel.v;
            out<<'\n';
        }
    }
    return out.str();
}
std::optional<ReferenceConfiguration> parseReferenceConfiguration(std::string_view text) {
    if(text.size()>MaximumWorkspaceProfileBytes) return {};
    std::istringstream in{std::string{text}}; in.imbue(std::locale::classic());
    auto quoted=[&](std::string& s){in>>std::ws; return in.peek()=='"' && bool(in>>std::quoted(s));};
    ReferenceConfiguration c; std::string token; int version,flag; std::int64_t cadence,age,duration; std::size_t count;
    if(!(in>>token>>version) || token!="micro_robot_references" || version!=1 || !quoted(c.revision) || !quoted(c.tabletopBinding)
        || !quoted(c.sharedErrorNotes) || !quoted(c.validatedLocalizationRevision)
        || !(in>>cadence>>age>>duration>>c.policy.consecutiveDriftSamples>>c.policy.maximumResidualPixels>>c.policy.maximumResidualAxisMeters>>count)
        || count>MaximumReferenceMarkers) return {};
    c.policy.cadence=std::chrono::milliseconds{cadence}; c.policy.maximumEvidenceAge=std::chrono::milliseconds{age}; c.policy.minimumDriftDuration=std::chrono::milliseconds{duration};
    for(std::size_t i=0;i<count;++i) {
        SurveyedReferenceMarker m; int family,role;
        if(!(in>>family>>m.id>>role>>m.surfaceHeightMeters>>m.blackEdgeMeters>>m.standardUncertaintyMeters>>flag)
            || family!=0 || role<0 || role>2 || (flag!=0 && flag!=1) || !quoted(m.installationRevision) || !quoted(m.surveyRevision) || !quoted(m.method)) return {};
        m.family=static_cast<MarkerFamily>(family); m.role=static_cast<ReferenceRole>(role); m.independentlySurveyed=flag;
        for(auto& corner:m.corners) if(!(in>>corner.x.value>>corner.y.value)) return {};
        c.markers.push_back(std::move(m));
    }
    if(!(in>>flag) || (flag!=0 && flag!=1)) return {};
    if(flag) {
        ReferenceFrame f; std::int64_t ticks;
        if(!quoted(f.cameraBinding) || !quoted(f.worldFrameId) || !quoted(f.registryRevision) || !quoted(f.inputCalibrationRevision)
            || !(in>>f.frameSequence>>f.inputGeneration>>ticks) || !quoted(f.error) || !(in>>count) || count>MaximumReferenceMarkers) return {};
        f.capturedAt=Timestamp{std::chrono::duration_cast<Duration>(std::chrono::nanoseconds{ticks})};
        for(std::size_t i=0;i<count;++i) {
            ReferenceObservation row; int reason;
            if(!(in>>row.id>>reason) || reason<0 || reason>4) return {};
            row.rejection=static_cast<ReferenceRejection>(reason);
            for(auto* corners:{&row.raw,&row.undistorted}) for(auto& pixel:*corners) if(!(in>>pixel.u>>pixel.v)) return {};
            f.observations.push_back(row);
        }
        c.acquisition=std::move(f);
    }
    in>>std::ws; if(!in.eof()) return {};
    return c;
}
} // namespace micro_robot
