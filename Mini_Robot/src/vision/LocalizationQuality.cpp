#include "vision/MetricObservation.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace micro_robot {
const char* measurementRejectionName(MeasurementRejection r) {
    switch(r) {
    case MeasurementRejection::None: return "accepted geometry (tracking may still reject)";
    case MeasurementRejection::UnsupportedFamily: return "unsupported family";
    case MeasurementRejection::UnassignedId: return "unassigned tag";
    case MeasurementRejection::DuplicateRobot: return "duplicate robot identity";
    case MeasurementRejection::InvalidGeometry: return "invalid geometry";
    case MeasurementRejection::OutsideWorkspace: return "outside workspace";
    case MeasurementRejection::TooSmall: return "shortest edge below sampling gate";
    case MeasurementRejection::MarginUnavailable: return "required native margin unavailable";
    case MeasurementRejection::LowMargin: return "native margin below gate";
    case MeasurementRejection::TooOld: return "observation exceeds age gate";
    case MeasurementRejection::InvalidClock: return "invalid clock/measurement binding";
    }
    return "unknown rejection";
}
void validateQualityGates(const LocalizationQualityGates& g) {
    if(!std::isfinite(g.minimumEdgePixels) || g.minimumEdgePixels<0
        || (g.minimumDecisionMargin && !std::isfinite(*g.minimumDecisionMargin))
        || (g.maximumObservationAge && *g.maximumObservationAge<=Duration::zero()))
        throw std::invalid_argument("Quality gates require finite sampling/margin and positive age limits");
}
void applyQualityGates(MetricObservationBatch& b,const LocalizationQualityGates& g,Timestamp now) {
    validateQualityGates(g);
    std::erase_if(b.measurements,[&](const auto& m) {
        auto reason=MeasurementRejection::None;
        if((!m.provenance.sourceId.empty() && (m.provenance.observedAt!=m.capturedAt || m.provenance.frameSequence!=m.frameSequence))
            || (m.provenance.timing.observationSource==ObservationTimeSource::Receipt
                && (!m.provenance.timing.receivedAt || m.capturedAt!=*m.provenance.timing.receivedAt))
            || m.capturedAt>now || (m.provenance.timing.receivedAt && *m.provenance.timing.receivedAt>now)
            || (m.provenance.timing.exposureAt && !hasMappedExposure(m.provenance.timing))
            || (m.provenance.timing.observationSource==ObservationTimeSource::MappedExposure
                && (!hasMappedExposure(m.provenance.timing) || m.capturedAt!=*m.provenance.timing.exposureAt))) reason=MeasurementRejection::InvalidClock;
        else if(g.maximumObservationAge && now-m.capturedAt>*g.maximumObservationAge) reason=MeasurementRejection::TooOld;
        else if(!std::isfinite(m.minimumEdgePixels) || m.minimumEdgePixels<g.minimumEdgePixels) reason=MeasurementRejection::TooSmall;
        else if(g.minimumDecisionMargin && !m.decisionMargin) reason=MeasurementRejection::MarginUnavailable;
        else if(g.minimumDecisionMargin && (!std::isfinite(*m.decisionMargin) || *m.decisionMargin<*g.minimumDecisionMargin)) reason=MeasurementRejection::LowMargin;
        if(reason==MeasurementRejection::None) return false;
        auto row=std::find_if(b.diagnostics.begin(),b.diagnostics.end(),[&](const auto& d){return d.robotId==m.robotId && d.reason==MeasurementRejection::None;});
        if(row!=b.diagnostics.end()) row->reason=reason;
        else b.diagnostics.push_back({0,m.robotId,reason,m.minimumEdgePixels});
        return true;
    });
}
} // namespace micro_robot
