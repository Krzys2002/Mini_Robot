#pragma once
#include "common/Time.hpp"
#include <cstdint>
#include <optional>
#include <memory>
#include <string>

namespace micro_robot {
enum class ObservationTimeSource { Unknown, Receipt, ReplayTimeline, Simulated, MappedExposure };
inline const char* observationTimeSourceName(ObservationTimeSource s) {
    switch(s) {
    case ObservationTimeSource::Unknown: return "unknown observation clock";
    case ObservationTimeSource::Receipt: return "receipt estimate (not exposure)";
    case ObservationTimeSource::ReplayTimeline: return "mapped replay timeline (not exposure)";
    case ObservationTimeSource::Simulated: return "simulated observation time";
    case ObservationTimeSource::MappedExposure: return "mapped sensor exposure";
    }
    return "unknown";
}
// Device/media clocks are never implicitly comparable to steady_clock. An
// adapter may supply exposureAt only with a documented mapping and uncertainty.
struct FrameTiming {
    std::optional<Timestamp> receivedAt;
    std::optional<Timestamp> exposureAt;
    ObservationTimeSource observationSource{ObservationTimeSource::Unknown};
    std::string exposureClockMapping;
    std::optional<Duration> exposureUncertainty;
    bool upstreamBufferingKnown{};
};
inline bool hasMappedExposure(const FrameTiming& t) {
    return t.exposureAt && t.receivedAt && *t.exposureAt <= *t.receivedAt
        && !t.exposureClockMapping.empty() && t.exposureUncertainty
        && *t.exposureUncertainty >= Duration::zero();
}
struct MeasurementProvenance {
    std::string sourceId;
    std::uint64_t frameSequence{}, inputGeneration{};
    std::uint32_t width{}, height{};
    Timestamp observedAt{}; // Original observation time, including receipt/replay estimates.
    FrameTiming timing;
    std::optional<std::string> frameId, mountingRevision;
    // Exact serialized profile identity, shared across measurements; no lossy hash.
    std::shared_ptr<const std::string> calibrationRevision;
};
} // namespace micro_robot
