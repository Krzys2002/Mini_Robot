#include "vision/LocalizationMetrics.hpp"
#include <algorithm>
#include <cmath>

namespace micro_robot {
void LocalizationMetrics::Samples::add(Duration d) {
    if(d < Duration::zero()) return;
    values.push_back(std::chrono::duration<double,std::milli>(d).count());
    if(values.size()>SampleLimit) values.pop_front();
}
LatencyDistribution LocalizationMetrics::Samples::distribution() const {
    if(values.empty()) return {};
    auto sorted=std::vector<double>(values.begin(),values.end()); std::sort(sorted.begin(),sorted.end());
    const auto percentile=[&](double p) { return sorted[static_cast<std::size_t>(std::ceil(p*sorted.size()))-1]; };
    constexpr double Median=.50, Percentile95=.95, Percentile99=.99;
    return {sorted.size(),percentile(Median),percentile(Percentile95),percentile(Percentile99)};
}
void LocalizationMetrics::append(std::deque<Timestamp>& times, Timestamp t) {
    if(!times.empty() && t<=times.back()) return;
    times.push_back(t); if(times.size()>SampleLimit) times.pop_front();
    while(times.size()>1 && t-times.front()>RateWindow) times.pop_front();
}
std::optional<double> LocalizationMetrics::rate(const std::deque<Timestamp>& times,Timestamp now) {
    const auto first=std::lower_bound(times.begin(),times.end(),now-RateWindow);
    if (times.empty() || now < times.back()) return std::nullopt;
    if (first == times.end()) return 0.0;
    const auto n=std::distance(first,times.end());
    if(n<2 || now<=*first) return std::nullopt;
    return static_cast<double>(n-1)/std::chrono::duration<double>(now-*first).count();
}
void LocalizationMetrics::rejection(RobotId id,std::string reason) {
    if(!robots_.contains(id) && robots_.size()>=RobotLimit) return;
    robots_[id].lastRejection=std::move(reason);
}
void LocalizationMetrics::reset() { *this={}; }
void LocalizationMetrics::input(Timestamp t) { ++inputs_; append(inputsTimes_,t); }
void LocalizationMetrics::pipeline(const PipelineTiming& t,bool error,bool metricUnavailable) {
    if(error) ++errors_;
    if(metricUnavailable) ++metricUnavailable_;
    // Reject malformed ordering instead of turning negative durations into zero latency.
    if(t.enqueuedAt>t.detectionStarted || t.detectionStarted>t.detectionFinished
        || t.detectionFinished>t.conversionFinished || t.conversionFinished>t.preparedAt) return;
    queue_.add(t.detectionStarted-t.enqueuedAt); detection_.add(t.detectionFinished-t.detectionStarted);
    if(t.converted) conversion_.add(t.conversionFinished-t.detectionFinished);
    if(t.debugEnabled) debug_.add(t.debugPreparation);
    if(t.loggingEnabled) logging_.add(t.logging);
    delivery_.add(t.delivery);
}
void LocalizationMetrics::publication(const MeasurementProvenance& p,Timestamp start,Timestamp end,
    const std::vector<RobotId>& accepted,const std::vector<RobotId>& rejected,const std::vector<RobotId>& missing) {
    if(end<start || start<p.observedAt) return;
    publication_.add(end-start);
    if(!accepted.empty()) {
        age_.add(end-p.observedAt);
        if(p.timing.receivedAt && *p.timing.receivedAt<=start) receipt_.add(end-*p.timing.receivedAt);
        if(hasMappedExposure(p.timing) && *p.timing.receivedAt<=start) exposure_.add(end-*p.timing.exposureAt);
    }
    const auto get=[&](RobotId id)->RobotSamples* {
        if(!robots_.contains(id) && robots_.size()>=RobotLimit) return nullptr;
        return &robots_[id];
    };
    for(auto id:accepted) if(auto* r=get(id)) {
        ++r->accepted; if(!r->outputs.empty() && end>r->outputs.back()) r->gap=end-r->outputs.back();
        append(r->outputs,end); r->observation=p.observedAt;
    }
    for(auto id:rejected) if(auto* r=get(id)) ++r->rejected;
    for(auto id:missing) if(auto* r=get(id)) ++r->missing;
}
LocalizationReport LocalizationMetrics::report(Timestamp now) const {
    LocalizationReport r{inputs_,pendingReplaced,sequenceGaps,invalidFrames,outOfOrder,errors_,metricUnavailable_,rate(inputsTimes_,now),
        queue_.distribution(),detection_.distribution(),conversion_.distribution(),debug_.distribution(),logging_.distribution(),
        delivery_.distribution(),publication_.distribution(),receipt_.distribution(),exposure_.distribution(),age_.distribution(),paint_.distribution(),diagnostics_.distribution(),instrumentation_.distribution(),{}};
    for(const auto& [id,s]:robots_) r.robots.push_back({id,s.accepted,s.rejected,s.missing,rate(s.outputs,now),
        s.observation && now>=*s.observation?std::optional<Duration>{now-*s.observation}:std::nullopt,s.gap,s.lastRejection});
    return r;
}
} // namespace micro_robot
