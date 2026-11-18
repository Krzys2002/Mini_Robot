#pragma once
#include "common/MeasurementProvenance.hpp"
#include "common/RobotId.hpp"
#include <deque>
#include <map>
#include <vector>

namespace micro_robot {
struct PipelineTiming {
    std::uint64_t statisticsEpoch{};
    Timestamp enqueuedAt{}, detectionStarted{}, detectionFinished{}, conversionFinished{}, preparedAt{};
    Duration debugPreparation{}, logging{}, delivery{};
    bool converted{}, debugEnabled{}, loggingEnabled{};
};
struct LatencyDistribution {
    std::size_t samples{};
    std::optional<double> p50Milliseconds, p95Milliseconds, p99Milliseconds;
};
struct RobotCadence {
    RobotId id;
    std::uint64_t accepted{}, rejected{}, missingFrames{};
    std::optional<double> usefulUpdatesPerSecond;
    std::optional<Duration> outputAge, latestInterUpdateGap;
    std::string lastRejection;
};
struct LocalizationReport {
    std::uint64_t inputFrames{}, pendingReplaced{}, sequenceGaps{}, invalidFrames{}, outOfOrder{}, detectorErrors{}, metricUnavailableFrames{};
    std::optional<double> inputFramesPerSecond;
    LatencyDistribution queue, detection, conversion, debugPreparation, logging, delivery,
        publication, receiptToOutput, exposureToOutput, observationAge, overlayPaint, diagnostics, instrumentation;
    std::vector<RobotCadence> robots;
};
// Owner-thread bounded instrumentation. No raw images/history or automatic clock
// reads. Distributions are rolling last 256 samples; rate windows are five seconds.
class LocalizationMetrics final {
public:
    static constexpr std::size_t SampleLimit = 256;
    static constexpr auto RateWindow = std::chrono::seconds{5};
    static constexpr std::size_t RobotLimit = 587;
    void reset();
    void rejection(RobotId, std::string reason);
    void instrumentation(Duration d) { instrumentation_.add(d); }
    void overlayPaint(Duration d) { paint_.add(d); }
    void diagnostics(Duration d) { diagnostics_.add(d); }
    void input(Timestamp received);
    void pipeline(const PipelineTiming&, bool detectorError, bool metricUnavailable = false);
    void publication(const MeasurementProvenance&, Timestamp started, Timestamp completed,
        const std::vector<RobotId>& accepted, const std::vector<RobotId>& rejected,
        const std::vector<RobotId>& missing);
    [[nodiscard]] LocalizationReport report(Timestamp now) const;
    std::uint64_t pendingReplaced{}, sequenceGaps{}, invalidFrames{}, outOfOrder{};
private:
    struct Samples { std::deque<double> values; void add(Duration); LatencyDistribution distribution() const; };
    struct RobotSamples {
        std::uint64_t accepted{}, rejected{}, missing{};
        std::deque<Timestamp> outputs;
        std::optional<Timestamp> observation;
        std::optional<Duration> gap;
        std::string lastRejection;
    };
    static void append(std::deque<Timestamp>&, Timestamp);
    static std::optional<double> rate(const std::deque<Timestamp>&, Timestamp);
    std::uint64_t inputs_{}, errors_{}, metricUnavailable_{};
    std::deque<Timestamp> inputsTimes_;
    Samples queue_, detection_, conversion_, debug_, logging_, delivery_, publication_, receipt_, exposure_, age_, paint_, diagnostics_, instrumentation_;
    std::map<RobotId, RobotSamples> robots_;
};
} // namespace micro_robot
