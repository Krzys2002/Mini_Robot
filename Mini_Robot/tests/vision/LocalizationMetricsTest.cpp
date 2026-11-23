#include "vision/LocalizationMetrics.hpp"
#include "vision/MetricObservation.hpp"
#include <gtest/gtest.h>
using namespace micro_robot;
using namespace std::chrono_literals;
namespace {
Timestamp at(Duration d) { return Timestamp{}+d; }
MeasurementProvenance sample(Timestamp t) {
    MeasurementProvenance p; p.sourceId="camera"; p.frameSequence=1; p.observedAt=t;
    p.timing={t,{},ObservationTimeSource::Receipt,{},{},false}; return p;
}
TEST(LocalizationMetricsTest, UnknownExposureAndInvalidClockCannotFabricateLatency) {
    LocalizationMetrics m; auto p=sample(at(1s));
    m.publication(p,at(1010ms),at(1015ms),{RobotId{1}},{},{});
    auto r=m.report(at(1020ms));
    ASSERT_TRUE(r.receiptToOutput.p50Milliseconds); EXPECT_DOUBLE_EQ(*r.receiptToOutput.p50Milliseconds,15);
    EXPECT_FALSE(r.exposureToOutput.p50Milliseconds); EXPECT_EQ(r.exposureToOutput.samples,0U);
    EXPECT_FALSE(p.timing.upstreamBufferingKnown);
    p.timing.exposureAt=at(990ms); // Unmapped device time must stay unavailable.
    m.publication(p,at(1010ms),at(1015ms),{RobotId{1}},{},{});
    EXPECT_EQ(m.report(at(1020ms)).exposureToOutput.samples,0U);
    p.timing.exposureClockMapping="documented steady mapping"; p.timing.exposureUncertainty=2ms;
    m.publication(p,at(1010ms),at(1015ms),{RobotId{1}},{},{});
    EXPECT_DOUBLE_EQ(*m.report(at(1020ms)).exposureToOutput.p50Milliseconds,25);
    const auto n=m.report(at(1020ms)).publication.samples;
    m.publication(p,at(1010ms),at(1009ms),{RobotId{1}},{},{});
    EXPECT_EQ(m.report(at(1020ms)).publication.samples,n);
}
TEST(LocalizationMetricsTest, PercentilesUseActualOrderedStagesAndRemainBounded) {
    LocalizationMetrics m;
    for(int i=1;i<=300;++i) {
        PipelineTiming t; t.enqueuedAt=at(1s); t.detectionStarted=at(1001ms);
        t.detectionFinished=t.detectionStarted+std::chrono::milliseconds{i};
        t.conversionFinished=t.detectionFinished+2ms; t.preparedAt=t.conversionFinished+3ms;
        t.converted=true; t.debugEnabled=true; t.debugPreparation=3ms; m.pipeline(t,false);
    }
    auto r=m.report(at(2s)); EXPECT_EQ(r.detection.samples,256U);
    EXPECT_DOUBLE_EQ(*r.detection.p50Milliseconds,172); EXPECT_DOUBLE_EQ(*r.detection.p95Milliseconds,288);
    EXPECT_DOUBLE_EQ(*r.detection.p99Milliseconds,298); EXPECT_DOUBLE_EQ(*r.queue.p50Milliseconds,1);
    EXPECT_DOUBLE_EQ(*r.conversion.p50Milliseconds,2); EXPECT_FALSE(r.logging.p50Milliseconds);
    PipelineTiming broken; broken.enqueuedAt=at(1s); m.pipeline(broken,true);
    EXPECT_EQ(m.report(at(2s)).detection.samples,256U); EXPECT_EQ(m.report(at(2s)).detectorErrors,1U);
}
TEST(LocalizationMetricsTest, UsefulPerRobotCadenceIgnoresAbsenceAndRejectedMeasurementsAndExpires) {
    LocalizationMetrics m;
    for(int i=0;i<31;++i) {
        const auto t=at(1s)+std::chrono::milliseconds{100*i}; m.input(t);
        auto p=sample(t);
        m.publication(p,t,t,i%2==0?std::vector<RobotId>{RobotId{1}}:std::vector<RobotId>{},
            i%3==0?std::vector<RobotId>{RobotId{2}}:std::vector<RobotId>{}, {RobotId{3}});
    }
    const auto r=m.report(at(4s)); ASSERT_EQ(r.robots.size(),3U);
    EXPECT_DOUBLE_EQ(*r.inputFramesPerSecond,10); EXPECT_DOUBLE_EQ(*r.robots[0].usefulUpdatesPerSecond,5);
    EXPECT_EQ(r.robots[0].accepted,16U); EXPECT_EQ(r.robots[1].accepted,0U);
    EXPECT_EQ(r.robots[1].rejected,11U); EXPECT_EQ(r.robots[2].missingFrames,31U);
    EXPECT_FALSE(r.robots[2].outputAge); EXPECT_EQ(*m.report(at(10s)).robots[0].usefulUpdatesPerSecond,0);
    m.reset(); EXPECT_TRUE(m.report(at(10s)).robots.empty()); EXPECT_EQ(m.report(at(10s)).inputFrames,0U);
}
MetricObservationBatch measurement() {
    MetricObservationBatch b{MetricObservationStatus::Valid,{},{}};
    b.measurements.push_back({RobotId{1},{Meters{.1},Meters{.2},Radians{0}},at(1s),1,12,{}});
    b.measurements.back().provenance=sample(at(1s)); return b;
}
TEST(LocalizationQualityTest, GatesAreExplicitAndUnavailableMarginIsNotInvented) {
    auto b=measurement(); LocalizationQualityGates g; g.minimumEdgePixels=15;
    applyQualityGates(b,g,at(1010ms)); ASSERT_TRUE(b.measurements.empty());
    EXPECT_EQ(b.diagnostics[0].reason,MeasurementRejection::TooSmall);
    b=measurement(); g.minimumEdgePixels=0; g.minimumDecisionMargin=1;
    applyQualityGates(b,g,at(1010ms)); EXPECT_EQ(b.diagnostics[0].reason,MeasurementRejection::MarginUnavailable);
    b=measurement(); g.minimumDecisionMargin.reset(); g.maximumObservationAge=5ms;
    applyQualityGates(b,g,at(1010ms)); EXPECT_EQ(b.diagnostics[0].reason,MeasurementRejection::TooOld);
    b=measurement(); applyQualityGates(b,{},at(1010ms)); EXPECT_EQ(b.measurements.size(),1U);
    EXPECT_THROW(validateQualityGates({-1,{},{}}),std::invalid_argument);
    g.maximumObservationAge=0ms; EXPECT_THROW(validateQualityGates(g),std::invalid_argument);
}
TEST(LocalizationQualityTest, FutureUnmappedExposureAndMismatchedObservationAreRejected) {
    auto b=measurement(); applyQualityGates(b,{},at(999ms));
    EXPECT_EQ(b.diagnostics[0].reason,MeasurementRejection::InvalidClock);
    b=measurement(); b.measurements[0].provenance.timing.exposureAt=at(990ms);
    applyQualityGates(b,{},at(1010ms)); EXPECT_EQ(b.diagnostics[0].reason,MeasurementRejection::InvalidClock);
    b=measurement(); b.measurements[0].provenance.frameSequence=2;
    applyQualityGates(b,{},at(1010ms)); EXPECT_EQ(b.diagnostics[0].reason,MeasurementRejection::InvalidClock);
}
}
