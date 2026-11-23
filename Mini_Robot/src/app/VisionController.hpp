#pragma once
#include <set>
#include "vision/VisionDebugFrame.hpp"
#include "vision/MetricObservation.hpp"
#include "infrastructure/logging/Logger.hpp"

#include <QObject>
#include <QString>
#include <QThread>
#include <QTimer>

namespace micro_robot {
// Owner-thread application service. Detector runs serially on a dedicated worker;
// one job plus one newest pending frame, never an unbounded frame queue. Connect
// to DesktopController's cameraInputChanged/frameAvailable at composition time.
// Call shutdown(), await stopped(), then destroy outside callbacks. Destruction
// joins as a fallback; production keeps the GUI event loop alive while draining.
class VisionController final : public QObject {
    Q_OBJECT
public:
    VisionController(std::unique_ptr<IMarkerDetector> detector, LogChannel log, QObject* parent = nullptr);
    // Use the same immutable identity map as observation consumers. Defaults
    // preserve the standard tag 0–15 / robot 1–16 allocation.
    VisionController(std::unique_ptr<IMarkerDetector> detector, LogChannel log,
                     RobotMarkerMap markerMap, QObject* parent = nullptr);
    ~VisionController() override;
    void setInput(const QString& sourceId, bool streaming);
    void receiveFrame(FramePtr frame);
    void setMetricConverter(std::unique_ptr<IMetricObservationConverter> converter);
    void setWorkspaceCalibration(std::optional<WorkspaceCalibrationProfile> profile);
    void setReferenceDraft(std::optional<WorkspaceCalibrationProfile> profile);
    void setQualityGates(LocalizationQualityGates gates);
    [[nodiscard]] const LocalizationQualityGates& qualityGates() const { return qualityGates_; }
    void setDebugEnabled(bool enabled);
    void recordTrackingRejection(RobotId id,std::string reason) { metrics_.rejection(id,std::move(reason)); }
    void recordOverlayPaint(Duration d) { metrics_.overlayPaint(d); }
    void recordDiagnostics(Duration d) { metrics_.diagnostics(d); }
    void setFrameLoggingEnabled(bool enabled);
    [[nodiscard]] bool debugEnabled() const { return debugEnabled_; }
    [[nodiscard]] bool frameLoggingEnabled() const { return frameLoggingEnabled_; }
    [[nodiscard]] LocalizationReport localizationReport(Timestamp now) const { return metrics_.report(now); }
    bool recordPublication(const MetricObservationBatch&, const std::vector<RobotId>& accepted, const std::vector<RobotId>& rejected, Timestamp, Timestamp);
    [[nodiscard]] std::uint64_t inputGeneration() const noexcept { return generation_; }
    [[nodiscard]] bool busy() const noexcept { return busy_; }
    [[nodiscard]] bool isStopped() const { return worker_.isFinished(); }
    [[nodiscard]] const QString& description() const noexcept { return description_; }
public slots:
    void shutdown();
signals:
    // Owner-thread delivery. Immutable values may be retained. Null clears old
    // observations on input invalidation/shutdown; source ID scopes the sequence.
    void observationsAvailable(const QString& sourceId, MarkerDetectionsPtr detections);
    // Owner-thread measurement boundary; null invalidates previous batches.
    void metricObservationsAvailable(const QString& sourceId, MetricObservationBatchPtr batch);
    void previewAvailable(const QString& sourceId, VisionDebugFramePtr preview);
    void descriptionChanged(const QString& description);
    void stopped();
    void referenceObservationsAvailable(ReferenceFramePtr frame);
private:
    void invalidate();
    void dispatch();
    void describe(const QString& text);
    LogChannel log_;
    RobotMarkerMap markerMap_;
    VisionOutputRate outputRate_;
    LocalizationMetrics metrics_;
    LocalizationQualityGates qualityGates_;
    std::set<RobotId> seenRobots_;
    bool debugEnabled_{true}, frameLoggingEnabled_{true};
    std::uint64_t statisticsEpoch_{};
    Timestamp pendingEnqueued_{};
    std::shared_ptr<const std::string> calibrationRevision_;
    std::shared_ptr<IMarkerDetector> detector_;
    std::shared_ptr<IMetricObservationConverter> metricConverter_;
    std::optional<WorkspaceCalibrationProfile> workspace_;
    std::optional<WorkspaceCalibrationProfile> referenceDraft_;
    std::uint64_t referenceDraftGeneration_{};
    // Worker-only acquisition cadence; no separate image queue.
    std::uint64_t workerReferenceGeneration_{};
    std::optional<Timestamp> workerReferenceCapture_;
    std::optional<PlacementReport> lastPlacement_;
    QTimer referenceWatchdog_;
    bool referenceExpired_{};
    QThread worker_;
    QObject* workerContext_{}; // Deleted on worker completion.
    FramePtr pending_;
    QString sourceId_;
    QString description_{QStringLiteral("AprilTag: waiting for camera frames")};
    std::optional<std::uint64_t> lastSequence_;
    Timestamp lastCapture_{};
    std::uint32_t width_{}, height_{};
    std::uint64_t generation_{};
    bool streaming_{}, busy_{}, shuttingDown_{};
};
} // namespace micro_robot
Q_DECLARE_METATYPE(micro_robot::MarkerDetectionsPtr)

Q_DECLARE_METATYPE(micro_robot::VisionDebugFramePtr)
Q_DECLARE_METATYPE(micro_robot::MetricObservationBatchPtr)
Q_DECLARE_METATYPE(micro_robot::ReferenceFramePtr)
