#pragma once
#include "vision/VisionDebugFrame.hpp"
#include "infrastructure/logging/Logger.hpp"

#include <QObject>
#include <QString>
#include <QThread>

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
    ~VisionController() override;
    void setInput(const QString& sourceId, bool streaming);
    void receiveFrame(FramePtr frame);
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
    void previewAvailable(const QString& sourceId, VisionDebugFramePtr preview);
    void descriptionChanged(const QString& description);
    void stopped();
private:
    void invalidate();
    void dispatch();
    void describe(const QString& text);
    LogChannel log_;
    RobotMarkerMap markerMap_;
    VisionOutputRate outputRate_;
    std::shared_ptr<IMarkerDetector> detector_;
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
