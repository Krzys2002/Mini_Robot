#pragma once

#include "calibration/CalibrationProfile.hpp"
#include "calibration/CalibrationCornerTracker.hpp"
#include "infrastructure/logging/Logger.hpp"

#include <QObject>
#include <QThread>
#include <QTimer>
#include <QString>

#include <chrono>
#include <functional>
#include <memory>

namespace micro_robot {
enum class CalibrationLivePreviewMode { Hold = 0, Track = 1, Matched = 2 };

// GUI-thread adapter. Detection/solving/store I/O use one worker; optical flow
// and remapping share another. Preview can hold/track overlays on camera-rate
// video or advance matched images/corners at detection rate. Capture uses only a
// full detection of its exact
// raw frame. Call shutdown() and await stopped() before destruction.
class CalibrationController final : public QObject {
    Q_OBJECT
public:
    static constexpr auto CaptureReviewDuration = std::chrono::seconds{5};
    static constexpr auto MaximumLiveFrameAge = std::chrono::seconds{2};

    // Optional review clock is called on the GUI thread and must be monotonic.
    CalibrationController(std::unique_ptr<ICalibrationDetector> detector,
                          std::unique_ptr<IIntrinsicCalibrationSolver> solver, LogChannel log,
                          CalibrationCaptureConfig config = {},
                          IntrinsicCalibrationConfig intrinsicConfig = {}, QObject* parent = nullptr,
                          std::function<Timestamp()> clock = [] { return MonotonicClock::now(); });
    ~CalibrationController() override;
    // Inject once before input; used exclusively on the preview worker.
    void setLiveTracker(std::unique_ptr<ICalibrationCornerTracker> tracker);
    [[nodiscard]] bool canTrackLiveOverlay() const { return tracker_ && !shuttingDown_; }
    [[nodiscard]] CalibrationLivePreviewMode livePreviewMode() const noexcept { return previewMode_; }
    [[nodiscard]] bool liveTrackingBusy() const noexcept { return trackingBusy_; }
    // Inject once before camera input. Store I/O uses the capture worker;
    // undistortion uses the preview worker.
    void setProfileServices(std::unique_ptr<ICalibrationProfileStore> store,
                            std::unique_ptr<ICalibrationUndistorter> undistorter);
    [[nodiscard]] const std::optional<IntrinsicCalibration>& profile() const noexcept { return profile_; }
    [[nodiscard]] const QString& profileMessage() const noexcept { return profileMessage_; }
    [[nodiscard]] bool persistentIdentity() const noexcept { return persistentIdentity_; }
    [[nodiscard]] bool profileBusy() const noexcept { return profileIoBusy_; }
    [[nodiscard]] bool canSaveProfile() const { return persistentIdentity_ && profileStore_ && profile_ && !profileIoBusy_ && !shuttingDown_; }
    [[nodiscard]] bool canReloadProfile() const { return persistentIdentity_ && profileStore_ && latest_ && !profileIoBusy_ && !shuttingDown_; }
    [[nodiscard]] bool canUndistort() const { return undistorter_ && profile_ && latest_ && !shuttingDown_; }
    [[nodiscard]] bool undistortedPreview() const noexcept { return undistortedPreview_; }
    // Callers must explicitly attest persistence; the default is session-only.
    void setInput(const QString& sourceId, bool streaming, bool persistentIdentity = false);
    void receiveFrame(FramePtr frame);
    [[nodiscard]] const CalibrationCapture& session() const noexcept { return session_; }
    [[nodiscard]] const QString& message() const noexcept { return message_; }
    [[nodiscard]] const QString& captureFeedback() const noexcept { return captureFeedback_; }
    [[nodiscard]] bool busy() const noexcept { return busy_; }
    [[nodiscard]] bool canCapture() const;
    // GUI-thread access; copy the validated value before use on another worker.
    // Cleared on restart, cancellation, source/resolution change or source loss.
    [[nodiscard]] const std::optional<IntrinsicCalibration>& result() const noexcept { return result_; }
    [[nodiscard]] const IntrinsicCalibrationConfig& intrinsicConfig() const noexcept { return intrinsicConfig_; }
    [[nodiscard]] bool isStopped() const { return worker_.isFinished() && liveWorker_.isFinished(); }

public slots:
    void saveProfile();
    void reloadProfile();
    void setUndistortedPreview(bool enabled);
    void restart();
    void capture();
    void removeView(int index);
    void reviewView(int index);
    void showLive();
    // Wizard visibility controls background work; hidden dialogs never sample.
    void setLivePreviewActive(bool active);
    void setLivePreviewMode(CalibrationLivePreviewMode mode);
    void finish();
    void cancel();
    void shutdown();

signals:
    void changed();
    // Same-thread presentation signals. Frame storage remains owned/immutable.
    void previewChanged(FramePtr frame, const std::vector<ImageCorner>& corners, bool accepted, bool approximate = false);
    // Reusable non-modal event; retained by captureFeedback() across live transitions.
    void captureFeedbackAvailable(const QString& reason, bool accepted);
    void stopped();

private:
    void publish(const QString& message);
    void cancelReview();
    void emitPreview(FramePtr frame, const std::vector<ImageCorner>& corners, bool accepted, bool approximate);
    void expireReview();
    std::function<Timestamp()> clock_;
    QTimer reviewTimer_;
    std::optional<Timestamp> reviewDeadline_;
    FramePtr reviewFrame_;
    std::uint64_t reviewRevision_{};
    QString captureFeedback_;
    void invalidate();
    void presentPreview(FramePtr frame, const std::vector<ImageCorner>& corners, bool accepted, bool approximate = false);
    void processPreview();
    void sampleLive();
    void resetLiveOverlay();
    void trackLive();
    void resetTracking();
    void acceptCapture(FramePtr frame, CalibrationDetection detection, std::uint64_t reviewRevision);
    CalibrationCapture session_;
    LogChannel log_;
    QString sourceId_;
    QString message_{QStringLiteral("Select and start a camera, then start capture.")};
    FramePtr latest_;
    QThread worker_;
    bool stoppedEmitted_{};
    QThread liveWorker_; // Optical flow and preview remap; never capture/solve.
    QObject* liveContext_{};
    std::shared_ptr<ICalibrationCornerTracker> tracker_;
    CalibrationLivePreviewMode previewMode_{CalibrationLivePreviewMode::Hold};
    bool trackingBusy_{};
    std::uint64_t trackingRevision_{};
    FramePtr trackingFrame_;
    std::vector<ImageCorner> trackingCorners_;
    Timestamp lastFullDetection_{};
    std::vector<ImageCorner> liveCorners_;
    bool liveAccepted_{};
    QObject* workerContext_{}; // Deleted on worker completion.
    std::shared_ptr<ICalibrationDetector> detector_;
    std::shared_ptr<IIntrinsicCalibrationSolver> solver_;
    IntrinsicCalibrationConfig intrinsicConfig_;
    std::optional<IntrinsicCalibration> result_;
    std::uint64_t generation_{};
    bool streaming_{};
    bool persistentIdentity_{};
    bool busy_{};
    bool live_{true};
    bool livePreviewActive_{};
    bool liveDetectionBusy_{};
    std::uint64_t liveEpoch_{};
    Timestamp nextLiveSample_{};
    FramePtr detectedFrame_;
    std::optional<CalibrationDetection> liveDetection_;
    bool shuttingDown_{};
    std::shared_ptr<ICalibrationProfileStore> profileStore_;
    std::shared_ptr<ICalibrationUndistorter> undistorter_;
    std::optional<IntrinsicCalibration> profile_;
    QString profileMessage_{QStringLiteral("No saved profile selected.")};
    std::uint64_t profileGeneration_{}; // Camera/resolution validity, independent of capture restarts.
    bool profileIoBusy_{};
    bool reloadPending_{};
    bool profileLookupRequested_{};
    bool undistortedPreview_{};
    bool previewBusy_{};
    std::uint64_t previewEpoch_{};
    std::uint64_t previewRevision_{};
    FramePtr previewFrame_;
    FramePtr selectedFrame_; // Raw source of the last actually emitted presentation.
    std::vector<ImageCorner> previewCorners_;
    bool previewAccepted_{};
    bool previewApproximate_{};
};
} // namespace micro_robot
