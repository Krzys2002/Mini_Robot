#pragma once

#include "calibration/IntrinsicCalibration.hpp"
#include "infrastructure/logging/Logger.hpp"

#include <QObject>
#include <QThread>
#include <QString>

#include <memory>

namespace micro_robot {

// GUI-thread application adapter. One worker job at a time; only the latest
// camera frame is retained between capture requests. Injected detector and solver
// are used exclusively on the worker. Call shutdown() and await stopped() before
// destruction in production, so joining the worker never blocks the GUI.
class CalibrationController final : public QObject {
    Q_OBJECT
public:
    CalibrationController(std::unique_ptr<ICalibrationDetector> detector,
                          std::unique_ptr<IIntrinsicCalibrationSolver> solver, LogChannel log,
                          CalibrationCaptureConfig config = {},
                          IntrinsicCalibrationConfig intrinsicConfig = {}, QObject* parent = nullptr);
    ~CalibrationController() override;
    void setInput(const QString& sourceId, bool streaming);
    void receiveFrame(FramePtr frame);
    [[nodiscard]] const CalibrationCapture& session() const noexcept { return session_; }
    [[nodiscard]] const QString& message() const noexcept { return message_; }
    [[nodiscard]] bool busy() const noexcept { return busy_; }
    [[nodiscard]] bool canCapture() const;
    // GUI-thread access; copy the validated value before use on another worker.
    // Cleared on restart, cancellation, source/resolution change or source loss.
    [[nodiscard]] const std::optional<IntrinsicCalibration>& result() const noexcept { return result_; }
    [[nodiscard]] const IntrinsicCalibrationConfig& intrinsicConfig() const noexcept { return intrinsicConfig_; }
    [[nodiscard]] bool isStopped() const { return worker_.isFinished(); }

public slots:
    void restart();
    void capture();
    void removeView(int index);
    void reviewView(int index);
    void showLive();
    void finish();
    void cancel();
    void shutdown();

signals:
    void changed();
    // Same-thread presentation signals. Frame storage remains owned/immutable.
    void previewChanged(FramePtr frame, const std::vector<ImageCorner>& corners, bool accepted);
    void stopped();

private:
    void publish(const QString& message);
    void invalidate();
    CalibrationCapture session_;
    LogChannel log_;
    QString sourceId_;
    QString message_{QStringLiteral("Select and start a camera, then start capture.")};
    FramePtr latest_;
    QThread worker_;
    QObject* workerContext_{}; // Deleted on worker completion.
    std::shared_ptr<ICalibrationDetector> detector_;
    std::shared_ptr<IIntrinsicCalibrationSolver> solver_;
    IntrinsicCalibrationConfig intrinsicConfig_;
    std::optional<IntrinsicCalibration> result_;
    std::uint64_t generation_{};
    bool streaming_{};
    bool busy_{};
    bool live_{true};
    bool shuttingDown_{};
};
} // namespace micro_robot
