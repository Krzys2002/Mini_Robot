#pragma once

#include "calibration/CalibrationCapture.hpp"
#include "infrastructure/logging/Logger.hpp"

#include <QObject>
#include <QThread>
#include <QString>

#include <memory>

namespace micro_robot {

// GUI-thread application adapter. One worker job at a time; only the latest
// camera frame is retained between capture requests. Detector is injected and
// used exclusively on the worker. Call shutdown() and await stopped() before
// destruction in production, so joining the worker never blocks the GUI.
class CalibrationController final : public QObject {
    Q_OBJECT
public:
    CalibrationController(std::unique_ptr<ICalibrationDetector> detector, LogChannel log,
                          CalibrationCaptureConfig config = {}, QObject* parent = nullptr);
    ~CalibrationController() override;
    void setInput(const QString& sourceId, bool streaming);
    void receiveFrame(FramePtr frame);
    [[nodiscard]] const CalibrationCapture& session() const noexcept { return session_; }
    [[nodiscard]] const QString& message() const noexcept { return message_; }
    [[nodiscard]] bool busy() const noexcept { return busy_; }
    [[nodiscard]] bool canCapture() const;
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
    std::uint64_t generation_{};
    bool streaming_{};
    bool busy_{};
    bool live_{true};
    bool shuttingDown_{};
};
} // namespace micro_robot
