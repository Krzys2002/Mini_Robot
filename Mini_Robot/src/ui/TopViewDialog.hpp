#pragma once

#include "calibration/CalibrationProfile.hpp"
#include "calibration/WorkspaceCalibration.hpp"

#include <QDialog>
#include <QImage>
#include <QThread>
#include <QTimer>

class QLabel;

namespace micro_robot {
class TopViewImage;

// Worker-safe rendering of one undistorted native-K frame. The picture follows
// the existing metric axes (+X right, +Y up), with the table centered and cropped
// to its calibrated bounds. No pixels or coordinates are published to perception.
[[nodiscard]] QImage renderTopView(const Frame&, const WorkspaceBounds&, const WorkspaceHomography&);

class TopViewDialog final : public QDialog {
    Q_OBJECT
public:
    explicit TopViewDialog(std::unique_ptr<ICalibrationUndistorter> undistorter, QWidget* parent = nullptr);
    ~TopViewDialog() override;

    void setCalibration(const std::optional<WorkspaceCalibrationProfile>& profile);
    void setInput(const QString& calibrationIdentity, bool streaming);
    void receiveFrame(FramePtr frame);

protected:
    void showEvent(QShowEvent*) override;
    void hideEvent(QHideEvent*) override;

private:
    struct Calibration {
        WorkspaceBounds bounds;
        IntrinsicCalibration intrinsics;
        WorkspaceHomography homography;
    };
    void resetPreview();
    void tick();
    [[nodiscard]] bool compatible() const;

    static constexpr auto PreviewInterval = std::chrono::milliseconds{150};
    std::unique_ptr<ICalibrationUndistorter> undistorter_;
    QThread worker_;
    QObject* workerContext_{};
    QTimer timer_;
    std::optional<Calibration> calibration_;
    QString input_;
    FramePtr pending_;
    std::optional<Timestamp> latestAt_;
    std::optional<Timestamp> displayedAt_;
    std::uint64_t generation_{};
    bool streaming_{};
    bool running_{};
    TopViewImage* image_{};
    QLabel* status_{};
};
} // namespace micro_robot
