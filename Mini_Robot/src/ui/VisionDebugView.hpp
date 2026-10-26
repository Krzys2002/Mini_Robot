#pragma once

#include "vision/VisionDebugFrame.hpp"

#include <QDialog>
#include <QTimer>

class QPlainTextEdit;
namespace micro_robot {
class VisionController;
// GUI-thread presentation only. Paints one immutable, matched raw-frame/overlay
// snapshot; QImage borrows its retained storage only during painting.
class VisionOverlay final : public QWidget {
    Q_OBJECT
public:
    explicit VisionOverlay(QWidget* parent = nullptr);
    void present(VisionDebugFramePtr preview);
    [[nodiscard]] VisionDebugFramePtr preview() const { return preview_; }
    [[nodiscard]] QRectF imageRect() const;
    // Detector coordinates address pixel centers; QImage's rectangle addresses
    // pixel edges. Include the half-pixel offset under the same uniform scale.
    [[nodiscard]] QPointF imageToView(ImagePoint point) const;
protected:
    void paintEvent(QPaintEvent*) override;
private:
    VisionDebugFramePtr preview_;
};
class VisionDebugDialog final : public QDialog {
    Q_OBJECT
public:
    explicit VisionDebugDialog(VisionController& vision, QWidget* parent = nullptr);
protected:
    void showEvent(QShowEvent* event) override;
    void hideEvent(QHideEvent* event) override;
private:
    void present(const QString& sourceId, VisionDebugFramePtr preview);
    void refreshDiagnostics();
    VisionController& vision_; // Must outlive this dialog, like other injected UI services.
    VisionOverlay* overlay_{};
    QPlainTextEdit* diagnostics_{};
    QString sourceId_;
    VisionDebugFramePtr preview_;
    QTimer ageTimer_;
};
} // namespace micro_robot
