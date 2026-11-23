#pragma once

#include "calibration/WorkspaceAcquisition.hpp"
#include "camera/CameraTypes.hpp"

#include <QTransform>
#include <QWidget>

#include <functional>
#include <optional>
#include <utility>
#include <vector>

namespace micro_robot {

inline constexpr std::size_t MaximumWorkspaceOverlayReferences = 2048;
inline constexpr int MaximumWorkspaceOverlayReferenceLabels = 64;

// Shared workspace and tabletop image canvas. All overlays and point callbacks
// use the frozen frame's native pixel centres.
class WorkspaceImageView final : public QWidget {
    Q_OBJECT
public:
    explicit WorkspaceImageView(QWidget* parent = nullptr);
    QWidget* createControls(QWidget* parent);
    void setFrame(FramePtr frame);

    std::vector<std::pair<ImageCorner, bool>> markers;
    std::vector<ImageCorner> corners;
    std::optional<ImageCorner> origin;
    std::vector<ImageCorner> board;
    std::size_t boardAnchor{};
    std::function<void(ImageCorner)> clicked;

signals:
    void frameAvailabilityChanged(bool available);

protected:
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void paintEvent(QPaintEvent*) override;
    void resizeEvent(QResizeEvent* event) override;

private:
    [[nodiscard]] QSizeF scaledImageSize() const;
    [[nodiscard]] QTransform imageToView() const;
    void setZoom(double zoom, QPointF anchor);
    void fitImage();
    void constrainPan();
    void updateCursor();

    static constexpr double MinimumZoom = 1.0;
    static constexpr double MaximumZoom = 16.0;
    static constexpr double ZoomStep = 1.25;
    static constexpr double WheelUnitsPerStep = 120.0;
    static constexpr double TrackpadPixelsPerStep = 120.0;
    static constexpr std::size_t BoardColumns = WorkspaceBoardTarget.columns;
    static constexpr std::size_t BoardRows = WorkspaceBoardTarget.rows;
    FramePtr frame_;
    double zoom_{MinimumZoom};
    QPointF pan_;
    QPointF lastDragPosition_;
    Qt::MouseButton dragButton_{Qt::NoButton};
    bool panMode_{};
    bool horizontalFlip_{};
    bool verticalFlip_{};
};

} // namespace micro_robot
