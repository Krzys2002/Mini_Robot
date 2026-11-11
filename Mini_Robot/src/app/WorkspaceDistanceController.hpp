#pragma once
#include "calibration/WorkspaceDistance.hpp"
#include "camera/CameraTypes.hpp"
#include <QObject>
#include <QString>
namespace micro_robot
{
class WorkspaceCalibrationController;
// GUI-thread endpoint state; transforms/prediction live in the pure calibration core.
// Owns no worker or raw camera buffer. World endpoints survive view-only changes.
class WorkspaceDistanceController final : public QObject
{
    Q_OBJECT
public:
    explicit WorkspaceDistanceController(WorkspaceCalibrationController&, QObject* parent = nullptr);
    bool pickImage(std::size_t index, ImageCorner);
    bool setWorld(std::size_t index, WorldPoint2D);
    void setSelection(std::size_t index, WorkspaceEndpointSelection);
    void clear(std::optional<std::size_t> index = {});
    void freeze();
    [[nodiscard]] const auto& endpoints() const { return endpoints_; }
    [[nodiscard]] const auto& measurement() const { return measurement_; }
    [[nodiscard]] const auto& geometry() const { return geometry_; }
    [[nodiscard]] FramePtr frame() const;
    [[nodiscard]] bool canFreeze() const;
    [[nodiscard]] QString message() const { return message_; }

private:
    void recompute();
    void calibrationChanged();
    WorkspaceCalibrationController& source_;
    std::shared_ptr<const WorkspaceDiagnostics> geometry_;
    std::array<std::optional<WorkspaceDistanceEndpoint>, WorkspaceDistanceEndpointCount> endpoints_;
    WorkspaceDistanceOutcome measurement_;
    QString message_{QStringLiteral(
        "Freeze an undistorted camera frame, then select A and B. Selection bounds default to unknown.")};
    bool pendingFreeze_{}, shownCanFreeze_{};
signals:
    void changed();
};
} // namespace micro_robot
