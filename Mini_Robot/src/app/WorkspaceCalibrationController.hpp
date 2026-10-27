#pragma once
#include "calibration/TabletopHomography.hpp"
#include "calibration/WorkspaceProfile.hpp"
#include "infrastructure/logging/Logger.hpp"
#include <QObject>
#include <QThread>
#include <QString>
#include <memory>

namespace micro_robot {
// GUI-thread application service. Undistortion, estimation and disk I/O share
// one serialized worker. Input/configuration changes invalidate late results.
class WorkspaceCalibrationController final : public QObject {
    Q_OBJECT
public:
    WorkspaceCalibrationController(std::unique_ptr<IWorkspaceHomographySolver> solver,
        std::unique_ptr<ICalibrationUndistorter> undistorter,
        std::unique_ptr<IWorkspaceProfileStore> store, LogChannel log, QObject* parent = nullptr);
    ~WorkspaceCalibrationController() override;
    void setInput(std::optional<IntrinsicCalibration> intrinsics, bool persistent);
    void receiveFrame(FramePtr frame);
    void configure(QString workspaceId, double widthMeters, double heightMeters);
    void setPoints(std::vector<WorkspaceCorrespondence> fit, std::vector<WorkspaceCorrespondence> check);
    [[nodiscard]] const std::optional<WorkspaceCalibrationProfile>& profile() const { return profile_; }
    [[nodiscard]] FramePtr frozenFrame() const { return frozen_; }
    [[nodiscard]] QString message() const { return message_; }
    [[nodiscard]] bool busy() const { return busy_; }
    [[nodiscard]] bool canFreeze() const { return !shuttingDown_ && !busy_ && intrinsics_ && latest_; }
    [[nodiscard]] bool canCalculate() const { return !shuttingDown_ && !busy_ && frozen_ && intrinsics_ && bounds_ && !workspaceId_.isEmpty(); }
    [[nodiscard]] bool canSave() const { return !shuttingDown_ && !busy_ && persistent_ && profile_ && profile_->homography; }
    [[nodiscard]] bool canLoad() const { return !shuttingDown_ && !busy_ && persistent_ && intrinsics_ && !workspaceId_.isEmpty(); }
    [[nodiscard]] bool isStopped() const { return worker_.isFinished(); }
public slots:
    void freeze();
    void calculate();
    void save();
    void load();
    void shutdown();
signals:
    void changed();
    void profileChanged();
    void stopped();
private:
    void invalidate(bool clearFrame);
    void publish(QString message);
    std::shared_ptr<IWorkspaceHomographySolver> solver_;
    std::shared_ptr<ICalibrationUndistorter> undistorter_;
    std::shared_ptr<IWorkspaceProfileStore> store_;
    LogChannel log_;
    QThread worker_;
    QObject* workerContext_{}; // Deleted on worker completion.
    std::optional<IntrinsicCalibration> intrinsics_;
    std::optional<WorkspaceBounds> bounds_;
    std::optional<WorkspaceCalibrationProfile> profile_;
    QString workspaceId_;
    FramePtr latest_, frozen_;
    std::vector<WorkspaceCorrespondence> fit_, check_;
    QString message_{QStringLiteral("Load or calculate compatible intrinsics, then freeze an undistorted frame.")};
    std::uint64_t generation_{};
    bool persistent_{}, busy_{}, shuttingDown_{};
};
}
