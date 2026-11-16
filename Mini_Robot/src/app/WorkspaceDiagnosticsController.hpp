#pragma once
#include "calibration/WorkspaceDiagnostics.hpp"
#include <QObject>
#include <QString>
#include <QThread>
#include <functional>

namespace micro_robot
{
// GUI-thread service; one running job and one replaceable pending map request.
// Owns immutable active-calibration snapshots; no camera-frame backlog.
class WorkspaceDiagnosticsController final : public QObject
{
    Q_OBJECT
public:
    using Builder = std::function<WorkspaceDiagnosticsOutcome(
        WorkspaceCalibrationProfile, std::shared_ptr<const WorkspaceAcquisitionRecord>,
        WorkspaceSurveyEvidence, std::size_t, std::size_t)>;
    explicit WorkspaceDiagnosticsController(QObject* parent = nullptr,
                                            Builder builder = buildWorkspaceDiagnostics);
    ~WorkspaceDiagnosticsController() override;
    void setCalibration(std::optional<WorkspaceCalibrationProfile>,
                        std::shared_ptr<const WorkspaceAcquisitionRecord> = {});
    void request(WorkspaceSurveyEvidence = {}, std::size_t columns = DefaultWorkspaceMapColumns,
                 std::size_t rows = DefaultWorkspaceMapRows);
    void exportSnapshot(const QString& path);
    void shutdown();
    [[nodiscard]] const auto& snapshot() const { return snapshot_; }
    [[nodiscard]] const auto& profile() const { return profile_; }
    [[nodiscard]] const auto& acquisition() const { return acquisition_; }
    [[nodiscard]] const auto& evidence() const { return evidence_; }
    [[nodiscard]] bool busy() const { return running_; }
    [[nodiscard]] bool isStopped() const { return worker_.isFinished(); }
    [[nodiscard]] QString message() const { return message_; }
signals:
    void changed();
    void calibrationChanged();
    void stopped();

private:
    struct Task {
        WorkspaceCalibrationProfile profile;
        std::shared_ptr<const WorkspaceAcquisitionRecord> acquisition;
        WorkspaceSurveyEvidence evidence;
        std::size_t columns{}, rows{};
        std::uint64_t generation{};
    };
    void launch(Task);
    void finished();
    Builder builder_;
    QThread worker_;
    QObject* context_{};
    std::optional<WorkspaceCalibrationProfile> profile_;
    std::shared_ptr<const WorkspaceAcquisitionRecord> acquisition_;
    std::shared_ptr<const WorkspaceDiagnostics> snapshot_;
    WorkspaceSurveyEvidence evidence_;
    std::optional<Task> pending_;
    std::uint64_t generation_{};
    bool running_{}, shuttingDown_{};
    QString message_{QStringLiteral("Activate a compatible workspace transform to inspect maps.")};
};
// Worker-only canonical JSON export; atomic replacement and bounded bytes.
[[nodiscard]] std::string exportWorkspaceDiagnostics(const WorkspaceDiagnostics&, const QString& path);
} // namespace micro_robot
