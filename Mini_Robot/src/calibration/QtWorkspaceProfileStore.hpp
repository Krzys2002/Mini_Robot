#pragma once
#include "calibration/WorkspaceProfile.hpp"
#include "calibration/WorkspaceAcquisition.hpp"
#include <QString>

namespace micro_robot {
class QtWorkspaceProfileStore final : public IWorkspaceProfileStore {
public:
    explicit QtWorkspaceProfileStore(QString directory);
    [[nodiscard]] WorkspaceProfileLoadOutcome load(std::string_view workspaceId,
                                                   const IntrinsicCalibration& active) override;
    [[nodiscard]] std::string save(const WorkspaceCalibrationProfile& profile) override;

    [[nodiscard]] std::string saveAcquisition(const WorkspaceCalibrationProfile& profile,
                                             const WorkspaceAcquisitionRecord& record) override;
    [[nodiscard]] WorkspaceProfileLoadOutcome loadFile(std::string_view file, const IntrinsicCalibration& active) override;
    [[nodiscard]] std::string saveFile(std::string_view file, const WorkspaceCalibrationProfile& profile,
                                       const WorkspaceAcquisitionRecord* record) override;
private:
    [[nodiscard]] static QString acquisitionPath(const QString& file, const WorkspaceCalibrationProfile& profile);
    [[nodiscard]] static WorkspaceAcquisitionLoadOutcome readAcquisition(const QString& file, const WorkspaceCalibrationProfile& profile);
    [[nodiscard]] static WorkspaceProfileLoadOutcome readProfile(const QString& file, const IntrinsicCalibration& active,
                                                                  std::optional<std::string_view> workspace);
    [[nodiscard]] static std::string writeProfile(const QString& file, const WorkspaceCalibrationProfile& profile);
    [[nodiscard]] static std::string writeAcquisition(const QString& file, const WorkspaceCalibrationProfile& profile,
                                                      const WorkspaceAcquisitionRecord& record);
    static constexpr int StoreLockWaitMilliseconds = 0;
    [[nodiscard]] QString path(std::string_view workspaceId, const IntrinsicCalibration& active) const;
    QString directory_;
};
} // namespace micro_robot
