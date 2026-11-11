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
private:
    [[nodiscard]] QString acquisitionPath(const WorkspaceCalibrationProfile& profile) const;
    [[nodiscard]] WorkspaceAcquisitionLoadOutcome readAcquisition(const WorkspaceCalibrationProfile& profile) const;
    static constexpr int StoreLockWaitMilliseconds = 0;
    [[nodiscard]] QString path(std::string_view workspaceId, const IntrinsicCalibration& active) const;
    QString directory_;
};
} // namespace micro_robot
