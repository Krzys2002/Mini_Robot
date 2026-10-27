#pragma once
#include "calibration/WorkspaceProfile.hpp"
#include <QString>
namespace micro_robot {
class QtWorkspaceProfileStore final : public IWorkspaceProfileStore {
public:
    explicit QtWorkspaceProfileStore(QString directory);
    [[nodiscard]] WorkspaceProfileLoadOutcome load(std::string_view workspaceId, const IntrinsicCalibration& active) override;
    [[nodiscard]] std::string save(const WorkspaceCalibrationProfile& profile) override;
private:
    [[nodiscard]] QString path(std::string_view workspaceId, const IntrinsicCalibration& active) const;
    QString directory_;
};
}
