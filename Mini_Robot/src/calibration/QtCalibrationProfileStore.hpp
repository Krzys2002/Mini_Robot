#pragma once
#include "calibration/CalibrationProfile.hpp"
#include <QString>

namespace micro_robot {
// Filesystem adapter. Directory is injected by composition, never a global path.
class QtCalibrationProfileStore final : public ICalibrationProfileStore {
public:
    explicit QtCalibrationProfileStore(QString directory);
    [[nodiscard]] ProfileLoadOutcome load(std::string_view sourceId,
        std::uint32_t width, std::uint32_t height) override;
    [[nodiscard]] std::string save(const IntrinsicCalibration& calibration) override;
private:
    [[nodiscard]] QString path(std::string_view sourceId, std::uint32_t width, std::uint32_t height) const;
    QString directory_;
};
} // namespace micro_robot
