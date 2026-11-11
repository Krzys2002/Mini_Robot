#pragma once
#include "calibration/CalibrationProfile.hpp"
#include "calibration/WorkspaceCalibration.hpp"
#include <memory>

namespace micro_robot {
struct WorkspaceAcquisitionRecord;
inline constexpr std::size_t MaximumWorkspaceProfileBytes = 131072;

struct WorkspaceProfileLoadOutcome {
    ProfileLoadState state{ProfileLoadState::Missing};
    std::optional<WorkspaceCalibrationProfile> profile;
    std::string error;
    std::shared_ptr<WorkspaceAcquisitionRecord> acquisition{};
};

// Strict locale-independent v3 writer and v1/v2/v3 reader; lossless frame/optical/H doubles.
// Serialization throws invalid_argument for invalid or oversized metadata.
[[nodiscard]] std::string serializeWorkspaceProfile(const WorkspaceCalibrationProfile& profile);
[[nodiscard]] WorkspaceProfileLoadOutcome parseWorkspaceProfile(std::string_view text);

class IWorkspaceProfileStore {
public:
    virtual ~IWorkspaceProfileStore() = default;
    // Worker-only, serialized access; exact workspace/camera/resolution/optics.
    // Missing differs from Rejected. Atomic save preserves the old file on error.
    [[nodiscard]] virtual WorkspaceProfileLoadOutcome load(std::string_view workspaceId,
                                                           const IntrinsicCalibration& active) = 0;
    [[nodiscard]] virtual std::string save(const WorkspaceCalibrationProfile& profile) = 0;
    [[nodiscard]] virtual std::string saveAcquisition(const WorkspaceCalibrationProfile&,
                                                     const WorkspaceAcquisitionRecord&)
    { return "This store cannot persist acquisition provenance."; }
};
} // namespace micro_robot
