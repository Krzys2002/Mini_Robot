#pragma once
#include "calibration/CalibrationProfile.hpp"
#include "calibration/WorkspaceCalibration.hpp"

// Boundary/origin selection defines the tabletop z=0 world frame, not camera
// extrinsics or elevated robot poses. #270 owns height/mount conversion; #271
// owns reference monitoring outside robot travel bounds. Bounds remain travel geometry.
namespace micro_robot {
inline constexpr std::size_t MaximumWorkspaceProfileBytes = 131072;

struct WorkspaceProfileLoadOutcome {
    ProfileLoadState state{ProfileLoadState::Missing};
    std::optional<WorkspaceCalibrationProfile> profile;
    std::string error;
};

// Strict locale-independent version 1 format; lossless optical/H doubles.
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
};
} // namespace micro_robot
