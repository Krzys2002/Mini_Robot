#pragma once
#include "calibration/CalibrationProfile.hpp"
#include "calibration/WorkspaceCalibration.hpp"
#include <memory>

// Boundary/origin selection defines the tabletop z=0 world frame, not camera
// extrinsics or elevated robot poses. #270 owns height/mount conversion; #271
// owns reference monitoring outside robot travel bounds. Bounds remain travel geometry.
namespace micro_robot {
struct WorkspaceAcquisitionRecord;
inline constexpr std::size_t MaximumWorkspaceProfileBytes = 131072;

struct WorkspaceProfileLoadOutcome {
    ProfileLoadState state{ProfileLoadState::Missing};
    std::optional<WorkspaceCalibrationProfile> profile;
    std::string error;
    std::shared_ptr<const WorkspaceAcquisitionRecord> acquisition{};
};

// Strict locale-independent v3/v4/v5 writer and v1–v5 reader; lossless frame/optical/H doubles.
// Serialization throws invalid_argument for invalid or oversized metadata.
[[nodiscard]] std::string serializeWorkspaceProfile(const WorkspaceCalibrationProfile& profile);
[[nodiscard]] WorkspaceProfileLoadOutcome parseWorkspaceProfile(std::string_view text);

class IWorkspaceProfileStore {
public:
    virtual ~IWorkspaceProfileStore() = default;
    // Worker-only, serialized access; exact workspace/camera/resolution/optics.
    // Missing differs from Rejected. Loaded record snapshots are immutable.
    // Atomic save preserves the old file on error.
    [[nodiscard]] virtual WorkspaceProfileLoadOutcome load(std::string_view workspaceId,
                                                           const IntrinsicCalibration& active) = 0;
    [[nodiscard]] virtual std::string save(const WorkspaceCalibrationProfile& profile) = 0;
    [[nodiscard]] virtual std::string saveAcquisition(const WorkspaceCalibrationProfile&,
                                                     const WorkspaceAcquisitionRecord&)
    { return "This store cannot persist acquisition provenance."; }
    // Named files carry their workspace identity and retain linked acquisition data.
    [[nodiscard]] virtual WorkspaceProfileLoadOutcome loadFile(std::string_view, const IntrinsicCalibration&)
    { return {ProfileLoadState::Rejected, {}, "This store cannot load a named workspace file."}; }
    [[nodiscard]] virtual std::string saveFile(std::string_view, const WorkspaceCalibrationProfile&,
                                               const WorkspaceAcquisitionRecord*)
    { return "This store cannot save a named workspace file."; }
};
} // namespace micro_robot
