#pragma once

#include "calibration/IntrinsicCalibration.hpp"
#include "world/Workspace.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <string>

namespace micro_robot {
inline constexpr std::uint32_t WorkspaceCalibrationProfileVersion = 1;
enum class WorkspaceImageCoordinates { UndistortedNativeCameraMatrix };
// Row-major H, [x',y',w'] = H*[u,v,1], world metres = (x'/w', y'/w').
// Input is undistorted pixels using the bound intrinsic K, at its native size,
// without crop, resize or display mirroring. Estimation/application belongs to #57/#58.
struct WorkspaceHomography {
    std::array<double, 9> imageToWorld{};
};
// Separate from lens intrinsics: changing camera placement/table requires a new
// workspace calibration even when the lens model remains valid. No I/O or solver.
// Deliberately not default-constructible: parse/validate dimensions before
// aggregate construction; no placeholder or assumed physical table size.
struct WorkspaceCalibrationProfile {
    std::uint32_t version{WorkspaceCalibrationProfileVersion};
    std::string workspaceId;
    WorkspaceBounds bounds;
    // Owned exact camera/device, resolution, K and distortion binding. A later
    // recalibration cannot silently alter the pixel domain of a stored transform.
    IntrinsicCalibration intrinsics;
    WorkspaceImageCoordinates imageCoordinates{WorkspaceImageCoordinates::UndistortedNativeCameraMatrix};
    std::optional<WorkspaceHomography> homography{}; // Absent means uncalibrated.
};
// Empty error means structurally valid metadata (including an optional finite,
// numerically nonsingular H), not physical accuracy or permission to control a robot.
[[nodiscard]] std::string validateWorkspaceCalibrationProfile(const WorkspaceCalibrationProfile& profile);
// Rejects camera/resolution or optical-model changes; never rescales a transform.
// Optical doubles compare exactly. Persistence must round-trip them losslessly
// (e.g. max_digits10 for decimal text), rather than relaxing compatibility.
[[nodiscard]] std::string workspaceCalibrationCompatibilityError(
    const WorkspaceCalibrationProfile& profile, const IntrinsicCalibration& activeIntrinsics);
} // namespace micro_robot
