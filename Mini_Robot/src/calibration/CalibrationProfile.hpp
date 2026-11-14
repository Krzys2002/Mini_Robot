#pragma once
#include "calibration/IntrinsicCalibration.hpp"

#include <string_view>

namespace micro_robot {
inline constexpr std::size_t MaximumCalibrationProfileBytes = 65536;
// Version 1: Brown-Conrady pinhole K, k1,k2,p1,p2,k3; pixel RMS and per-view metadata.
// Locale-independent, strict format. Serialization throws invalid_argument for invalid models.
[[nodiscard]] std::string serializeCalibrationProfile(const IntrinsicCalibration& calibration);
[[nodiscard]] IntrinsicCalibrationOutcome parseCalibrationProfile(std::string_view text);
// Exact identity and delivered resolution only. Never rescale or silently substitute a profile.
[[nodiscard]] std::string calibrationCompatibilityError(const IntrinsicCalibration& calibration,
    std::string_view sourceId, std::uint32_t width, std::uint32_t height);

enum class ProfileLoadState { Missing, Loaded, Rejected };
struct ProfileLoadOutcome {
    ProfileLoadState state{ProfileLoadState::Missing};
    std::optional<IntrinsicCalibration> calibration;
    std::string error;
};
class ICalibrationProfileStore {
public:
    virtual ~ICalibrationProfileStore() = default;
    // Synchronous worker-only operations, serialized by the application service.
    // Missing is distinct from corrupt/incompatible. Save replaces one exact key atomically;
    // a failed save leaves the previous file intact. Empty error means successful save.
    [[nodiscard]] virtual ProfileLoadOutcome load(std::string_view sourceId,
        std::uint32_t width, std::uint32_t height) = 0;
    [[nodiscard]] virtual std::string save(const IntrinsicCalibration& calibration) = 0;
};
struct UndistortedPreviewOutcome {
    FramePtr frame;
    std::string error;
    std::vector<ImageCorner> corners{};
};
class ICalibrationUndistorter {
public:
    virtual ~ICalibrationUndistorter() = default;
    // Worker-only. Return independent immutable pixels, same dimensions/format and timing.
    // Original frame and capture/detection pipeline are never modified.
    [[nodiscard]] virtual UndistortedPreviewOutcome undistort(const Frame& frame,
        const IntrinsicCalibration& calibration) = 0;
    // Adapters without point remapping suppress overlays instead of returning raw
    // coordinates over remapped pixels. Capture always retains original points.
    [[nodiscard]] virtual UndistortedPreviewOutcome undistortWithCorners(const Frame& frame,
        const IntrinsicCalibration& calibration, const std::vector<ImageCorner>&)
    { return undistort(frame, calibration); }
};
} // namespace micro_robot
