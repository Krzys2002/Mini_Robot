#pragma once

#include "camera/ICameraDeviceDiscovery.hpp"
#include "camera/ICameraSourceFactory.hpp"

#include <optional>
#include <string_view>
#include <vector>

namespace micro_robot {

// Why a discovered device can or cannot be selected. Selection is keyed by the
// opaque device ID, so missing or ambiguous IDs are never selectable.
enum class CameraChoiceStatus { Available, MissingId, DuplicateId, Unsupported };

struct CameraChoice {
    CameraDevice device;
    CameraChoiceStatus status{CameraChoiceStatus::Available};
};

// Classifies one discovery snapshot, preserving its order and every entry. Every
// device sharing a non-empty ID is DuplicateId. Only IDs that are present and
// unique are passed to factory.supports(). Pure: no device is opened.
[[nodiscard]] std::vector<CameraChoice> classifyCameraChoices(
    const std::vector<CameraDevice>& devices, const ICameraSourceFactory& factory);

// Returns the Available device with this ID, or nullopt (including an empty ID).
[[nodiscard]] std::optional<CameraDevice> findSelectableCamera(
    const std::vector<CameraChoice>& choices, std::string_view id);

} // namespace micro_robot
