#include "camera/CameraSelection.hpp"

#include <algorithm>

namespace micro_robot {

std::vector<CameraChoice> classifyCameraChoices(
    const std::vector<CameraDevice>& devices, const ICameraSourceFactory& factory)
{
    std::vector<CameraChoice> choices;
    choices.reserve(devices.size());
    for (const auto& device : devices) {
        auto status = CameraChoiceStatus::Available;
        if (device.id.empty()) {
            status = CameraChoiceStatus::MissingId;
        } else if (std::count_if(devices.begin(), devices.end(),
                       [&](const CameraDevice& other) { return other.id == device.id; }) > 1) {
            status = CameraChoiceStatus::DuplicateId;
        } else if (!factory.supports(device)) {
            status = CameraChoiceStatus::Unsupported;
        }
        choices.push_back({device, status});
    }
    return choices;
}

std::optional<CameraDevice> findSelectableCamera(
    const std::vector<CameraChoice>& choices, std::string_view id)
{
    if (id.empty()) return std::nullopt;
    const auto found = std::find_if(choices.begin(), choices.end(), [&](const CameraChoice& choice) {
        return choice.status == CameraChoiceStatus::Available && choice.device.id == id;
    });
    if (found == choices.end()) return std::nullopt;
    return found->device;
}

} // namespace micro_robot
