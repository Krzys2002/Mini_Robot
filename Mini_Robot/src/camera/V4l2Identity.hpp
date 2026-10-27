#pragma once
#include <filesystem>
#include <optional>
#include <string>

namespace micro_robot {
// Adapter helper: node paths are access locations, not physical identities.
// USB vendor/product/serial + interface + video index; absent for missing serial,
// unreadable metadata or duplicate identities. Paths are injectable for host tests.
[[nodiscard]] std::optional<std::string> persistentV4l2Identity(
    const std::filesystem::path& node,
    const std::filesystem::path& sysfs = "/sys/class/video4linux");
} // namespace micro_robot
