#include "camera/V4l2Identity.hpp"
#include <fstream>

namespace micro_robot {
namespace {
std::string attribute(const std::filesystem::path& path)
{
    std::ifstream input(path);
    std::string value;
    std::getline(input, value);
    const auto end = value.find_last_not_of(" \t\r\n");
    return end == std::string::npos ? std::string{} : value.substr(0, end + 1);
}
std::optional<std::string> fingerprint(const std::filesystem::path& entry)
{
    const auto index = attribute(entry / "index");
    if (index.empty()) return {};
    std::error_code error;
    auto path = std::filesystem::canonical(entry / "device", error);
    if (error) return {};
    std::string interface;
    while (!path.empty() && path != path.parent_path()) {
        if (interface.empty()) interface = attribute(path / "bInterfaceNumber");
        const auto vendor = attribute(path / "idVendor");
        const auto product = attribute(path / "idProduct");
        if (!vendor.empty() || !product.empty()) {
            const auto serial = attribute(path / "serial");
            if (vendor.empty() || product.empty() || serial.empty() || interface.empty()) return {};
            std::string result = "v4l2-usb-v1:";
            // Length prefixes avoid collisions from punctuation in serial strings.
            for (const auto& part : {vendor, product, serial, interface, index})
                result += std::to_string(part.size()) + ":" + part;
            return result;
        }
        path = path.parent_path();
    }
    return {};
}
}
std::optional<std::string> persistentV4l2Identity(const std::filesystem::path& node,
                                                const std::filesystem::path& sysfs)
{
    std::error_code error;
    const auto resolved = std::filesystem::canonical(node, error);
    if (error) return {};
    const auto name = resolved.filename().string();
    if (!name.starts_with("video") || name.size() == 5
        || name.find_first_not_of("0123456789", 5) != std::string::npos) return {};
    const auto identity = fingerprint(sysfs / name);
    if (!identity) return {};
    std::size_t matches = 0;
    auto entry = std::filesystem::directory_iterator(sysfs, error);
    const auto end = std::filesystem::directory_iterator{};
    while (!error && entry != end) {
        if (fingerprint(entry->path()) == identity) ++matches;
        entry.increment(error);
    }
    // Ambiguous serials must not silently load another camera's calibration.
    if (error || matches != 1) return {};
    return identity;
}
} // namespace micro_robot
