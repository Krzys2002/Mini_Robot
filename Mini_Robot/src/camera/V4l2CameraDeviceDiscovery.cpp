#include "camera/V4l2CameraDeviceDiscovery.hpp"
#include "camera/V4l2Device.hpp"
#include "common/Time.hpp"

#include <QLoggingCategory>
#include <QTimer>

#include <algorithm>
#include <charconv>
#include <chrono>
#include <fstream>
#include <optional>
#include <string_view>
#include <system_error>
#include <utility>

namespace micro_robot {
namespace {
Q_LOGGING_CATEGORY(discoveryLog, "micro_robot.camera.discovery")

// Node number of a "video<N>" class entry, or nullopt for other entries.
std::optional<unsigned> nodeNumber(std::string_view name)
{
    constexpr std::string_view prefix = "video";
    if (name.substr(0, prefix.size()) != prefix || name.size() == prefix.size()) return std::nullopt;
    name.remove_prefix(prefix.size());
    unsigned number{};
    const auto [end, error] = std::from_chars(name.data(), name.data() + name.size(), number);
    if (error != std::errc{} || end != name.data() + name.size()) return std::nullopt;
    return number;
}

std::string readAttribute(const std::filesystem::path& file)
{
    std::ifstream stream{file};
    std::string line;
    std::getline(stream, line);
    const auto end = line.find_last_not_of(" \t\r\n");
    return end == std::string::npos ? std::string{} : line.substr(0, end + 1);
}

} // namespace

std::vector<CameraDevice> scanV4l2CaptureDevices(const V4l2DiscoveryPaths& paths)
{
    using Status = V4l2NodeProbe::Status;
    std::vector<std::pair<unsigned, CameraDevice>> found;
    std::error_code error;
    for (const auto& entry : std::filesystem::directory_iterator{paths.sysfsDirectory, error}) {
        const auto name = entry.path().filename().string();
        const auto number = nodeNumber(name);
        if (!number) continue;
        const auto node = (paths.deviceDirectory / name).string();
        const auto probe = probeV4l2Node(node);
        bool include = probe.status == Status::Capture;
        if (probe.status == Status::AccessDenied || probe.status == Status::Busy) {
            const auto index = readAttribute(entry.path() / "index");
            include = index.empty() || index == "0";
        }
        if (!include) continue;
        auto displayName = readAttribute(entry.path() / "name");
        if (displayName.empty()) displayName = probe.card;
        found.push_back({*number, {node, std::move(displayName)}});
    }
    // Directory order is unspecified; video10 must follow video2.
    std::sort(found.begin(), found.end(),
              [](const auto& left, const auto& right) { return left.first < right.first; });
    std::vector<CameraDevice> devices;
    devices.reserve(found.size());
    for (auto& [number, device] : found) devices.push_back(std::move(device));
    return devices;
}

V4l2CameraDeviceDiscovery::V4l2CameraDeviceDiscovery(V4l2DiscoveryPaths paths, QObject* parent)
    : QObject(parent), paths_(std::move(paths))
{
    const auto directory = QString::fromStdString(paths_.deviceDirectory.string());
    if (!watcher_.addPath(directory)) {
        qCWarning(discoveryLog) << "event=camera_watch_unavailable" << "path=" << directory
                                << "; camera hotplug is detected only on refresh";
    }
    connect(&watcher_, &QFileSystemWatcher::directoryChanged, this, [this] { scheduleScan(false); });
}

void V4l2CameraDeviceDiscovery::setEventSink(std::weak_ptr<ICameraDeviceEvents> sink)
{
    sink_ = std::move(sink);
}

std::vector<CameraDevice> V4l2CameraDeviceDiscovery::devices() const { return devices_; }

void V4l2CameraDeviceDiscovery::refresh() { scheduleScan(true); }

void V4l2CameraDeviceDiscovery::scheduleScan(bool publishUnchanged)
{
    publishRequested_ = publishRequested_ || publishUnchanged;
    if (scanPending_) return;
    scanPending_ = true;
    // Context-bound delivery is cancelled automatically on destruction. A burst of
    // directory notifications in one event-loop turn shares one scan.
    QTimer::singleShot(0, this, [this] {
        scanPending_ = false;
        const bool publish = std::exchange(publishRequested_, false);
        publishInventory(publish);
    });
}

void V4l2CameraDeviceDiscovery::publishInventory(bool publishUnchanged)
{
    auto snapshot = scanV4l2CaptureDevices(paths_);
    const bool changed = !scanned_ || snapshot.size() != devices_.size()
        || !std::equal(snapshot.begin(), snapshot.end(), devices_.begin(),
                       [](const CameraDevice& left, const CameraDevice& right) {
                           return left.id == right.id && left.displayName == right.displayName;
                       });
    scanned_ = true;
    if (!changed && !publishUnchanged) return;
    devices_ = std::move(snapshot);
    const auto at = std::chrono::duration_cast<std::chrono::milliseconds>(
        MonotonicClock::now().time_since_epoch()).count();
    qCInfo(discoveryLog) << "event=camera_inventory" << "backend=v4l2" << "steady_ms=" << at
                        << "count=" << devices_.size();
    for (const auto& device : devices_) {
        qCInfo(discoveryLog) << "event=camera_discovered" << "steady_ms=" << at
                            << "id=" << QString::fromStdString(device.id)
                            << "name=" << QString::fromStdString(device.displayName);
    }
    if (auto sink = sink_.lock()) sink->onCameraDevicesChanged(devices_);
}

} // namespace micro_robot
