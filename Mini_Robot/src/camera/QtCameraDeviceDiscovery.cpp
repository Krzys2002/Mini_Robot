#include "camera/QtCameraDeviceDiscovery.hpp"
#include "common/Time.hpp"

#include <QCameraDevice>
#include <QLoggingCategory>
#include <QTimer>

#include <chrono>
#include <utility>

namespace micro_robot {
namespace {
Q_LOGGING_CATEGORY(discoveryLog, "micro_robot.camera.discovery")
}

QtCameraDeviceDiscovery::QtCameraDeviceDiscovery(QObject* parent) : QObject(parent)
{
    connect(&mediaDevices_, &QMediaDevices::videoInputsChanged, this,
            [this] { refresh(); });
}

void QtCameraDeviceDiscovery::setEventSink(std::weak_ptr<ICameraDeviceEvents> sink)
{
    sink_ = std::move(sink);
}

std::vector<CameraDevice> QtCameraDeviceDiscovery::devices() const { return devices_; }

void QtCameraDeviceDiscovery::refresh()
{
    if (refreshPending_) return;
    refreshPending_ = true;
    // Context-bound delivery is cancelled automatically on destruction. Several
    // device-change notifications in one event-loop turn share one fresh snapshot.
    QTimer::singleShot(0, this, [this] {
        refreshPending_ = false;
        publishInventory();
    });
}

void QtCameraDeviceDiscovery::publishInventory()
{
    const auto inputs = QMediaDevices::videoInputs();
    std::vector<CameraDevice> snapshot;
    snapshot.reserve(static_cast<std::size_t>(inputs.size()));
    for (const auto& input : inputs) {
        // Hex encoding preserves arbitrary binary IDs; no lossy text conversion.
        snapshot.push_back({input.id().toHex().toStdString(), input.description().toStdString()});
    }
    devices_ = std::move(snapshot);
    const auto at = std::chrono::duration_cast<std::chrono::milliseconds>(
        MonotonicClock::now().time_since_epoch()).count();
    qCInfo(discoveryLog) << "event=camera_inventory" << "steady_ms=" << at
                        << "count=" << devices_.size();
    for (const auto& device : devices_) {
        qCInfo(discoveryLog) << "event=camera_discovered" << "steady_ms=" << at
                            << "id=" << QString::fromStdString(device.id)
                            << "name=" << QString::fromStdString(device.displayName);
    }
    if (auto sink = sink_.lock()) sink->onCameraDevicesChanged(devices_);
}

} // namespace micro_robot
