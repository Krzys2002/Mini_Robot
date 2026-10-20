#pragma once

#include "camera/ICameraDeviceDiscovery.hpp"

#include <QFileSystemWatcher>
#include <QObject>

#include <filesystem>
#include <vector>

namespace micro_robot {

// Where V4L2 device nodes and their sysfs class entries live. Tests substitute
// temporary directories; production uses the system locations.
struct V4l2DiscoveryPaths {
    std::filesystem::path deviceDirectory{"/dev"};
    std::filesystem::path sysfsDirectory{"/sys/class/video4linux"};
};

// Synchronous inventory of V4L2 video capture nodes, ordered by node number. The
// ID is the node path (for example /dev/video0), which V4l2CameraSourceFactory
// opens; the name is the driver-reported name. Nodes without video capture (for
// example UVC metadata nodes) are skipped. A node that cannot be opened for the
// capability query because of permissions or a busy driver is still listed when
// sysfs marks it as its device's first node (index 0), so starting it reports the
// actual problem instead of the camera silently disappearing. Opens and closes
// each node without streaming.
[[nodiscard]] std::vector<CameraDevice> scanV4l2CaptureDevices(const V4l2DiscoveryPaths& paths);

// Linux discovery adapter. Watches the device directory and publishes a fresh
// inventory when nodes appear or disappear; unrelated changes that leave the
// inventory unchanged publish nothing, while refresh() always publishes. Own on
// the Qt event-loop thread. Snapshots contain no open descriptors.
class V4l2CameraDeviceDiscovery final : public QObject, public ICameraDeviceDiscovery {
public:
    explicit V4l2CameraDeviceDiscovery(V4l2DiscoveryPaths paths = {}, QObject* parent = nullptr);
    void setEventSink(std::weak_ptr<ICameraDeviceEvents> sink) override;
    [[nodiscard]] std::vector<CameraDevice> devices() const override;
    void refresh() override;

private:
    void scheduleScan(bool publishUnchanged);
    void publishInventory(bool publishUnchanged);

    V4l2DiscoveryPaths paths_;
    QFileSystemWatcher watcher_;
    std::weak_ptr<ICameraDeviceEvents> sink_;
    std::vector<CameraDevice> devices_;
    bool scanPending_{};
    bool publishRequested_{};
    bool scanned_{};
};

} // namespace micro_robot
