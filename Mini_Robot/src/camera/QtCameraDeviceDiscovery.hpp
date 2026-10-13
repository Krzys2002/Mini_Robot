#pragma once

#include "camera/ICameraDeviceDiscovery.hpp"

#include <QMediaDevices>
#include <QObject>

namespace micro_robot {

// Adapter only: queries Qt's device inventory, never opens a camera or captures.
// Own on the Qt event-loop thread. Cached snapshots contain no native handles.
class QtCameraDeviceDiscovery final : public QObject, public ICameraDeviceDiscovery {
public:
    explicit QtCameraDeviceDiscovery(QObject* parent = nullptr);
    void setEventSink(std::weak_ptr<ICameraDeviceEvents> sink) override;
    [[nodiscard]] std::vector<CameraDevice> devices() const override;
    void refresh() override;

private:
    void publishInventory();
    QMediaDevices mediaDevices_;
    std::weak_ptr<ICameraDeviceEvents> sink_;
    std::vector<CameraDevice> devices_;
    bool refreshPending_{};
};

} // namespace micro_robot
