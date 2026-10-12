#pragma once

#include "camera/ICameraDeviceDiscovery.hpp"

#include <QObject>
#include <QString>

#include <memory>

namespace micro_robot {

// Presents device inventories; no Qt Multimedia or concrete backend dependency.
// Discovery outlives this controller. All calls/events use its owning Qt thread.
class CameraDiscoveryController final : public QObject {
    Q_OBJECT
public:
    explicit CameraDiscoveryController(ICameraDeviceDiscovery& discovery, QObject* parent = nullptr);
    ~CameraDiscoveryController() override;
    void initialize();

signals:
    void cameraDevicesDescriptionChanged(const QString& description);

private:
    class EventSink;
    void publishDevices(const std::vector<CameraDevice>& devices);
    ICameraDeviceDiscovery& discovery_;
    std::shared_ptr<EventSink> events_;
    bool initialized_{};
};

} // namespace micro_robot
