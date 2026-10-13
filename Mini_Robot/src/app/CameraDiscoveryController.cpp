#include "app/CameraDiscoveryController.hpp"

#include <QStringList>

namespace micro_robot {

class CameraDiscoveryController::EventSink final : public ICameraDeviceEvents {
public:
    explicit EventSink(CameraDiscoveryController& owner) : owner_(owner) {}
    void onCameraDevicesChanged(std::vector<CameraDevice> devices) override
    {
        owner_.publishDevices(devices);
    }
private:
    CameraDiscoveryController& owner_;
};

CameraDiscoveryController::CameraDiscoveryController(ICameraDeviceDiscovery& discovery, QObject* parent)
    : QObject(parent), discovery_(discovery), events_(std::make_shared<EventSink>(*this))
{
    discovery_.setEventSink(events_);
}

CameraDiscoveryController::~CameraDiscoveryController() { discovery_.setEventSink({}); }

void CameraDiscoveryController::initialize()
{
    if (initialized_) return;
    initialized_ = true;
    discovery_.refresh();
}

void CameraDiscoveryController::publishDevices(const std::vector<CameraDevice>& devices)
{
    if (devices.empty()) {
        emit cameraDevicesDescriptionChanged(QStringLiteral("No cameras available"));
        return;
    }
    QStringList rows;
    for (const auto& device : devices) {
        const auto name = device.displayName.empty() ? QStringLiteral("Unnamed camera")
            : QString::fromStdString(device.displayName);
        const auto id = device.id.empty() ? QStringLiteral("unavailable")
            : QString::fromStdString(device.id);
        rows.push_back(QStringLiteral("%1\nID: %2").arg(name, id));
    }
    emit cameraDevicesDescriptionChanged(QStringLiteral("%1 camera(s) available\n\n%2")
        .arg(static_cast<qulonglong>(devices.size())).arg(rows.join(QStringLiteral("\n\n"))));
}

} // namespace micro_robot
