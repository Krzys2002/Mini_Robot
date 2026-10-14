#include "app/CameraSelectionController.hpp"
#include "app/DesktopController.hpp"

#include <algorithm>
#include <string>
#include <utility>

namespace micro_robot {
namespace {

QString displayName(const CameraDevice& device)
{
    return device.displayName.empty() ? QStringLiteral("Unnamed camera")
                                      : QString::fromStdString(device.displayName);
}

QString displayId(const CameraDevice& device)
{
    return device.id.empty() ? QStringLiteral("unavailable") : QString::fromStdString(device.id);
}

std::string describe(const CameraDevice& device)
{
    return displayName(device).toStdString() + " (ID " + displayId(device).toStdString() + ")";
}

QString unavailableReason(CameraChoiceStatus status)
{
    switch (status) {
    case CameraChoiceStatus::Available: return {};
    case CameraChoiceStatus::MissingId: return QStringLiteral("no device ID");
    case CameraChoiceStatus::DuplicateId: return QStringLiteral("duplicate device ID");
    case CameraChoiceStatus::Unsupported: return QStringLiteral("not supported");
    }
    return QStringLiteral("unknown");
}

} // namespace

class CameraSelectionController::EventSink final : public ICameraDeviceEvents {
public:
    explicit EventSink(CameraSelectionController& owner) : owner_(owner) {}
    void onCameraDevicesChanged(std::vector<CameraDevice> devices) override
    {
        owner_.receiveDevices(devices);
    }
private:
    CameraSelectionController& owner_;
};

CameraSelectionController::CameraSelectionController(ICameraDeviceDiscovery& discovery,
    ICameraSourceFactory& factory, DesktopController& desktop, LogChannel log, QObject* parent)
    : QObject(parent), discovery_(discovery), factory_(factory), desktop_(desktop),
      log_(std::move(log)), events_(std::make_shared<EventSink>(*this))
{
    discovery_.setEventSink(events_);
}

CameraSelectionController::~CameraSelectionController() { discovery_.setEventSink({}); }

void CameraSelectionController::initialize()
{
    if (initialized_) return;
    initialized_ = true;
    discovery_.refresh();
}

void CameraSelectionController::selectCamera(const QString& id)
{
    const auto key = id.toStdString();
    if (selected_ && selected_->id == key) return;
    const auto device = findSelectableCamera(choices_, key);
    if (!device) {
        log_.warning("Camera selection rejected; ID is not selectable: " + key);
        emit errorOccurred(QStringLiteral("The chosen camera is not available"));
        publishSelection();
        return;
    }
    auto source = factory_.create(*device);
    if (!source) {
        log_.error("Could not create camera source for " + describe(*device));
        emit errorOccurred(QStringLiteral("Could not open camera \"%1\"").arg(displayName(*device)));
        publishSelection();
        return;
    }
    selected_ = *device;
    log_.info("Camera selected: " + describe(*device));
    desktop_.setCameraSource(std::move(source));
    publishSelection();
}

void CameraSelectionController::receiveDevices(const std::vector<CameraDevice>& devices)
{
    choices_ = classifyCameraChoices(devices, factory_);
    const auto selectable = std::count_if(choices_.begin(), choices_.end(),
        [](const CameraChoice& choice) { return choice.status == CameraChoiceStatus::Available; });
    log_.info("Camera inventory: " + std::to_string(choices_.size()) + " device(s), "
        + std::to_string(selectable) + " selectable");

    QList<CameraChoiceItem> items;
    for (const auto& choice : choices_) {
        const auto name = displayName(choice.device);
        auto label = name;
        // Same-name devices stay distinguishable before selection.
        if (std::count_if(choices_.begin(), choices_.end(), [&](const CameraChoice& other) {
                return displayName(other.device) == name; }) > 1) {
            label += QStringLiteral(" [%1]").arg(displayId(choice.device));
        }
        const bool available = choice.status == CameraChoiceStatus::Available;
        if (!available) {
            label += QStringLiteral(" — unavailable: %1").arg(unavailableReason(choice.status));
        }
        items.push_back({QString::fromStdString(choice.device.id), label,
                         QStringLiteral("ID: %1").arg(displayId(choice.device)), available});
    }

    if (selected_) {
        if (auto current = findSelectableCamera(choices_, selected_->id)) {
            selected_ = std::move(*current); // Keeps the selection across renames.
        } else {
            log_.warning("Selected camera no longer available: " + describe(*selected_)
                + "; active source cleared");
            emit errorOccurred(QStringLiteral("Camera \"%1\" is no longer available")
                .arg(displayName(*selected_)));
            selected_.reset();
            desktop_.setCameraSource(nullptr);
        }
    }
    emit camerasChanged(items);
    publishSelection();
}

void CameraSelectionController::publishSelection()
{
    if (!selected_) {
        emit selectionChanged({}, QStringLiteral("No camera selected"));
        return;
    }
    emit selectionChanged(QString::fromStdString(selected_->id),
        QStringLiteral("%1\nID: %2").arg(displayName(*selected_), displayId(*selected_)));
}

} // namespace micro_robot
