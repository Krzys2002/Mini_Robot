#pragma once

#include "camera/CameraSelection.hpp"
#include "camera/ICameraDeviceDiscovery.hpp"
#include "camera/ICameraSourceFactory.hpp"
#include "infrastructure/logging/Logger.hpp"

#include <QList>
#include <QObject>
#include <QString>

#include <memory>
#include <optional>
#include <vector>

namespace micro_robot {

class DesktopController;

// One selectable row for presentation. Unavailable rows are listed but disabled.
struct CameraChoiceItem {
    QString id;      // Opaque device ID; empty when the device has none.
    QString label;   // Display name, disambiguated and annotated when needed.
    QString detail;  // Plain-text identity/availability for tooltips.
    bool selectable{};
};

// Presents discovered devices for selection and turns a selection into the
// active camera source: the factory creates it and DesktopController owns it.
// No Qt Multimedia or concrete backend dependency. Discovery, factory, and
// desktop outlive this controller. All calls/events use its owning Qt thread.
// Selection is keyed by device ID only; if the selected device disappears or
// becomes unselectable, the selection and active source are cleared.
class CameraSelectionController final : public QObject {
    Q_OBJECT
public:
    CameraSelectionController(ICameraDeviceDiscovery& discovery, ICameraSourceFactory& factory,
                              DesktopController& desktop, LogChannel log,
                              QObject* parent = nullptr);
    ~CameraSelectionController() override;
    void initialize();
    [[nodiscard]] std::optional<CameraDevice> selectedCamera() const { return selected_; }

public slots:
    // Ignores the current selection; rejects unknown/unselectable IDs visibly.
    void selectCamera(const QString& id);

signals:
    // Complete replacement list after each inventory snapshot.
    void camerasChanged(const QList<CameraChoiceItem>& cameras);
    // Emitted after every selection attempt or inventory update; empty id means
    // nothing is selected. Views should restore this after a rejected request.
    void selectionChanged(const QString& id, const QString& description);
    void errorOccurred(const QString& message);

private:
    class EventSink;
    void receiveDevices(const std::vector<CameraDevice>& devices);
    void publishSelection();
    ICameraDeviceDiscovery& discovery_;
    ICameraSourceFactory& factory_;
    DesktopController& desktop_;
    LogChannel log_;
    std::shared_ptr<EventSink> events_;
    std::vector<CameraChoice> choices_;
    std::optional<CameraDevice> selected_;
    bool initialized_{};
};

} // namespace micro_robot
