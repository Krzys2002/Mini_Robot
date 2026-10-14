#include "app/CameraSelectionController.hpp"
#include "app/DesktopController.hpp"
#include "infrastructure/logging/Logger.hpp"
#include "ui/MainWindow.hpp"
#include "ManualBackends.hpp"

#include <QApplication>
#include <QComboBox>
#include <QLabel>
#include <QPushButton>
#include <QStandardItemModel>
#include <QStatusBar>
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace micro_robot {
namespace {

using namespace std::chrono_literals;

// Tests publish complete inventories explicitly after refresh requests. No Qt
// Multimedia object, physical device query, or actual hotplug is involved.
class ManualDiscovery final : public ICameraDeviceDiscovery {
public:
    void setEventSink(std::weak_ptr<ICameraDeviceEvents> sink) override { sink_ = std::move(sink); }
    std::vector<CameraDevice> devices() const override { return devices_; }
    void refresh() override { ++refreshRequests; }
    void publish(std::vector<CameraDevice> devices)
    {
        devices_ = std::move(devices);
        if (auto sink = sink_.lock()) sink->onCameraDevicesChanged(devices_);
    }
    bool hasSink() const { return !sink_.expired(); }
    int refreshRequests{};
private:
    std::vector<CameraDevice> devices_;
    std::weak_ptr<ICameraDeviceEvents> sink_;
};

// Creates manual cameras named after the device and keeps non-owning handles;
// check alive before using a handle the controller may have released.
class ManualCameraFactory final : public ICameraSourceFactory {
public:
    bool supports(const CameraDevice& device) const override
    {
        return !device.id.empty() && !unsupported.contains(device.id);
    }
    std::unique_ptr<ICameraSource> create(const CameraDevice& device) override
    {
        if (!supports(device) || failing.contains(device.id)) return nullptr;
        auto camera = std::make_unique<test::ManualCamera>(
            CameraInfo{"manual:" + device.id, device.displayName});
        created.push_back(camera.get());
        alive.push_back(camera->alive);
        return camera;
    }
    std::set<std::string> unsupported;
    std::set<std::string> failing;
    std::vector<test::ManualCamera*> created;
    std::vector<std::shared_ptr<bool>> alive;
};

class CapturingLogSink final : public ILogSink {
public:
    void write(const LogRecord& record) override { records.push_back(record); }
    [[nodiscard]] bool contains(LogLevel level, const std::string& message) const
    {
        return std::any_of(records.begin(), records.end(), [&](const LogRecord& record) {
            return record.level == level && record.module == "camera"
                && record.message.find(message) != std::string::npos;
        });
    }
    std::vector<LogRecord> records;
};

struct SelectionFixture {
    Logger logger{LogLevel::Debug}; // Outlives the controllers' non-owning log channels.
    std::shared_ptr<CapturingLogSink> log{std::make_shared<CapturingLogSink>()};
    ManualDiscovery discovery;
    ManualCameraFactory cameras;
    test::ManualTransport transport;
    WorldModel world;
    DesktopController controller{transport, world, RobotId{1}, logger.channel("controller")};
    CameraSelectionController selection{discovery, cameras, controller, logger.channel("camera")};
    MainWindow window{controller};

    SelectionFixture()
    {
        logger.addSink(log);
        window.bindCameraSelection(selection);
        controller.initialize(Timestamp{1s});
    }
    ~SelectionFixture() { controller.shutdown(); }
    QComboBox* selector() { return window.findChild<QComboBox*>(QStringLiteral("cameraSelector")); }
    QLabel* label(const char* name) { return window.findChild<QLabel*>(QString::fromLatin1(name)); }
    QPushButton* button(const char* name)
    {
        return window.findChild<QPushButton*>(QString::fromLatin1(name));
    }
    // Simulates the user picking a row; activated is not emitted programmatically.
    void choose(int index) { emit selector()->activated(index); }
    bool rowEnabled(int index)
    {
        return qobject_cast<QStandardItemModel*>(selector()->model())->item(index)->isEnabled();
    }
    QString status() { return window.statusBar()->currentMessage(); }
};

TEST(CameraSelectionControllerTest, InitialRefreshIsIdempotentAndEmptyInventoryDisablesSelection)
{
    SelectionFixture fixture;
    fixture.selection.initialize();
    fixture.selection.initialize();
    EXPECT_EQ(fixture.discovery.refreshRequests, 1);
    // Requests alone must not invent a device snapshot or create a source.
    EXPECT_FALSE(fixture.selector()->isEnabled());
    EXPECT_EQ(fixture.selector()->placeholderText(), QStringLiteral("Looking for cameras…"));
    fixture.discovery.publish({});
    EXPECT_FALSE(fixture.selector()->isEnabled());
    EXPECT_EQ(fixture.selector()->count(), 0);
    EXPECT_EQ(fixture.selector()->placeholderText(), QStringLiteral("No cameras available"));
    EXPECT_EQ(fixture.label("selectedCamera")->text(), QStringLiteral("No camera selected"));
    EXPECT_TRUE(fixture.cameras.created.empty());
    EXPECT_FALSE(fixture.controller.hasCameraSource());
    EXPECT_FALSE(fixture.button("startCamera")->isEnabled());
    EXPECT_TRUE(fixture.log->contains(LogLevel::Info, "Camera inventory: 0 device(s), 0 selectable"));
}

TEST(CameraSelectionControllerTest, ListsEveryDeviceAndDisablesUnselectableOnes)
{
    SelectionFixture fixture;
    fixture.cameras.unsupported = {"exotic"};
    fixture.discovery.publish({{"internal", "Built-in"}, {"", "No ID"}, {"dup", "Twin"},
        {"dup", "Twin"}, {"exotic", "Exotic"}, {"usb-a", "USB Camera"}, {"usb-b", "USB Camera"},
        {"html", "<b>Camera & sensor</b>"}, {"unnamed", ""}});
    auto* selector = fixture.selector();
    ASSERT_EQ(selector->count(), 9);
    EXPECT_TRUE(selector->isEnabled());
    EXPECT_EQ(selector->currentIndex(), -1);
    EXPECT_EQ(selector->placeholderText(), QStringLiteral("Select a camera…"));

    EXPECT_EQ(selector->itemText(0), QStringLiteral("Built-in"));
    EXPECT_EQ(selector->itemText(1), QStringLiteral("No ID — unavailable: no device ID"));
    EXPECT_EQ(selector->itemText(2), QStringLiteral("Twin [dup] — unavailable: duplicate device ID"));
    EXPECT_EQ(selector->itemText(4), QStringLiteral("Exotic — unavailable: not supported"));
    // Same-name devices are distinguishable by ID before anything is selected.
    EXPECT_EQ(selector->itemText(5), QStringLiteral("USB Camera [usb-a]"));
    EXPECT_EQ(selector->itemText(6), QStringLiteral("USB Camera [usb-b]"));
    EXPECT_EQ(selector->itemText(7), QStringLiteral("<b>Camera & sensor</b>"));
    EXPECT_EQ(selector->itemText(8), QStringLiteral("Unnamed camera"));
    EXPECT_EQ(selector->itemData(5, Qt::ToolTipRole).toString(), QStringLiteral("ID: usb-a"));
    EXPECT_EQ(selector->itemData(1, Qt::ToolTipRole).toString(), QStringLiteral("ID: unavailable"));

    const bool enabled[]{true, false, false, false, false, true, true, true, true};
    for (int index = 0; index < selector->count(); ++index) {
        EXPECT_EQ(fixture.rowEnabled(index), enabled[index]) << "row " << index;
    }
    EXPECT_TRUE(fixture.cameras.created.empty());
    EXPECT_TRUE(fixture.log->contains(LogLevel::Info, "Camera inventory: 9 device(s), 5 selectable"));
}

TEST(CameraSelectionControllerTest, SelectingACameraActivatesItsSource)
{
    SelectionFixture fixture;
    fixture.discovery.publish({{"a", "Camera A"}, {"b", "Camera B"}});
    fixture.choose(1);
    ASSERT_EQ(fixture.cameras.created.size(), 1U);
    EXPECT_EQ(fixture.cameras.created[0]->info().sourceId, "manual:b");
    EXPECT_TRUE(fixture.controller.hasCameraSource());
    ASSERT_TRUE(fixture.selection.selectedCamera());
    EXPECT_EQ(fixture.selection.selectedCamera()->id, "b");
    EXPECT_EQ(fixture.selector()->currentIndex(), 1);
    EXPECT_EQ(fixture.label("selectedCamera")->text(), QStringLiteral("Camera B\nID: b"));
    EXPECT_EQ(fixture.label("activeCamera")->text(), QStringLiteral("Camera B"));
    EXPECT_EQ(fixture.label("cameraStatus")->text(), QStringLiteral("Stopped"));
    EXPECT_TRUE(fixture.button("startCamera")->isEnabled());
    EXPECT_TRUE(fixture.log->contains(LogLevel::Info, "Camera selected: Camera B (ID b)"));

    // Re-choosing the current camera keeps the existing source.
    fixture.choose(1);
    EXPECT_EQ(fixture.cameras.created.size(), 1U);
    fixture.button("startCamera")->click();
    EXPECT_EQ(fixture.cameras.created[0]->startRequests, 1);
}

TEST(CameraSelectionControllerTest, ChangingSelectionReplacesTheActiveSource)
{
    SelectionFixture fixture;
    fixture.discovery.publish({{"a", "Camera A"}, {"b", "Camera B"}});
    fixture.choose(0);
    auto& first = *fixture.cameras.created.at(0);
    const auto firstAlive = fixture.cameras.alive.at(0);
    fixture.button("startCamera")->click();
    first.publishState(CameraState::Streaming, Timestamp{2s});

    fixture.choose(1);
    ASSERT_EQ(fixture.cameras.created.size(), 2U);
    EXPECT_EQ(first.stopRequests, 1);
    EXPECT_EQ(fixture.label("selectedCamera")->text(), QStringLiteral("Camera B\nID: b"));
    // The running source finishes stopping before it is replaced.
    EXPECT_EQ(fixture.label("activeCamera")->text(), QStringLiteral("Camera A"));
    first.publishState(CameraState::Stopped, Timestamp{3s});
    QApplication::processEvents();
    EXPECT_FALSE(*firstAlive);
    EXPECT_EQ(fixture.label("activeCamera")->text(), QStringLiteral("Camera B"));
    EXPECT_TRUE(fixture.cameras.created[1]->hasSink());
    EXPECT_EQ(fixture.cameras.created[1]->startRequests, 0);
    EXPECT_TRUE(fixture.button("startCamera")->isEnabled());
}

TEST(CameraSelectionControllerTest, UnselectableOrFailingChoicesAreRejectedVisibly)
{
    SelectionFixture fixture;
    fixture.cameras.unsupported = {"exotic"};
    fixture.cameras.failing = {"broken"};
    fixture.discovery.publish({{"a", "Camera A"}, {"exotic", "Exotic"}, {"broken", "Broken"}});
    fixture.choose(0);
    ASSERT_EQ(fixture.cameras.created.size(), 1U);

    for (const auto* id : {"exotic", "missing", ""}) {
        fixture.selection.selectCamera(QString::fromLatin1(id));
        EXPECT_EQ(fixture.status(), QStringLiteral("The chosen camera is not available")) << id;
    }
    EXPECT_TRUE(fixture.log->contains(LogLevel::Warning, "Camera selection rejected"));
    fixture.choose(2);
    EXPECT_EQ(fixture.status(), QStringLiteral("Could not open camera \"Broken\""));
    EXPECT_TRUE(fixture.log->contains(LogLevel::Error, "Could not create camera source for Broken"));

    // The previous selection and its source remain; the view reverts to it.
    EXPECT_EQ(fixture.cameras.created.size(), 1U);
    EXPECT_TRUE(*fixture.cameras.alive[0]);
    EXPECT_EQ(fixture.selection.selectedCamera()->id, "a");
    EXPECT_EQ(fixture.selector()->currentIndex(), 0);
    EXPECT_EQ(fixture.label("selectedCamera")->text(), QStringLiteral("Camera A\nID: a"));
    EXPECT_EQ(fixture.label("activeCamera")->text(), QStringLiteral("Camera A"));
}

TEST(CameraSelectionControllerTest, RemovingTheSelectedStoppedDeviceClearsSelectionAndSource)
{
    SelectionFixture fixture;
    fixture.discovery.publish({{"a", "Camera A"}, {"b", "Camera B"}});
    fixture.choose(0);
    const auto firstAlive = fixture.cameras.alive.at(0);

    fixture.discovery.publish({{"b", "Camera B"}});
    EXPECT_FALSE(*firstAlive);
    EXPECT_FALSE(fixture.selection.selectedCamera());
    EXPECT_FALSE(fixture.controller.hasCameraSource());
    EXPECT_EQ(fixture.selector()->count(), 1);
    EXPECT_EQ(fixture.selector()->currentIndex(), -1);
    EXPECT_EQ(fixture.label("selectedCamera")->text(), QStringLiteral("No camera selected"));
    EXPECT_EQ(fixture.label("activeCamera")->text(), QStringLiteral("No camera selected"));
    EXPECT_FALSE(fixture.button("startCamera")->isEnabled());
    EXPECT_EQ(fixture.status(), QStringLiteral("Camera \"Camera A\" is no longer available"));
    EXPECT_TRUE(fixture.log->contains(LogLevel::Warning, "Selected camera no longer available: Camera A"));

    // A remaining camera can be selected afterwards.
    fixture.choose(0);
    EXPECT_EQ(fixture.label("activeCamera")->text(), QStringLiteral("Camera B"));
}

TEST(CameraSelectionControllerTest, RemovingAStreamingDeviceStopsItAndPermitsReselection)
{
    SelectionFixture fixture;
    fixture.discovery.publish({{"a", "Camera A"}});
    fixture.choose(0);
    auto& first = *fixture.cameras.created.at(0);
    const auto firstAlive = fixture.cameras.alive.at(0);
    fixture.button("startCamera")->click();
    first.publishState(CameraState::Streaming, Timestamp{2s});

    fixture.discovery.publish({});
    EXPECT_EQ(first.stopRequests, 1);
    EXPECT_FALSE(fixture.selector()->isEnabled());
    EXPECT_EQ(fixture.label("selectedCamera")->text(), QStringLiteral("No camera selected"));
    // A backend reports loss as Disconnected; the source is released afterwards.
    first.publishState(CameraState::Disconnected, Timestamp{3s});
    EXPECT_TRUE(*firstAlive);
    QApplication::processEvents();
    EXPECT_FALSE(*firstAlive);
    EXPECT_FALSE(fixture.controller.hasCameraSource());
    EXPECT_FALSE(fixture.button("startCamera")->isEnabled());
    EXPECT_FALSE(fixture.button("stopCamera")->isEnabled());

    // Reconnection lists the device again without silently reselecting it.
    fixture.discovery.publish({{"a", "Camera A"}});
    EXPECT_FALSE(fixture.selection.selectedCamera());
    EXPECT_EQ(fixture.selector()->currentIndex(), -1);
    fixture.choose(0);
    ASSERT_EQ(fixture.cameras.created.size(), 2U);
    EXPECT_TRUE(fixture.controller.hasCameraSource());
}

TEST(CameraSelectionControllerTest, SelectionFollowsTheDeviceIdAcrossRenamesOnly)
{
    SelectionFixture fixture;
    fixture.discovery.publish({{"a", "Camera A"}, {"b", "Camera B"}});
    fixture.choose(1);
    fixture.discovery.publish({{"b", "Renamed"}, {"a", "Camera A"}});
    EXPECT_EQ(fixture.cameras.created.size(), 1U);
    EXPECT_TRUE(*fixture.cameras.alive[0]);
    EXPECT_EQ(fixture.selector()->currentIndex(), 0);
    EXPECT_EQ(fixture.label("selectedCamera")->text(), QStringLiteral("Renamed\nID: b"));

    // An ID that becomes ambiguous no longer identifies the selected device.
    fixture.discovery.publish({{"b", "Renamed"}, {"b", "Impostor"}});
    EXPECT_FALSE(fixture.selection.selectedCamera());
    EXPECT_FALSE(*fixture.cameras.alive[0]);
    EXPECT_FALSE(fixture.controller.hasCameraSource());
}

TEST(CameraSelectionControllerTest, ControllerDetachesSubscriptionOnDestruction)
{
    Logger logger;
    ManualDiscovery discovery;
    ManualCameraFactory cameras;
    test::ManualTransport transport;
    WorldModel world;
    DesktopController controller{transport, world, RobotId{1}, logger.channel("controller")};
    {
        CameraSelectionController selection{discovery, cameras, controller, logger.channel("camera")};
        EXPECT_TRUE(discovery.hasSink());
        selection.initialize();
        discovery.publish({{"id", "Camera"}});
    }
    EXPECT_FALSE(discovery.hasSink());
    EXPECT_NO_THROW(discovery.publish({}));
}

} // namespace
} // namespace micro_robot
