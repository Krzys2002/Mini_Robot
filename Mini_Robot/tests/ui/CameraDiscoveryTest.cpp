#include "app/CameraDiscoveryController.hpp"
#include "app/DesktopController.hpp"
#include "infrastructure/logging/Logger.hpp"
#include "ui/MainWindow.hpp"
#include "simulation/SimulationTransport.hpp"

#include <QLabel>
#include <gtest/gtest.h>

#include <memory>
#include <utility>
#include <vector>

namespace micro_robot {
namespace {

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

struct DiscoveryFixture {
    Logger logger; // Outlives the controller's non-owning log channel.
    ManualDiscovery discovery;
    CameraDiscoveryController presenter{discovery};
    SimulationTransport transport;
    WorldModel world;
    DesktopController controller{transport, world, RobotId{1}, logger.channel("controller")};
    MainWindow window{controller};

    DiscoveryFixture()
    {
        QObject::connect(&presenter, &CameraDiscoveryController::cameraDevicesDescriptionChanged,
                         &window, &MainWindow::setCameraDevicesDescription);
    }
    QLabel* inventory() { return window.findChild<QLabel*>(QStringLiteral("cameraDevicesDescription")); }
};

TEST(CameraDiscoveryTest, InitialRefreshIsIdempotentAndEmptySnapshotIsExplicit)
{
    DiscoveryFixture fixture;
    fixture.presenter.initialize();
    fixture.presenter.initialize();
    EXPECT_EQ(fixture.discovery.refreshRequests, 1);
    // Requests alone must not invent a device snapshot or imply capture.
    EXPECT_EQ(fixture.inventory()->text(), QStringLiteral("Looking for cameras…"));
    fixture.discovery.publish({});
    EXPECT_EQ(fixture.inventory()->text(), QStringLiteral("No cameras available"));
    EXPECT_FALSE(fixture.controller.hasCameraSource());
    EXPECT_EQ(fixture.world.robotCount(), 0U);
}

TEST(CameraDiscoveryTest, SameNameDevicesRemainDistinctByTheirIdentifiers)
{
    DiscoveryFixture fixture;
    fixture.discovery.publish({{"usb-a", "USB Camera"}, {"usb-b", "USB Camera"}});
    const auto text = fixture.inventory()->text();
    EXPECT_TRUE(text.contains(QStringLiteral("2 camera(s) available")));
    EXPECT_EQ(text.count(QStringLiteral("USB Camera")), 2);
    EXPECT_TRUE(text.contains(QStringLiteral("ID: usb-a")));
    EXPECT_TRUE(text.contains(QStringLiteral("ID: usb-b")));
    const auto devices = fixture.discovery.devices();
    ASSERT_EQ(devices.size(), 2U);
    EXPECT_NE(devices[0].id, devices[1].id);
}

TEST(CameraDiscoveryTest, AddRemoveAndRenameReplaceTheDisplayedInventory)
{
    DiscoveryFixture fixture;
    fixture.discovery.publish({{"internal", "Built-in"}});
    fixture.discovery.publish({{"internal", "Built-in"}, {"usb", "External"}});
    EXPECT_TRUE(fixture.inventory()->text().contains(QStringLiteral("External")));
    fixture.discovery.publish({{"usb", "Renamed camera"}});
    EXPECT_FALSE(fixture.inventory()->text().contains(QStringLiteral("Built-in")));
    EXPECT_FALSE(fixture.inventory()->text().contains(QStringLiteral("External")));
    EXPECT_TRUE(fixture.inventory()->text().contains(QStringLiteral("Renamed camera")));
    EXPECT_TRUE(fixture.inventory()->text().contains(QStringLiteral("ID: usb")));
    fixture.discovery.publish({});
    EXPECT_EQ(fixture.inventory()->text(), QStringLiteral("No cameras available"));
}

TEST(CameraDiscoveryTest, MissingMetadataHasFallbacksAndNamesArePlainText)
{
    DiscoveryFixture fixture;
    fixture.discovery.publish({{"", ""}, {"id-2", "<b>Camera & sensor</b>"}});
    EXPECT_TRUE(fixture.inventory()->text().contains(QStringLiteral("Unnamed camera")));
    EXPECT_TRUE(fixture.inventory()->text().contains(QStringLiteral("ID: unavailable")));
    EXPECT_TRUE(fixture.inventory()->text().contains(QStringLiteral("<b>Camera & sensor</b>")));
    EXPECT_EQ(fixture.inventory()->textFormat(), Qt::PlainText);
    EXPECT_TRUE(fixture.discovery.devices()[0].id.empty());
}

TEST(CameraDiscoveryTest, InventoryQueriesAreDetachedCopies)
{
    DiscoveryFixture fixture;
    fixture.discovery.publish({{"first", "Camera"}});
    auto snapshot = fixture.discovery.devices();
    snapshot[0].id = "modified";
    snapshot.clear();
    EXPECT_EQ(fixture.discovery.devices()[0].id, "first");
    fixture.discovery.publish({{"second", "Camera"}});
    EXPECT_FALSE(fixture.inventory()->text().contains(QStringLiteral("ID: first")));
    EXPECT_TRUE(fixture.inventory()->text().contains(QStringLiteral("ID: second")));
}

TEST(CameraDiscoveryTest, PresenterDetachesSubscriptionOnDestruction)
{
    ManualDiscovery discovery;
    {
        CameraDiscoveryController presenter{discovery};
        EXPECT_TRUE(discovery.hasSink());
        presenter.initialize();
        discovery.publish({{"id", "Camera"}});
    }
    EXPECT_FALSE(discovery.hasSink());
    EXPECT_NO_THROW(discovery.publish({}));
}

} // namespace
} // namespace micro_robot
