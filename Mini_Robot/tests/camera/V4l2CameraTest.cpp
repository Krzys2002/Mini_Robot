#include "camera/V4l2CameraDeviceDiscovery.hpp"
#include "camera/V4l2CameraSource.hpp"

#include <QEventLoop>
#include <QThread>
#include <QTimer>
#include <gtest/gtest.h>

#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace {
using namespace micro_robot;
namespace fs = std::filesystem;

// Runs the Qt event loop until the predicate holds or the deadline expires.
bool waitUntil(const std::function<bool()>& done, std::chrono::milliseconds limit = std::chrono::seconds{5})
{
    QEventLoop loop;
    QTimer poll;
    QObject::connect(&poll, &QTimer::timeout, &loop, [&] { if (done()) loop.quit(); });
    QTimer deadline;
    deadline.setSingleShot(true);
    QObject::connect(&deadline, &QTimer::timeout, &loop, &QEventLoop::quit);
    poll.start(5);
    deadline.start(limit);
    if (!done()) loop.exec();
    return done();
}

// Device and sysfs trees in a private temporary directory. Device nodes are
// plain files: unreadable ones behave like nodes without access permission,
// readable ones like character devices that are not V4L2 capture nodes.
class FakeV4l2Tree {
public:
    FakeV4l2Tree()
    {
        root_ = fs::temp_directory_path()
            / ("micro-robot-v4l2-" + std::to_string(::getpid()) + "-" + std::to_string(counter_++));
        fs::remove_all(root_);
        fs::create_directories(paths.deviceDirectory = root_ / "dev");
        fs::create_directories(paths.sysfsDirectory = root_ / "sys");
    }
    ~FakeV4l2Tree()
    {
        std::error_code ignored;
        fs::remove_all(root_, ignored);
    }

    void addClassEntry(const std::string& node, const std::optional<std::string>& name,
                       const std::optional<std::string>& index)
    {
        const auto entry = paths.sysfsDirectory / node;
        fs::create_directories(entry);
        if (name) std::ofstream{entry / "name"} << *name << '\n';
        if (index) std::ofstream{entry / "index"} << *index << '\n';
    }

    // Staged outside the device directory and renamed in, so a watcher-triggered
    // scan never observes the node before its permissions are final.
    void addNode(const std::string& node, bool accessible)
    {
        const auto staged = root_ / ("staged-" + node);
        std::ofstream{staged} << "not a device";
        fs::permissions(staged, accessible ? fs::perms::owner_read | fs::perms::owner_write
                                           : fs::perms::none);
        fs::rename(staged, paths.deviceDirectory / node);
    }

    std::string nodePath(const std::string& node) const { return (paths.deviceDirectory / node).string(); }

    V4l2DiscoveryPaths paths;

private:
    static inline int counter_{};
    fs::path root_;
};

// Permission bits do not restrict root, so access-denied cases cannot be staged.
bool runningAsRoot() { return ::geteuid() == 0; }

std::vector<std::string> ids(const std::vector<CameraDevice>& devices)
{
    std::vector<std::string> result;
    for (const auto& device : devices) result.push_back(device.id);
    return result;
}

TEST(V4l2DiscoveryScanTest, MissingSysfsDirectoryYieldsEmptyInventory)
{
    FakeV4l2Tree tree;
    tree.paths.sysfsDirectory /= "absent";
    EXPECT_TRUE(scanV4l2CaptureDevices(tree.paths).empty());
}

TEST(V4l2DiscoveryScanTest, ListsPrimaryNodesItCannotQueryInNumericOrder)
{
    if (runningAsRoot()) GTEST_SKIP() << "Permission bits do not apply to root";
    FakeV4l2Tree tree;
    tree.addClassEntry("video10", "Late camera", std::nullopt);
    tree.addNode("video10", false);
    tree.addClassEntry("video2", "USB camera  ", "0");
    tree.addNode("video2", false);
    tree.addClassEntry("video0", "Integrated camera", "0");
    tree.addNode("video0", false);
    tree.addClassEntry("video1", "Integrated camera", "1"); // Secondary (metadata) node.
    tree.addNode("video1", false);
    tree.addClassEntry("video3", "Not capture", "0");      // Queryable, but no V4L2 capture.
    tree.addNode("video3", true);
    tree.addClassEntry("video4", "Removed", "0");          // Node already gone.
    tree.addClassEntry("vbi0", "Teletext", "0");
    tree.addNode("vbi0", false);
    tree.addClassEntry("videoX", "Malformed", "0");
    tree.addNode("videoX", false);

    const auto devices = scanV4l2CaptureDevices(tree.paths);
    EXPECT_EQ(ids(devices), (std::vector<std::string>{
        tree.nodePath("video0"), tree.nodePath("video2"), tree.nodePath("video10")}));
    ASSERT_EQ(devices.size(), 3U);
    EXPECT_EQ(devices[0].displayName, "Integrated camera");
    EXPECT_EQ(devices[1].displayName, "USB camera");
    EXPECT_EQ(devices[2].displayName, "Late camera");
}

TEST(V4l2DiscoveryScanTest, MissingNameIsLeftEmptyRatherThanInvented)
{
    if (runningAsRoot()) GTEST_SKIP() << "Permission bits do not apply to root";
    FakeV4l2Tree tree;
    tree.addClassEntry("video0", std::nullopt, "0");
    tree.addNode("video0", false);
    const auto devices = scanV4l2CaptureDevices(tree.paths);
    ASSERT_EQ(devices.size(), 1U);
    EXPECT_EQ(devices[0].id, tree.nodePath("video0"));
    EXPECT_TRUE(devices[0].displayName.empty());
}

class RecordingDeviceEvents final : public ICameraDeviceEvents {
public:
    void onCameraDevicesChanged(std::vector<CameraDevice> devices) override
    {
        snapshots.push_back(std::move(devices));
    }
    std::vector<std::vector<CameraDevice>> snapshots;
};

TEST(V4l2CameraDeviceDiscoveryTest, RefreshPublishesAsynchronouslyEvenWhenUnchanged)
{
    if (runningAsRoot()) GTEST_SKIP() << "Permission bits do not apply to root";
    FakeV4l2Tree tree;
    tree.addClassEntry("video0", "Camera", "0");
    tree.addNode("video0", false);
    V4l2CameraDeviceDiscovery discovery{tree.paths};
    auto events = std::make_shared<RecordingDeviceEvents>();
    discovery.setEventSink(events);

    discovery.refresh();
    discovery.refresh(); // Coalesced with the pending request.
    EXPECT_TRUE(events->snapshots.empty()) << "refresh() must not publish inline";
    ASSERT_TRUE(waitUntil([&] { return events->snapshots.size() == 1; }));
    EXPECT_EQ(ids(events->snapshots[0]), (std::vector<std::string>{tree.nodePath("video0")}));
    EXPECT_EQ(ids(discovery.devices()), ids(events->snapshots[0]));

    discovery.refresh();
    ASSERT_TRUE(waitUntil([&] { return events->snapshots.size() == 2; }));
    EXPECT_EQ(ids(events->snapshots[1]), ids(events->snapshots[0]));
}

TEST(V4l2CameraDeviceDiscoveryTest, DeviceDirectoryChangesPublishOnlyInventoryChanges)
{
    if (runningAsRoot()) GTEST_SKIP() << "Permission bits do not apply to root";
    FakeV4l2Tree tree;
    V4l2CameraDeviceDiscovery discovery{tree.paths};
    auto events = std::make_shared<RecordingDeviceEvents>();
    discovery.setEventSink(events);
    discovery.refresh();
    ASSERT_TRUE(waitUntil([&] { return events->snapshots.size() == 1; }));
    EXPECT_TRUE(events->snapshots[0].empty());

    // Hotplug: the class entry exists before the kernel creates the device node.
    tree.addClassEntry("video0", "Camera", "0");
    tree.addNode("video0", false);
    ASSERT_TRUE(waitUntil([&] { return events->snapshots.size() == 2; }));
    EXPECT_EQ(ids(events->snapshots[1]), (std::vector<std::string>{tree.nodePath("video0")}));

    // An unrelated device node changes the directory but not the inventory.
    tree.addNode("ttyUSB0", true);
    EXPECT_FALSE(waitUntil([&] { return events->snapshots.size() > 2; }, std::chrono::milliseconds{300}));

    // Unplug removes the node first; the stale class entry alone is not listed.
    fs::remove(tree.paths.deviceDirectory / "video0");
    ASSERT_TRUE(waitUntil([&] { return events->snapshots.size() == 3; }));
    EXPECT_TRUE(events->snapshots[2].empty());
    EXPECT_TRUE(discovery.devices().empty());
}

class SourceEvents final : public ICameraEvents {
public:
    explicit SourceEvents(ICameraSource& source) : source_(source) {}
    void onFrame(FramePtr frame) override
    {
        EXPECT_EQ(QThread::currentThread(), thread);
        EXPECT_EQ(source_.state(), CameraState::Streaming);
        frames.push_back(std::move(frame));
    }
    void onCameraStateChanged(CameraState state, Timestamp) override
    {
        EXPECT_EQ(QThread::currentThread(), thread);
        EXPECT_EQ(source_.state(), state);
        states.push_back(state);
    }
    void onCameraError(const CameraError& error) override
    {
        EXPECT_EQ(QThread::currentThread(), thread);
        errors.push_back(error);
    }
    bool awaitState(CameraState state, std::chrono::milliseconds limit = std::chrono::seconds{5})
    {
        return waitUntil([&] { return !states.empty() && states.back() == state; }, limit);
    }

    QThread* thread{QThread::currentThread()};
    ICameraSource& source_;
    std::vector<CameraState> states;
    std::vector<CameraError> errors;
    std::vector<FramePtr> frames;
};

TEST(V4l2CameraSourceTest, FactoryAcceptsAbsoluteNodePathsWithoutOpeningThem)
{
    V4l2CameraSourceFactory factory;
    EXPECT_FALSE(factory.supports({{}, "No ID"}));
    EXPECT_FALSE(factory.supports({"video0", "Relative path"}));
    EXPECT_FALSE(factory.create({"video0", "Relative path"}));
    auto source = factory.create({"/dev/mini-robot-test-video-absent", "Absent camera"});
    ASSERT_TRUE(source);
    EXPECT_EQ(source->state(), CameraState::Stopped);
    EXPECT_EQ(source->info().sourceId, "/dev/mini-robot-test-video-absent");
    EXPECT_EQ(source->info().displayName, "Absent camera");
}

TEST(V4l2CameraSourceTest, MissingDeviceFaultsAsynchronouslyAndCanRetry)
{
    V4l2CameraSourceFactory factory;
    auto source = factory.create({"/dev/mini-robot-test-video-absent", "Absent camera"});
    ASSERT_TRUE(source);
    auto events = std::make_shared<SourceEvents>(*source);
    source->setEventSink(events);

    for (int attempt = 0; attempt < 2; ++attempt) {
        events->states.clear();
        events->errors.clear();
        source->start();
        EXPECT_TRUE(events->states.empty()) << "start() must not dispatch inline";
        ASSERT_TRUE(events->awaitState(CameraState::Fault));
        EXPECT_EQ(events->states, (std::vector<CameraState>{CameraState::Starting, CameraState::Fault}));
        ASSERT_EQ(events->errors.size(), 1U);
        EXPECT_EQ(events->errors[0].code, CameraErrorCode::Unavailable);
        EXPECT_TRUE(events->frames.empty());
    }
}

TEST(V4l2CameraSourceTest, InaccessibleDeviceReportsPermissionDenied)
{
    if (runningAsRoot()) GTEST_SKIP() << "Permission bits do not apply to root";
    FakeV4l2Tree tree;
    tree.addNode("video0", false);
    V4l2CameraSource source{{tree.nodePath("video0"), "Locked camera"}, {}};
    auto events = std::make_shared<SourceEvents>(source);
    source.setEventSink(events);
    source.start();
    ASSERT_TRUE(events->awaitState(CameraState::Fault));
    ASSERT_EQ(events->errors.size(), 1U);
    EXPECT_EQ(events->errors[0].code, CameraErrorCode::PermissionDenied);
    EXPECT_NE(events->errors[0].message.find(tree.nodePath("video0")), std::string::npos);
}

TEST(V4l2CameraSourceTest, NonCaptureNodeReportsBackendFailure)
{
    FakeV4l2Tree tree;
    tree.addNode("video0", true);
    V4l2CameraSource source{{tree.nodePath("video0"), "Not a camera"}, {}};
    auto events = std::make_shared<SourceEvents>(source);
    source.setEventSink(events);
    source.start();
    ASSERT_TRUE(events->awaitState(CameraState::Fault));
    ASSERT_EQ(events->errors.size(), 1U);
    EXPECT_EQ(events->errors[0].code, CameraErrorCode::BackendFailure);
}

TEST(V4l2CameraSourceTest, StopCancelsPendingStartAndAcknowledgesQuiescence)
{
    V4l2CameraSource source{{"/dev/mini-robot-test-video-absent", "Absent camera"}, {}};
    auto events = std::make_shared<SourceEvents>(source);
    source.setEventSink(events);
    source.start();
    source.stop();
    source.stop(); // Idempotent.
    ASSERT_TRUE(events->awaitState(CameraState::Stopped));
    EXPECT_TRUE(events->errors.empty());
    EXPECT_FALSE(waitUntil([&] { return events->states.back() != CameraState::Stopped; },
                           std::chrono::milliseconds{200}));
}

// Physical capture on a real node, e.g. MICRO_ROBOT_TEST_V4L2_DEVICE=/dev/video0.
// MICRO_ROBOT_TEST_V4L2_RESOLUTION=<width>x<height> additionally requests a mode
// the device offers and checks that frames arrive at that size.
std::optional<std::string> hardwareDevice()
{
    const char* value = std::getenv("MICRO_ROBOT_TEST_V4L2_DEVICE");
    if (!value || !*value) return std::nullopt;
    return std::string{value};
}

std::optional<CameraResolution> hardwareResolution()
{
    const char* value = std::getenv("MICRO_ROBOT_TEST_V4L2_RESOLUTION");
    unsigned width{};
    unsigned height{};
    if (!value || std::sscanf(value, "%ux%u", &width, &height) != 2) return std::nullopt;
    return CameraResolution{width, height};
}

TEST(V4l2CameraHardwareTest, DiscoveryListsConfiguredDevice)
{
    const auto device = hardwareDevice();
    if (!device) GTEST_SKIP() << "Set MICRO_ROBOT_TEST_V4L2_DEVICE to a camera node";
    const auto listed = ids(scanV4l2CaptureDevices({}));
    EXPECT_NE(std::find(listed.begin(), listed.end(), *device), listed.end())
        << *device << " is not listed as a capture device";
}

TEST(V4l2CameraHardwareTest, StreamsValidFramesAcrossRestarts)
{
    const auto device = hardwareDevice();
    if (!device) GTEST_SKIP() << "Set MICRO_ROBOT_TEST_V4L2_DEVICE to a camera node";
    const auto resolution = hardwareResolution();
    V4l2CameraSource source{{*device, "Hardware camera"}, {resolution}};
    auto events = std::make_shared<SourceEvents>(source);
    source.setEventSink(events);

    std::uint64_t lastSequence{};
    for (int cycle = 0; cycle < 2; ++cycle) {
        events->states.clear();
        events->frames.clear();
        source.start();
        ASSERT_TRUE(waitUntil([&] { return events->frames.size() >= 10 || !events->errors.empty(); },
                              std::chrono::seconds{10}));
        ASSERT_TRUE(events->errors.empty()) << events->errors[0].message;
        EXPECT_EQ(events->states.front(), CameraState::Starting);
        EXPECT_EQ(events->states.back(), CameraState::Streaming);
        for (const auto& frame : events->frames) {
            ASSERT_TRUE(frame && isValidFrame(*frame));
            EXPECT_EQ(frame->pixelFormat, PixelFormat::Rgb8);
            if (resolution) {
                EXPECT_EQ(frame->width, resolution->width);
                EXPECT_EQ(frame->height, resolution->height);
            }
            if (cycle > 0 || frame != events->frames.front()) {
                EXPECT_GT(frame->sequence, lastSequence);
            }
            lastSequence = frame->sequence;
        }
        RecordProperty("frame_size", std::to_string(events->frames.back()->width) + "x"
                       + std::to_string(events->frames.back()->height));
        source.stop();
        ASSERT_TRUE(events->awaitState(CameraState::Stopped));
    }
}

} // namespace
