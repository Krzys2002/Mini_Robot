#include "camera/V4l2Identity.hpp"
#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <chrono>

namespace {
using namespace micro_robot;
namespace fs = std::filesystem;
struct UsbTree {
    fs::path root = fs::temp_directory_path() / ("robot-usb-identity-" +
        std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    UsbTree() { fs::create_directories(root / "sys"); fs::create_directories(root / "dev"); }
    ~UsbTree() { std::error_code error; fs::remove_all(root, error); }
    fs::path add(const char* node, const char* device, const char* serial, const char* index = "0") {
        auto usb = root / device;
        fs::create_directories(usb / "interface");
        std::ofstream(usb / "idVendor") << "1234\n";
        std::ofstream(usb / "idProduct") << "5678\n";
        std::ofstream(usb / "serial") << serial << '\n';
        std::ofstream(usb / "interface/bInterfaceNumber") << "00\n";
        fs::create_directories(root / "sys" / node);
        std::ofstream(root / "sys" / node / "index") << index << '\n';
        fs::create_directory_symlink(usb / "interface", root / "sys" / node / "device");
        std::ofstream(root / "dev" / node) << "fake";
        return root / "dev" / node;
    }
    auto id(const fs::path& node) { return persistentV4l2Identity(node, root / "sys"); }
};
TEST(V4l2IdentityTest, RenumberingPreservesPhysicalIdentityAndNodeReuseChangesIt) {
    UsbTree t;
    auto node = t.add("video0", "usb-a", "serial-a");
    const auto original = t.id(node); ASSERT_TRUE(original);
    fs::rename(t.root / "dev/video0", t.root / "dev/video4");
    fs::rename(t.root / "sys/video0", t.root / "sys/video4");
    EXPECT_EQ(t.id(t.root / "dev/video4"), original);
    auto reused = t.add("video0", "usb-b", "serial-b");
    ASSERT_TRUE(t.id(reused)); EXPECT_NE(t.id(reused), original);
}
TEST(V4l2IdentityTest, RejectsMissingSerialAndDuplicatePhysicalIdentity) {
    UsbTree t;
    auto a = t.add("video0", "usb-a", "");
    EXPECT_FALSE(t.id(a));
    std::ofstream(t.root / "usb-a/serial") << "same";
    auto b = t.add("video2", "usb-b", "same");
    EXPECT_FALSE(t.id(a)); EXPECT_FALSE(t.id(b));
    fs::remove_all(t.root / "sys/video2");
    EXPECT_TRUE(t.id(a));
}
TEST(V4l2IdentityTest, SeparatesInterfacesAndVideoIndicesAndRejectsIncompleteMetadata) {
    UsbTree t;
    auto a = t.add("video0", "usb-a", "serial", "0");
    auto b = t.add("video1", "usb-a", "serial", "1");
    ASSERT_TRUE(t.id(a)); ASSERT_TRUE(t.id(b)); EXPECT_NE(t.id(a), t.id(b));
    fs::remove(t.root / "usb-a/interface/bInterfaceNumber");
    EXPECT_FALSE(t.id(a)); EXPECT_FALSE(t.id(b));
}
TEST(V4l2IdentityTest, ResolvesAccessSymlinkButNeverInventsIdentityFromPathOrName) {
    UsbTree t;
    auto a = t.add("video0", "usb-a", "serial");
    fs::create_symlink(a, t.root / "dev/by-id-camera");
    EXPECT_EQ(t.id(t.root / "dev/by-id-camera"), t.id(a));
    EXPECT_FALSE(t.id(t.root / "dev/missing"));
    fs::remove(t.root / "sys/video0/index");
    EXPECT_FALSE(t.id(a));
}
} // namespace
