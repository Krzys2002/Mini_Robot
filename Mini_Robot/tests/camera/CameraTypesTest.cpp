#include "camera/ICameraSource.hpp"
#include "camera/ICameraDeviceDiscovery.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <type_traits>
#include <utility>
#include <vector>

namespace micro_robot {
namespace {

static_assert(std::is_abstract_v<ICameraSource>);
static_assert(std::is_abstract_v<ICameraDeviceDiscovery>);
static_assert(std::has_virtual_destructor_v<ICameraDeviceDiscovery>);
static_assert(std::has_virtual_destructor_v<ICameraSource>);
static_assert(std::is_const_v<FramePtr::element_type>);
static_assert(std::is_const_v<decltype(Frame::pixels)::element_type>);
static_assert(noexcept(isValidFrame(std::declval<const Frame&>())));

Frame image(PixelFormat format, std::size_t stride, std::size_t storageSize)
{
    return {0, Timestamp{}, 2, 3, stride, format,
        std::make_shared<const std::vector<std::byte>>(storageSize)};
}

TEST(CameraFrameTest, SupportsAllPackedFormatsAndPaddedRows)
{
    EXPECT_TRUE(isValidFrame(image(PixelFormat::Gray8, 2, 6)));
    EXPECT_TRUE(isValidFrame(image(PixelFormat::Rgb8, 6, 18)));
    EXPECT_TRUE(isValidFrame(image(PixelFormat::Rgba8, 8, 24)));
    EXPECT_TRUE(isValidFrame(image(PixelFormat::Rgb8, 8, 24)));
    // Larger owned buffers are allowed; no requirement to trim allocation capacity.
    EXPECT_TRUE(isValidFrame(image(PixelFormat::Gray8, 2, 10)));
}

TEST(CameraFrameTest, RejectsMissingDimensionsOrStorage)
{
    EXPECT_FALSE(isValidFrame(Frame{}));
    auto frame = image(PixelFormat::Gray8, 2, 6);
    frame.width = 0;
    EXPECT_FALSE(isValidFrame(frame));
    frame.width = 2;
    frame.height = 0;
    EXPECT_FALSE(isValidFrame(frame));
    frame.height = 3;
    frame.pixels.reset();
    EXPECT_FALSE(isValidFrame(frame));
    EXPECT_FALSE(isValidFrame(image(PixelFormat::Gray8, 2, 0)));
}

TEST(CameraFrameTest, RejectsShortRowsAndTruncatedStorage)
{
    EXPECT_FALSE(isValidFrame(image(PixelFormat::Gray8, 0, 6)));
    EXPECT_FALSE(isValidFrame(image(PixelFormat::Gray8, 1, 6)));
    EXPECT_FALSE(isValidFrame(image(PixelFormat::Rgb8, 5, 18)));
    EXPECT_FALSE(isValidFrame(image(PixelFormat::Rgba8, 7, 24)));
    EXPECT_FALSE(isValidFrame(image(PixelFormat::Gray8, 2, 5)));
    // Final-row padding is part of the storage contract too.
    EXPECT_FALSE(isValidFrame(image(PixelFormat::Rgb8, 8, 22)));
}

TEST(CameraFrameTest, RejectsUnknownFormatsAndOverflowSizedMetadata)
{
    auto frame = image(static_cast<PixelFormat>(-1), 2, 6);
    EXPECT_FALSE(isValidFrame(frame));
    frame.pixelFormat = PixelFormat::Rgba8;
    frame.width = std::numeric_limits<std::uint32_t>::max();
    EXPECT_FALSE(isValidFrame(frame));
    frame.width = 2;
    frame.strideBytes = std::numeric_limits<std::size_t>::max() / 2 + 1;
    frame.height = 2; // A stride * height check would wrap to zero.
    EXPECT_FALSE(isValidFrame(frame));
    frame.strideBytes = 8;
    frame.height = std::numeric_limits<std::uint32_t>::max();
    EXPECT_FALSE(isValidFrame(frame));
}

} // namespace
} // namespace micro_robot
