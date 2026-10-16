#include "camera/CameraSelection.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <set>
#include <string>
#include <type_traits>
#include <vector>

namespace micro_robot {
namespace {

static_assert(std::is_abstract_v<ICameraSourceFactory>);
static_assert(std::has_virtual_destructor_v<ICameraSourceFactory>);

// Supports every ID except those listed; records which devices were queried.
class ListedFactory final : public ICameraSourceFactory {
public:
    explicit ListedFactory(std::set<std::string> unsupported = {}) : unsupported_(std::move(unsupported)) {}
    bool supports(const CameraDevice& device) const override
    {
        queried.push_back(device.id);
        return !unsupported_.contains(device.id);
    }
    std::unique_ptr<ICameraSource> create(const CameraDevice&) override { return nullptr; }
    mutable std::vector<std::string> queried;
private:
    std::set<std::string> unsupported_;
};

TEST(CameraSelectionTest, EmptyInventoryHasNoChoices)
{
    ListedFactory factory;
    EXPECT_TRUE(classifyCameraChoices({}, factory).empty());
    EXPECT_FALSE(findSelectableCamera({}, "any"));
}

TEST(CameraSelectionTest, ClassificationPreservesOrderAndEveryDevice)
{
    ListedFactory factory{{"unsupported"}};
    const std::vector<CameraDevice> devices{
        {"internal", "Built-in"}, {"", "No ID"}, {"usb", "USB Camera"},
        {"unsupported", "Exotic"}, {"usb", "USB Camera"}, {"usb-2", "USB Camera"}};
    const auto choices = classifyCameraChoices(devices, factory);
    ASSERT_EQ(choices.size(), devices.size());
    for (std::size_t index = 0; index < devices.size(); ++index) {
        EXPECT_EQ(choices[index].device.id, devices[index].id);
        EXPECT_EQ(choices[index].device.displayName, devices[index].displayName);
    }
    EXPECT_EQ(choices[0].status, CameraChoiceStatus::Available);
    EXPECT_EQ(choices[1].status, CameraChoiceStatus::MissingId);
    // Both entries of an ambiguous ID are rejected, not just the later one.
    EXPECT_EQ(choices[2].status, CameraChoiceStatus::DuplicateId);
    EXPECT_EQ(choices[3].status, CameraChoiceStatus::Unsupported);
    EXPECT_EQ(choices[4].status, CameraChoiceStatus::DuplicateId);
    // Same display name with a distinct ID remains selectable.
    EXPECT_EQ(choices[5].status, CameraChoiceStatus::Available);
    // The factory is asked only about identifiable, unambiguous devices.
    EXPECT_EQ(factory.queried, (std::vector<std::string>{"internal", "unsupported", "usb-2"}));
}

TEST(CameraSelectionTest, LookupReturnsOnlyAvailableDevicesById)
{
    ListedFactory factory{{"unsupported"}};
    const auto choices = classifyCameraChoices(
        {{"internal", "Built-in"}, {"", "No ID"}, {"dup", "A"}, {"dup", "B"},
         {"unsupported", "Exotic"}}, factory);
    const auto found = findSelectableCamera(choices, "internal");
    ASSERT_TRUE(found);
    EXPECT_EQ(found->displayName, "Built-in");
    EXPECT_FALSE(findSelectableCamera(choices, ""));
    EXPECT_FALSE(findSelectableCamera(choices, "dup"));
    EXPECT_FALSE(findSelectableCamera(choices, "unsupported"));
    EXPECT_FALSE(findSelectableCamera(choices, "removed"));
    // Display names are never identity.
    EXPECT_FALSE(findSelectableCamera(choices, "Built-in"));
}

} // namespace
} // namespace micro_robot
