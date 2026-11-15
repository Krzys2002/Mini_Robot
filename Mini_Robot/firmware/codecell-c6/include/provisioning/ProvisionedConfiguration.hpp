#pragma once
// Runtime configuration from the provisioned record (#159). The runtime calls load()
// once during start(), after the drive has been disabled. Without a validated record it
// returns nothing, so the runtime enters Fault (ConfigurationUnavailable) and never
// becomes ready: an absent, corrupt, partial or out-of-version record cannot arm outputs.
//
// The advertised maxCommandValidityMs is also the enforced validity ceiling, so Robot
// Info can never promise a longer validity than the watchdog applies. bootState() is the
// record this image runs with until the next reset; maintenance writes never change it.
#include "provisioning/ConfigStore.hpp"
#include "provisioning/RobotIdentity.hpp"
#include "runtime/RuntimeConfig.hpp"
#include "runtime/RuntimePorts.hpp"
#include <optional>

namespace codecell::provisioning {

[[nodiscard]] constexpr runtime::RuntimeConfig runtimeConfigFor(const ValidatedRobotConfig& config) noexcept
{
    runtime::RuntimeConfig result{};
    result.commandValidityCeiling = config.maxCommandValidity();
    return result;
}

class ProvisionedConfiguration final : public runtime::IConfigurationSource {
public:
    explicit ProvisionedConfiguration(ConfigStore& store) noexcept : store_(store) {}

    [[nodiscard]] std::optional<runtime::RuntimeConfig> load() noexcept override
    {
        if (!loaded_) {
            boot_ = store_.load();
            loaded_ = true;
        }
        if (!boot_.armable()) return std::nullopt;
        return runtimeConfigFor(*boot_.config);
    }

    // State read at start-up. Before load() it reports StorageUnavailable (not armable).
    [[nodiscard]] const LoadResult& bootState() const noexcept { return boot_; }

    // Identity for Robot Info and advertising (#161/#163); nullopt when not provisioned,
    // in which case the robot has no identity to publish and must not advertise.
    [[nodiscard]] std::optional<RobotInfoValues> robotInfo() const noexcept
    {
        if (!boot_.armable()) return std::nullopt;
        return robotInfoValues(*boot_.config);
    }

private:
    ConfigStore& store_;
    LoadResult boot_{};
    bool loaded_{false};
};
} // namespace codecell::provisioning
