#pragma once
// Runtime configuration from the provisioned record (#159). The runtime calls load()
// once during start(), after the drive has been disabled. Without a validated record it
// returns nothing, so the runtime enters Fault (ConfigurationUnavailable) and never
// becomes ready: an absent, corrupt, partial or out-of-version record cannot arm outputs.
//
// The advertised maxCommandValidityMs is also the enforced validity ceiling, so Robot
// Info can never promise a longer validity than the watchdog applies. Likewise the
// advertised maxLinear/maxAngular are the body limits the drive enforces (#160), and the
// drive profile is the record's calibrated actuator data in SI units. bootState() is the
// record this image runs with until the next reset; maintenance writes never change it.
#include "drive/DriveProfile.hpp"
#include "provisioning/ConfigStore.hpp"
#include "provisioning/RobotIdentity.hpp"
#include "runtime/RuntimeConfig.hpp"
#include "runtime/RuntimePorts.hpp"
#include <optional>

namespace codecell::provisioning {

// The record stores the output bound in DriveCell percent; the drive uses a fraction.
[[nodiscard]] constexpr drive::DriveProfile driveProfileFor(const ValidatedRobotConfig& config) noexcept
{
    drive::DriveProfile profile{};
    profile.trackWidthMetres = config.trackWidthMetres();
    profile.maxWheelSpeedMetresPerSecond = config.maxWheelSpeedMetresPerSecond();
    profile.maxOutputFraction =
        static_cast<double>(config.maxDriveOutputPercent()) / static_cast<double>(kDriveOutputPercentFullScale);
    profile.maxLinearMetresPerSecond = config.maxLinearMetresPerSecond();
    profile.maxAngularRadiansPerSecond = config.maxAngularRadiansPerSecond();
    profile.leftReversed = config.leftWheelReversed();
    profile.rightReversed = config.rightWheelReversed();
    return profile;
}

[[nodiscard]] constexpr runtime::RuntimeConfig runtimeConfigFor(const ValidatedRobotConfig& config) noexcept
{
    runtime::RuntimeConfig result{};
    result.commandValidityCeiling = config.maxCommandValidity();
    result.driveProfile = driveProfileFor(config);
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
