#include "transport/ble/BleSessionSecurity.hpp"

namespace micro_robot {
namespace {
TransportSecurityStatus insecureStatus()
{
    return {kRestrictedLabSecurityProfile, false, false, true};
}
} // namespace

BleSecuritySession RestrictedLabPassThroughSecurity::beginSession(RobotId, const BleRobotInfo&)
{
    return {true, insecureStatus(), kRestrictedLabSecurityWarning};
}

BleSecurityDecision RestrictedLabPassThroughSecurity::authorize(RobotId, BleSecuredOperation)
{
    return {true, true, "restricted-lab pass-through: no authorization performed"};
}

BleProtectedPayload RestrictedLabPassThroughSecurity::protectOutgoing(RobotId, BleEndpoint,
                                                                      std::span<const std::uint8_t> bytes)
{
    return {std::vector<std::uint8_t>(bytes.begin(), bytes.end()), true,
            "restricted-lab pass-through: sent without protection"};
}

BleProtectedPayload RestrictedLabPassThroughSecurity::verifyIncoming(RobotId, BleEndpoint,
                                                                     std::span<const std::uint8_t> bytes)
{
    return {std::vector<std::uint8_t>(bytes.begin(), bytes.end()), true,
            "restricted-lab pass-through: received without verification"};
}

std::shared_ptr<IBleSessionSecurity> makeRestrictedLabPassThroughSecurity()
{
    return std::make_shared<RestrictedLabPassThroughSecurity>();
}

std::optional<std::string> bleSecuritySelectionError(BleSecurityMode mode, const IBleSessionSecurity* policy)
{
    if (!policy) return "BLE requires an explicit session-security policy";
    switch (mode) {
    case BleSecurityMode::RestrictedLabPassThrough:
        if (!policy->isPassThrough())
            return "Restricted-lab mode selects the pass-through policy only";
        return std::nullopt;
    case BleSecurityMode::SecureRequired:
        if (policy->isPassThrough())
            return "Secure BLE operation was required, but only the restricted-lab pass-through is available; "
                   "authentication and encryption are not implemented";
        return std::nullopt;
    }
    return "Unknown BLE security mode";
}

const char* bleSecurityModeName(BleSecurityMode mode) noexcept
{
    switch (mode) {
    case BleSecurityMode::SecureRequired: return "secure-required";
    case BleSecurityMode::RestrictedLabPassThrough: return "restricted-lab-pass-through";
    }
    return "unknown";
}
} // namespace micro_robot
