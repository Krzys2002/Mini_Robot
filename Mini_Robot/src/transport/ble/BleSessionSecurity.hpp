#pragma once
// Session-security extension points of the BLE adapter (#157 review of PR #274).
//
// v1 has no authenticated controller ownership and no guaranteed encryption. These hooks
// make the eventual implementation an explicit architectural requirement and give it real
// call sites in BleTransport (session establishment, Velocity/Clear authorization,
// incoming verification) and in the backends (outgoing protection at native write).
// They stay behind the transport boundary: planning, WorldModel and widgets never see
// BLE or cryptographic types.
//
// The only implementation today is RestrictedLabPassThroughSecurity: it permits every
// operation and leaves bytes unchanged, and always reports bypassed, unauthenticated and
// unencrypted. It is scaffolding, not security. It must be selected explicitly
// (BleSecurityMode::RestrictedLabPassThrough); SecureRequired rejects it.
//
// The dedicated E-stop path never calls these hooks: v1 latches on any write to the
// Emergency Stop characteristic, and no authentication or encryption work may block,
// queue or reject it. Stop is protected like any frame but is not subject to
// authorization; Velocity and ClearEmergencyStop are.
//
// A real implementation needs (security-profile.md section 9): mutual peer
// authentication, confidentiality plus integrity, session-bound replay protection,
// enrollment and key storage, replacement/revocation/recovery and downgrade prevention.
// It must not change the frozen v1 frames under the same version.
#include "transport/TransportTypes.hpp"
#include "transport/ble/BleProtocol.hpp"
#include "transport/ble/BleTypes.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace micro_robot {
inline constexpr const char* kRestrictedLabSecurityProfile = "restricted-lab-pass-through";
inline constexpr const char* kRestrictedLabSecurityWarning =
    "TODO: authentication and encryption are not implemented; restricted-lab pass-through security is active.";

enum class BleSecuredOperation { Velocity, EmergencyClear };

struct BleSecuritySession {
    bool established{false};  // False: the session must not become Connected.
    TransportSecurityStatus status;
    std::string reason;
};
struct BleSecurityDecision {
    bool permitted{false};
    bool bypassed{false};  // Permitted without any authorization check.
    std::string reason;
};
struct BleProtectedPayload {
    std::optional<std::vector<std::uint8_t>> bytes;  // nullopt: rejected, nothing is sent or delivered.
    bool bypassed{false};                            // Returned unchanged, without protection.
    std::string reason;
};

// All calls on the adapter's owning thread. endSession() is idempotent.
class IBleSessionSecurity {
public:
    virtual ~IBleSessionSecurity() = default;
    [[nodiscard]] virtual const char* profile() const noexcept = 0;
    // True only for the restricted-lab pass-through; SecureRequired refuses it.
    [[nodiscard]] virtual bool isPassThrough() const noexcept = 0;
    [[nodiscard]] virtual BleSecuritySession beginSession(RobotId id, const BleRobotInfo& info) = 0;
    [[nodiscard]] virtual BleSecurityDecision authorize(RobotId id, BleSecuredOperation operation) = 0;
    [[nodiscard]] virtual BleProtectedPayload protectOutgoing(RobotId id, BleEndpoint endpoint,
                                                              std::span<const std::uint8_t> bytes) = 0;
    [[nodiscard]] virtual BleProtectedPayload verifyIncoming(RobotId id, BleEndpoint endpoint,
                                                             std::span<const std::uint8_t> bytes) = 0;
    virtual void endSession(RobotId id) = 0;
};

// Temporary restricted-lab mock. Permits everything, returns bytes unchanged and reports
// bypassed/unauthenticated/unencrypted. NOT authentication and NOT encryption.
class RestrictedLabPassThroughSecurity final : public IBleSessionSecurity {
public:
    [[nodiscard]] const char* profile() const noexcept override { return kRestrictedLabSecurityProfile; }
    [[nodiscard]] bool isPassThrough() const noexcept override { return true; }
    [[nodiscard]] BleSecuritySession beginSession(RobotId id, const BleRobotInfo& info) override;
    [[nodiscard]] BleSecurityDecision authorize(RobotId id, BleSecuredOperation operation) override;
    [[nodiscard]] BleProtectedPayload protectOutgoing(RobotId id, BleEndpoint endpoint,
                                                      std::span<const std::uint8_t> bytes) override;
    [[nodiscard]] BleProtectedPayload verifyIncoming(RobotId id, BleEndpoint endpoint,
                                                     std::span<const std::uint8_t> bytes) override;
    void endSession(RobotId) override {}
};

[[nodiscard]] std::shared_ptr<IBleSessionSecurity> makeRestrictedLabPassThroughSecurity();

// nullopt when `policy` may run in `mode`; otherwise why not. SecureRequired refuses a
// missing policy and the pass-through, so it fails closed instead of falling back.
[[nodiscard]] std::optional<std::string> bleSecuritySelectionError(BleSecurityMode mode,
                                                                   const IBleSessionSecurity* policy);
[[nodiscard]] const char* bleSecurityModeName(BleSecurityMode mode) noexcept;
} // namespace micro_robot
