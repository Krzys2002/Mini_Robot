#pragma once
// Session-security extension points of the v1 BLE peripheral (#157 review of PR #274,
// integrated here for #163). The restricted-lab v1 profile provides neither authenticated
// controller ownership nor guaranteed encryption. These hooks make the eventual
// implementation an explicit requirement and give it real call sites in the peripheral
// controller:
//   - beginSession() when a session would become ready (both CCCDs, F2); a refusal keeps
//     the link connected but unready;
//   - verifyIncoming() on every Command write of a ready session, before it is parsed;
//   - authorize() for Velocity and ClearEmergencyStop (protocol::v1::ICommandAuthorizer);
//   - protectOutgoing() on every CommandResult (and Telemetry, #164) before it is queued;
//   - endSession() whenever the session ends (CCCD disable, disconnect, overload).
// The Emergency Stop characteristic never reaches any of them before its latch (F4): no
// authentication or encryption step may block, queue or reject E-stop.
//
// The only implementation is RestrictedLabPassThroughSecurity: it permits everything,
// returns the bytes unchanged and reports bypassed, unauthenticated and unencrypted. It is
// scaffolding, not security, and runs only when selected explicitly
// (SecurityMode::RestrictedLabPassThrough). SecureRequired refuses it: the peripheral then
// never advertises. Nothing falls back to the pass-through after a failure.
//
// A real implementation needs (security-profile.md section 9): mutual peer authentication,
// confidentiality plus integrity, session-bound replay protection, enrollment and key
// storage, replacement/revocation/recovery and downgrade prevention, and a coordinated
// protocol/GATT version decision. It must not change the frozen v1 frames.
//
// Threading: every call runs in the BLE host callback context; implementations must not
// block, allocate or log. Portable C++20, no radio types.
#include "protocol/V1Wire.hpp"
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace codecell::ble {

enum class SecurityMode : std::uint8_t { SecureRequired, RestrictedLabPassThrough };

struct SecuritySessionStatus {
    bool established{false};  // False: the session must not become ready.
    bool authenticated{false};
    bool encrypted{false};
    bool bypassed{true};       // No authentication or payload protection was performed.
};

// Largest frame a security implementation may produce or consume (scratch buffers).
inline constexpr std::size_t kSecuredFrameMax = 64;

class ISessionSecurity {
public:
    virtual ~ISessionSecurity() = default;
    [[nodiscard]] virtual const char* profile() const noexcept = 0;
    [[nodiscard]] virtual bool isPassThrough() const noexcept = 0;
    [[nodiscard]] virtual SecuritySessionStatus beginSession() noexcept = 0;
    // Velocity or ClearEmergencyStop only. False: refused, nothing reaches the authority.
    [[nodiscard]] virtual bool authorize(protocol::v1::MessageType operation) noexcept = 0;
    // nullopt: refused. Otherwise the bytes to use, either `frame` itself or a view into
    // `scratch` (kSecuredFrameMax bytes, owned by the caller for the duration of the call).
    [[nodiscard]] virtual std::optional<std::span<const std::uint8_t>>
    verifyIncoming(std::span<const std::uint8_t> frame, std::span<std::uint8_t> scratch) noexcept = 0;
    [[nodiscard]] virtual std::optional<std::span<const std::uint8_t>>
    protectOutgoing(std::span<const std::uint8_t> frame, std::span<std::uint8_t> scratch) noexcept = 0;
    virtual void endSession() noexcept = 0;  // Idempotent.
};

inline constexpr const char* kRestrictedLabSecurityProfile = "restricted-lab-pass-through";
inline constexpr const char* kRestrictedLabSecurityWarning =
    "TODO: authentication and encryption are not implemented; restricted-lab pass-through security is active.";

// Temporary restricted-lab mock: NOT authentication and NOT encryption.
class RestrictedLabPassThroughSecurity final : public ISessionSecurity {
public:
    [[nodiscard]] const char* profile() const noexcept override { return kRestrictedLabSecurityProfile; }
    [[nodiscard]] bool isPassThrough() const noexcept override { return true; }
    [[nodiscard]] SecuritySessionStatus beginSession() noexcept override { return {true, false, false, true}; }
    [[nodiscard]] bool authorize(protocol::v1::MessageType) noexcept override { return true; }
    [[nodiscard]] std::optional<std::span<const std::uint8_t>>
    verifyIncoming(std::span<const std::uint8_t> frame, std::span<std::uint8_t>) noexcept override
    {
        return frame;
    }
    [[nodiscard]] std::optional<std::span<const std::uint8_t>>
    protectOutgoing(std::span<const std::uint8_t> frame, std::span<std::uint8_t>) noexcept override
    {
        return frame;
    }
    void endSession() noexcept override {}
};

// The explicit selection of this image (F10: defined in one place). No secure
// implementation exists, so the restricted-lab pass-through is the only one that runs.
inline constexpr SecurityMode kSecurityMode = SecurityMode::RestrictedLabPassThrough;

// Whether `security` may run in `mode`. SecureRequired refuses the pass-through, so it
// fails closed; restricted-lab mode runs only the pass-through.
[[nodiscard]] inline bool securityModeAdmits(SecurityMode mode, const ISessionSecurity& security) noexcept
{
    switch (mode) {
    case SecurityMode::RestrictedLabPassThrough: return security.isPassThrough();
    case SecurityMode::SecureRequired: return !security.isPassThrough();
    }
    return false;
}

[[nodiscard]] constexpr const char* toString(SecurityMode mode) noexcept
{
    switch (mode) {
    case SecurityMode::SecureRequired: return "secure-required";
    case SecurityMode::RestrictedLabPassThrough: return "restricted-lab-pass-through";
    }
    return "unknown";
}
} // namespace codecell::ble
