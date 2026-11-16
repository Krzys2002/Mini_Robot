#pragma once
// Session readiness policy of the v1 BLE peripheral (#163). It applies the restricted-lab
// v1 security profile proposed by #157 (maintainer decision of 2026-11-07, PR #274, not yet
// merged): v1 has no pairing, bonding, encryption, controller keys or authenticated
// ownership, and nothing here is authentication. The only enforceable controls are session
// exclusivity (one connected central, F1) and the v1 readiness rule (both CCCDs, F2).
// Final enforcement and its attack tests belong to #165.
//
// Portable C++20, no radio types.
#include <cstdint>

namespace codecell::ble {

enum class SecurityProfile : std::uint8_t { RestrictedLabV1 };

// The one profile this image is built for. No secure profile exists, and nothing falls back
// to the restricted-lab pass-through: SecurityMode::SecureRequired (ble/SessionSecurity.hpp)
// refuses it and the peripheral never advertises.
inline constexpr SecurityProfile kSecurityProfile = SecurityProfile::RestrictedLabV1;

[[nodiscard]] constexpr const char* toString(SecurityProfile profile) noexcept
{
    switch (profile) {
    case SecurityProfile::RestrictedLabV1: return "restricted-lab-v1";
    }
    return "unknown";
}

// What the peripheral knows about the connection when readiness is decided.
struct ReadinessInputs {
    bool ownsSession{false};          // The connection is the robot's only accepted central (F1).
    bool closing{false};              // The session was ended for overload and the link is being terminated.
    bool commandResultNotify{false};  // Command Result CCCD has notifications enabled.
    bool telemetryNotify{false};      // Telemetry CCCD has notifications enabled.
};

// A session is ready only on the owning connection, with notifications enabled on both CCCDs,
// and never while it is being closed. Link encryption is deliberately not an input: under the
// restricted-lab profile no behaviour may depend on it (F6), because a central can obtain at
// most unauthenticated Just Works encryption.
[[nodiscard]] constexpr bool sessionMayBeReady(SecurityProfile profile, const ReadinessInputs& in) noexcept
{
    switch (profile) {
    case SecurityProfile::RestrictedLabV1:
        return in.ownsSession && !in.closing && in.commandResultNotify && in.telemetryNotify;
    }
    return false;
}
} // namespace codecell::ble
