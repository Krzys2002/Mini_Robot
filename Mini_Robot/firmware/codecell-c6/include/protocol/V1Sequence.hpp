#pragma once
// Protocol version 1 sequence rules (#161, robot-protocol.md "Sequence numbers").
// Wire sequences are per-session uint16 counters. They are not domain metadata: the
// robot never passes them to the motion authority, and the desktop's
// RobotCommand::metadata.sequence is never transmitted.
//
// Plain values, not thread-safe: the BLE adapter (#163) owns one session's instances and
// uses them from its single callback context.
#include "protocol/V1Wire.hpp"
#include <cstdint>
#include <optional>

namespace codecell::protocol::v1 {

// Serial-number comparison: `a` is newer than `b` when 0 < (uint16)(a - b) < 32768.
// Equal values and values exactly half the range apart are not newer in either
// direction; wraparound from 65535 to 0 is newer.
[[nodiscard]] constexpr bool isNewer(std::uint16_t a, std::uint16_t b) noexcept
{
    const auto distance = static_cast<std::uint16_t>(a - b);
    return distance != 0 && distance < kSequenceHalfRange;
}

// The robot's record of desktop command sequences in one session (`lastAccepted`).
// Unset at session start. check() never changes it; record() is called only after the
// command was accepted, so a rejected frame never changes it.
class CommandSequenceState {
public:
    [[nodiscard]] std::optional<std::uint16_t> lastAccepted() const noexcept { return lastAccepted_; }

    // True when the sequence rule lets this command proceed to the state checks:
    // - EmergencyStop is never sequence-checked;
    // - Stop is always executed, because stopping is always safe;
    // - Velocity and ClearEmergencyStop must be newer than lastAccepted, except for the
    //   first accepted command of the session, which is accepted without comparison.
    [[nodiscard]] constexpr bool admits(MessageType type, std::uint16_t sequence) const noexcept
    {
        switch (type) {
        case MessageType::EmergencyStop:
        case MessageType::Stop: return true;
        case MessageType::Velocity:
        case MessageType::ClearEmergencyStop: return isNewerThanLast(sequence);
        case MessageType::Telemetry:
        case MessageType::CommandResult: return false;
        }
        return false;
    }

    // An accepted command. EmergencyStop never changes lastAccepted; every other command
    // advances it only when newer (a stale Stop is accepted without advancing it).
    constexpr void record(MessageType type, std::uint16_t sequence) noexcept
    {
        if (type == MessageType::EmergencyStop) return;
        if (isNewerThanLast(sequence)) lastAccepted_ = sequence;
    }

    // Session end: all sequence state is reset.
    constexpr void reset() noexcept { lastAccepted_.reset(); }

private:
    [[nodiscard]] constexpr bool isNewerThanLast(std::uint16_t sequence) const noexcept
    {
        return !lastAccepted_ || isNewer(sequence, *lastAccepted_);
    }

    std::optional<std::uint16_t> lastAccepted_{};
};

// The robot's single transmit counter for Telemetry and CommandResult. It starts at 0
// for the first frame of a session and increments by one per frame, modulo 65536.
class TransmitSequence {
public:
    [[nodiscard]] constexpr std::uint16_t peek() const noexcept { return next_; }
    // The sequence of the frame being sent now.
    [[nodiscard]] constexpr std::uint16_t take() noexcept { return next_++; }  // Unsigned wrap.
    constexpr void reset() noexcept { next_ = 0; }

private:
    std::uint16_t next_{0};
};
} // namespace codecell::protocol::v1
