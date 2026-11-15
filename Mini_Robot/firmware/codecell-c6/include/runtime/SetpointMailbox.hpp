#pragma once
// Depth-1 setpoint handoff from the command path to the control task (#156 B12). The
// newest accepted setpoint wins; nothing is ever queued behind it, so the mailbox cannot
// fill up or grow a motion backlog. Reads return a copy and do not consume it: the gate
// decides on every write whether that copy may still drive (generation and deadline).
#include "runtime/RuntimePorts.hpp"
#include <cstdint>
#include <optional>

namespace codecell::runtime {

struct SetpointSnapshot {
    BodyVelocity velocity{};
    std::uint32_t generation{0};  // SafetyState generation at admission.
};

class SetpointMailbox {
public:
    // Single-threaded until bind(); the runtime binds before starting any task.
    void bind(ICriticalSection& section) noexcept { section_ = &section; }

    // Returns true when an unread setpoint was replaced (counted as a B12 overwrite).
    bool publish(const SetpointSnapshot& snapshot) noexcept
    {
        CriticalSectionGuard guard(section_);
        const bool overwrote = unread_;
        slot_ = snapshot;
        unread_ = true;
        return overwrote;
    }

    [[nodiscard]] std::optional<SetpointSnapshot> latest() noexcept
    {
        CriticalSectionGuard guard(section_);
        unread_ = false;
        return slot_;
    }

private:
    ICriticalSection* section_{nullptr};
    std::optional<SetpointSnapshot> slot_{};
    bool unread_{false};
};
} // namespace codecell::runtime
