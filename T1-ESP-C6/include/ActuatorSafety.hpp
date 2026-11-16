#pragma once
#include <cstdint>

namespace prototype {
// Pure local actuator gate. The adapter serializes access with its command mutex;
// no clock reads, vendor APIs or desktop dependency. Time uses wrap-safe millis.
class ActuatorSafety {
public:
    explicit ActuatorSafety(std::uint32_t timeoutMs) : timeoutMs_(timeoutMs) {}
    std::uint64_t accept(std::uint32_t now) {
        if (stopped_ || faulted_) return 0;
        issuedAt_ = now; active_ = true; return ++generation_;
    }
    bool permits(std::uint64_t generation, std::uint32_t now) {
        if (active_ && static_cast<std::uint32_t>(now - issuedAt_) >= timeoutMs_) invalidate();
        return active_ && !stopped_ && !faulted_ && generation != 0 && generation == generation_;
    }
    void stop() { stopped_ = true; invalidate(); }
    bool clear() { invalidate(); stopped_ = false; return !faulted_; }
    void fault() { faulted_ = true; stop(); }
private:
    void invalidate() { active_ = false; ++generation_; }
    const std::uint32_t timeoutMs_;
    std::uint32_t issuedAt_{};
    std::uint64_t generation_{};
    bool active_{}, stopped_{}, faulted_{};
};
} // namespace prototype
