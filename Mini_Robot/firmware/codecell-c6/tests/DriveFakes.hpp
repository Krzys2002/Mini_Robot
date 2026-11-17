#pragma once
// Deterministic motor HAL fake for the #160 bounded drive. It records every call in order
// so tests can check exact final-write traces, disable precedence and bounds.
#include "drive/MotorPorts.hpp"
#include "RuntimeFakes.hpp"
#include <optional>
#include <vector>

namespace codecell::drive::fakes {

struct MotorEvent {
    enum class Kind { Enable, Coast, Write, Sleep } kind;
    MotorDuty duty{};
};

class FakeMotorDriver final : public IMotorDriver {
public:
    std::vector<MotorEvent> events;
    bool failEnable{false};
    bool failCoast{false};
    bool failWrite{false};
    // When set, every coast/write must happen inside this critical section (gate contract).
    const runtime::fakes::FakeCriticalSection* section{nullptr};
    bool calledOutsideSection{false};

    bool enable() noexcept override
    {
        events.push_back({MotorEvent::Kind::Enable, {}});
        return !failEnable;
    }
    bool coast() noexcept override
    {
        checkSection();
        events.push_back({MotorEvent::Kind::Coast, {}});
        return !failCoast;
    }
    bool write(const MotorDuty& duty) noexcept override
    {
        checkSection();
        events.push_back({MotorEvent::Kind::Write, duty});
        return !failWrite;
    }
    void sleep() noexcept override
    {
        checkSection();
        events.push_back({MotorEvent::Kind::Sleep, {}});
        asleep = true;
    }
    bool asleep{false};

    [[nodiscard]] std::vector<MotorDuty> writes() const
    {
        std::vector<MotorDuty> result;
        for (const auto& e : events)
            if (e.kind == MotorEvent::Kind::Write) result.push_back(e.duty);
        return result;
    }
    [[nodiscard]] std::optional<MotorDuty> lastWrite() const
    {
        for (auto it = events.rbegin(); it != events.rend(); ++it)
            if (it->kind == MotorEvent::Kind::Write) return it->duty;
        return {};
    }
    [[nodiscard]] int count(MotorEvent::Kind kind) const
    {
        int n = 0;
        for (const auto& e : events) n += e.kind == kind ? 1 : 0;
        return n;
    }
    [[nodiscard]] bool lastIsCoast() const { return !events.empty() && events.back().kind == MotorEvent::Kind::Coast; }
    // The fault path (#162): an immediate coast, then the bridges put to sleep, nothing after.
    [[nodiscard]] bool endsCoastThenSleep() const
    {
        const auto n = events.size();
        return n >= 2 && events[n - 2].kind == MotorEvent::Kind::Coast && events[n - 1].kind == MotorEvent::Kind::Sleep;
    }

private:
    void checkSection()
    {
        if (section && section->depth != 1) calledOutsideSection = true;
    }
};
} // namespace codecell::drive::fakes
