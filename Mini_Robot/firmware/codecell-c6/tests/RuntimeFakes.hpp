#pragma once
// Deterministic fakes for the #158 runtime ports. Nothing here sleeps, spawns a thread
// or reads a real clock: tests advance time and interleave the periodic steps by hand.
#include "runtime/FirmwareRuntime.hpp"
#include <algorithm>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace codecell::runtime::fakes {

class FakeClock final : public IFirmwareClock {
public:
    Instant time{};
    // Runs on each read once, then clears: models a preemption at that exact point.
    mutable std::function<void()> onNextRead;

    Milliseconds now() const override { return std::chrono::duration_cast<Milliseconds>(nowMicroseconds()); }
    Microseconds nowMicroseconds() const override
    {
        if (onNextRead) {
            auto hook = std::move(onNextRead);
            onNextRead = nullptr;
            hook();
        }
        return time;
    }
    void advance(Instant by) { time += by; }
};

class FakeCriticalSection final : public ICriticalSection {
public:
    int depth{0};
    int maxDepth{0};
    int entries{0};
    void enter() noexcept override
    {
        ++entries;
        maxDepth = std::max(maxDepth, ++depth);
    }
    void exit() noexcept override { --depth; }
};

struct DriveEvent {
    enum class Kind { Disable, Command } kind;
    BodyVelocity velocity{};
};

class FakeDrive final : public IDriveOutput {
public:
    std::vector<DriveEvent> events;
    bool failDisable{false};
    bool failCommand{false};
    bool enabled{false};
    const FakeCriticalSection* section{nullptr};  // When set, every call must be inside it.
    bool calledOutsideSection{false};

    bool disable() noexcept override
    {
        checkSection();
        events.push_back({DriveEvent::Kind::Disable, {}});
        if (failDisable) return false;
        enabled = false;
        return true;
    }
    bool command(const BodyVelocity& velocity) noexcept override
    {
        checkSection();
        events.push_back({DriveEvent::Kind::Command, velocity});
        if (failCommand) return false;
        enabled = true;
        return true;
    }
    [[nodiscard]] int commandCount() const
    {
        int count = 0;
        for (const auto& e : events) count += e.kind == DriveEvent::Kind::Command ? 1 : 0;
        return count;
    }
    [[nodiscard]] std::optional<BodyVelocity> lastCommand() const
    {
        for (auto it = events.rbegin(); it != events.rend(); ++it)
            if (it->kind == DriveEvent::Kind::Command) return it->velocity;
        return {};
    }

private:
    void checkSection()
    {
        if (section && section->depth != 1) calledOutsideSection = true;
    }
};

class FakePlatform final : public IPlatform {
public:
    FakeCriticalSection section;
    bool failCriticalSection{false};
    std::optional<std::string> failTaskNamed;
    struct Started {
        TaskSpec spec;
        IPeriodicStep* body;
    };
    std::vector<Started> tasks;

    ICriticalSection* createCriticalSection() noexcept override { return failCriticalSection ? nullptr : &section; }
    bool startPeriodicTask(const TaskSpec& spec, IPeriodicStep& body) noexcept override
    {
        if (failTaskNamed && *failTaskNamed == spec.name) return false;
        tasks.push_back({spec, &body});
        return true;
    }
    [[nodiscard]] IPeriodicStep* task(const char* name) const
    {
        for (const auto& t : tasks)
            if (std::string{t.spec.name} == name) return t.body;
        return nullptr;
    }
};

class FakeConfiguration final : public IConfigurationSource {
public:
    std::optional<RuntimeConfig> value{RuntimeConfig{}};
    std::optional<RuntimeConfig> load() noexcept override { return value; }
};

class FakeInertialSensor final : public IInertialSensor {
public:
    std::optional<InertialSample> next{InertialSample{}};
    std::function<void()> whileReading;  // Models a blocking read: runs during read().
    int reads{0};
    std::optional<InertialSample> read() noexcept override
    {
        ++reads;
        if (whileReading) whileReading();
        return next;
    }
};
} // namespace codecell::runtime::fakes
