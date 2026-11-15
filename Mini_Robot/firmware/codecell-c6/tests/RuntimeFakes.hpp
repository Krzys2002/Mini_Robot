#pragma once
// Deterministic fakes for the #158 runtime ports (and the #160 drive port). Nothing here sleeps, spawns a thread
// or reads a real clock: tests advance time and interleave the periodic steps by hand.
#include "DriveTestProfile.hpp"
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

using drive::fixtures::kTestDriveProfile;

[[nodiscard]] inline RuntimeConfig validRuntimeConfig()
{
    RuntimeConfig config;
    config.driveProfile = kTestDriveProfile;
    return config;
}

struct DriveEvent {
    enum class Kind { Disable, Command, RampToStop } kind;
    BodyVelocity velocity{};
    Instant at{};
};

// Body-level drive fake. Its stop ramp reaches zero after `rampSteps` steps (0: the first
// ramp step is already at rest); `rampNeverEnds` models an adapter stuck above zero.
class FakeDrive final : public IDriveOutput {
public:
    std::vector<DriveEvent> events;
    bool failDisable{false};
    bool failCommand{false};
    bool failConfigure{false};
    bool failRamp{false};
    bool rampNeverEnds{false};
    int rampSteps{0};
    bool enabled{false};
    std::optional<drive::DriveProfile> configured;
    const FakeCriticalSection* section{nullptr};  // When set, every call must be inside it.
    bool calledOutsideSection{false};

    bool configure(const drive::DriveProfile& profile, std::chrono::milliseconds) noexcept override
    {
        if (failConfigure) return false;
        configured = profile;
        return true;
    }
    bool disable() noexcept override
    {
        checkSection();
        events.push_back({DriveEvent::Kind::Disable, {}, {}});
        rampLeft_.reset();
        if (failDisable) return false;
        enabled = false;
        return true;
    }
    bool command(const BodyVelocity& velocity, Instant now) noexcept override
    {
        checkSection();
        events.push_back({DriveEvent::Kind::Command, velocity, now});
        rampLeft_.reset();
        if (failCommand) return false;
        enabled = true;
        return true;
    }
    StopRampStep rampToStop(Instant now) noexcept override
    {
        checkSection();
        events.push_back({DriveEvent::Kind::RampToStop, {}, now});
        if (failRamp) return StopRampStep::Refused;
        if (rampNeverEnds) return StopRampStep::Ramping;
        if (!rampLeft_) rampLeft_ = rampSteps;
        if (*rampLeft_ == 0) return StopRampStep::AtRest;
        --*rampLeft_;
        return StopRampStep::Ramping;
    }
    [[nodiscard]] int rampStepCount() const
    {
        int count = 0;
        for (const auto& e : events) count += e.kind == DriveEvent::Kind::RampToStop ? 1 : 0;
        return count;
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
    std::optional<int> rampLeft_{};
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
    std::optional<RuntimeConfig> value{validRuntimeConfig()};
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
