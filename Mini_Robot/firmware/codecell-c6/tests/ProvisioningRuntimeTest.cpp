// Host tests of the #159 runtime integration: only a validated record makes the runtime
// ready, the advertised validity ceiling is the enforced one, maintenance clears active
// intent and cannot be undone without a reset, and nothing resumes after a reboot.
#include "ProvisioningFakes.hpp"
#include "RuntimeFakes.hpp"
#include "provisioning/MaintenanceConsole.hpp"
#include "provisioning/ProvisionedConfiguration.hpp"
#include "runtime/FirmwareRuntime.hpp"
#include <gtest/gtest.h>
#include <optional>

using namespace codecell;
using namespace codecell::runtime;
using namespace codecell::runtime::fakes;
using namespace codecell::provisioning;
using namespace codecell::provisioning::fakes;
using namespace std::chrono_literals;

namespace {
constexpr BodyVelocity kForward{0.05, 0.0};

// One boot of the robot over persistent fake storage.
struct Boot {
    FakeClock clock;
    FakePlatform platform;
    FakeDrive drive;
    ConfigStore store;
    ProvisionedConfiguration configuration;
    FirmwareRuntime runtime;

    explicit Boot(FakeRecordStorage& storage)
        : store(storage), configuration(store),
          runtime(RuntimeDependencies{clock, platform, drive, configuration, nullptr})
    {
        clock.time = 1s;
        drive.section = &platform.section;
    }
    StartResult start() { return runtime.start(); }
    void tick()
    {
        clock.advance(Instant{timing::kSafetyCheckPeriod});
        runtime.safetyStep().step();
        runtime.controlStep().step();
    }
    CommandOutcome velocity(milliseconds validFor = 100ms)
    {
        return runtime.commands().velocity(kForward, validFor, clock.nowMicroseconds());
    }
};

class RuntimeTarget final : public IMaintenanceTarget {
public:
    explicit RuntimeTarget(FirmwareRuntime& runtime) : runtime_(runtime) {}
    bool enterMaintenance() noexcept override { return runtime_.enterMaintenance(); }
    void restart() noexcept override { ++restarts; }
    int restarts{0};

private:
    FirmwareRuntime& runtime_;
};

std::vector<std::uint8_t> damaged(const RecordBytes& record)
{
    std::vector<std::uint8_t> bytes(record.begin(), record.end());
    bytes[25] ^= 0x10;
    return bytes;
}
} // namespace

TEST(ProvisioningRuntimeTest, MissingOrBadConfigurationNeverArmsOutputs)
{
    struct Case {
        const char* name;
        void (*prepare)(FakeRecordStorage&);
    };
    const Case cases[] = {
        {"absent", [](FakeRecordStorage&) {}},
        {"corrupt", [](FakeRecordStorage& s) { s.put(Slot::A, damaged(encode(validConfig(), 1))); }},
        {"partial",
         [](FakeRecordStorage& s) {
             const auto r = encode(validConfig(), 1);
             s.put(Slot::A, std::vector<std::uint8_t>(r.begin(), r.begin() + 30));
         }},
        {"out-of-version",
         [](FakeRecordStorage& s) {
             const auto record = encode(validConfig(), 1);
             auto bytes = std::vector<std::uint8_t>(record.begin(), record.end());
             bytes[4] = 2;
             const auto crc = crc32(std::span<const std::uint8_t>{bytes}.first(bytes.size() - 4));
             for (int i = 0; i < 4; ++i) bytes[bytes.size() - 4 + i] = static_cast<std::uint8_t>(crc >> (8 * i));
             s.put(Slot::A, bytes);
         }},
        {"invalid limits",
         [](FakeRecordStorage& s) {
             auto c = validConfig();
             c.maxAngularMilliradiansPerSecond = 0;
             s.put(Slot::A, encode(c, 1));
         }},
        {"unreadable", [](FakeRecordStorage& s) { s.failRead[0] = true; }},
    };
    for (const auto& test : cases) {
        FakeRecordStorage storage;
        test.prepare(storage);
        Boot boot{storage};
        EXPECT_EQ(boot.start(), StartResult::Faulted) << test.name;
        EXPECT_EQ(boot.runtime.status().fault, FaultCode::ConfigurationUnavailable) << test.name;
        EXPECT_FALSE(boot.runtime.status().ready) << test.name;
        EXPECT_FALSE(boot.configuration.robotInfo()) << test.name;  // Nothing to advertise.
        EXPECT_EQ(boot.velocity(), CommandOutcome::RejectedFault) << test.name;
        for (int i = 0; i < 20; ++i) boot.tick();
        EXPECT_EQ(boot.drive.commandCount(), 0) << test.name;
        EXPECT_FALSE(boot.drive.events.empty()) << test.name;
        EXPECT_EQ(boot.drive.events.front().kind, DriveEvent::Kind::Disable) << test.name;  // Off before the load.
        EXPECT_EQ(storage.mutations(), 0) << test.name;
    }
}

TEST(ProvisioningRuntimeTest, AdvertisedValidityIsTheEnforcedCeiling)
{
    FakeRecordStorage storage;
    auto config = validConfig(9);
    config.maxCommandValidityMilliseconds = 120;
    ASSERT_EQ(ConfigStore{storage}.provision(config).outcome, WriteOutcome::Written);
    Boot boot{storage};
    ASSERT_EQ(boot.start(), StartResult::Running);
    const auto info = boot.configuration.robotInfo();
    ASSERT_TRUE(info);
    EXPECT_EQ(info->robotId, 9U);
    EXPECT_EQ(milliseconds{info->maxCommandValidityMilliseconds}, boot.runtime.config()->commandValidityCeiling);

    // A longer requested window is capped at the advertised ceiling.
    ASSERT_EQ(boot.velocity(250ms), CommandOutcome::Accepted);
    const Instant receipt = boot.clock.time;
    while (boot.clock.time < receipt + Instant{milliseconds{115}}) boot.tick();  // Last tick at +115 ms.
    EXPECT_EQ(boot.runtime.status().state, MotionState::Moving);
    while (boot.clock.time < receipt + Instant{milliseconds{125}}) boot.tick();
    EXPECT_EQ(boot.runtime.status().state, MotionState::Idle);
    EXPECT_FALSE(boot.drive.enabled);
}

TEST(ProvisioningRuntimeTest, MaintenanceClearsActiveIntentAndRefusesMotionUntilReset)
{
    FakeRecordStorage storage;
    ASSERT_EQ(ConfigStore{storage}.provision(validConfig(3)).outcome, WriteOutcome::Written);
    Boot boot{storage};
    ASSERT_EQ(boot.start(), StartResult::Running);
    ASSERT_EQ(boot.velocity(), CommandOutcome::Accepted);
    boot.tick();
    ASSERT_TRUE(boot.drive.enabled);
    const auto commandsBefore = boot.drive.commandCount();

    ASSERT_TRUE(boot.runtime.enterMaintenance());
    EXPECT_FALSE(boot.drive.enabled);
    EXPECT_EQ(boot.runtime.status().state, MotionState::Idle);
    EXPECT_TRUE(boot.runtime.status().maintenance);
    EXPECT_FALSE(boot.runtime.status().ready);
    for (int i = 0; i < 10; ++i) boot.tick();  // The stale mailbox setpoint must not drive.
    EXPECT_EQ(boot.drive.commandCount(), commandsBefore);
    EXPECT_EQ(boot.velocity(), CommandOutcome::RejectedNotReady);
    EXPECT_FALSE(boot.drive.enabled);
}

TEST(ProvisioningRuntimeTest, MaintenanceKeepsTheEmergencyStopLatchAndFaults)
{
    FakeRecordStorage storage;
    ASSERT_EQ(ConfigStore{storage}.provision(validConfig(3)).outcome, WriteOutcome::Written);
    Boot boot{storage};
    ASSERT_EQ(boot.start(), StartResult::Running);
    (void)boot.runtime.commands().emergencyStop();
    ASSERT_TRUE(boot.runtime.enterMaintenance());
    EXPECT_EQ(boot.runtime.status().state, MotionState::EmergencyStop);

    FakeRecordStorage empty;
    Boot faulted{empty};
    ASSERT_EQ(faulted.start(), StartResult::Faulted);
    EXPECT_TRUE(faulted.runtime.enterMaintenance());
    EXPECT_EQ(faulted.runtime.status().state, MotionState::Fault);
}

TEST(ProvisioningRuntimeTest, MaintenanceBeforeStartPreventsReadiness)
{
    FakeRecordStorage storage;
    ASSERT_EQ(ConfigStore{storage}.provision(validConfig(3)).outcome, WriteOutcome::Written);
    Boot boot{storage};
    ASSERT_TRUE(boot.runtime.enterMaintenance());
    EXPECT_EQ(boot.start(), StartResult::Faulted);
    EXPECT_FALSE(boot.runtime.status().ready);
    EXPECT_EQ(boot.velocity(), CommandOutcome::RejectedNotReady);
}

TEST(ProvisioningRuntimeTest, ProvisionRebootReprovisionAndResetThroughTheConsole)
{
    FakeRecordStorage storage;
    std::optional<RobotId> previousId;
    for (const RobotId id : {RobotId{0}, RobotId{4294967295U}, RobotId{12}}) {
        Boot boot{storage};
        (void)boot.start();
        RuntimeTarget target{boot.runtime};
        RecordingOutput output;
        MaintenanceConsole console{boot.store, target, output};
        if (previousId) {
            // The previous identity survived the reboot and is what Robot Info would carry.
            ASSERT_TRUE(boot.configuration.robotInfo());
            EXPECT_EQ(boot.configuration.robotInfo()->robotId, *previousId);
            ASSERT_EQ(boot.velocity(), CommandOutcome::Accepted);
            boot.tick();
            ASSERT_TRUE(boot.drive.enabled);
        }
        console.handleLine("maintenance");
        EXPECT_FALSE(boot.drive.enabled);
        const auto c = validConfig(id);
        for (const auto& info : kConfigFields)
            console.handleLine("set " + std::string{info.name} + " " + std::to_string(fieldValue(c, info.field)));
        console.handleLine("commit " + std::to_string(id));
        ASSERT_EQ(target.restarts, 1) << output.last();
        // Until the restart takes effect nothing can move, and the running identity is unchanged.
        EXPECT_EQ(boot.velocity(), previousId ? CommandOutcome::RejectedNotReady : CommandOutcome::RejectedFault);
        EXPECT_FALSE(boot.drive.enabled);
        if (previousId) {
            EXPECT_EQ(boot.configuration.robotInfo()->robotId, *previousId);
        }
        previousId = id;
    }

    // After the last reboot: new identity, Idle, outputs off, no resumed motion.
    Boot rebooted{storage};
    ASSERT_EQ(rebooted.start(), StartResult::Running);
    EXPECT_EQ(rebooted.configuration.robotInfo()->robotId, 12U);
    for (int i = 0; i < 20; ++i) rebooted.tick();
    EXPECT_EQ(rebooted.drive.commandCount(), 0);
    EXPECT_EQ(rebooted.runtime.status().state, MotionState::Idle);

    // Factory reset through the console, then reboot: unprovisioned and never armed.
    RuntimeTarget target{rebooted.runtime};
    RecordingOutput output;
    MaintenanceConsole console{rebooted.store, target, output};
    storage.bondsPresent = true;
    console.handleLine("maintenance");
    console.handleLine("factory-reset ERASE");
    ASSERT_EQ(target.restarts, 1) << output.last();
    EXPECT_FALSE(storage.bondsPresent);
    Boot afterReset{storage};
    EXPECT_EQ(afterReset.start(), StartResult::Faulted);
    EXPECT_EQ(afterReset.configuration.bootState().state, ProvisioningState::Unprovisioned);
    EXPECT_EQ(afterReset.velocity(), CommandOutcome::RejectedFault);
    for (int i = 0; i < 20; ++i) afterReset.tick();
    EXPECT_EQ(afterReset.drive.commandCount(), 0);
}

TEST(ProvisioningRuntimeTest, OrdinaryOperationPerformsNoFlashWrites)
{
    FakeRecordStorage storage;
    ASSERT_EQ(ConfigStore{storage}.provision(validConfig(3)).outcome, WriteOutcome::Written);
    const int afterProvisioning = storage.mutations();
    Boot boot{storage};
    ASSERT_EQ(boot.start(), StartResult::Running);
    for (int i = 0; i < 200; ++i) {
        (void)boot.velocity();
        boot.tick();
        if (i % 50 == 0) (void)boot.runtime.commands().stop();
    }
    (void)boot.runtime.commands().emergencyStop();
    (void)boot.runtime.commands().clearEmergencyStop();
    boot.runtime.commands().sessionEnded(SessionEndReason::Disconnected);
    EXPECT_EQ(storage.mutations(), afterProvisioning);
}
