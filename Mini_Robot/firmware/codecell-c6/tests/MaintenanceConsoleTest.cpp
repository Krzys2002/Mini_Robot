// Host tests of the #159 USB maintenance console: maintenance is entered (outputs off,
// intent cleared) before any write, drafts are explicit and confirmed, reset needs a
// confirmation word, and nothing on the console can command motion.
#include "ProvisioningFakes.hpp"
#include "provisioning/MaintenanceConsole.hpp"
#include <gtest/gtest.h>
#include <string>

using namespace codecell::provisioning;
using namespace codecell::provisioning::fakes;

namespace {
struct ConsoleRig {
    FakeRecordStorage storage;
    ConfigStore store{storage};
    FakeMaintenanceTarget target;
    RecordingOutput output;
    MaintenanceConsole console{store, target, output};

    ConsoleRig() { target.storage = &storage; }
    void send(const std::string& line) { console.handleLine(line); }
    void setAll(const RobotConfig& c)
    {
        for (const auto& info : kConfigFields)
            send("set " + std::string{info.name} + " " + std::to_string(fieldValue(c, info.field)));
    }
};
} // namespace

TEST(MaintenanceConsoleTest, MutatingCommandsRequireMaintenance)
{
    ConsoleRig rig;
    for (const char* line : {"set robot_id 3", "commit 3", "factory-reset ERASE", "draft"}) {
        rig.send(line);
        EXPECT_EQ(rig.output.last(), "ERR not in maintenance") << line;
    }
    EXPECT_EQ(rig.target.entered, 0);
    EXPECT_EQ(rig.storage.mutations(), 0);
}

TEST(MaintenanceConsoleTest, InitialProvisioningEntersMaintenanceBeforeWritingAndRestarts)
{
    ConsoleRig rig;
    rig.send("maintenance");
    EXPECT_EQ(rig.target.entered, 1);
    EXPECT_EQ(rig.target.mutationsAtEntry, 0);
    rig.setAll(validConfig(0));
    rig.send("commit 0");
    EXPECT_TRUE(rig.output.contains("OK committed robot_id=0 generation=1 slot=A"));
    EXPECT_EQ(rig.target.restarts, 1);
    EXPECT_EQ(rig.store.load().config->raw(), validConfig(0));
}

TEST(MaintenanceConsoleTest, AnUnsetFieldBlocksCommitEvenThoughZeroIsAValidRobotId)
{
    ConsoleRig rig;
    rig.send("maintenance");
    auto c = validConfig(0);
    for (const auto& info : kConfigFields)
        if (info.field != ConfigField::RobotId)
            rig.send("set " + std::string{info.name} + " " + std::to_string(fieldValue(c, info.field)));
    rig.send("commit 0");
    EXPECT_EQ(rig.output.last(), "ERR draft incomplete: robot_id unset");
    EXPECT_EQ(rig.storage.mutations(), 0);
    EXPECT_EQ(rig.target.restarts, 0);
}

TEST(MaintenanceConsoleTest, CommitNeedsTheRobotIdTypedAgain)
{
    ConsoleRig rig;
    rig.send("maintenance");
    rig.setAll(validConfig(42));
    for (const char* wrong : {"commit 41", "commit x", "commit -42", "commit 4294967296"}) {
        rig.send(wrong);
        EXPECT_EQ(rig.output.last(), "ERR confirmation must repeat the draft robot_id") << wrong;
    }
    EXPECT_EQ(rig.storage.mutations(), 0);
}

TEST(MaintenanceConsoleTest, InvalidDraftIsReportedAndNothingIsWritten)
{
    ConsoleRig rig;
    rig.send("maintenance");
    auto c = validConfig(5);
    c.telemetryPeriodMilliseconds = 50;
    rig.setAll(c);
    rig.send("commit 5");
    EXPECT_TRUE(rig.output.contains("ERR commit outcome=RejectedInvalid error=TelemetryPeriodMismatch"));
    EXPECT_EQ(rig.storage.mutations(), 0);
    EXPECT_EQ(rig.target.restarts, 0);
}

TEST(MaintenanceConsoleTest, ValuesAreRangeCheckedForTheirStoredWidth)
{
    ConsoleRig rig;
    rig.send("maintenance");
    rig.send("set max_output_percent 256");
    EXPECT_EQ(rig.output.last(), "ERR max_output_percent needs a decimal integer 0..255");
    rig.send("set max_linear_mm_s 65536");
    EXPECT_EQ(rig.output.last(), "ERR max_linear_mm_s needs a decimal integer 0..65535");
    rig.send("set robot_id 4294967295");
    EXPECT_EQ(rig.output.last(), "OK robot_id=4294967295");
    rig.send("set robot_id 0x10");
    EXPECT_TRUE(rig.output.last().rfind("ERR robot_id", 0) == 0);
    rig.send("set no_such_field 1");
    EXPECT_EQ(rig.output.last(), "ERR unknown field; type help");
}

TEST(MaintenanceConsoleTest, ReprovisioningStartsFromTheStoredRecord)
{
    ConsoleRig rig;
    ASSERT_EQ(rig.store.provision(validConfig(7)).outcome, WriteOutcome::Written);
    rig.send("maintenance");
    rig.send("set robot_id 8");
    rig.send("commit 8");
    EXPECT_TRUE(rig.output.contains("OK committed robot_id=8 generation=2 slot=B"));
    auto expected = validConfig(7);
    expected.robotId = 8;
    EXPECT_EQ(rig.store.load().config->raw(), expected);
}

TEST(MaintenanceConsoleTest, MaintenanceIsRefusedWhenTheDriveDisableIsNotConfirmed)
{
    ConsoleRig rig;
    rig.target.refuse = true;
    rig.send("maintenance");
    EXPECT_TRUE(rig.output.contains("ERR maintenance refused"));
    EXPECT_FALSE(rig.console.inMaintenance());
    rig.send("set robot_id 1");
    EXPECT_EQ(rig.output.last(), "ERR not in maintenance");
}

TEST(MaintenanceConsoleTest, FactoryResetWithAnUnreadableSlotErasesNothingAndStays)
{
    ConsoleRig rig;
    ASSERT_EQ(rig.store.provision(validConfig(7)).outcome, WriteOutcome::Written);
    rig.storage.bondsPresent = true;
    rig.send("maintenance");
    rig.storage.failRead[1] = true;
    rig.send("factory-reset ERASE");
    EXPECT_EQ(rig.output.last(),
              "ERR factory-reset outcome=StorageUnavailable nothing erased; check the storage and run it again");
    EXPECT_EQ(rig.target.restarts, 0);
    EXPECT_TRUE(rig.storage.bondsPresent);
    rig.storage.failRead[1] = false;
    EXPECT_TRUE(rig.store.load().armable());
}

TEST(MaintenanceConsoleTest, FactoryResetNeedsConfirmationAndRestarts)
{
    ConsoleRig rig;
    ASSERT_EQ(rig.store.provision(validConfig(7)).outcome, WriteOutcome::Written);
    rig.storage.bondsPresent = true;
    rig.send("maintenance");
    rig.send("factory-reset yes");
    EXPECT_EQ(rig.output.last(), "ERR factory-reset needs the confirmation word ERASE");
    EXPECT_TRUE(rig.store.load().armable());
    rig.send("factory-reset ERASE");
    EXPECT_TRUE(rig.output.contains("OK factory-reset state=Unprovisioned bonds=erased; restarting"));
    EXPECT_EQ(rig.target.restarts, 1);
    EXPECT_EQ(rig.store.load().state, ProvisioningState::Unprovisioned);
    EXPECT_FALSE(rig.storage.bondsPresent);
}

TEST(MaintenanceConsoleTest, StatusReportsIdentityAndHasNoCredentialFields)
{
    ConsoleRig rig;
    rig.send("status");
    EXPECT_TRUE(rig.output.contains("PROV state=Unprovisioned"));
    ASSERT_EQ(rig.store.provision(validConfig(4294967295U)).outcome, WriteOutcome::Written);
    rig.send("status");
    EXPECT_TRUE(rig.output.contains("PROV robot_id=4294967295 name=MR-4294967295"));
    EXPECT_TRUE(rig.output.contains("CONFIG max_validity_ms=200"));
    EXPECT_TRUE(rig.output.contains("pending=None"));
    // An unfinished change (here a marker that could not be erased) is visible.
    rig.storage.failMarkerErase = true;
    ASSERT_EQ(rig.store.provision(validConfig(5)).outcome, WriteOutcome::CommitFailed);
    rig.send("status");
    EXPECT_TRUE(rig.output.contains("pending=Provisioning"));
    for (const auto& line : rig.output.lines) {
        for (const char* secret : {"key", "pass", "secret", "token", "pin"})
            EXPECT_EQ(line.find(secret), std::string::npos) << line;
    }
}

TEST(MaintenanceConsoleTest, NoConsoleCommandCanRequestMotion)
{
    ConsoleRig rig;
    for (const char* line : {"velocity 0.1 0", "move 10", "clear", "{\"type\":\"velocity\"}", "estop-clear"}) {
        rig.send(line);
        EXPECT_EQ(rig.output.last(), "ERR unknown command or wrong arguments; type help") << line;
    }
    rig.send("set a b c d");
    EXPECT_EQ(rig.output.last(), "ERR too many arguments");
    rig.send("   ");
    EXPECT_EQ(rig.target.entered, 0);
}

TEST(MaintenanceConsoleTest, RestartClearsIntentFirst)
{
    ConsoleRig rig;
    rig.send("restart");
    EXPECT_EQ(rig.target.entered, 1);
    EXPECT_EQ(rig.target.restarts, 1);
}
