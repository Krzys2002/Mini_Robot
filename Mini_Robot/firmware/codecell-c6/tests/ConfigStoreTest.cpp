// Host tests of the #159 A/B store: absent/corrupt/partial/out-of-version records, slot
// selection and rollback, power interruption at every step and byte of provisioning and
// factory reset, storage failures and the write (wear) policy.
#include "ProvisioningFakes.hpp"
#include "provisioning/ConfigStore.hpp"
#include <gtest/gtest.h>
#include <limits>
#include <vector>

using namespace codecell::provisioning;
using namespace codecell::provisioning::fakes;

namespace {
std::vector<std::uint8_t> withVersion(const RecordBytes& record, std::uint8_t version)
{
    std::vector<std::uint8_t> bytes(record.begin(), record.end());
    bytes[4] = version;
    const auto at = bytes.size() - kChecksumBytes;
    const auto crc = crc32(std::span<const std::uint8_t>{bytes}.first(at));
    for (std::size_t i = 0; i < kChecksumBytes; ++i) bytes[at + i] = static_cast<std::uint8_t>(crc >> (8 * i));
    return bytes;
}

// The only states an interrupted operation may leave: exactly `before`, exactly `after`,
// or (where one side is "no record") a safe, non-armable provisioning state.
void expectOldOrNew(const LoadResult& state, const std::optional<RobotConfig>& before,
                    const std::optional<RobotConfig>& after, const char* what, int cut, std::size_t torn)
{
    const auto matches = [&](const std::optional<RobotConfig>& expected) {
        if (!expected)
            return !state.armable() && !state.config
                && (state.state == ProvisioningState::Unprovisioned || state.state == ProvisioningState::Invalid);
        return state.armable() && state.config->raw() == *expected;
    };
    EXPECT_TRUE(matches(before) || matches(after))
        << what << " cut=" << cut << " torn=" << torn << " state=" << toString(state.state)
        << " reason=" << toString(state.reason);
}
} // namespace

TEST(ConfigStoreTest, EmptyStorageIsUnprovisionedAndNotArmable)
{
    FakeRecordStorage storage;
    ConfigStore store{storage};
    const auto state = store.load();
    EXPECT_EQ(state.state, ProvisioningState::Unprovisioned);
    EXPECT_FALSE(state.armable());
    EXPECT_FALSE(state.config);
}

TEST(ConfigStoreTest, ProvisionedRecordSurvivesARebootOfTheStore)
{
    FakeRecordStorage storage;
    {
        ConfigStore store{storage};
        const auto written = store.provision(validConfig(0));
        ASSERT_EQ(written.outcome, WriteOutcome::Written);
        EXPECT_EQ(written.after.newestSlot, Slot::A);
        EXPECT_EQ(written.after.generation, ConfigStore::kFirstGeneration);
    }
    storage.reboot();
    ConfigStore afterReboot{storage};
    const auto state = afterReboot.load();
    ASSERT_TRUE(state.armable());
    EXPECT_EQ(state.config->robotId(), 0U);
    EXPECT_EQ(state.config->raw(), validConfig(0));
}

TEST(ConfigStoreTest, ReprovisioningAlternatesSlotsAndKeepsThePreviousRecord)
{
    FakeRecordStorage storage;
    ConfigStore store{storage};
    ASSERT_EQ(store.provision(validConfig(1)).outcome, WriteOutcome::Written);
    const auto second = store.provision(validConfig(2));
    ASSERT_EQ(second.outcome, WriteOutcome::Written);
    EXPECT_EQ(second.after.newestSlot, Slot::B);
    EXPECT_EQ(second.after.generation, 2U);
    const auto third = store.provision(validConfig(std::numeric_limits<RobotId>::max()));
    ASSERT_EQ(third.outcome, WriteOutcome::Written);
    EXPECT_EQ(third.after.newestSlot, Slot::A);
    EXPECT_EQ(third.after.config->robotId(), std::numeric_limits<RobotId>::max());
    // Slot B still holds the previous record (the rollback copy).
    ASSERT_TRUE(storage.slots[1]);
    EXPECT_EQ(decode(*storage.slots[1]).config.robotId, 2U);
}

TEST(ConfigStoreTest, InvalidDraftIsRejectedBeforeAnyWrite)
{
    FakeRecordStorage storage;
    ConfigStore store{storage};
    auto bad = validConfig();
    bad.maxLinearMillimetresPerSecond = 0;
    const auto result = store.provision(bad);
    EXPECT_EQ(result.outcome, WriteOutcome::RejectedInvalid);
    EXPECT_EQ(result.error, RecordError::LinearLimitOutOfRange);
    EXPECT_EQ(storage.mutations(), 0);
}

TEST(ConfigStoreTest, TornNewestSlotFallsBackToThePreviousValidRecord)
{
    FakeRecordStorage storage;
    storage.put(Slot::A, encode(validConfig(1), 5));
    auto torn = encode(validConfig(2), 6);
    std::vector<std::uint8_t> partial(torn.begin(), torn.begin() + 20);
    storage.put(Slot::B, partial);
    ConfigStore store{storage};
    const auto state = store.load();
    ASSERT_TRUE(state.armable());
    EXPECT_EQ(state.config->robotId(), 1U);
    EXPECT_EQ(state.generation, 5U);
}

TEST(ConfigStoreTest, OnlyDamagedRecordsAreInvalidNotUnprovisioned)
{
    FakeRecordStorage storage;
    auto record = encode(validConfig(), 1);
    record[20] ^= 0x01;
    storage.put(Slot::A, record);
    ConfigStore store{storage};
    const auto state = store.load();
    EXPECT_EQ(state.state, ProvisioningState::Invalid);
    EXPECT_EQ(state.reason, RecordError::ChecksumMismatch);
    EXPECT_FALSE(state.armable());
}

TEST(ConfigStoreTest, NewerUnsupportedVersionIsNotReplacedByTheOlderRecord)
{
    // A downgrade must not silently resurrect the configuration that a newer image replaced.
    FakeRecordStorage storage;
    storage.put(Slot::A, encode(validConfig(1), 5));
    storage.put(Slot::B, withVersion(encode(validConfig(2), 6), 2));
    ConfigStore store{storage};
    const auto state = store.load();
    EXPECT_EQ(state.state, ProvisioningState::Invalid);
    EXPECT_EQ(state.reason, RecordError::UnsupportedVersion);
    EXPECT_FALSE(state.armable());
    EXPECT_EQ(state.newestSlot, Slot::B);
}

TEST(ConfigStoreTest, OlderUnsupportedVersionDoesNotHideANewerValidRecord)
{
    FakeRecordStorage storage;
    storage.put(Slot::A, withVersion(encode(validConfig(1), 5), 0));
    storage.put(Slot::B, encode(validConfig(2), 6));
    ConfigStore store{storage};
    const auto state = store.load();
    ASSERT_TRUE(state.armable());
    EXPECT_EQ(state.config->robotId(), 2U);
}

TEST(ConfigStoreTest, NewerRecordWithInvalidContentBlocksArming)
{
    FakeRecordStorage storage;
    storage.put(Slot::A, encode(validConfig(1), 5));
    auto bad = validConfig(2);
    bad.maxCommandValidityMilliseconds = 0;
    storage.put(Slot::B, encode(bad, 6));
    ConfigStore store{storage};
    const auto state = store.load();
    EXPECT_EQ(state.state, ProvisioningState::Invalid);
    EXPECT_EQ(state.reason, RecordError::ValidityOutOfRange);
}

TEST(ConfigStoreTest, ReprovisioningRecoversFromAnInvalidNewestRecord)
{
    FakeRecordStorage storage;
    storage.put(Slot::A, encode(validConfig(1), 5));
    storage.put(Slot::B, withVersion(encode(validConfig(2), 6), 2));
    ConfigStore store{storage};
    const auto result = store.provision(validConfig(3));
    ASSERT_EQ(result.outcome, WriteOutcome::Written);
    EXPECT_EQ(result.after.newestSlot, Slot::A);  // Overwrites the older slot, not the newest.
    EXPECT_EQ(result.after.generation, 7U);
    EXPECT_EQ(result.after.config->robotId(), 3U);
}

TEST(ConfigStoreTest, ConflictingRecordsWithTheSameGenerationAreInvalid)
{
    FakeRecordStorage storage;
    storage.put(Slot::A, encode(validConfig(1), 5));
    storage.put(Slot::B, encode(validConfig(2), 5));
    ConfigStore store{storage};
    EXPECT_EQ(store.load().reason, RecordError::ConflictingRecords);
    EXPECT_FALSE(store.load().armable());

    storage.put(Slot::B, encode(validConfig(1), 5));  // Identical copies are not a conflict.
    EXPECT_TRUE(store.load().armable());
}

TEST(ConfigStoreTest, GenerationOrderSurvivesWrap)
{
    FakeRecordStorage storage;
    storage.put(Slot::A, encode(validConfig(1), std::numeric_limits<std::uint32_t>::max()));
    storage.put(Slot::B, encode(validConfig(2), 0));
    ConfigStore store{storage};
    EXPECT_EQ(store.load().config->robotId(), 2U);
    const auto next = store.provision(validConfig(3));
    ASSERT_EQ(next.outcome, WriteOutcome::Written);
    EXPECT_EQ(next.after.generation, 1U);
    EXPECT_EQ(next.after.newestSlot, Slot::A);
}

TEST(ConfigStoreTest, OversizedOrUnreadableSlotsAreSafe)
{
    FakeRecordStorage storage;
    storage.put(Slot::A, std::vector<std::uint8_t>(kMaxRecordBytes + 1, 0));
    ConfigStore store{storage};
    EXPECT_EQ(store.load().state, ProvisioningState::Invalid);
    EXPECT_EQ(store.load().reason, RecordError::LengthMismatch);

    FakeRecordStorage failing;
    failing.put(Slot::A, encode(validConfig(), 1));
    failing.failRead[1] = true;
    ConfigStore unreadable{failing};
    EXPECT_EQ(unreadable.load().state, ProvisioningState::StorageUnavailable);
    EXPECT_FALSE(unreadable.load().armable());
    EXPECT_EQ(unreadable.provision(validConfig(2)).outcome, WriteOutcome::StorageUnavailable);
    EXPECT_EQ(failing.writes, 0);
}

TEST(ConfigStoreTest, WriteFailureAndLostWritesKeepThePreviousRecord)
{
    FakeRecordStorage storage;
    ConfigStore store{storage};
    ASSERT_EQ(store.provision(validConfig(1)).outcome, WriteOutcome::Written);
    storage.failWrite = true;
    const auto failed = store.provision(validConfig(2));
    EXPECT_EQ(failed.outcome, WriteOutcome::WriteFailed);
    EXPECT_EQ(failed.after.config->robotId(), 1U);

    storage.failWrite = false;
    storage.dropWrites = true;  // Reports success, stores nothing: caught by read-back.
    const auto lost = store.provision(validConfig(3));
    EXPECT_EQ(lost.outcome, WriteOutcome::VerifyFailed);
    EXPECT_EQ(store.load().config->robotId(), 1U);
}

TEST(ConfigStoreTest, PowerCutAtAnyByteOfProvisioningLeavesOldOrNew)
{
    for (const bool initiallyProvisioned : {false, true}) {
        const std::optional<RobotConfig> before =
            initiallyProvisioned ? std::optional<RobotConfig>{validConfig(1)} : std::nullopt;
        const RobotConfig after = validConfig(2);
        for (std::size_t torn = 0; torn <= kRecordV1Bytes; ++torn) {
            FakeRecordStorage storage;
            if (initiallyProvisioned) storage.put(Slot::A, encode(*before, 4));
            storage.mutationsBeforeCut = 0;
            storage.tornBytes = torn;
            ConfigStore store{storage};
            (void)store.provision(after);
            storage.reboot();
            ConfigStore rebooted{storage};
            expectOldOrNew(rebooted.load(), before, after, "provision", 0, torn);
            // Recovery: provisioning again after the cut completes normally.
            const auto retry = rebooted.provision(after);
            EXPECT_EQ(retry.outcome, WriteOutcome::Written) << torn;
            EXPECT_EQ(rebooted.load().config->raw(), after) << torn;
        }
    }
}

TEST(ConfigStoreTest, RepeatedInterruptedReprovisioningNeverLosesTheLastCommittedRecord)
{
    FakeRecordStorage storage;
    ConfigStore store{storage};
    RobotConfig committed = validConfig(100);
    ASSERT_EQ(store.provision(committed).outcome, WriteOutcome::Written);
    for (RobotId id = 101; id < 140; ++id) {
        const RobotConfig previous = committed;
        if (id % 3 != 0) {
            storage.mutationsBeforeCut = 0;
            storage.tornBytes = static_cast<std::size_t>(id) % (kRecordV1Bytes + 1);
        }
        const auto result = store.provision(validConfig(id));
        storage.reboot();
        ConfigStore rebooted{storage};
        const auto state = rebooted.load();
        ASSERT_TRUE(state.armable()) << id;
        if (result.outcome == WriteOutcome::Written) {
            EXPECT_EQ(state.config->raw(), validConfig(id)) << id;
        } else {
            expectOldOrNew(state, previous, validConfig(id), "repeat", 0, storage.tornBytes);
        }
        committed = state.config->raw();
    }
}

TEST(ConfigStoreTest, FactoryResetErasesRecordsAndBonds)
{
    FakeRecordStorage storage;
    ConfigStore store{storage};
    ASSERT_EQ(store.provision(validConfig(1)).outcome, WriteOutcome::Written);
    ASSERT_EQ(store.provision(validConfig(2)).outcome, WriteOutcome::Written);
    storage.bondsPresent = true;
    const auto reset = store.factoryReset();
    EXPECT_EQ(reset.outcome, ResetOutcome::Reset);
    EXPECT_EQ(reset.after.state, ProvisioningState::Unprovisioned);
    EXPECT_FALSE(storage.bondsPresent);
    EXPECT_FALSE(storage.slots[0]);
    EXPECT_FALSE(storage.slots[1]);
}

TEST(ConfigStoreTest, FactoryResetRefusesToEraseWhenBondsCannotBeRemoved)
{
    FakeRecordStorage storage;
    ConfigStore store{storage};
    ASSERT_EQ(store.provision(validConfig(1)).outcome, WriteOutcome::Written);
    storage.failBondErase = true;
    EXPECT_EQ(store.factoryReset().outcome, ResetOutcome::BondEraseFailed);
    EXPECT_TRUE(store.load().armable());  // Retry is possible; nothing half-erased.
}

// Review finding: with one slot unreadable the reset cannot tell which slot is newest. If it
// guessed (B last) and was cut after erasing A, the superseded record in B became armable.
// It must refuse before erasing anything, including the bonds.
TEST(ConfigStoreTest, FactoryResetRefusesWhenSlotOrderingIsUnknown)
{
    for (int cut = 0; cut < 3; ++cut) {
        FakeRecordStorage storage;
        storage.put(Slot::A, encode(validConfig(30), 3));  // Newest.
        storage.put(Slot::B, encode(validConfig(20), 2));  // Superseded.
        storage.bondsPresent = true;
        storage.failRead[0] = true;
        storage.mutationsBeforeCut = cut;
        ConfigStore store{storage};
        const auto reset = store.factoryReset();
        EXPECT_EQ(reset.outcome, ResetOutcome::StorageUnavailable) << cut;
        EXPECT_EQ(reset.after.state, ProvisioningState::StorageUnavailable) << cut;
        EXPECT_EQ(storage.mutations(), 0) << cut;
        EXPECT_TRUE(storage.bondsPresent) << cut;

        storage.failRead[0] = false;
        storage.reboot();
        ConfigStore rebooted{storage};
        const auto state = rebooted.load();
        ASSERT_TRUE(state.armable()) << cut;
        EXPECT_EQ(state.config->raw(), validConfig(30)) << cut;  // Never the superseded 20.
    }
}

TEST(ConfigStoreTest, PowerCutDuringFactoryResetNeverResurrectsAnOlderRecord)
{
    for (int cut = 0; cut < 3; ++cut) {
        FakeRecordStorage storage;
        storage.put(Slot::A, encode(validConfig(1), 4));  // Older.
        storage.put(Slot::B, encode(validConfig(2), 5));  // Newest.
        storage.bondsPresent = true;
        storage.mutationsBeforeCut = cut;
        ConfigStore store{storage};
        (void)store.factoryReset();
        storage.reboot();
        ConfigStore rebooted{storage};
        const auto state = rebooted.load();
        expectOldOrNew(state, validConfig(2), std::nullopt, "reset", cut, 0);
        // Re-running the reset completes it.
        EXPECT_EQ(rebooted.factoryReset().outcome, ResetOutcome::Reset) << cut;
        EXPECT_EQ(rebooted.load().state, ProvisioningState::Unprovisioned) << cut;
        EXPECT_FALSE(storage.bondsPresent) << cut;
    }
}

TEST(ConfigStoreTest, LoadNeverWritesAndProvisioningWritesOneSlot)
{
    FakeRecordStorage storage;
    ConfigStore store{storage};
    for (int i = 0; i < 100; ++i) (void)store.load();
    EXPECT_EQ(storage.mutations(), 0);
    ASSERT_EQ(store.provision(validConfig()).outcome, WriteOutcome::Written);
    EXPECT_EQ(storage.writes, 1);
    EXPECT_EQ(storage.erases, 0);
    for (int i = 0; i < 100; ++i) (void)store.load();
    EXPECT_EQ(storage.mutations(), 1);
}
