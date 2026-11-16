// Host tests of the #159 A/B store: absent/corrupt/partial/out-of-version records, slot
// selection and rollback, the transaction marker (interrupted write versus damaged committed
// record, unfinished factory reset), power interruption at every step and byte of
// provisioning and factory reset, storage failures and the write (wear) policy.
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

std::vector<std::uint8_t> flipPayloadByte(const std::vector<std::uint8_t>& record)
{
    auto bytes = record;
    bytes[detail::kRobotIdAt + 7] ^= 0x01;  // Inside the payload; the CRC is left as it was.
    return bytes;
}

RobotConfig withOutputBound(RobotId id, std::uint8_t percent)
{
    auto config = validConfig(id);
    config.maxDriveOutputPercent = percent;
    return config;
}

// Mutations of one provisioning: marker write, slot write, marker erase.
constexpr int kProvisioningMutations = 3;
// Mutations of one factory reset: bonds, marker write, two slot erases, marker erase.
constexpr int kResetMutations = 5;
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

TEST(ConfigStoreTest, TornSlotNamedByTheMarkerFallsBackToThePreviousValidRecord)
{
    FakeRecordStorage storage;
    storage.put(Slot::A, encode(validConfig(1), 5));
    auto torn = encode(validConfig(2), 6);
    std::vector<std::uint8_t> partial(torn.begin(), torn.begin() + 20);
    storage.put(Slot::B, partial);
    storage.putMarker(encodeMarker({MarkerKind::Provisioning, Slot::B, true}));
    ConfigStore store{storage};
    const auto state = store.load();
    ASSERT_TRUE(state.armable());
    EXPECT_EQ(state.config->robotId(), 1U);
    EXPECT_EQ(state.generation, 5U);
    EXPECT_EQ(state.pending, PendingChange::Provisioning);
}

TEST(ConfigStoreTest, DamagedSlotWithoutAnInterruptedWriteIsInvalid)
{
    // The same bytes without the marker: nothing explains the damage, so it may be the
    // committed newest record. Neither slot may arm.
    FakeRecordStorage storage;
    storage.put(Slot::A, encode(validConfig(1), 5));
    auto torn = encode(validConfig(2), 6);
    storage.put(Slot::B, std::vector<std::uint8_t>(torn.begin(), torn.begin() + 20));
    ConfigStore store{storage};
    const auto state = store.load();
    EXPECT_EQ(state.state, ProvisioningState::Invalid);
    EXPECT_EQ(state.reason, RecordError::CommittedRecordDamaged);
    EXPECT_FALSE(state.armable());

    // A marker that names the other slot does not explain it either.
    storage.putMarker(encodeMarker({MarkerKind::Provisioning, Slot::A, true}));
    EXPECT_EQ(store.load().reason, RecordError::CommittedRecordDamaged);
    EXPECT_FALSE(store.load().armable());
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
    EXPECT_EQ(state.reason, RecordError::CommittedRecordDamaged);
    EXPECT_FALSE(state.armable());
}

// Review finding (P1): a committed newest record that is corrupted later, with no
// provisioning interrupted, used to hand control back to the superseded record and its
// higher output bound.
TEST(ConfigStoreTest, CorruptedCommittedNewestRecordNeverRestoresTheSupersededRecord)
{
    FakeRecordStorage storage;
    ConfigStore store{storage};
    ASSERT_EQ(store.provision(withOutputBound(7, 80)).outcome, WriteOutcome::Written);
    const auto reduced = store.provision(withOutputBound(8, 10));
    ASSERT_EQ(reduced.outcome, WriteOutcome::Written);
    ASSERT_EQ(reduced.after.newestSlot, Slot::B);
    ASSERT_FALSE(storage.marker);

    storage.put(Slot::B, flipPayloadByte(*storage.slots[1]));
    storage.reboot();
    ConfigStore rebooted{storage};
    const auto state = rebooted.load();
    EXPECT_FALSE(state.armable());
    EXPECT_EQ(state.state, ProvisioningState::Invalid);
    EXPECT_EQ(state.reason, RecordError::CommittedRecordDamaged);
    EXPECT_FALSE(state.config);

    // Deliberate reprovisioning overwrites the damaged slot and recovers.
    const auto recovered = rebooted.provision(withOutputBound(8, 10));
    ASSERT_EQ(recovered.outcome, WriteOutcome::Written);
    EXPECT_EQ(recovered.after.newestSlot, Slot::B);
    EXPECT_EQ(recovered.after.config->raw(), withOutputBound(8, 10));
}

TEST(ConfigStoreTest, CorruptedOlderSlotAlsoFailsClosed)
{
    // A damaged record's generation cannot be read, so the store cannot tell a damaged older
    // copy from a damaged newer one: it refuses to arm either way.
    FakeRecordStorage storage;
    ConfigStore store{storage};
    ASSERT_EQ(store.provision(validConfig(1)).outcome, WriteOutcome::Written);
    ASSERT_EQ(store.provision(validConfig(2)).outcome, WriteOutcome::Written);
    storage.put(Slot::A, flipPayloadByte(*storage.slots[0]));
    EXPECT_EQ(store.load().reason, RecordError::CommittedRecordDamaged);
    EXPECT_FALSE(store.load().armable());
}

// Reprovisioning a store that is already invalid must not let the older intact record take
// over when the write is cut: the marker records that no armable record preceded it.
TEST(ConfigStoreTest, PowerCutWhileReprovisioningAnInvalidStoreNeverArmsTheOlderRecord)
{
    for (int cut = 0; cut < kProvisioningMutations; ++cut) {
        for (std::size_t torn = 0; torn <= kRecordV1Bytes; ++torn) {
            FakeRecordStorage storage;
            storage.put(Slot::A, encode(withOutputBound(7, 80), 5));
            const auto newer = encode(withOutputBound(8, 10), 6);
            storage.put(Slot::B, flipPayloadByte(std::vector<std::uint8_t>(newer.begin(), newer.end())));
            storage.mutationsBeforeCut = cut;
            storage.tornBytes = torn;
            ConfigStore store{storage};
            ASSERT_FALSE(store.load().armable());
            (void)store.provision(withOutputBound(9, 10));
            storage.reboot();
            ConfigStore rebooted{storage};
            const auto state = rebooted.load();
            expectOldOrNew(state, std::nullopt, withOutputBound(9, 10), "reprovision-invalid", cut, torn);
            EXPECT_FALSE(state.armable() && state.config->raw() == withOutputBound(7, 80)) << cut << " " << torn;
            const auto retry = rebooted.provision(withOutputBound(9, 10));
            EXPECT_EQ(retry.outcome, WriteOutcome::Written) << cut << " " << torn;
        }
    }
}

TEST(ConfigStoreTest, FirstProvisioningCutWhileWritingTheSlotIsNotArmable)
{
    FakeRecordStorage storage;
    storage.mutationsBeforeCut = 1;  // The marker is written; the slot write is cut.
    storage.tornBytes = 20;
    ConfigStore store{storage};
    EXPECT_EQ(store.provision(validConfig(1)).outcome, WriteOutcome::WriteFailed);
    storage.reboot();
    const auto state = ConfigStore{storage}.load();
    EXPECT_EQ(state.state, ProvisioningState::Invalid);
    EXPECT_EQ(state.reason, RecordError::ProvisioningIncomplete);
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
    EXPECT_EQ(store.load().reason, RecordError::CommittedRecordDamaged);

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

TEST(ConfigStoreTest, PowerCutAtAnyStepAndByteOfProvisioningLeavesOldOrNew)
{
    for (const bool initiallyProvisioned : {false, true}) {
        const std::optional<RobotConfig> before =
            initiallyProvisioned ? std::optional<RobotConfig>{validConfig(1)} : std::nullopt;
        const RobotConfig after = validConfig(2);
        for (int cut = 0; cut < kProvisioningMutations; ++cut) {
            for (std::size_t torn = 0; torn <= kRecordV1Bytes; ++torn) {
                FakeRecordStorage storage;
                if (initiallyProvisioned) storage.put(Slot::A, encode(*before, 4));
                storage.mutationsBeforeCut = cut;
                storage.tornBytes = torn;
                ConfigStore store{storage};
                EXPECT_NE(store.provision(after).outcome, WriteOutcome::Written) << cut << " " << torn;
                storage.reboot();
                ConfigStore rebooted{storage};
                expectOldOrNew(rebooted.load(), before, after, "provision", cut, torn);
                // Recovery: provisioning again after the cut completes normally.
                const auto retry = rebooted.provision(after);
                EXPECT_EQ(retry.outcome, WriteOutcome::Written) << cut << " " << torn;
                EXPECT_EQ(rebooted.load().config->raw(), after) << cut << " " << torn;
                EXPECT_EQ(rebooted.load().pending, PendingChange::None) << cut << " " << torn;
            }
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
        if (id % 4 != 0) {
            storage.mutationsBeforeCut = static_cast<int>(id % kProvisioningMutations);
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

TEST(ConfigStoreTest, MarkerEraseFailureIsNotReportedAsWritten)
{
    FakeRecordStorage storage;
    ConfigStore store{storage};
    ASSERT_EQ(store.provision(validConfig(1)).outcome, WriteOutcome::Written);
    storage.failMarkerErase = true;
    const auto result = store.provision(validConfig(2));
    EXPECT_EQ(result.outcome, WriteOutcome::CommitFailed);
    // The new record was read back and is in charge; the change is reported as unfinished.
    ASSERT_TRUE(result.after.armable());
    EXPECT_EQ(result.after.config->robotId(), 2U);
    EXPECT_EQ(result.after.pending, PendingChange::Provisioning);

    storage.failMarkerErase = false;
    const auto retry = store.provision(validConfig(2));
    EXPECT_EQ(retry.outcome, WriteOutcome::Written);
    EXPECT_EQ(retry.after.pending, PendingChange::None);
}

TEST(ConfigStoreTest, UnreadableMarkerIsStorageUnavailableAndBlocksEveryWrite)
{
    FakeRecordStorage storage;
    storage.put(Slot::A, encode(validConfig(1), 1));
    storage.failMarkerRead = true;
    storage.bondsPresent = true;
    ConfigStore store{storage};
    EXPECT_EQ(store.load().state, ProvisioningState::StorageUnavailable);
    EXPECT_FALSE(store.load().armable());
    EXPECT_EQ(store.provision(validConfig(2)).outcome, WriteOutcome::StorageUnavailable);
    EXPECT_EQ(store.factoryReset().outcome, ResetOutcome::StorageUnavailable);
    EXPECT_EQ(storage.mutations(), 0);
    EXPECT_TRUE(storage.bondsPresent);
}

TEST(ConfigStoreTest, DamagedMarkerIsIgnoredOnlyWhileBothSlotsAreIntact)
{
    // A torn marker means the cut came while the marker was written, before any slot.
    FakeRecordStorage storage;
    storage.put(Slot::A, encode(validConfig(1), 4));
    storage.put(Slot::B, encode(validConfig(2), 5));
    const auto marker = encodeMarker({MarkerKind::FactoryReset, Slot::A, false});
    storage.marker = std::vector<std::uint8_t>(marker.begin(), marker.begin() + 7);
    ConfigStore store{storage};
    auto state = store.load();
    ASSERT_TRUE(state.armable());
    EXPECT_EQ(state.config->robotId(), 2U);
    EXPECT_EQ(state.pending, PendingChange::DamagedMarker);

    // Two faults at once (damaged marker and damaged slot) fail closed.
    storage.put(Slot::B, flipPayloadByte(*storage.slots[1]));
    state = store.load();
    EXPECT_FALSE(state.armable());
    EXPECT_EQ(state.reason, RecordError::CommittedRecordDamaged);

    // An oversized marker is damaged too.
    storage.put(Slot::B, encode(validConfig(2), 5));
    storage.marker = std::vector<std::uint8_t>(kMarkerBytes + 1, 0);
    EXPECT_EQ(store.load().pending, PendingChange::DamagedMarker);
    EXPECT_TRUE(store.load().armable());
}

TEST(ConfigStoreTest, MarkerCodecRejectsAnythingButACompleteKnownMarker)
{
    for (const Marker marker : {Marker{MarkerKind::Provisioning, Slot::A, false},
                                Marker{MarkerKind::Provisioning, Slot::B, true},
                                Marker{MarkerKind::FactoryReset, Slot::A, false}}) {
        const auto bytes = encodeMarker(marker);
        EXPECT_EQ(decodeMarker(bytes), marker);
        for (std::size_t length = 0; length < bytes.size(); ++length)
            EXPECT_FALSE(decodeMarker(std::span<const std::uint8_t>{bytes}.first(length))) << length;
        for (std::size_t i = 0; i < bytes.size(); ++i) {
            for (int bit = 0; bit < 8; ++bit) {
                auto flipped = bytes;
                flipped[i] ^= static_cast<std::uint8_t>(1U << bit);
                EXPECT_FALSE(decodeMarker(flipped)) << i << " " << bit;
            }
        }
    }
    // Reserved values with a correct CRC are still refused.
    const auto withByte = [](std::size_t at, std::uint8_t value) {
        auto bytes = encodeMarker({MarkerKind::Provisioning, Slot::A, false});
        bytes[at] = value;
        const auto crc = crc32(std::span<const std::uint8_t>{bytes}.first(8));
        for (std::size_t i = 0; i < kChecksumBytes; ++i) bytes[8 + i] = static_cast<std::uint8_t>(crc >> (8 * i));
        return bytes;
    };
    EXPECT_FALSE(decodeMarker(withByte(4, 2)));     // Version.
    EXPECT_FALSE(decodeMarker(withByte(5, 3)));     // Kind.
    EXPECT_FALSE(decodeMarker(withByte(6, 2)));     // Slot.
    EXPECT_FALSE(decodeMarker(withByte(7, 0x02)));  // Reserved flag.
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
    for (int cut = 0; cut < kResetMutations; ++cut) {
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

// Review finding (P2): with two conflicting records (not armable), a reset cut after the
// bonds and the first slot erase used to leave the other record alone, and armable.
TEST(ConfigStoreTest, InterruptedResetOfConflictingRecordsNeverArms)
{
    for (int cut = 0; cut < kResetMutations; ++cut) {
        for (const std::size_t torn : {std::size_t{0}, std::size_t{7}, kMarkerBytes - 1}) {
            FakeRecordStorage storage;
            storage.put(Slot::A, encode(validConfig(7), 5));
            storage.put(Slot::B, encode(validConfig(8), 5));
            storage.bondsPresent = true;
            ConfigStore store{storage};
            ASSERT_FALSE(store.load().armable());
            storage.mutationsBeforeCut = cut;
            storage.tornBytes = torn;
            EXPECT_NE(store.factoryReset().outcome, ResetOutcome::Reset) << cut;
            storage.reboot();
            ConfigStore rebooted{storage};
            const auto state = rebooted.load();
            EXPECT_FALSE(state.armable()) << "cut=" << cut << " torn=" << torn << " state=" << toString(state.state)
                                          << " reason=" << toString(state.reason);
            EXPECT_EQ(rebooted.factoryReset().outcome, ResetOutcome::Reset) << cut;
            EXPECT_EQ(rebooted.load().state, ProvisioningState::Unprovisioned) << cut;
            EXPECT_FALSE(storage.marker) << cut;
        }
    }
}

TEST(ConfigStoreTest, ProvisioningIsRefusedUntilAnUnfinishedResetCompletes)
{
    FakeRecordStorage storage;
    ConfigStore store{storage};
    ASSERT_EQ(store.provision(validConfig(1)).outcome, WriteOutcome::Written);
    ASSERT_EQ(store.provision(validConfig(2)).outcome, WriteOutcome::Written);
    storage.mutationsBeforeCut = 2;  // Bonds and marker done; the first slot erase is cut.
    (void)store.factoryReset();
    storage.reboot();
    ConfigStore rebooted{storage};
    ASSERT_EQ(rebooted.load().reason, RecordError::FactoryResetIncomplete);
    const int before = storage.mutations();
    EXPECT_EQ(rebooted.provision(validConfig(3)).outcome, WriteOutcome::ResetIncomplete);
    EXPECT_EQ(storage.mutations(), before);
    ASSERT_EQ(rebooted.factoryReset().outcome, ResetOutcome::Reset);
    const auto fresh = rebooted.provision(validConfig(3));
    ASSERT_EQ(fresh.outcome, WriteOutcome::Written);
    EXPECT_EQ(fresh.after.config->robotId(), 3U);
}

TEST(ConfigStoreTest, LoadNeverWritesAndProvisioningWritesOneSlotAndTheMarker)
{
    FakeRecordStorage storage;
    ConfigStore store{storage};
    for (int i = 0; i < 100; ++i) (void)store.load();
    EXPECT_EQ(storage.mutations(), 0);
    ASSERT_EQ(store.provision(validConfig()).outcome, WriteOutcome::Written);
    EXPECT_EQ(storage.writes, 2);  // The marker, then the record.
    EXPECT_EQ(storage.markerWrites, 1);
    EXPECT_EQ(storage.erases, 1);  // The marker.
    EXPECT_FALSE(storage.marker);
    for (int i = 0; i < 100; ++i) (void)store.load();
    EXPECT_EQ(storage.mutations(), 3);
}

// Review follow-up: a second interruption of the retried change must not undo the first
// one's protection. Every pair of cut points, with the interrupted write torn.
TEST(ConfigStoreTest, TwoConsecutiveInterruptedResetsOfConflictingRecordsNeverArm)
{
    for (int first = 0; first < kResetMutations; ++first) {
        for (int second = 0; second < kResetMutations; ++second) {
            for (const std::size_t torn : {std::size_t{0}, std::size_t{7}, kMarkerBytes - 1}) {
                FakeRecordStorage storage;
                storage.put(Slot::A, encode(validConfig(7), 5));
                storage.put(Slot::B, encode(validConfig(8), 5));
                storage.tornBytes = torn;
                for (const int cut : {first, second}) {
                    storage.mutationsBeforeCut = cut;
                    (void)ConfigStore{storage}.factoryReset();
                    storage.reboot();
                    const auto state = ConfigStore{storage}.load();
                    EXPECT_FALSE(state.armable()) << first << "," << second << " torn=" << torn << " at " << cut
                                                  << " reason=" << toString(state.reason);
                }
                EXPECT_EQ(ConfigStore{storage}.factoryReset().outcome, ResetOutcome::Reset);
            }
        }
    }
}

TEST(ConfigStoreTest, TwoConsecutiveInterruptedProvisioningsLeaveOldOrNew)
{
    for (int first = 0; first < kProvisioningMutations; ++first) {
        for (int second = 0; second < kProvisioningMutations; ++second) {
            for (const std::size_t torn : {std::size_t{0}, std::size_t{7}, std::size_t{20}, kRecordV1Bytes - 1}) {
                FakeRecordStorage storage;
                storage.put(Slot::A, encode(withOutputBound(1, 80), 4));
                storage.tornBytes = torn;
                for (const int cut : {first, second}) {
                    storage.mutationsBeforeCut = cut;
                    (void)ConfigStore{storage}.provision(withOutputBound(2, 10));
                    storage.reboot();
                    expectOldOrNew(ConfigStore{storage}.load(), withOutputBound(1, 80), withOutputBound(2, 10),
                                   "provision-twice", cut, torn);
                }
                ConfigStore store{storage};
                EXPECT_EQ(store.provision(withOutputBound(2, 10)).outcome, WriteOutcome::Written);
            }
        }
    }
}

TEST(ConfigStoreTest, IdenticalMarkerIsNotRewritten)
{
    FakeRecordStorage storage;
    storage.put(Slot::A, encode(validConfig(7), 5));
    storage.putMarker(encodeMarker({MarkerKind::FactoryReset, Slot::A, false}));
    ConfigStore store{storage};
    ASSERT_EQ(store.factoryReset().outcome, ResetOutcome::Reset);
    EXPECT_EQ(storage.markerWrites, 0);
}
