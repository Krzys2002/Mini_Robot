#pragma once
// Persistent configuration store with atomic replacement and safe rollback (#159).
//
// Two storage slots (A/B) hold encoded records. Each record carries a generation; the
// newest intact record decides the state:
//   - both slots empty                      -> Unprovisioned (cannot arm);
//   - newest intact record is valid         -> Provisioned;
//   - newest intact record is out-of-version or has invalid content, two intact
//     records conflict, or only damaged records exist -> Invalid (cannot arm);
//   - a slot cannot be read                 -> StorageUnavailable (cannot arm).
// An older intact record never replaces a newer one that is unusable, so a superseded
// configuration cannot come back silently. A damaged (torn/corrupt) record is ignored:
// provisioning only ever writes the slot that does not hold the newest record, so a
// torn slot is the interrupted write and the other slot is the previous valid record.
//
// Writes happen only through provision() and factoryReset(), which the maintenance
// console calls with outputs disabled. Nothing on a command, telemetry or boot path
// writes, so flash wear is bounded by deliberate maintenance actions.
#include "provisioning/RobotConfigRecord.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace codecell::provisioning {

enum class Slot : std::uint8_t { A = 0, B = 1 };
inline constexpr std::array<Slot, 2> kSlots{Slot::A, Slot::B};
[[nodiscard]] constexpr Slot otherSlot(Slot slot) noexcept { return slot == Slot::A ? Slot::B : Slot::A; }
[[nodiscard]] constexpr const char* toString(Slot slot) noexcept { return slot == Slot::A ? "A" : "B"; }

enum class StorageStatus : std::uint8_t {
    Ok,
    NotFound,  // The slot holds nothing.
    TooLarge,  // The stored value does not fit the buffer; reported as LengthMismatch.
    Failed,    // The storage could not be read.
};

struct StorageRead {
    StorageStatus status{StorageStatus::Failed};
    std::size_t length{0};
};

// Non-volatile storage boundary. Production: ESP NVS (src/EspProvisioningAdapters.hpp).
// Called from the maintenance/loop context and from start-up only, never from a
// timing-critical task or a radio callback.
class IRecordStorage {
public:
    virtual ~IRecordStorage() = default;
    [[nodiscard]] virtual StorageRead read(Slot slot, std::span<std::uint8_t> buffer) noexcept = 0;
    // Replaces one slot. A write cut by power loss may leave that slot torn, never the other.
    [[nodiscard]] virtual bool write(Slot slot, std::span<const std::uint8_t> record) noexcept = 0;
    // Removes one slot. An absent slot counts as erased.
    [[nodiscard]] virtual bool erase(Slot slot) noexcept = 0;
    // Removes every stored BLE bond record (#157 factory-reset policy). Absent counts as erased.
    [[nodiscard]] virtual bool eraseControllerBonds() noexcept = 0;
};

enum class ProvisioningState : std::uint8_t { Provisioned, Unprovisioned, Invalid, StorageUnavailable };

[[nodiscard]] constexpr const char* toString(ProvisioningState state) noexcept
{
    switch (state) {
    case ProvisioningState::Provisioned: return "Provisioned";
    case ProvisioningState::Unprovisioned: return "Unprovisioned";
    case ProvisioningState::Invalid: return "Invalid";
    case ProvisioningState::StorageUnavailable: return "StorageUnavailable";
    }
    return "Unknown";
}

struct LoadResult {
    ProvisioningState state{ProvisioningState::StorageUnavailable};
    RecordError reason{RecordError::None};          // Why the state is Invalid.
    std::optional<ValidatedRobotConfig> config{};   // Set only when Provisioned.
    std::optional<std::uint32_t> generation{};      // Of the newest intact record.
    std::optional<Slot> newestSlot{};               // Slot of that record.

    // The only condition under which the runtime may become ready.
    [[nodiscard]] bool armable() const noexcept { return state == ProvisioningState::Provisioned && config; }
};

enum class WriteOutcome : std::uint8_t { Written, RejectedInvalid, StorageUnavailable, WriteFailed, VerifyFailed };
enum class ResetOutcome : std::uint8_t { Reset, BondEraseFailed, StorageEraseFailed, VerifyFailed };

[[nodiscard]] constexpr const char* toString(WriteOutcome outcome) noexcept
{
    switch (outcome) {
    case WriteOutcome::Written: return "Written";
    case WriteOutcome::RejectedInvalid: return "RejectedInvalid";
    case WriteOutcome::StorageUnavailable: return "StorageUnavailable";
    case WriteOutcome::WriteFailed: return "WriteFailed";
    case WriteOutcome::VerifyFailed: return "VerifyFailed";
    }
    return "Unknown";
}

[[nodiscard]] constexpr const char* toString(ResetOutcome outcome) noexcept
{
    switch (outcome) {
    case ResetOutcome::Reset: return "Reset";
    case ResetOutcome::BondEraseFailed: return "BondEraseFailed";
    case ResetOutcome::StorageEraseFailed: return "StorageEraseFailed";
    case ResetOutcome::VerifyFailed: return "VerifyFailed";
    }
    return "Unknown";
}

struct ProvisionResult {
    WriteOutcome outcome{WriteOutcome::RejectedInvalid};
    RecordError error{RecordError::None};  // Validation error of the rejected draft.
    LoadResult after{};                    // Stored state after the attempt.
};

struct ResetResult {
    ResetOutcome outcome{ResetOutcome::VerifyFailed};
    LoadResult after{};
};

// Serial-number comparison, so a wrapped uint32 generation still orders correctly.
[[nodiscard]] constexpr bool isNewer(std::uint32_t candidate, std::uint32_t reference) noexcept
{
    return static_cast<std::int32_t>(candidate - reference) > 0;
}

class ConfigStore {
public:
    explicit ConfigStore(IRecordStorage& storage) noexcept : storage_(storage) {}

    // Reads both slots; never writes.
    [[nodiscard]] LoadResult load() noexcept
    {
        std::array<SlotView, kSlots.size()> views{};
        for (const auto slot : kSlots) {
            auto& view = views[index(slot)];
            view = inspect(slot);
            if (view.kind == SlotView::Kind::Unreadable) return {};
        }
        const SlotView* newest = nullptr;
        bool conflict = false;
        for (const auto& view : views) {
            if (view.kind != SlotView::Kind::Intact) continue;
            if (!newest || isNewer(*view.decoded.generation, *newest->decoded.generation)) {
                newest = &view;
                conflict = false;
            } else if (*view.decoded.generation == *newest->decoded.generation && !sameBytes(view, *newest)) {
                conflict = true;
            }
        }
        LoadResult result;
        if (!newest) {
            for (const auto& view : views) {
                if (view.kind == SlotView::Kind::Damaged) {
                    result.state = ProvisioningState::Invalid;
                    result.reason = view.decoded.error;
                    return result;
                }
            }
            result.state = ProvisioningState::Unprovisioned;
            return result;
        }
        result.generation = newest->decoded.generation;
        result.newestSlot = newest->slot;
        if (conflict) {
            result.state = ProvisioningState::Invalid;
            result.reason = RecordError::ConflictingRecords;
            return result;
        }
        if (newest->decoded.error != RecordError::None) {
            result.state = ProvisioningState::Invalid;
            result.reason = newest->decoded.error;
            return result;
        }
        result.config = ValidatedRobotConfig::validated(newest->decoded.config);
        result.state = result.config ? ProvisioningState::Provisioned : ProvisioningState::Invalid;
        return result;
    }

    // Initial provisioning and deliberate reprovisioning. The draft is validated before
    // any write; the new record goes to the slot without the newest record, with the
    // next generation, and is read back. A failure at any point leaves the previous
    // newest record (or the unprovisioned state) in charge.
    [[nodiscard]] ProvisionResult provision(const RobotConfig& config) noexcept
    {
        if (const auto error = validate(config); error != RecordError::None)
            return {WriteOutcome::RejectedInvalid, error, load()};
        const auto before = load();
        if (before.state == ProvisioningState::StorageUnavailable)
            return {WriteOutcome::StorageUnavailable, RecordError::None, before};
        const Slot target = before.newestSlot ? otherSlot(*before.newestSlot) : Slot::A;
        const std::uint32_t generation = before.generation ? *before.generation + 1 : kFirstGeneration;
        const auto bytes = encode(config, generation);
        if (!storage_.write(target, bytes)) return {WriteOutcome::WriteFailed, RecordError::None, load()};
        auto after = load();
        if (!after.armable() || after.generation != generation || after.newestSlot != target
            || after.config->raw() != config)
            return {WriteOutcome::VerifyFailed, after.reason, after};
        return {WriteOutcome::Written, RecordError::None, after};
    }

    // Factory reset: bonds first, then the older slot, then the newest one. An
    // interruption therefore leaves either the unchanged newest record or nothing,
    // never an older record; running the reset again completes it.
    [[nodiscard]] ResetResult factoryReset() noexcept
    {
        const auto before = load();
        if (!storage_.eraseControllerBonds()) return {ResetOutcome::BondEraseFailed, before};
        const Slot last = before.newestSlot.value_or(Slot::B);
        if (!storage_.erase(otherSlot(last)) || !storage_.erase(last))
            return {ResetOutcome::StorageEraseFailed, load()};
        auto after = load();
        if (after.state != ProvisioningState::Unprovisioned) return {ResetOutcome::VerifyFailed, after};
        return {ResetOutcome::Reset, after};
    }

    static constexpr std::uint32_t kFirstGeneration = 1;

private:
    struct SlotView {
        enum class Kind : std::uint8_t { Empty, Intact, Damaged, Unreadable } kind{Kind::Unreadable};
        Slot slot{Slot::A};
        DecodedRecord decoded{};
        std::array<std::uint8_t, kMaxRecordBytes> bytes{};
        std::size_t length{0};
    };

    [[nodiscard]] static constexpr std::size_t index(Slot slot) noexcept { return static_cast<std::size_t>(slot); }

    [[nodiscard]] static bool sameBytes(const SlotView& a, const SlotView& b) noexcept
    {
        if (a.length != b.length) return false;
        for (std::size_t i = 0; i < a.length; ++i)
            if (a.bytes[i] != b.bytes[i]) return false;
        return true;
    }

    [[nodiscard]] SlotView inspect(Slot slot) noexcept
    {
        SlotView view;
        view.slot = slot;
        const auto read = storage_.read(slot, view.bytes);
        switch (read.status) {
        case StorageStatus::NotFound: view.kind = SlotView::Kind::Empty; return view;
        case StorageStatus::TooLarge:
            view.kind = SlotView::Kind::Damaged;
            view.decoded.error = RecordError::LengthMismatch;
            return view;
        case StorageStatus::Failed: view.kind = SlotView::Kind::Unreadable; return view;
        case StorageStatus::Ok: break;
        }
        if (read.length > view.bytes.size()) {
            view.kind = SlotView::Kind::Damaged;
            view.decoded.error = RecordError::LengthMismatch;
            return view;
        }
        view.length = read.length;
        view.decoded = decode(std::span<const std::uint8_t>{view.bytes}.first(read.length));
        view.kind = view.decoded.generation ? SlotView::Kind::Intact : SlotView::Kind::Damaged;
        return view;
    }

    IRecordStorage& storage_;
};
} // namespace codecell::provisioning
