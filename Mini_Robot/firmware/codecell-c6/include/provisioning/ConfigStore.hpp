#pragma once
// Persistent configuration store with atomic replacement and safe rollback (#159).
//
// Two storage slots (A/B) hold encoded records. Each record carries a generation; the
// newest intact record decides the state. A third, small value, the transaction marker,
// records which multi-step change is in progress, so that an interrupted change can be
// told apart from a committed record that was damaged later:
//   - a provisioning write puts "provisioning slot X" in the marker before it writes X and
//     erases the marker only after X was read back;
//   - a factory reset puts "factory reset" in the marker before it erases a slot and erases
//     the marker only after both slots are gone.
//
// load() rules:
//   - a slot or the marker cannot be read          -> StorageUnavailable (cannot arm);
//   - the marker says "factory reset"              -> Invalid / FactoryResetIncomplete;
//   - a slot is damaged (torn or corrupt) and the marker does not name it as the slot
//     being provisioned                            -> Invalid / CommittedRecordDamaged.
//     A committed record that went bad therefore never hands control to the older record:
//     a completed reduction of a safety limit cannot silently roll back;
//   - a damaged slot that the marker names is the interrupted write. It is ignored, so the
//     other slot (the previous record) stays in charge, only if the marker also records that
//     the store was armable when the write began: old or new after a power cut. Otherwise
//     (first provisioning, or reprovisioning a store that was already invalid) the cut
//     leaves Invalid / ProvisioningIncomplete, so a record that was not in charge before
//     cannot take over;
//   - otherwise as before: no record -> Unprovisioned; newest intact record valid ->
//     Provisioned; newest out-of-version, invalid content or two conflicting records at the
//     same generation -> Invalid.
// An older intact record never replaces a newer one that is unusable, so a superseded
// configuration cannot come back silently.
//
// A damaged marker means the cut came while the marker itself was written, before any slot
// was touched, so the slots are judged as if no change were in progress. One fault at a
// time is assumed; two at once (a damaged marker and a damaged slot) fail closed. For the
// same reason a marker is never rewritten with the value it already holds: a retried reset
// or provisioning whose marker write is cut would otherwise tear the marker that was still
// protecting the slots from the first interruption.
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
    // The transaction marker: one more value, separate from both slots, with the same
    // read/write/erase semantics. A cut write may leave it torn, never a slot.
    [[nodiscard]] virtual StorageRead readMarker(std::span<std::uint8_t> buffer) noexcept = 0;
    [[nodiscard]] virtual bool writeMarker(std::span<const std::uint8_t> marker) noexcept = 0;
    [[nodiscard]] virtual bool eraseMarker() noexcept = 0;
    // Removes every stored BLE bond record (#157 factory-reset policy). Absent counts as erased.
    [[nodiscard]] virtual bool eraseControllerBonds() noexcept = 0;
};

// Transaction marker layout, little-endian and packed:
//   0 magic "MRTX"   4 version u8 (1)   5 kind u8 (MarkerKind)   6 slot u8   7 flags u8
//   8 CRC-32 of bytes 0..7
// `slot` is the slot being provisioned (0 = A, 1 = B). Flag bit 0 (kMarkerFallback): the
// store was armable when the provisioning began, so its previous record may stay in charge
// if the write is cut. A factory reset has slot 0 and no flags; other bits are reserved.
enum class MarkerKind : std::uint8_t { Provisioning = 1, FactoryReset = 2 };
inline constexpr std::array<std::uint8_t, 4> kMarkerMagic{'M', 'R', 'T', 'X'};
inline constexpr std::uint8_t kMarkerVersion = 1;
inline constexpr std::size_t kMarkerBytes = 12;
inline constexpr std::uint8_t kMarkerFallback = 1U << 0;
using MarkerBytes = std::array<std::uint8_t, kMarkerBytes>;

struct Marker {
    MarkerKind kind{MarkerKind::Provisioning};
    Slot slot{Slot::A};
    bool fallback{false};  // Provisioning only: the previous record was armable.
    friend constexpr bool operator==(const Marker&, const Marker&) = default;
};

[[nodiscard]] constexpr MarkerBytes encodeMarker(const Marker& marker) noexcept
{
    MarkerBytes out{};
    for (std::size_t i = 0; i < kMarkerMagic.size(); ++i) out[i] = kMarkerMagic[i];
    out[4] = kMarkerVersion;
    out[5] = static_cast<std::uint8_t>(marker.kind);
    const bool provisioning = marker.kind == MarkerKind::Provisioning;
    out[6] = provisioning ? static_cast<std::uint8_t>(marker.slot) : 0;
    out[7] = provisioning && marker.fallback ? kMarkerFallback : 0;
    const auto crc = crc32(std::span<const std::uint8_t>{out}.first(8));
    for (std::size_t i = 0; i < kChecksumBytes; ++i) out[8 + i] = static_cast<std::uint8_t>(crc >> (8 * i));
    return out;
}

// nullopt: not a complete, well-formed marker (torn, corrupt or unknown).
[[nodiscard]] constexpr std::optional<Marker> decodeMarker(std::span<const std::uint8_t> bytes) noexcept
{
    if (bytes.size() != kMarkerBytes) return std::nullopt;
    for (std::size_t i = 0; i < kMarkerMagic.size(); ++i)
        if (bytes[i] != kMarkerMagic[i]) return std::nullopt;
    std::uint32_t stored = 0;
    for (std::size_t i = 0; i < kChecksumBytes; ++i) stored |= static_cast<std::uint32_t>(bytes[8 + i]) << (8 * i);
    if (stored != crc32(bytes.first(8))) return std::nullopt;
    if (bytes[4] != kMarkerVersion) return std::nullopt;
    if (bytes[5] == static_cast<std::uint8_t>(MarkerKind::Provisioning) && bytes[6] <= 1
        && (bytes[7] & ~kMarkerFallback) == 0)
        return Marker{MarkerKind::Provisioning, static_cast<Slot>(bytes[6]), (bytes[7] & kMarkerFallback) != 0};
    if (bytes[5] == static_cast<std::uint8_t>(MarkerKind::FactoryReset) && bytes[6] == 0 && bytes[7] == 0)
        return Marker{MarkerKind::FactoryReset, Slot::A, false};
    return std::nullopt;
}

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

// What the transaction marker held when the store was loaded.
enum class PendingChange : std::uint8_t { None, Provisioning, FactoryReset, DamagedMarker };

[[nodiscard]] constexpr const char* toString(PendingChange change) noexcept
{
    switch (change) {
    case PendingChange::None: return "None";
    case PendingChange::Provisioning: return "Provisioning";
    case PendingChange::FactoryReset: return "FactoryReset";
    case PendingChange::DamagedMarker: return "DamagedMarker";
    }
    return "Unknown";
}

struct LoadResult {
    ProvisioningState state{ProvisioningState::StorageUnavailable};
    RecordError reason{RecordError::None};          // Why the state is Invalid.
    std::optional<ValidatedRobotConfig> config{};   // Set only when Provisioned.
    std::optional<std::uint32_t> generation{};      // Of the newest intact record.
    std::optional<Slot> newestSlot{};               // Slot of that record.
    PendingChange pending{PendingChange::None};     // An unfinished change (diagnostics).

    // The only condition under which the runtime may become ready.
    [[nodiscard]] bool armable() const noexcept { return state == ProvisioningState::Provisioned && config; }
};

enum class WriteOutcome : std::uint8_t {
    Written,
    RejectedInvalid,
    StorageUnavailable,
    WriteFailed,
    VerifyFailed,
    ResetIncomplete,  // A factory reset is unfinished; run it again before provisioning.
    CommitFailed,     // The record was written and read back, but the marker could not be erased.
};
enum class ResetOutcome : std::uint8_t {
    Reset,
    StorageUnavailable,
    BondEraseFailed,
    MarkerWriteFailed,
    StorageEraseFailed,
    VerifyFailed,
};

[[nodiscard]] constexpr const char* toString(WriteOutcome outcome) noexcept
{
    switch (outcome) {
    case WriteOutcome::Written: return "Written";
    case WriteOutcome::RejectedInvalid: return "RejectedInvalid";
    case WriteOutcome::StorageUnavailable: return "StorageUnavailable";
    case WriteOutcome::WriteFailed: return "WriteFailed";
    case WriteOutcome::VerifyFailed: return "VerifyFailed";
    case WriteOutcome::ResetIncomplete: return "ResetIncomplete";
    case WriteOutcome::CommitFailed: return "CommitFailed";
    }
    return "Unknown";
}

[[nodiscard]] constexpr const char* toString(ResetOutcome outcome) noexcept
{
    switch (outcome) {
    case ResetOutcome::Reset: return "Reset";
    case ResetOutcome::StorageUnavailable: return "StorageUnavailable";
    case ResetOutcome::BondEraseFailed: return "BondEraseFailed";
    case ResetOutcome::MarkerWriteFailed: return "MarkerWriteFailed";
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

    // Reads both slots and the marker; never writes.
    [[nodiscard]] LoadResult load() noexcept
    {
        std::array<SlotView, kSlots.size()> views{};
        for (const auto slot : kSlots) {
            auto& view = views[index(slot)];
            view = inspect(slot);
            if (view.kind == SlotView::Kind::Unreadable) return {};
        }
        const auto marker = inspectMarker();
        if (!marker) return {};

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
        result.pending = marker->pending;
        if (newest) {
            result.generation = newest->decoded.generation;
            result.newestSlot = newest->slot;
        }
        const auto invalid = [&result](RecordError reason) {
            result.state = ProvisioningState::Invalid;
            result.reason = reason;
            return result;
        };
        if (marker->pending == PendingChange::FactoryReset) return invalid(RecordError::FactoryResetIncomplete);
        for (const auto& view : views) {
            if (view.kind != SlotView::Kind::Damaged) continue;
            const bool interruptedWrite = marker->pending == PendingChange::Provisioning && marker->slot == view.slot;
            if (!interruptedWrite) return invalid(RecordError::CommittedRecordDamaged);
            if (!marker->fallback) return invalid(RecordError::ProvisioningIncomplete);
        }
        if (!newest) {
            result.state = ProvisioningState::Unprovisioned;
            return result;
        }
        if (conflict) return invalid(RecordError::ConflictingRecords);
        if (newest->decoded.error != RecordError::None) return invalid(newest->decoded.error);
        result.config = ValidatedRobotConfig::validated(newest->decoded.config);
        result.state = result.config ? ProvisioningState::Provisioned : ProvisioningState::Invalid;
        return result;
    }

    // Initial provisioning and deliberate reprovisioning. The draft is validated before
    // any write; the marker names the target slot, the new record goes to the slot without
    // the newest record, with the next generation, and is read back; only then is the marker
    // erased. Written is returned only after that: the change is complete. A failure at any
    // point leaves the previous newest record (or a non-armable state) in charge.
    [[nodiscard]] ProvisionResult provision(const RobotConfig& config) noexcept
    {
        if (const auto error = validate(config); error != RecordError::None)
            return {WriteOutcome::RejectedInvalid, error, load()};
        const auto before = load();
        if (before.state == ProvisioningState::StorageUnavailable)
            return {WriteOutcome::StorageUnavailable, RecordError::None, before};
        // Overwriting the reset marker would let a record the reset was erasing come back.
        if (before.pending == PendingChange::FactoryReset)
            return {WriteOutcome::ResetIncomplete, RecordError::None, before};
        const Slot target = before.newestSlot ? otherSlot(*before.newestSlot) : Slot::A;
        const std::uint32_t generation = before.generation ? *before.generation + 1 : kFirstGeneration;
        if (!putMarker({MarkerKind::Provisioning, target, before.armable()}))
            return {WriteOutcome::WriteFailed, RecordError::None, load()};
        const auto bytes = encode(config, generation);
        if (!storage_.write(target, bytes)) return {WriteOutcome::WriteFailed, RecordError::None, load()};
        if (const auto written = load(); !holds(written, config, generation, target))
            return {WriteOutcome::VerifyFailed, written.reason, written};
        if (!storage_.eraseMarker()) return {WriteOutcome::CommitFailed, RecordError::None, load()};
        auto after = load();
        if (!holds(after, config, generation, target) || after.pending != PendingChange::None)
            return {WriteOutcome::VerifyFailed, after.reason, after};
        return {WriteOutcome::Written, RecordError::None, after};
    }

    // Factory reset: bonds first, then the reset marker, then the older slot, the newest one
    // and finally the marker. While the marker is present the store cannot arm, whatever the
    // slots hold, so an interruption leaves either the unchanged store (cut before the marker)
    // or a non-armable one; running the reset again completes it. In particular, an
    // interrupted reset can never make a previously non-armable store (for example two
    // conflicting records) armable. A slot or marker that cannot be read refuses the reset
    // before anything is erased (StorageUnavailable).
    [[nodiscard]] ResetResult factoryReset() noexcept
    {
        const auto before = load();
        if (before.state == ProvisioningState::StorageUnavailable) return {ResetOutcome::StorageUnavailable, before};
        if (!storage_.eraseControllerBonds()) return {ResetOutcome::BondEraseFailed, before};
        if (!putMarker({MarkerKind::FactoryReset, Slot::A, false})) return {ResetOutcome::MarkerWriteFailed, load()};
        const Slot last = before.newestSlot.value_or(Slot::B);
        if (!storage_.erase(otherSlot(last)) || !storage_.erase(last) || !storage_.eraseMarker())
            return {ResetOutcome::StorageEraseFailed, load()};
        auto after = load();
        if (after.state != ProvisioningState::Unprovisioned || after.pending != PendingChange::None)
            return {ResetOutcome::VerifyFailed, after};
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

    struct MarkerView {
        PendingChange pending{PendingChange::None};
        Slot slot{Slot::A};    // The slot being provisioned when pending is Provisioning.
        bool fallback{false};  // The previous record may stay in charge if that write was cut.
    };

    [[nodiscard]] static constexpr std::size_t index(Slot slot) noexcept { return static_cast<std::size_t>(slot); }

    [[nodiscard]] static bool holds(const LoadResult& state, const RobotConfig& config, std::uint32_t generation,
                                    Slot slot) noexcept
    {
        return state.armable() && state.generation == generation && state.newestSlot == slot
            && state.config->raw() == config;
    }

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

    // Writes the marker unless it already holds exactly this value.
    [[nodiscard]] bool putMarker(const Marker& marker) noexcept
    {
        MarkerBytes stored{};
        const auto read = storage_.readMarker(stored);
        if (read.status == StorageStatus::Ok && read.length == stored.size() && decodeMarker(stored) == marker)
            return true;
        return storage_.writeMarker(encodeMarker(marker));
    }

    // nullopt: the marker cannot be read.
    [[nodiscard]] std::optional<MarkerView> inspectMarker() noexcept
    {
        MarkerBytes bytes{};
        const auto read = storage_.readMarker(bytes);
        switch (read.status) {
        case StorageStatus::NotFound: return MarkerView{};
        case StorageStatus::TooLarge: return MarkerView{PendingChange::DamagedMarker};
        case StorageStatus::Failed: return std::nullopt;
        case StorageStatus::Ok: break;
        }
        if (read.length > bytes.size()) return MarkerView{PendingChange::DamagedMarker};
        const auto marker = decodeMarker(std::span<const std::uint8_t>{bytes}.first(read.length));
        if (!marker) return MarkerView{PendingChange::DamagedMarker};
        if (marker->kind == MarkerKind::FactoryReset) return MarkerView{PendingChange::FactoryReset};
        return MarkerView{PendingChange::Provisioning, marker->slot, marker->fallback};
    }

    IRecordStorage& storage_;
};
} // namespace codecell::provisioning
