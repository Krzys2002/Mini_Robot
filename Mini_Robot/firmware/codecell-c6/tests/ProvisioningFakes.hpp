#pragma once
// Deterministic fakes for the #159 provisioning ports. The storage fake can cut power
// after a chosen number of mutating operations and tear the interrupted write at any
// byte, which models the worst case (a partial value persisted) for every step.
#include "provisioning/ConfigStore.hpp"
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace codecell::provisioning::fakes {

// Test values that satisfy validate(). They are NOT CodeCell calibration data.
inline RobotConfig validConfig(RobotId id = 7)
{
    RobotConfig c;
    c.robotId = id;
    c.boardProfile = static_cast<std::uint8_t>(BoardProfile::CodeCellC6Drive);
    c.requiredCapabilities = capability::kDifferentialDrive;
    c.drivePolarity = polarity::kRightReversed;
    c.maxDriveOutputPercent = 40;
    c.wheelDiameterMicrometres = 17'000;
    c.trackWidthMicrometres = 60'000;
    c.maxWheelSpeedMillimetresPerSecond = 200;
    c.maxLinearMillimetresPerSecond = 150;
    c.maxAngularMilliradiansPerSecond = 3'000;
    c.maxCommandValidityMilliseconds = 200;
    c.telemetryPeriodMilliseconds = static_cast<std::uint16_t>(timing::kTelemetryPeriod.count());
    return c;
}

class FakeRecordStorage final : public IRecordStorage {
public:
    std::array<std::optional<std::vector<std::uint8_t>>, kSlots.size()> slots{};
    bool bondsPresent{false};
    std::array<bool, kSlots.size()> failRead{false, false};
    bool failWrite{false};
    bool failBondErase{false};
    bool dropWrites{false};  // Reports success but stores nothing (verify must catch it).

    // Power cut: after `mutationsBeforeCut` successful mutations, the next mutation is
    // interrupted (a write keeps only `tornBytes` bytes) and nothing further persists.
    std::optional<int> mutationsBeforeCut{};
    std::size_t tornBytes{0};
    bool powerLost{false};

    int reads{0};
    int writes{0};
    int erases{0};

    StorageRead read(Slot slot, std::span<std::uint8_t> buffer) noexcept override
    {
        ++reads;
        if (failRead[index(slot)]) return {StorageStatus::Failed, 0};
        const auto& value = slots[index(slot)];
        if (!value) return {StorageStatus::NotFound, 0};
        if (value->size() > buffer.size()) return {StorageStatus::TooLarge, 0};
        std::copy(value->begin(), value->end(), buffer.begin());
        return {StorageStatus::Ok, value->size()};
    }

    bool write(Slot slot, std::span<const std::uint8_t> record) noexcept override
    {
        ++writes;
        if (failWrite || powerLost) return false;
        if (cutNow()) {
            slots[index(slot)] = std::vector<std::uint8_t>(record.begin(),
                                                           record.begin() + std::min(tornBytes, record.size()));
            return false;
        }
        if (!dropWrites) slots[index(slot)] = std::vector<std::uint8_t>(record.begin(), record.end());
        return true;
    }

    bool erase(Slot slot) noexcept override
    {
        ++erases;
        if (powerLost || cutNow()) return false;
        slots[index(slot)].reset();
        return true;
    }

    bool eraseControllerBonds() noexcept override
    {
        if (failBondErase || powerLost || cutNow()) return false;
        bondsPresent = false;
        return true;
    }

    void put(Slot slot, std::vector<std::uint8_t> bytes) { slots[index(slot)] = std::move(bytes); }
    void put(Slot slot, const RecordBytes& bytes) { slots[index(slot)] = std::vector<std::uint8_t>(bytes.begin(), bytes.end()); }
    // Restores power with the persisted contents unchanged (a reboot).
    void reboot()
    {
        powerLost = false;
        mutationsBeforeCut.reset();
    }
    [[nodiscard]] int mutations() const { return writes + erases; }

private:
    static std::size_t index(Slot slot) { return static_cast<std::size_t>(slot); }
    bool cutNow()
    {
        if (!mutationsBeforeCut) return false;
        if (*mutationsBeforeCut > 0) {
            --*mutationsBeforeCut;
            return false;
        }
        powerLost = true;
        return true;
    }
};

} // namespace codecell::provisioning::fakes
