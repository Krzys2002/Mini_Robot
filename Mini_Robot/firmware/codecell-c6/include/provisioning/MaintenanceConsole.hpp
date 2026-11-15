#pragma once
// Local USB-serial maintenance path (#159; #157 F8). Line commands for status,
// initial provisioning, deliberate reprovisioning and factory reset. Portable C++20:
// the serial port, restart and runtime are injected, so the console is host-tested.
//
// Rules:
// - There is no motion, clear or JSON command here; the console cannot request motion.
// - Every mutating command needs `maintenance` first. Entering maintenance stops the
//   runtime, clears active intent, disables the drive and refuses motion until reset;
//   it is one-way. If the drive disable cannot be confirmed, maintenance (and every
//   write) is refused.
// - A commit needs every field set explicitly (or copied from the valid stored record)
//   and the RobotId typed again as confirmation, because every uint32, including 0,
//   is a valid RobotId and a forgotten value must not become an identity.
// - A successful commit or factory reset restarts the robot, which boots Idle with
//   outputs disabled and moves only after a new session sends new Velocity frames.
// - No field is a credential; the restricted-lab profile has none (#157).
//
// Runs in the lowest-priority loop context. Flash writes happen only here, with outputs
// already disabled, so a flash stall cannot delay an enabled-output safety decision.
#include "provisioning/ConfigStore.hpp"
#include "provisioning/RobotConfigRecord.hpp"
#include "provisioning/RobotIdentity.hpp"
#include <array>
#include <charconv>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <string_view>

namespace codecell::provisioning {

class IMaintenanceOutput {
public:
    virtual ~IMaintenanceOutput() = default;
    virtual void line(const char* text) noexcept = 0;
};

class IMaintenanceTarget {
public:
    virtual ~IMaintenanceTarget() = default;
    // Stop, clear intent, disable the drive and refuse motion until reset. False when
    // the drive disable could not be confirmed.
    [[nodiscard]] virtual bool enterMaintenance() noexcept = 0;
    // Production: flush serial and reset the chip (does not return).
    virtual void restart() noexcept = 0;
};

// Maximum accepted line, terminator excluded. Longer lines are discarded by the reader.
inline constexpr std::size_t kMaintenanceLineMax = 96;
inline constexpr std::string_view kFactoryResetConfirmation = "ERASE";

enum class ConfigField : std::uint8_t {
    RobotId,
    BoardProfile,
    Capabilities,
    DrivePolarity,
    MaxOutputPercent,
    WheelDiameterUm,
    TrackWidthUm,
    MaxWheelSpeedMmS,
    MaxLinearMmS,
    MaxAngularMradS,
    MaxValidityMs,
    TelemetryPeriodMs,
    Count_,
};

inline constexpr std::size_t kConfigFieldCount = static_cast<std::size_t>(ConfigField::Count_);

struct ConfigFieldInfo {
    ConfigField field;
    std::string_view name;
    std::uint32_t maxStorable;  // Width of the stored integer; content rules are validate().
};

inline constexpr std::uint32_t kU8Max = 0xFFU;
inline constexpr std::uint32_t kU16Max = 0xFFFFU;
inline constexpr std::uint32_t kU32Max = 0xFFFFFFFFU;

inline constexpr std::array<ConfigFieldInfo, kConfigFieldCount> kConfigFields{{
    {ConfigField::RobotId, "robot_id", kU32Max},
    {ConfigField::BoardProfile, "board_profile", kU8Max},
    {ConfigField::Capabilities, "capabilities", kU8Max},
    {ConfigField::DrivePolarity, "drive_polarity", kU8Max},
    {ConfigField::MaxOutputPercent, "max_output_percent", kU8Max},
    {ConfigField::WheelDiameterUm, "wheel_diameter_um", kU32Max},
    {ConfigField::TrackWidthUm, "track_width_um", kU32Max},
    {ConfigField::MaxWheelSpeedMmS, "max_wheel_speed_mm_s", kU16Max},
    {ConfigField::MaxLinearMmS, "max_linear_mm_s", kU16Max},
    {ConfigField::MaxAngularMradS, "max_angular_mrad_s", kU16Max},
    {ConfigField::MaxValidityMs, "max_validity_ms", kU16Max},
    {ConfigField::TelemetryPeriodMs, "telemetry_period_ms", kU16Max},
}};

[[nodiscard]] constexpr std::uint32_t fieldValue(const RobotConfig& c, ConfigField field) noexcept
{
    switch (field) {
    case ConfigField::RobotId: return c.robotId;
    case ConfigField::BoardProfile: return c.boardProfile;
    case ConfigField::Capabilities: return c.requiredCapabilities;
    case ConfigField::DrivePolarity: return c.drivePolarity;
    case ConfigField::MaxOutputPercent: return c.maxDriveOutputPercent;
    case ConfigField::WheelDiameterUm: return c.wheelDiameterMicrometres;
    case ConfigField::TrackWidthUm: return c.trackWidthMicrometres;
    case ConfigField::MaxWheelSpeedMmS: return c.maxWheelSpeedMillimetresPerSecond;
    case ConfigField::MaxLinearMmS: return c.maxLinearMillimetresPerSecond;
    case ConfigField::MaxAngularMradS: return c.maxAngularMilliradiansPerSecond;
    case ConfigField::MaxValidityMs: return c.maxCommandValidityMilliseconds;
    case ConfigField::TelemetryPeriodMs: return c.telemetryPeriodMilliseconds;
    case ConfigField::Count_: break;
    }
    return 0;
}

// The caller has checked `value` against the field's maxStorable.
constexpr void setFieldValue(RobotConfig& c, ConfigField field, std::uint32_t value) noexcept
{
    const auto u8 = static_cast<std::uint8_t>(value);
    const auto u16 = static_cast<std::uint16_t>(value);
    switch (field) {
    case ConfigField::RobotId: c.robotId = value; break;
    case ConfigField::BoardProfile: c.boardProfile = u8; break;
    case ConfigField::Capabilities: c.requiredCapabilities = u8; break;
    case ConfigField::DrivePolarity: c.drivePolarity = u8; break;
    case ConfigField::MaxOutputPercent: c.maxDriveOutputPercent = u8; break;
    case ConfigField::WheelDiameterUm: c.wheelDiameterMicrometres = value; break;
    case ConfigField::TrackWidthUm: c.trackWidthMicrometres = value; break;
    case ConfigField::MaxWheelSpeedMmS: c.maxWheelSpeedMillimetresPerSecond = u16; break;
    case ConfigField::MaxLinearMmS: c.maxLinearMillimetresPerSecond = u16; break;
    case ConfigField::MaxAngularMradS: c.maxAngularMilliradiansPerSecond = u16; break;
    case ConfigField::MaxValidityMs: c.maxCommandValidityMilliseconds = u16; break;
    case ConfigField::TelemetryPeriodMs: c.telemetryPeriodMilliseconds = u16; break;
    case ConfigField::Count_: break;
    }
}

[[nodiscard]] constexpr std::optional<std::uint32_t> parseUnsigned(std::string_view text) noexcept
{
    std::uint32_t value = 0;
    if (text.empty()) return std::nullopt;
    const auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), value);
    if (error != std::errc{} || end != text.data() + text.size()) return std::nullopt;
    return value;
}

class MaintenanceConsole {
public:
    MaintenanceConsole(ConfigStore& store, IMaintenanceTarget& target, IMaintenanceOutput& output) noexcept
        : store_(store), target_(target), output_(output)
    {
    }

    [[nodiscard]] bool inMaintenance() const noexcept { return maintenance_; }

    void handleLine(std::string_view line) noexcept
    {
        std::array<std::string_view, kMaxTokens + 1> tokens{};
        std::size_t count = 0;
        while (!line.empty()) {
            const auto start = line.find_first_not_of(" \t");
            if (start == std::string_view::npos) break;
            line.remove_prefix(start);
            const auto end = line.find_first_of(" \t");
            if (count == tokens.size()) {
                print("ERR too many arguments");
                return;
            }
            tokens[count++] = line.substr(0, end);
            line.remove_prefix(end == std::string_view::npos ? line.size() : end);
        }
        if (count == 0) return;
        if (count > kMaxTokens) {
            print("ERR too many arguments");
            return;
        }
        const auto command = tokens[0];
        const auto argument = [&](std::size_t i) { return i < count ? tokens[i] : std::string_view{}; };
        const std::size_t arguments = count - 1;
        if (command == "help" && arguments == 0) return help();
        if (command == "status" && arguments == 0) return status();
        if (command == "draft" && arguments == 0) return showDraft();
        if (command == "maintenance" && arguments == 0) return enter();
        if (command == "set" && arguments == 2) return set(argument(1), argument(2));
        if (command == "commit" && arguments == 1) return commit(argument(1));
        if (command == "factory-reset" && arguments == 1) return factoryReset(argument(1));
        if (command == "restart" && arguments == 0) return restart();
        print("ERR unknown command or wrong arguments; type help");
    }

private:
    static constexpr std::size_t kMaxTokens = 3;
    static constexpr std::size_t kLineBuffer = 192;
    static constexpr std::uint16_t kAllFields = (1U << kConfigFieldCount) - 1U;
    static_assert(kConfigFieldCount <= 16, "assigned-field mask is 16 bits");

    void print(const char* text) noexcept { output_.line(text); }

    template <typename... Args>
    void printLine(const char* format, Args... args) noexcept
    {
        std::array<char, kLineBuffer> buffer{};
        std::snprintf(buffer.data(), buffer.size(), format, args...);
        output_.line(buffer.data());
    }

    void help() noexcept
    {
        print("Maintenance commands (no motion over USB):");
        print("  status | draft | help");
        print("  maintenance            stop, clear intent, disable drive, refuse motion until restart");
        print("  set <field> <uint>     edit the draft (maintenance only)");
        print("  commit <robot_id>      validate, store atomically and restart (maintenance only)");
        print("  factory-reset ERASE    erase identity, configuration and BLE bonds, then restart");
        print("  restart");
        for (const auto& info : kConfigFields) printLine("  field %.*s", static_cast<int>(info.name.size()), info.name.data());
    }

    void status() noexcept
    {
        const auto stored = store_.load();
        printLine("PROV state=%s reason=%s generation=%lu slot=%s maintenance=%d", toString(stored.state),
               toString(stored.reason), static_cast<unsigned long>(stored.generation.value_or(0)),
               stored.newestSlot ? toString(*stored.newestSlot) : "-", maintenance_ ? 1 : 0);
        if (stored.config) {
            const auto name = advertisedName(stored.config->robotId());
            printLine("PROV robot_id=%lu name=%s", static_cast<unsigned long>(stored.config->robotId()), name.data());
            printFields("CONFIG", stored.config->raw(), kAllFields);
        }
    }

    void showDraft() noexcept
    {
        if (!maintenance_) {
            print("ERR not in maintenance");
            return;
        }
        printFields("DRAFT", draft_, assigned_);
    }

    void printFields(const char* tag, const RobotConfig& config, std::uint16_t assigned) noexcept
    {
        for (std::size_t i = 0; i < kConfigFields.size(); ++i) {
            const auto& info = kConfigFields[i];
            if ((assigned & (1U << i)) != 0)
                printLine("%s %.*s=%lu", tag, static_cast<int>(info.name.size()), info.name.data(),
                       static_cast<unsigned long>(fieldValue(config, info.field)));
            else
                printLine("%s %.*s=unset", tag, static_cast<int>(info.name.size()), info.name.data());
        }
    }

    void enter() noexcept
    {
        if (!maintenance_) {
            if (!target_.enterMaintenance()) {
                print("ERR maintenance refused: drive disable not confirmed; no configuration change");
                return;
            }
            maintenance_ = true;
            const auto stored = store_.load();
            if (stored.config) {
                draft_ = stored.config->raw();
                assigned_ = kAllFields;
            } else {
                draft_ = RobotConfig{};
                assigned_ = 0;
            }
        }
        print("OK maintenance outputs=disabled intent=cleared motion=refused-until-restart");
    }

    void set(std::string_view name, std::string_view text) noexcept
    {
        if (!maintenance_) {
            print("ERR not in maintenance");
            return;
        }
        for (std::size_t i = 0; i < kConfigFields.size(); ++i) {
            const auto& info = kConfigFields[i];
            if (info.name != name) continue;
            const auto value = parseUnsigned(text);
            if (!value || *value > info.maxStorable) {
                printLine("ERR %.*s needs a decimal integer 0..%lu", static_cast<int>(info.name.size()), info.name.data(),
                       static_cast<unsigned long>(info.maxStorable));
                return;
            }
            setFieldValue(draft_, info.field, *value);
            assigned_ = static_cast<std::uint16_t>(assigned_ | (1U << i));
            printLine("OK %.*s=%lu", static_cast<int>(info.name.size()), info.name.data(),
                   static_cast<unsigned long>(*value));
            return;
        }
        print("ERR unknown field; type help");
    }

    void commit(std::string_view confirmation) noexcept
    {
        if (!maintenance_) {
            print("ERR not in maintenance");
            return;
        }
        for (std::size_t i = 0; i < kConfigFields.size(); ++i) {
            if ((assigned_ & (1U << i)) == 0) {
                const auto& info = kConfigFields[i];
                printLine("ERR draft incomplete: %.*s unset", static_cast<int>(info.name.size()), info.name.data());
                return;
            }
        }
        const auto confirmed = parseUnsigned(confirmation);
        if (!confirmed || *confirmed != draft_.robotId) {
            print("ERR confirmation must repeat the draft robot_id");
            return;
        }
        const auto result = store_.provision(draft_);
        if (result.outcome != WriteOutcome::Written) {
            printLine("ERR commit outcome=%s error=%s stored=%s", toString(result.outcome), toString(result.error),
                   toString(result.after.state));
            return;
        }
        printLine("OK committed robot_id=%lu generation=%lu slot=%s; restarting",
               static_cast<unsigned long>(draft_.robotId),
               static_cast<unsigned long>(result.after.generation.value_or(0)),
               result.after.newestSlot ? toString(*result.after.newestSlot) : "-");
        target_.restart();
    }

    void factoryReset(std::string_view confirmation) noexcept
    {
        if (!maintenance_) {
            print("ERR not in maintenance");
            return;
        }
        if (confirmation != kFactoryResetConfirmation) {
            print("ERR factory-reset needs the confirmation word ERASE");
            return;
        }
        const auto result = store_.factoryReset();
        if (result.outcome != ResetOutcome::Reset) {
            printLine("ERR factory-reset outcome=%s stored=%s; run it again", toString(result.outcome),
                   toString(result.after.state));
            return;
        }
        draft_ = RobotConfig{};
        assigned_ = 0;
        print("OK factory-reset state=Unprovisioned bonds=erased; restarting");
        target_.restart();
    }

    void restart() noexcept
    {
        // Clear intent first even here; a reset never resumes motion either way.
        if (!maintenance_ && target_.enterMaintenance()) maintenance_ = true;
        print("OK restarting");
        target_.restart();
    }

    ConfigStore& store_;
    IMaintenanceTarget& target_;
    IMaintenanceOutput& output_;
    bool maintenance_{false};
    RobotConfig draft_{};
    std::uint16_t assigned_{0};
};
} // namespace codecell::provisioning
